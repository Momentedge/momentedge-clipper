//! Shared harness for the live ROS2 end-to-end suite (`tests/e2e.rs`).
//!
//! The test process spawns everything as child processes — a continuous
//! `ros2 bag record` (the production recording the extractor tails), a `ros2
//! topic pub` message source, the extractor binary under test, and a `ros2
//! topic echo` listener for `Recorded` announcements — and creates no ROS node
//! of its own, keeping the test free of process-global DDS state.
//!
//! Isolation: every test gets its own `ROS_DOMAIN_ID` (band 80–101, clear of
//! the host's domain 0) and its own temp tree (`record/`, `triggered/`,
//! `logs/`). Teardown: every child is spawned into its own process group and
//! [`Proc`]'s `Drop` SIGTERMs then SIGKILLs that group, so a panicking test
//! strands nothing; the nextest leak-timeout is the backstop. Child
//! stdout/stderr are teed to per-test log files for post-mortem.
//!
//! Bring-up is composed through methods on [`TestEnv`] rather than through
//! rstest fixtures depending on a `domain`/`tmp` fixture: rstest resolves a
//! fixture fresh at every injection site, so two fixtures sharing a `domain`
//! dependency would receive two *different* domains. One `TestEnv` per test
//! carries the shared identity instead.

use std::fs::File;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use std::process::{Child, Command, ExitStatus, Stdio};
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

pub const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";
pub const RECORDED_TOPIC: &str = "/events/momentedge/recorded";

/// Gate for the whole suite. Unset `CLIPPER_E2E` means skip-and-pass, so
/// plain `cargo test` / `cargo llvm-cov` stay green without ROS. Set, a
/// missing prerequisite is a loud panic — a misconfigured "enabled" run must
/// fail, never silently skip.
pub fn require_e2e() -> bool {
    if std::env::var_os("CLIPPER_E2E").is_none() {
        eprintln!(
            "skipping: CLIPPER_E2E is unset \
             (run inside the dev shell: CLIPPER_E2E=1 cargo nextest run --profile e2e)"
        );
        return false;
    }
    let out = Command::new("ros2")
        .args(["pkg", "prefix", "momentedge_msgs"])
        .output()
        .expect("CLIPPER_E2E is set but `ros2` is not on PATH — run inside `nix develop`");
    assert!(
        out.status.success(),
        "CLIPPER_E2E is set but `momentedge_msgs` does not resolve from \
         AMENT_PREFIX_PATH — run inside `nix develop`: {}",
        String::from_utf8_lossy(&out.stderr),
    );
    true
}

/// Opt-out gate for tests the project flags as inherently flaky under CI-grade
/// timing. Set `CLIPPER_E2E_SKIP_FLAKY` (CI does) to skip them; unset
/// locally, so the full suite — including the live-corruption race
/// `corrupt_tail_health_live` — runs. A skipped test returns early and so
/// reports as passed, the same skip-and-pass convention as [`require_e2e`].
pub fn skip_flaky() -> bool {
    if std::env::var_os("CLIPPER_E2E_SKIP_FLAKY").is_some() {
        eprintln!("skipping: flaky-under-CI test (CLIPPER_E2E_SKIP_FLAKY is set)");
        return true;
    }
    false
}

/// Poll `path` until it contains `needle`, or `timeout` elapses. Used to
/// confirm a child logged an expected line when only its log file (not its
/// [`Proc`]) is in scope.
fn wait_for_file_contains(path: &Path, needle: &str, timeout: Duration) -> bool {
    let deadline = Instant::now() + timeout;
    loop {
        if std::fs::read_to_string(path)
            .unwrap_or_default()
            .contains(needle)
        {
            return true;
        }
        if Instant::now() >= deadline {
            return false;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}

/// Nanoseconds since the Unix epoch on the system clock — the time base the
/// recorder, the trigger stamp, and MCAP `log_time` all share.
pub fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock before the epoch")
        .as_nanos() as u64
}

/// A `ROS_DOMAIN_ID` unique to this test: nextest runs each test in its own
/// process, so a plain counter would restart identically everywhere — the PID
/// disambiguates across test processes, the counter within one. Band 80–101
/// keeps clear of the host's domain 0 and stays inside the range whose DDS
/// ports all fit the default port plan.
fn unique_domain() -> u32 {
    static SEQ: AtomicU32 = AtomicU32::new(0);
    80 + (std::process::id() + SEQ.fetch_add(1, Ordering::Relaxed)) % 22
}

/// One test's isolated world: a unique DDS domain and a temp tree holding the
/// recording, the clips, and the child logs.
pub struct TestEnv {
    pub domain: u32,
    root: tempfile::TempDir,
    /// Whether the running recorder subscribes to the trigger topic (it does
    /// when recording `--all`), so [`Self::fire_trigger`] knows whether to wait
    /// for one matched subscriber (the extractor) or two (the extractor and the
    /// recorder). Set by [`Self::start_recorder`] / [`Self::start_recorder_topics`].
    records_trigger_topic: AtomicBool,
}

impl TestEnv {
    pub fn new() -> Self {
        let root = tempfile::Builder::new()
            .prefix("clipper-e2e-")
            .tempdir()
            .expect("creating the test temp dir");
        let env = TestEnv {
            domain: unique_domain(),
            root,
            records_trigger_topic: AtomicBool::new(false),
        };
        std::fs::create_dir_all(env.log_dir()).expect("creating the log dir");
        eprintln!(
            "e2e env: ROS_DOMAIN_ID={} root={}",
            env.domain,
            env.root.path().display()
        );
        env
    }

    pub fn record_dir(&self) -> PathBuf {
        self.root.path().join("record")
    }

    pub fn out_dir(&self) -> PathBuf {
        self.root.path().join("triggered")
    }

    fn log_dir(&self) -> PathBuf {
        self.root.path().join("logs")
    }

    /// Base command in this test's world: the test domain and the temp root
    /// as working directory (so the extractor finds no stray TOML config).
    fn command(&self, program: impl AsRef<std::ffi::OsStr>) -> Command {
        let mut cmd = Command::new(program);
        cmd.env("ROS_DOMAIN_ID", self.domain.to_string())
            .current_dir(self.root.path());
        cmd
    }

    /// Spawn `cmd` into its own process group with stdout/stderr teed to
    /// `logs/<name>.log`, wrapped in the kill-on-drop [`Proc`] guard.
    fn spawn(&self, name: &str, mut cmd: Command) -> Proc {
        use std::os::unix::process::CommandExt;
        let log = self.log_dir().join(format!("{name}.log"));
        let out = File::create(&log).expect("creating the child log file");
        let err = out.try_clone().expect("cloning the child log handle");
        let child = cmd
            .stdin(Stdio::null())
            .stdout(Stdio::from(out))
            .stderr(Stdio::from(err))
            .process_group(0)
            .spawn()
            .unwrap_or_else(|e| panic!("spawning {name}: {e}"));
        Proc {
            name: name.to_string(),
            child,
            log,
        }
    }

    /// The production recording the extractor tails: a continuous `ros2 bag
    /// record --all` writing one growing MCAP file into this test's `record/`.
    pub fn start_recorder(&self, preset: &str, cache: u64) -> Proc {
        // `--all` records every live topic, so the recorder also subscribes to
        // the trigger topic; `fire_trigger` then waits for two matched
        // subscribers (the extractor and the recorder).
        self.spawn_recorder(&["--all"], preset, cache, true, 0)
    }

    /// [`Self::start_recorder`] restricted to exactly `topics` (via
    /// `--topics`), for tests that must keep ambient topics (`/rosout`, the
    /// trigger) out of the recording.
    pub fn start_recorder_topics(&self, topics: &[&str], preset: &str, cache: u64) -> Proc {
        let mut selection = vec!["--topics"];
        selection.extend_from_slice(topics);
        // A topic-restricted recorder does not subscribe to the trigger topic,
        // so the extractor is the sole trigger subscriber (`fire_trigger` -w 1).
        self.spawn_recorder(&selection, preset, cache, false, 0)
    }

    /// [`Self::start_recorder`] that rolls the bag over every
    /// `max_bag_duration_secs` (`--max-bag-duration`), keeping every split on
    /// disk — the rosbag2 split recording a clip window can straddle, which the
    /// collection tailer recovers across.
    pub fn start_recorder_split(
        &self,
        preset: &str,
        cache: u64,
        max_bag_duration_secs: u64,
    ) -> Proc {
        self.spawn_recorder(&["--all"], preset, cache, true, max_bag_duration_secs)
    }

    /// Spawn `ros2 bag record <selection> --storage mcap
    /// --storage-preset-profile <preset> --max-cache-size <cache> --output
    /// record/` — the same invocation the examples document. `record/` is wiped
    /// first: rosbag2 refuses to record into an existing bag directory, and a
    /// recorder restart relies on the wipe so the extractor's tail sees a fresh
    /// inode (its replacement signal). `split_secs > 0` adds `--max-bag-duration`
    /// so the bag rolls over into numbered splits.
    fn spawn_recorder(
        &self,
        selection: &[&str],
        preset: &str,
        cache: u64,
        records_trigger: bool,
        split_secs: u64,
    ) -> Proc {
        let _ = std::fs::remove_dir_all(self.record_dir());
        let cache = cache.to_string();
        let split = split_secs.to_string();
        let mut cmd = self.command("ros2");
        cmd.args(["bag", "record"])
            .args(selection)
            .args(["--storage", "mcap", "--storage-preset-profile", preset])
            .args(["--max-cache-size", &cache]);
        // 0 means no duration split (rosbag2's own default), so only pass the
        // flag when a test asks the bag to roll over.
        if split_secs > 0 {
            cmd.args(["--max-bag-duration", &split]);
        }
        cmd.arg("--output").arg(self.record_dir());
        self.records_trigger_topic
            .store(records_trigger, Ordering::Relaxed);
        self.spawn("recorder", cmd)
    }

    /// Block until the recorder has created its MCAP file; returns its path.
    pub fn wait_for_recording(&self, timeout: Duration) -> PathBuf {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(p) = self.newest_recording() {
                return p;
            }
            assert!(
                Instant::now() < deadline,
                "no .mcap appeared under {} within {timeout:?} — recorder log: see logs/recorder.log",
                self.record_dir().display(),
            );
            std::thread::sleep(Duration::from_millis(200));
        }
    }

    /// Delete the recording file out from under the live recorder — the
    /// external-cleanup fault the deletion tests inject. The recorder keeps
    /// appending to the unlinked inode; the tail sees the path vanish.
    pub fn delete_recording(&self) {
        std::fs::remove_file(self.newest_recording().expect("the recording exists"))
            .expect("deleting the recording");
    }

    /// The production recorder-restart protocol: stop the recorder cleanly,
    /// relaunch it ([`Self::start_recorder`] wipes the bag dir), and gate on the
    /// extractor noticing the replacement and the new recording existing on
    /// disk. Returns the new recorder and the instant between the two
    /// recorders' lifetimes — every message in the new recording is stamped
    /// at receive and therefore at/after it.
    pub fn restart_recorder(
        &self,
        recorder: &mut Proc,
        extractor: &Proc,
        preset: &str,
        cache: u64,
    ) -> (Proc, u64) {
        recorder.stop(libc::SIGINT, Duration::from_secs(30));
        let restart_ns = now_ns();
        let recorder2 = self.start_recorder(preset, cache);
        // The tail indexes the replacement recording: the startup recording is
        // the first "indexing recording" line, the restart's fresh inode the
        // second (the bag dir is wiped and recreated with the same name, so the
        // path repeats — the second occurrence is the signal).
        extractor.expect_log_count("indexing recording", 2, Duration::from_secs(60));
        self.wait_for_recording(Duration::from_secs(60));
        (recorder2, restart_ns)
    }

    /// Newest `*.mcap` under `record/` by mtime — the same startup-adoption rule
    /// the extractor's tail uses to pick the live recording.
    pub fn newest_recording(&self) -> Option<PathBuf> {
        let entries = std::fs::read_dir(self.record_dir()).ok()?;
        entries
            .flatten()
            .map(|e| e.path())
            .filter(|p| p.extension().is_some_and(|x| x == "mcap"))
            .max_by_key(|p| {
                p.metadata()
                    .map(|m| (m.mtime(), m.mtime_nsec()))
                    .unwrap_or((i64::MIN, i64::MIN))
            })
    }

    /// A steady message stream so the bag has content and the tail's coverage
    /// high-water advances. Runs until dropped.
    pub fn start_source(&self, topic: &str, rate: u32) -> Proc {
        let payload = "clipper-e2e-payload ".repeat(10);
        let mut cmd = self.command("ros2");
        cmd.args([
            "topic",
            "pub",
            "-r",
            &rate.to_string(),
            topic,
            "std_msgs/msg/String",
            &format!("{{data: {payload}}}"),
        ]);
        self.spawn("source", cmd)
    }

    /// The binary under test, configured purely via `MOMENTEDGE_*` env onto
    /// this test's temp tree. Blocks until its "up" line is logged.
    pub fn start_extractor(&self, grace_secs: u64) -> Proc {
        let mut cmd = self.command(env!("CARGO_BIN_EXE_clipper"));
        cmd.env("MOMENTEDGE_RECORD_DIR", self.record_dir())
            .env("MOMENTEDGE_OUT_DIR", self.out_dir())
            .env("MOMENTEDGE_GRACE_SECS", grace_secs.to_string());
        let proc = self.spawn("extractor", cmd);
        proc.expect_log("clipper up", Duration::from_secs(30));
        proc
    }

    /// The binary under test on the MCAP interface: it reads triggers out of the
    /// tailed recording (decoding each by its `message_encoding`) and runs
    /// ROS-free — no trigger subscription, no `Recorded` publish. Same temp-tree
    /// wiring as [`Self::start_extractor`], plus `MOMENTEDGE_INTERFACE=mcap`.
    pub fn start_extractor_mcap(&self, grace_secs: u64) -> Proc {
        let mut cmd = self.command(env!("CARGO_BIN_EXE_clipper"));
        cmd.env("MOMENTEDGE_RECORD_DIR", self.record_dir())
            .env("MOMENTEDGE_OUT_DIR", self.out_dir())
            .env("MOMENTEDGE_GRACE_SECS", grace_secs.to_string())
            .env("MOMENTEDGE_INTERFACE", "mcap");
        let proc = self.spawn("extractor", cmd);
        proc.expect_log("clipper up", Duration::from_secs(30));
        proc
    }

    /// A `ros2 topic echo --once` capturing the next `Recorded` announcement
    /// into its log. Started (and given a discovery head start) BEFORE the
    /// trigger fires, so the announcement publisher is already matched by the
    /// time it publishes.
    pub fn start_recorded_listener(&self, tag: &str) -> Proc {
        let mut cmd = self.command("ros2");
        cmd.args([
            "topic",
            "echo",
            "--no-daemon",
            "--once",
            RECORDED_TOPIC,
            "momentedge_msgs/msg/Recorded",
        ]);
        let proc = self.spawn(&format!("recorded-{tag}"), cmd);
        // No readiness signal exists for the echo's subscription; the python
        // CLI needs a moment to create it. The announcement follows the
        // trigger by at least its postroll, which dwarfs this head start.
        std::thread::sleep(Duration::from_secs(2));
        proc
    }

    /// Publish one `Trigger` and confirm the extractor actually received it.
    ///
    /// A bare `ros2 topic pub --once -w 1` waits for *one* matched subscription
    /// before its single publish. But the `--all` recorder also subscribes to
    /// the trigger topic, so that one match can be the recorder's, not the
    /// extractor's — the one-shot trigger then fires before the extractor is
    /// matched and is lost to it. The race is invisible on a fast host that
    /// matches the extractor first and brutal on a slow one that does not.
    ///
    /// Wait for every subscriber instead: `-w 2` when the recorder records the
    /// trigger topic (so the publish cannot fire until the extractor is matched
    /// too), `-w 1` when a topic-restricted recorder leaves the extractor the
    /// sole subscriber (as `quiet_topics` does, where `-w 2` would never be
    /// satisfied). One publish settles it — no extra `ros2 topic pub` round
    /// trips, whose ~1 s startup each would skew the deletion-timing tests.
    /// Receipt is still confirmed in the extractor log, with a rare-miss
    /// republish; re-firing is idempotent, since only a trigger the extractor
    /// receives cuts a clip. The wait is bounded by `wait_exit` rather than
    /// `--max-wait-time-secs`, Jazzy+ syntax Humble's `ros2 topic pub` rejects.
    pub fn fire_trigger(&self, name: &str, trigger_ns: u64, preroll_ns: u64, postroll_ns: u64) {
        let sec = (trigger_ns / 1_000_000_000) as i64;
        let nanosec = trigger_ns % 1_000_000_000;
        let yaml = format!(
            "{{name: {name}, description: e2e, \
             trigger_time: {{sec: {sec}, nanosec: {nanosec}}}, \
             preroll: {preroll_ns}, postroll: {postroll_ns}}}"
        );
        let waiters = if self.records_trigger_topic.load(Ordering::Relaxed) {
            "2"
        } else {
            "1"
        };
        let extractor_log = self.log_dir().join("extractor.log");
        let needle = format!("trigger name=\"{name}\"");
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            let mut cmd = self.command("ros2");
            cmd.args([
                "topic",
                "pub",
                "--once",
                "-w",
                waiters,
                TRIGGER_TOPIC,
                "momentedge_msgs/msg/Trigger",
                &yaml,
            ]);
            let mut proc = self.spawn(&format!("trigger-{name}"), cmd);
            let status = proc.wait_exit(Duration::from_secs(30)).unwrap_or_else(|| {
                proc.dump_log();
                panic!("trigger publish {name} did not complete");
            });
            assert!(status.success(), "trigger publish {name} failed: {status}");

            // The publish reached the extractor iff it logs the trigger — the
            // first thing the handler logs on receipt, so a short window settles
            // it and keeps a rare miss's republish prompt.
            if wait_for_file_contains(&extractor_log, &needle, Duration::from_millis(1500)) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "extractor never logged receipt of trigger {name:?}; see {}",
                extractor_log.display(),
            );
        }
    }

    /// `out_dir/.capturing` must exist (the extractor ran) and hold nothing
    /// (no finished clip ever lingers there).
    pub fn assert_capturing_drained(&self) {
        let capturing = self.out_dir().join(".capturing");
        assert!(
            capturing.is_dir(),
            "the extractor creates {}",
            capturing.display()
        );
        let leftover: Vec<_> = std::fs::read_dir(&capturing)
            .expect("reading .capturing")
            .flatten()
            .map(|e| e.path())
            .collect();
        assert!(
            leftover.is_empty(),
            ".capturing must hold no finished clip, found: {leftover:?}"
        );
    }

    /// Publish one `Trigger` so it lands in the recording, for the MCAP
    /// interface to read back out. clipper on `--interface mcap` does NOT
    /// subscribe to the trigger topic, so the only subscriber to wait for is the
    /// `--all` recorder (`-w 1`); the publish lands in the bag and clipper reads
    /// it off the file. Publishes exactly once — a republish would write a second
    /// trigger record into the bag and cut a duplicate clip — then confirms
    /// clipper logged receipt after decoding it out of the MCAP.
    pub fn fire_trigger_into_bag(
        &self,
        name: &str,
        trigger_ns: u64,
        preroll_ns: u64,
        postroll_ns: u64,
    ) {
        let sec = (trigger_ns / 1_000_000_000) as i64;
        let nanosec = trigger_ns % 1_000_000_000;
        let yaml = format!(
            "{{name: {name}, description: e2e, \
             trigger_time: {{sec: {sec}, nanosec: {nanosec}}}, \
             preroll: {preroll_ns}, postroll: {postroll_ns}}}"
        );
        let mut cmd = self.command("ros2");
        cmd.args([
            "topic",
            "pub",
            "--once",
            "-w",
            "1",
            TRIGGER_TOPIC,
            "momentedge_msgs/msg/Trigger",
            &yaml,
        ]);
        let mut proc = self.spawn(&format!("trigger-{name}"), cmd);
        let status = proc.wait_exit(Duration::from_secs(30)).unwrap_or_else(|| {
            proc.dump_log();
            panic!("trigger publish {name} did not complete");
        });
        assert!(status.success(), "trigger publish {name} failed: {status}");

        // clipper logs `trigger name="<name>"` the moment it decodes the trigger
        // out of the tailed MCAP — the receipt confirmation for the mcap path.
        let extractor_log = self.log_dir().join("extractor.log");
        let needle = format!("trigger name=\"{name}\"");
        assert!(
            wait_for_file_contains(&extractor_log, &needle, Duration::from_secs(30)),
            "clipper never logged receipt of the in-bag trigger {name:?}; see {}",
            extractor_log.display(),
        );
    }

    /// Poll `out_dir` for the clip a cut produces, returning its path once it
    /// appears. The MCAP interface publishes no `Recorded`, so the finished
    /// clip's atomic appearance in `out_dir` is the only completion signal.
    pub fn wait_for_clip(&self, file_name: &str, timeout: Duration) -> PathBuf {
        let path = self.out_dir().join(file_name);
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if path.is_file() {
                return path;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
        panic!("clip {} never appeared in out_dir", path.display());
    }
}

/// A child process in its own process group, killed group-wide on drop.
pub struct Proc {
    name: String,
    child: Child,
    log: PathBuf,
}

impl Proc {
    pub fn log_text(&self) -> String {
        std::fs::read_to_string(&self.log).unwrap_or_default()
    }

    pub fn dump_log(&self) {
        eprintln!(
            "--- {} log ({}) ---\n{}\n--- end {} log ---",
            self.name,
            self.log.display(),
            self.log_text(),
            self.name,
        );
    }

    pub fn is_running(&mut self) -> bool {
        self.child.try_wait().expect("try_wait").is_none()
    }

    /// Signal the child's whole process group.
    pub fn signal_group(&self, signal: libc::c_int) {
        // Safety: plain kill(2); the group exists for the child's lifetime.
        unsafe {
            libc::kill(-(self.child.id() as i32), signal);
        }
    }

    /// Poll for exit up to `timeout`; `None` means still running.
    pub fn wait_exit(&mut self, timeout: Duration) -> Option<ExitStatus> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(status) = self.child.try_wait().expect("try_wait") {
                return Some(status);
            }
            if Instant::now() >= deadline {
                return None;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// Signal the group and require the child to exit within `timeout`.
    pub fn stop(&mut self, signal: libc::c_int, timeout: Duration) -> ExitStatus {
        self.signal_group(signal);
        self.wait_exit(timeout).unwrap_or_else(|| {
            self.dump_log();
            panic!(
                "{} did not exit within {timeout:?} after signal {signal}",
                self.name
            );
        })
    }

    /// Poll the child's log until `needle` appears.
    pub fn wait_for_log(&self, needle: &str, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        loop {
            if self.log_text().contains(needle) {
                return true;
            }
            if Instant::now() >= deadline {
                return false;
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }

    /// [`Self::wait_for_log`], but a missing line is a failure with the full
    /// log dumped for post-mortem.
    pub fn expect_log(&self, needle: &str, timeout: Duration) {
        if !self.wait_for_log(needle, timeout) {
            self.dump_log();
            panic!("{}: {needle:?} not logged within {timeout:?}", self.name);
        }
    }

    /// [`Self::expect_log`] for a line that repeats: poll until `needle` has
    /// appeared at least `count` times — e.g. the tail's "tailing <path>"
    /// attach line, whose second occurrence proves the replacement recording
    /// is attached (the path is identical across a restart, so presence alone
    /// cannot). The needle must be specific enough that no other line
    /// contributes a substring match toward the count — qualify it with the
    /// full path rather than a bare prefix another log line shares.
    pub fn expect_log_count(&self, needle: &str, count: usize, timeout: Duration) {
        let deadline = Instant::now() + timeout;
        while self.log_text().matches(needle).count() < count {
            if Instant::now() >= deadline {
                self.dump_log();
                panic!(
                    "{}: {needle:?} not logged {count} times within {timeout:?}",
                    self.name
                );
            }
            std::thread::sleep(Duration::from_millis(100));
        }
    }
}

impl Drop for Proc {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_some() {
            return;
        }
        self.signal_group(libc::SIGTERM);
        if self.wait_exit(Duration::from_secs(2)).is_none() {
            self.signal_group(libc::SIGKILL);
            let _ = self.child.wait();
        }
    }
}

/// The fields of one `Recorded` announcement the suite asserts on. `filenames`
/// lists every segment the window produced — one for a window that stayed in a
/// single recording, several when it straddled a rollover.
#[derive(Debug)]
pub struct Recorded {
    pub name: String,
    pub filenames: Vec<String>,
}

impl Recorded {
    /// The sole clip path, for the common case of a window that stayed in one
    /// recording. Panics if the window produced several segments.
    pub fn only(&self) -> &str {
        assert_eq!(
            self.filenames.len(),
            1,
            "expected a single clip segment, got {:?}",
            self.filenames
        );
        &self.filenames[0]
    }
}

/// Parse `ros2 topic echo` YAML output: top-level scalars sit at column 0, so
/// the nested `trigger_time` fields can never shadow them. `filenames` is a
/// `string[]`, printed either inline empty (`filenames: []`) or as a block of
/// `- <path>` items following the key.
fn parse_recorded(yaml: &str) -> Option<Recorded> {
    fn unquote(v: &str) -> String {
        let v = v.trim();
        v.strip_prefix('\'')
            .and_then(|v| v.strip_suffix('\''))
            .or_else(|| v.strip_prefix('"').and_then(|v| v.strip_suffix('"')))
            .unwrap_or(v)
            .to_string()
    }
    let mut name = None;
    let mut filenames: Option<Vec<String>> = None;
    let mut lines = yaml.lines().peekable();
    while let Some(line) = lines.next() {
        if let Some(v) = line.strip_prefix("name: ") {
            name = Some(unquote(v));
        } else if let Some(rest) = line.strip_prefix("filenames:") {
            let rest = rest.trim();
            let mut files = Vec::new();
            if rest.is_empty() {
                // Block style: subsequent `- <path>` items until the next key.
                while let Some(peek) = lines.peek() {
                    match peek.trim_start().strip_prefix("- ") {
                        Some(item) => {
                            files.push(unquote(item));
                            lines.next();
                        }
                        None => break,
                    }
                }
            } else if rest != "[]" {
                // Inline flow style: `filenames: [a, b]`.
                for item in rest
                    .trim_start_matches('[')
                    .trim_end_matches(']')
                    .split(',')
                {
                    let item = item.trim();
                    if !item.is_empty() {
                        files.push(unquote(item));
                    }
                }
            }
            filenames = Some(files);
        }
    }
    Some(Recorded {
        name: name?,
        filenames: filenames?,
    })
}

/// Wait for the listener to have echoed one complete announcement (echo
/// terminates each message with a `---` line) and parse it.
pub fn wait_for_recorded(listener: &mut Proc, timeout: Duration) -> Recorded {
    try_wait_for_recorded(listener, timeout).unwrap_or_else(|| {
        listener.dump_log();
        panic!("no Recorded announcement within {timeout:?}");
    })
}

/// [`wait_for_recorded`] for tests where no announcement is a legal outcome.
pub fn try_wait_for_recorded(listener: &mut Proc, timeout: Duration) -> Option<Recorded> {
    if !listener.wait_for_log("---", timeout) {
        return None;
    }
    parse_recorded(&listener.log_text())
}

/// Read a finished clip back as `(topic, log_time)` pairs. `MessageStream`
/// insists on a complete summary/footer/magic, so this doubles as the
/// completeness check on every announced file.
pub fn read_clip(path: &Path) -> Vec<(String, u64)> {
    let buf = std::fs::read(path)
        .unwrap_or_else(|e| panic!("reading announced clip {}: {e}", path.display()));
    mcap::MessageStream::new(&buf)
        .unwrap_or_else(|e| {
            panic!(
                "announced clip {} is not a complete MCAP: {e}",
                path.display()
            )
        })
        .map(|msg| {
            let msg = msg.unwrap_or_else(|e| {
                panic!("announced clip {} fails to parse: {e}", path.display())
            });
            (msg.channel.topic.clone(), msg.log_time)
        })
        .collect()
}

/// Every message in the clip lies inside the inclusive trigger window.
pub fn assert_clip_within_window(msgs: &[(String, u64)], start_ns: u64, end_ns: u64) {
    for (topic, log_time) in msgs {
        assert!(
            (start_ns..=end_ns).contains(log_time),
            "message on {topic} at {log_time} outside window [{start_ns}, {end_ns}]"
        );
    }
}

/// Walk the top-level record framing of a possibly unfinished (footer-less)
/// recording and return the `log_time` of every complete top-level `Message`
/// record — the 14-byte prefix read the extractor's tail performs, so this
/// works on a live-copied file [`read_clip`] would reject. A torn final
/// record ends the walk. Top-level only: messages inside `Chunk` records are
/// not seen, which suffices for the suite's unchunked fastwrite recordings.
pub fn partial_recording_stamps(path: &Path) -> Vec<u64> {
    let buf = std::fs::read(path).expect("reading recording");
    let mut stamps = Vec::new();
    let mut off = 8usize; // past the opening magic
    while off + 9 <= buf.len() {
        let opcode = buf[off];
        let len = u64::from_le_bytes(buf[off + 1..off + 9].try_into().unwrap()) as usize;
        let end = off + 9 + len;
        if end > buf.len() {
            break; // still being appended when the file was copied
        }
        if opcode == 0x05 && len >= 14 {
            // Message body: channel_id u16, sequence u32, log_time u64 (LE).
            stamps.push(u64::from_le_bytes(
                buf[off + 15..off + 23].try_into().unwrap(),
            ));
        }
        off = end;
    }
    stamps
}

/// Walk the top-level record framing (1-byte opcode + u64le length) and
/// return every record boundary offset, magic excluded — the same framing the
/// tail scans, used to place deterministic damage at a known record edge.
pub fn record_boundaries(path: &Path) -> Vec<u64> {
    let buf = std::fs::read(path).expect("reading recording");
    let mut boundaries = Vec::new();
    let mut off = 8u64; // past the opening magic
    while (off as usize) + 9 <= buf.len() {
        boundaries.push(off);
        let len = u64::from_le_bytes(buf[off as usize + 1..off as usize + 9].try_into().unwrap());
        off += 9 + len;
    }
    boundaries
}

/// Truncate `path` at a mid-file record boundary and append a record header
/// whose declared length exceeds any plausible record: a framing fault with
/// no resync point, exactly where the scan will arrive.
pub fn inject_framing_fault(path: &Path) {
    use std::io::{Seek, SeekFrom, Write};
    let boundaries = record_boundaries(path);
    assert!(
        boundaries.len() >= 4,
        "recording too short to damage mid-file ({} records)",
        boundaries.len()
    );
    let cut = boundaries[boundaries.len() / 2];
    let file = std::fs::OpenOptions::new()
        .write(true)
        .open(path)
        .expect("opening recording for damage");
    file.set_len(cut).expect("truncating recording");
    let mut file = file;
    file.seek(SeekFrom::End(0)).expect("seeking to the cut");
    let mut fault = vec![0x05u8]; // Message opcode
    fault.extend_from_slice(&u64::MAX.to_le_bytes()); // absurd declared length
    fault.extend_from_slice(&[0xFFu8; 16]); // a few garbage body bytes
    file.write_all(&fault).expect("appending the framing fault");
    file.sync_all().expect("syncing the damaged recording");
}

/// Overwrite `len` bytes at `offset` with 0xFF — localized in-place damage in
/// a region the tail has typically already consumed, surfacing at extraction.
pub fn overwrite_bytes(path: &Path, offset: u64, len: usize) {
    use std::os::unix::fs::FileExt;
    let file = std::fs::OpenOptions::new()
        .write(true)
        .open(path)
        .expect("opening recording for damage");
    file.write_all_at(&vec![0xFFu8; len], offset)
        .expect("overwriting recording bytes");
    file.sync_all().expect("syncing the damaged recording");
}
