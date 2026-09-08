//! Triggered clip recorder tailing a continuous MCAP recording.
//!
//! A continuous `ros2 bag record` (started separately — see the README and
//! `scripts/record.sh`) writes a growing MCAP file, rolling over to a
//! successor on a bag split. This binary discovers each recording, keeps it
//! open, and tails it ([`tail`]): an incremental scan over the record framing
//! that maintains a byte-extent index, a schema/channel registry, and a
//! collection-wide coverage watch (the highest `log_time` on disk). Rollovers
//! are recovered from the files themselves — there is no
//! `/events/write_split` dependency.
//!
//! A trigger requests the window `[trigger_time - preroll, trigger_time +
//! postroll]`: the [`handler`] waits until the wall clock passes the window end,
//! waits until the tail's coverage reaches it (the recording provably holds the
//! window), then bulk-copies the in-window messages out of the planned extents
//! into a clip at `./clipped/<trigger_ns>_<name>.mcap` (see [`clip`] — a
//! raw-bytes copy, no CDR decode, finished with a proper summary + footer,
//! assembled in a capturing dir and moved atomically into place so observers
//! never see a footer-less file).
//!
//! Where triggers come from and how completion is signalled is the [`interface`],
//! one active per run (`--interface`). The `mcap` interface reads triggers out
//! of the tailed recording itself — decoding each by its MCAP `message_encoding`
//! ([`decode`]) — and runs ROS-free, the clip's atomic move into the output
//! directory standing in for a completion announcement. The `ros` interface
//! subscribes to `/events/momentedge/trigger` (`momentedge_msgs/Trigger`) on a
//! ROS node and publishes `/events/momentedge/recorded`
//! (`momentedge_msgs/Recorded`) naming every durable segment. The handler
//! cutting the clip is identical either way; it knows only the neutral
//! [`trigger`] contract.
//!
//! **Two builds.** The `ros` cargo feature is what links the ROS client and
//! compiles the `ros` interface in. With it — the device build, which every
//! packaging path selects — `--interface` takes `ros` (the default there) or
//! `mcap`. Without it the binary links no ROS, builds and runs on a host with no
//! ROS installation, and offers `mcap` alone. Nothing else differs: the tail, the
//! window plan, the cut, and every other flag are the same code either way.
//!
//! Time base: MCAP `log_time`, the trigger stamp, and the wait clock are all
//! treated as nanoseconds on the system (ROS) clock — this assumes the default
//! (no `use_sim_time`). Each trigger is handled on its own thread, so
//! overlapping windows are cut concurrently against one shared tail — at most
//! [`MAX_ACTIVE_TRIGGERS`] at once; a trigger beyond that limit is rejected,
//! logged, and ignored.
//!
//! Everything runs on plain OS threads — there is no async runtime. The main
//! thread supervises ([`supervise`]) two long-lived companions over crossbeam
//! channels — the tail thread (file scan) and the interface thread (draining the
//! active trigger source; the `ros` interface owns its node spin and
//! subscription drain internally) — plus a signal forwarder (SIGINT/SIGTERM →
//! orderly exit 0). Clip copies run on a fixed pool of `extract_parallelism`
//! worker threads consuming one FIFO job channel.
//!
//! `clipper` is one binary and the mode is a subcommand ([`Mode`]): the device
//! recorder this module describes is `clipper tail`. `clipper --help` lists the
//! modes, `clipper tail --help` lists the recorder's own flags, and a flag
//! offered to `clipper` itself is a parse error pointing at the subcommand.
//!
//! **The other mode is `clipper clip`** ([`clip_mode`]): one clip out of one
//! finished recording, named by a trigger on the command line, then exit. It
//! shares everything below the trigger — the window plan, the copy, the
//! manifest, atomic publication — and differs in the two things a finished input
//! makes meaningless. The recording is indexed from its own summary
//! ([`clip::whole::WholeFileIndex`]) rather than by a scan that keeps resuming,
//! and neither wait above runs: there is no later data to wait for, so a window
//! reaching past the recording's end is simply short and the clip's manifest
//! says so. A run's result is the output directory's contents when the process
//! exits, and the exit status is the verdict.
//!
//! Configuration is parsed by clap into [`Config`]: each setting is a flag of
//! `clipper tail` that falls back to a `MOMENTEDGE_<KEY>` environment variable,
//! then to a built-in default — the CLI flag wins over the env var, which wins
//! over the default. The `MOMENTEDGE_*` env names are derived from one prefix
//! applied to every field of every mode ([`with_env_prefix`]). `--version`
//! prints the version; [`Config`]'s field docs are the `clipper tail --help`
//! text and the authoritative per-flag reference (the README configuration
//! table is the user-facing copy of the same set).
//!
//! Logging uses the `log` facade with a pretty_env_logger backend and goes to
//! **stdout**; `RUST_LOG` controls verbosity. Under `--interface ros` the ROS
//! layer's own diagnostics are a separate stream — rcutils writes them to
//! stderr unless `RCUTILS_LOGGING_USE_STDOUT=1`.

mod interface;
mod supervision;

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::Context;
use clap::{Args, CommandFactory, FromArgMatches, Parser, Subcommand, ValueEnum};
use clip::manifest::Producer;
use clip::trigger::{Trigger, now_ns};
use clip::{TimeSource, segment};
use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
#[cfg(feature = "ros")]
use interface::ros::RosInterface;
use interface::{Anchor, Interface, McapInterface};
use log::{error, info, warn};
use signal_hook::consts::{SIGINT, SIGTERM};
use supervision::{Supervised, harvest_panic, spawn_supervised};
use tail::{Coverage, Tailer, Watch, handler};

const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";
/// The topic the ROS interface announces finished clips on. Only that interface
/// publishes, so only the `ros` build has one.
#[cfg(feature = "ros")]
const RECORDED_TOPIC: &str = "/events/momentedge/recorded";

/// How many trigger handlers may be active (admitted, waiting, or extracting)
/// at once. Beyond this limit an arriving trigger is rejected at admission:
/// `error!`-logged and ignored — no handler is spawned, no clip is cut, and no
/// `Recorded` message is published.
///
/// An active handler is one parked thread: it sleeps out its postroll window
/// and waits on the coverage watch. The heavy work — the bulk file copy — is
/// already serialized by the extraction worker pool (`extract_parallelism`).
/// This constant is therefore a flood-sanity bound on thread and announcement
/// growth, not a resource budget; 16 comfortably exceeds any legitimate
/// concurrent burst.
///
/// **Failure mode for downstream automation.** A rejected trigger produces no
/// `Recorded` announcement, and there is no negative acknowledgement on the
/// wire: a consumer waiting on `/events/momentedge/recorded` to learn that a
/// clip was written simply never hears back for that trigger and would hang if
/// it blocks on the reply. The `error!` log line is the *only* signal that a
/// trigger was dropped, so alerting on it is how an operator detects a
/// sustained trigger flood that is outrunning the recorder.
const MAX_ACTIVE_TRIGGERS: usize = 16;

/// Compression codec for written clips, the clap surface of the otherwise
/// implicit `mcap::WriteOptions` compression. [`to_mcap`](ClipCompression::to_mcap)
/// maps it to the `Option<mcap::Compression>` the clip writer takes: `None` →
/// uncompressed, `Zstd`/`Lz4` → the matching codec.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum ClipCompression {
    None,
    Zstd,
    Lz4,
}

impl ClipCompression {
    /// The `mcap::WriteOptions::compression` argument this codec selects.
    fn to_mcap(self) -> Option<mcap::Compression> {
        match self {
            ClipCompression::None => None,
            ClipCompression::Zstd => Some(mcap::Compression::Zstd),
            ClipCompression::Lz4 => Some(mcap::Compression::Lz4),
        }
    }
}

impl std::fmt::Display for ClipCompression {
    /// Render as the clap value name (`none`/`zstd`/`lz4`) so the `--help`
    /// default rendered by `default_value_t` and the accepted flag values share
    /// one source — the `ValueEnum` possible-value names.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.to_possible_value()
            .expect("no ClipCompression variant is skipped")
            .get_name()
            .fmt(f)
    }
}

/// Where clipper takes triggers from and where it announces completions — one
/// interface to the outside world, chosen by `--interface`. The variants are
/// mutually exclusive; clipper drives exactly one per run.
///
/// The variant set is the build: `Ros` exists only under the `ros` feature, so a
/// ROS-free build's `--interface` accepts (and its `--help` lists) `mcap` alone.
/// [`DEFAULT_INTERFACE`] is the one clipper takes when the flag is absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum InterfaceKind {
    /// Subscribe to the trigger topic on a ROS node and publish `Recorded` on
    /// completion. The deployed, ROS-native path, and the default where it
    /// exists.
    #[cfg(feature = "ros")]
    Ros,
    /// Read triggers out of the tailed MCAP (decoding each by its
    /// `message_encoding`) and signal completion by the clip's move into
    /// `out_dir`. Runs ROS-free: no node, executor, subscription, or publish.
    Mcap,
}

/// The interface clipper drives when `--interface` is not given: the ROS one
/// where the feature built it, and otherwise the only one there is.
#[cfg(feature = "ros")]
const DEFAULT_INTERFACE: InterfaceKind = InterfaceKind::Ros;
#[cfg(not(feature = "ros"))]
const DEFAULT_INTERFACE: InterfaceKind = InterfaceKind::Mcap;

/// The `--interface` short help. It names the values this build actually
/// accepts, which is the feature's one visible difference on the command line.
#[cfg(feature = "ros")]
const INTERFACE_HELP: &str = "Where triggers come from and completions go: `ros` or `mcap`";
#[cfg(not(feature = "ros"))]
const INTERFACE_HELP: &str = "Where triggers come from and completions go: `mcap`";

/// The `--interface` long help (`--help`, not `-h`), likewise per build: the ROS
/// arm is described only where it can be selected, and the ROS-free build says
/// outright that it was built without it.
#[cfg(feature = "ros")]
const INTERFACE_LONG_HELP: &str = "\
Where triggers come from and completions go: `ros` or `mcap`.

`ros` (the default) subscribes to the trigger topic on a ROS node and publishes \
`Recorded` on completion. `mcap` reads triggers out of the tailed recording \
(decoding each by its `message_encoding`) and signals completion by moving the \
clip into `out_dir` — it runs ROS-free, with no node, subscription, or publish. \
Exactly one interface is active per run.";
#[cfg(not(feature = "ros"))]
const INTERFACE_LONG_HELP: &str = "\
Where triggers come from and completions go: `mcap`.

`mcap` reads triggers out of the tailed recording (decoding each by its \
`message_encoding`) and signals completion by moving the clip into `out_dir` — \
it runs ROS-free, with no node, subscription, or publish. It is the only \
interface this binary has: the `ros` interface, which subscribes to the trigger \
topic on a ROS node and publishes `Recorded`, is compiled in by the `ros` cargo \
feature, and this build was made without it.";

impl std::fmt::Display for InterfaceKind {
    /// Render as the clap value name (`ros`/`mcap`) so the `--help` default and
    /// the accepted flag values share the `ValueEnum` possible-value names.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.to_possible_value()
            .expect("no InterfaceKind variant is skipped")
            .get_name()
            .fmt(f)
    }
}

/// The command line: one binary, one mode per invocation, and the mode is a
/// subcommand. `clipper tail` is the device recorder; every flag belongs to the
/// mode rather than to `clipper` itself, so a bare `clipper --record-dir …` is
/// rejected ([`mode_hint`] says where the flag belongs).
///
/// The `mode` field is what makes naming no mode an error: it is not an
/// `Option`, so clap's derive requires the subcommand and answers a bare
/// `clipper` with the mode listing and a non-zero exit. No
/// `subcommand_required`/`arg_required_else_help` is needed on top of that, and
/// adding them changes nothing.
///
/// `long_about` is written out rather than taken from this doc comment: clap's
/// derive would otherwise put the rationale above — rustdoc links and all — in
/// front of an operator running `clipper --help`.
#[derive(Debug, Parser)]
#[command(
    name = "clipper",
    version,
    about = "Triggered MCAP clip recorder",
    long_about = "Triggered MCAP clip recorder.\n\n\
                  One mode runs per invocation, and the mode is a subcommand: \
                  `clipper tail` follows a continuous recording and cuts a clip \
                  per trigger. Every flag belongs to a mode, so `clipper tail \
                  --help` is the recorder's own flag reference."
)]
struct Cli {
    #[command(subcommand)]
    mode: Mode,
}

/// The modes clipper runs, one per invocation. Each carries its own flags, so
/// `clipper <mode> --help` lists that mode's surface and nothing else.
#[derive(Debug, Subcommand)]
enum Mode {
    /// Tail a continuous recording and cut a clip per trigger.
    ///
    /// Discovers and follows the growing MCAP files a continuous `ros2 bag
    /// record` writes, and cuts the window each trigger asks for out of them.
    /// Runs until a shutdown signal.
    Tail(Config),
    /// Cut one clip out of one finished recording and exit.
    ///
    /// Takes a recording nobody is writing any more and a trigger named on the
    /// command line, and writes the window that trigger asks for. The recording
    /// is indexed from its own summary rather than by walking it, and nothing
    /// is waited for — the input has an end. The run's result is the contents
    /// of the output directory when the process exits; the exit status is the
    /// verdict.
    Clip(ClipConfig),
}

/// The program name every clip's manifest carries under `producer.name`: this
/// binary, as an operator invokes it.
const PROGRAM: &str = "clipper";

impl Mode {
    /// What this mode's clips record as having cut them. The subcommand name is
    /// spelled here rather than recovered from the parser, so a mode added to
    /// the enum is a compile error until it says what its clips are stamped
    /// with — and a clip cut by a later mode is told apart from a recorder's
    /// without opening the file it came from.
    fn producer(&self) -> Producer {
        let mode = match self {
            Mode::Tail(_) => "tail",
            Mode::Clip(_) => "clip",
        };
        Producer {
            program: PROGRAM,
            mode,
        }
    }
}

/// Recorder configuration for [`Mode::Tail`], parsed by clap from CLI flags with
/// a `MOMENTEDGE_*` environment-variable fallback per field (see
/// [`load_cli`]). The field doc comments are the `clipper tail --help` text:
/// the first line is the short help, the rest is shown under `--help`.
#[derive(Debug, Args)]
struct Config {
    /// Bag directory of the continuous recording that is tailed.
    #[arg(long, default_value = "./record")]
    record_dir: PathBuf,
    /// Directory finished clips are written to.
    #[arg(long, default_value = "./clipped")]
    out_dir: PathBuf,
    /// Seconds to wait past the window end for coverage before cutting.
    ///
    /// How long past the window end to keep waiting for the recording to
    /// cover `trigger_time + postroll` before cutting the clip from what is
    /// on disk. Coverage normally lags the wall clock by the recorder's flush
    /// latency only; the timeout fires when the recorded topics go quiet, and
    /// the clip then simply ends at the last data that exists.
    #[arg(long, default_value_t = 30)]
    grace_secs: u64,
    /// Number of concurrent clip extractions (extraction worker-pool size).
    ///
    /// How many clip extractions may run at once — the size of the extraction
    /// worker pool. The default of 1 serializes the bulk copies FIFO:
    /// extraction reads compete with the recorder's writes on the same disk,
    /// and concurrent copies inflate the recorder's flush latency (rosbag2's
    /// cache drops messages when it cannot drain). Waiting — postroll,
    /// coverage — is always concurrent; only the copy is queued. Raise on
    /// storage with IO headroom.
    #[arg(long, default_value_t = 1)]
    extract_parallelism: usize,
    /// Compression codec for written clips (none, lz4, zstd).
    ///
    /// `zstd` (the default) writes the smallest clips; `lz4` trades size for
    /// lower CPU; `none` skips recompression entirely. The codec is set
    /// explicitly on the clip writer rather than inherited from the mcap crate
    /// default. Only the codec is configurable here — chunk size and chunking
    /// stay at the mcap default.
    #[arg(long, value_enum, default_value_t = ClipCompression::Zstd)]
    clip_compression: ClipCompression,

    /// Where triggers come from and completions go.
    ///
    /// The one flag whose surface the `ros` cargo feature changes, so its help
    /// text is per build ([`INTERFACE_HELP`] / [`INTERFACE_LONG_HELP`]) rather
    /// than this doc comment, and its default is [`DEFAULT_INTERFACE`]. clap
    /// derives the accepted values from [`InterfaceKind`]'s variants, so
    /// `--help` lists exactly what this build can select.
    #[arg(
        long,
        value_enum,
        default_value_t = DEFAULT_INTERFACE,
        help = INTERFACE_HELP,
        long_help = INTERFACE_LONG_HELP,
    )]
    interface: InterfaceKind,

    /// Clock domain the clip window lives in: `log` or `publish`.
    ///
    /// `log` (the default) windows on each message's `log_time` — when the
    /// producer received it — the completeness-proof clock. `publish` windows on
    /// each message's `publish_time` — whatever the producer wrote there (a DDS
    /// source timestamp, a capture time); clipper never interprets it. Publish
    /// times can arrive out of order, so a message may land after the cut with an
    /// in-window `publish_time` and be lost — `--grace-secs` bounds the wait.
    /// The flag governs the anchor, window membership, extent selection, and the
    /// coverage a handler waits on; retention still ages files on `log_time`.
    #[arg(long, value_enum, default_value_t = TimeSource::Log)]
    time_source: TimeSource,

    /// Seconds to keep a finished recording indexed (and its fd open) for clip
    /// preroll.
    ///
    /// A finished (split/restart) recording is watched while it holds data newer
    /// than now minus this duration, so a trigger's preroll can still reach into
    /// it. The tail prunes expired recordings every poll (whole files only,
    /// never the one being recorded), so open fds and index memory stay bounded
    /// even when the recorder stops splitting or goes idle. Pruning forgets a
    /// recording in-memory; it does NOT delete the file unless
    /// --delete-old-files is set. Set this comfortably above the largest preroll
    /// any trigger will request: a preroll reaching past the watch floor may
    /// lose its oldest segment.
    #[arg(long, default_value_t = 600)]
    watch_old_files_duration: u64,

    /// Also delete a recording from disk when it is pruned past the watch floor.
    ///
    /// Off by default: clipper forgets old recordings in-memory but leaves the
    /// .mcap files for `ros2 bag record` / other consumers. When set, a prune
    /// unlinks the expired file too (whole files only, never the current one);
    /// an in-flight extraction's open fd keeps the inode readable until it
    /// finishes.
    #[arg(long, default_value_t = false)]
    delete_old_files: bool,
}

impl Config {
    fn grace(&self) -> Duration {
        Duration::from_secs(self.grace_secs)
    }

    fn watch_old_files(&self) -> Duration {
        Duration::from_secs(self.watch_old_files_duration)
    }
}

/// Configuration for [`Mode::Clip`]: the recording to cut from, where the clip
/// goes, and the trigger — spelled out as the five fields a
/// `momentedge_msgs/Trigger` carries, so a clip cut here states the same trigger
/// a clip cut from a live topic does. As with every mode, each field falls back
/// to its `MOMENTEDGE_*` environment variable ([`load_cli`]) and the field doc
/// comments are the `clipper clip --help` text.
///
/// There is no clock-domain flag. The window lives on `log_time` ([`CLIP_TIME_SOURCE`]),
/// the clock a recording's summary states its message times on and the one a
/// completeness claim can be made about; `--time-source` belongs to the tail,
/// where coverage is something a caller waits for.
#[derive(Debug, Args)]
struct ClipConfig {
    /// The finished MCAP recording to cut the clip out of.
    recording: PathBuf,

    /// Directory the finished clip is written to.
    #[arg(long)]
    out_dir: PathBuf,

    /// The instant the clip window centres on, in nanoseconds since the epoch.
    ///
    /// The window is `[trigger-time - preroll, trigger-time + postroll]` on the
    /// recording's `log_time`, and the instant also names the clip
    /// (`<trigger-time>_<trigger-name>.mcap`). There is no default: the one
    /// thing only the caller knows is which moment the clip is about.
    #[arg(long)]
    trigger_time: u64,

    /// Nanoseconds before the trigger instant to include in the clip.
    #[arg(long)]
    preroll: u64,

    /// Nanoseconds after the trigger instant to include in the clip.
    #[arg(long)]
    postroll: u64,

    /// The trigger's name, which also names the clip file.
    ///
    /// Carried into the clip's manifest under `trigger.name` and embedded in the
    /// output filename, so it is bounded and kept filename-safe the same way a
    /// name arriving on a topic is.
    #[arg(long, default_value = "clip")]
    trigger_name: String,

    /// The trigger's description, carried into the clip's manifest.
    #[arg(long, default_value = "")]
    trigger_description: String,
}

/// Prefix shared by every `MOMENTEDGE_*` environment variable.
/// [`with_env_prefix`] applies it to every argument of every [`Mode`], so the
/// env names track the field names (`grace_secs` → `MOMENTEDGE_GRACE_SECS`)
/// with no per-field wiring.
const ENV_PREFIX: &str = "MOMENTEDGE";

/// Give every mode's arguments an environment-variable fallback named
/// `<ENV_PREFIX>_` + the field name upper-cased (`record_dir` →
/// `MOMENTEDGE_RECORD_DIR`). One place defines the prefix; the auto-generated
/// `--help`/`--version` flags are left without an env binding.
///
/// The flags live on the subcommands, not on `clipper` itself, so the walk goes
/// through every subcommand. The names are collected first because
/// [`clap::Command::mut_subcommand`] consumes the command it mutates.
fn with_env_prefix(cmd: clap::Command) -> clap::Command {
    let modes: Vec<String> = cmd
        .get_subcommands()
        .map(|sub| sub.get_name().to_owned())
        .collect();
    modes.into_iter().fold(cmd, |cmd, mode| {
        cmd.mut_subcommand(mode, |sub| {
            sub.mut_args(|arg| match arg.get_id().as_str() {
                "help" | "version" => arg,
                id => {
                    let env = format!("{ENV_PREFIX}_{}", id.to_uppercase());
                    arg.env(env)
                }
            })
        })
    })
}

/// The line appended to a parse failure that left the mode unnamed, since
/// clap's own text says the argument is unexpected without saying where it
/// belongs. `None` for a failure that named a mode (a bad value for one of its
/// flags), and for `--help`/`--version`, which are not failures at all.
fn mode_hint(kind: clap::error::ErrorKind) -> Option<&'static str> {
    use clap::error::ErrorKind::{
        DisplayHelpOnMissingArgumentOrSubcommand, InvalidSubcommand, MissingSubcommand,
        UnknownArgument,
    };
    matches!(
        kind,
        UnknownArgument
            | InvalidSubcommand
            | MissingSubcommand
            | DisplayHelpOnMissingArgumentOrSubcommand
    )
    .then_some(
        "clipper runs one mode per invocation, named as a subcommand. \
         The recorder is `clipper tail` and the one-shot cutter is \
         `clipper clip` — try `clipper tail --help`.",
    )
}

/// Parse the [`Cli`] from the command line, each mode's field falling back to
/// its `MOMENTEDGE_*` environment variable and then its default (CLI > env >
/// default). Diverges the way [`clap::Error::exit`] does — printing the message
/// and ending the process — for `--help`, `--version` and any parse error, so
/// this returns only a fully-populated mode. A failure that left the mode
/// unnamed carries [`mode_hint`] after clap's own text.
fn load_cli() -> Cli {
    let parsed = with_env_prefix(Cli::command())
        .try_get_matches()
        .and_then(|matches| Cli::from_arg_matches(&matches));
    match parsed {
        Ok(cli) => cli,
        Err(err) => {
            let hint = mode_hint(err.kind());
            let code = err.exit_code();
            let _ = err.print();
            if let Some(hint) = hint {
                eprintln!("\n{hint}");
            }
            std::process::exit(code);
        }
    }
}

/// Deliver SIGINT/SIGTERM as a message on the returned channel: a dedicated
/// thread blocks on signal-hook's iterator and forwards the first shutdown
/// signal for [`supervise`] to select on. Both signals mean the same
/// requested, orderly stop (the process exits zero) — SIGTERM is what process
/// supervisors send first.
fn signal_channel() -> anyhow::Result<Receiver<i32>> {
    let (tx, rx) = bounded(1);
    let mut signals = signal_hook::iterator::Signals::new([SIGINT, SIGTERM])?;
    thread::Builder::new()
        .name("signals".to_string())
        .spawn(move || {
            if let Some(sig) = signals.forever().next() {
                let _ = tx.send(sig);
            }
        })?;
    Ok(rx)
}

fn signal_name(sig: i32) -> &'static str {
    match sig {
        SIGINT => "SIGINT",
        SIGTERM => "SIGTERM",
        _ => "shutdown signal",
    }
}

/// Bounded admission for trigger handlers: [`MAX_ACTIVE_TRIGGERS`] permits
/// bound how many may be active (admitted, waiting, or extracting) at once.
/// The consumer takes a permit without waiting before spawning a handler
/// thread; the permit rides in the thread and returns when the handler
/// finishes — panic included, since it returns on drop, which unwinding
/// covers — so the bound never ratchets down.
struct Admission {
    active: AtomicUsize,
    limit: usize,
}

impl Admission {
    fn new(limit: usize) -> Arc<Self> {
        Arc::new(Admission {
            active: AtomicUsize::new(0),
            limit,
        })
    }

    /// Take a permit if one is free, without waiting. `None` means every
    /// permit is held by an active handler — the caller rejects the trigger.
    fn try_acquire(self: Arc<Self>) -> Option<AdmissionPermit> {
        self.active
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                (n < self.limit).then_some(n + 1)
            })
            .ok()
            .map(|_| AdmissionPermit(self))
    }
}

/// An admitted handler's slot; returns to the [`Admission`] count on drop.
struct AdmissionPermit(Arc<Admission>);

impl Drop for AdmissionPermit {
    fn drop(&mut self) {
        self.0.active.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Entry point: install logging, then run the mode the command line named.
///
/// The `match` is the dispatch table over [`Mode`], so a mode added to the enum
/// is a compile error here until it has a body to run.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Logs go to stdout: there is no machine-readable contract on that stream
    // and there will not be one — a run's result is the contents of `out_dir`
    // when the process exits, each clip carrying its own metadata record — so
    // the stream an operator reads first is free for human-readable output.
    // `Target` comes through the `env_logger` that `pretty_env_logger`
    // re-exports; no direct dependency on it.
    pretty_env_logger::formatted_builder()
        .target(pretty_env_logger::env_logger::fmt::Target::Stdout)
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    // The producer is read off the mode before it is destructured, so every clip
    // the run writes is stamped with the subcommand that produced it.
    let mode = load_cli().mode;
    let producer = mode.producer();
    match mode {
        Mode::Tail(cfg) => tail_mode(cfg, producer),
        Mode::Clip(cfg) => clip_mode(cfg, producer).map_err(Into::into),
    }
}

/// `clipper tail`: supervisor for the device recorder. Spawns the long-lived
/// threads — the tail, the interface (which owns its own trigger source, and for
/// ROS its node spin), the staging worker pool, and the signal forwarder — then
/// blocks in [`supervise`] until a shutdown signal (exit 0) or the first dead
/// critical thread (exit non-zero, for a supervisor to restart the process).
///
/// Returning ends the process, which kills the remaining threads: the
/// immortal tail and interface loops, parked handlers, and any in-flight
/// extraction.
/// That is safe for clips by construction — the capturing-dir reset at
/// startup reclaims any stranded staged file, and `out_dir` only ever holds
/// complete clips.
fn tail_mode(cfg: Config, producer: Producer) -> Result<(), Box<dyn std::error::Error>> {
    let cfg = Arc::new(cfg);

    // Start each run with a clean capturing dir: a crash mid-publish can strand
    // a stale staged file there, and clearing it at startup bounds that clutter
    // to a single run. This also creates out_dir, so the first clip can be
    // published without further setup. Fatal if it fails — a recorder that
    // cannot prepare its output directory must not start.
    clip::cut::reset_capturing_dir(&cfg.out_dir)?;

    // One staging worker per allowed concurrent clip copy; see
    // Config::extract_parallelism. The clip compression codec is process-global,
    // captured in the workers; each window's clock domain travels with its job.
    let extract_tx =
        segment::spawn_stage_workers(cfg.extract_parallelism, cfg.clip_compression.to_mcap());

    // Admission gate for trigger handlers; see [`Admission`].
    let admission = Admission::new(MAX_ACTIVE_TRIGGERS);

    if !cfg.record_dir.is_dir() {
        warn!(
            "record dir {} does not exist; the tail idles until the continuous \
             recording (scripts/record.sh) creates it",
            cfg.record_dir.display()
        );
    }

    // Build the tailer and the selected interface together, then drive the
    // recorder with them. Exactly one interface is active; `drive` is generic
    // over it (static dispatch, no `Box<dyn>`). The MCAP interface drives off a
    // decode-free trigger tap — the tail lifts trigger-topic messages out of the
    // recording — so its arm wires the tap channel and hands the receiver to the
    // interface; the ROS interface reads triggers from a live subscription and
    // needs no tap, so its tailer is built without one. The ROS arm exists only
    // where the `ros` feature compiled that interface in — without it the
    // variant does not exist and this match has the one arm.
    let result = match cfg.interface {
        #[cfg(feature = "ros")]
        InterfaceKind::Ros => {
            let (tailer, coverage) = Tailer::new();
            let iface = RosInterface::new(TRIGGER_TOPIC, RECORDED_TOPIC, cfg.time_source)?;
            drive(
                iface, cfg, tailer, coverage, extract_tx, admission, producer,
            )
        }
        InterfaceKind::Mcap => {
            let (tx, rx) = unbounded();
            let (tailer, coverage) = Tailer::with_trigger_tap(TRIGGER_TOPIC, tx);
            let iface = McapInterface::new(TRIGGER_TOPIC, rx, cfg.time_source);
            drive(
                iface, cfg, tailer, coverage, extract_tx, admission, producer,
            )
        }
    };
    result.map_err(Into::into)
}

/// The clock domain `clipper clip` cuts on.
///
/// Not a flag: a finished recording's summary states its message times on
/// `log_time` alone, so that is the clock a window over one can be planned and a
/// completeness claim made on. `--time-source` is the tail's, where the choice
/// changes what a handler waits for.
const CLIP_TIME_SOURCE: TimeSource = TimeSource::Log;

/// The codec `clipper clip` writes its clip with — the recorder's default,
/// spelled once here because this mode takes no compression flag.
const CLIP_MODE_COMPRESSION: ClipCompression = ClipCompression::Zstd;

/// `clipper clip`: cut one window out of one finished recording and exit.
///
/// The recording is indexed from its own summary ([`clip::whole::WholeFileIndex`])
/// — a footer seek and one read, no chunk decompressed — and handed to the same
/// [`clip::segment::cut_window`] the recorder drives, so the clip is byte-for-byte
/// what the device would have cut from the same recording and window.
///
/// **The waits are what is absent.** `tail::handler` sleeps until the wall clock
/// passes the window end, then blocks until the tail's coverage reaches it,
/// because a window may reach past the last byte on disk. This input has an end:
/// there is nothing to wait for, so a window reaching past it is simply short,
/// and the manifest says so ([`clip::manifest::WindowCoverage`]).
///
/// Nothing is printed for a caller to parse. The result is the output
/// directory's contents when the process exits, each clip carrying its own
/// manifest; the exit status is the verdict.
fn clip_mode(cfg: ClipConfig, producer: Producer) -> anyhow::Result<()> {
    // A trigger name reaches the filesystem through the clip's pathname, so it
    // passes the gate every trigger passes, whichever source it arrived from.
    if let Err(why) = validate_name(&cfg.trigger_name) {
        anyhow::bail!("--trigger-name {:?} {why}", cfg.trigger_name);
    }

    // Start from a clean capturing dir, which also creates out_dir: a clip is
    // assembled there and hard-linked into place, so the output directory only
    // ever holds complete clips.
    clip::cut::reset_capturing_dir(&cfg.out_dir)?;

    let index = clip::whole::WholeFileIndex::open(&cfg.recording)?;

    let anchor_ns = cfg.trigger_time;
    let trigger = Trigger {
        name: cfg.trigger_name,
        description: cfg.trigger_description,
        trigger_time: clip::Stamp::from_ns(anchor_ns),
        preroll: cfg.preroll,
        postroll: cfg.postroll,
    };
    let request = Arc::new(clip::CutRequest::new(
        producer,
        trigger.clone(),
        anchor_ns,
        CLIP_TIME_SOURCE,
    ));

    // The one thing a finished clip cannot show from its own contents: whether
    // the recording ever reached the window end, or simply stops inside it.
    let coverage = if index
        .log_end_ns()
        .is_some_and(|end_ns| end_ns >= request.end_ns())
    {
        clip::WindowCoverage::Covered
    } else {
        warn!(
            "{} ends before the window end {}; the clip stops where the \
             recording does",
            cfg.recording.display(),
            request.end_ns(),
        );
        clip::WindowCoverage::Short
    };

    info!(
        "cutting {} window=[{}, {}] source={CLIP_TIME_SOURCE} from {} into {}",
        trigger.name,
        request.start_ns(),
        request.end_ns(),
        cfg.recording.display(),
        cfg.out_dir.display(),
    );

    // One window, one recording, one copy: the pool is sized to the work there
    // is. It exists at all because staging is the pool's job either way.
    let stage_tx = segment::spawn_stage_workers(1, CLIP_MODE_COMPRESSION.to_mcap());
    let base_out_path = cfg.out_dir.join(format!(
        "{anchor_ns}_{}.mcap",
        segment::sanitize(&trigger.name)
    ));
    let segments = segment::cut_window(&index, &request, coverage, &base_out_path, &stage_tx)?;

    for stats in &segments {
        info!(
            "clip {} written: {} msgs from {} extents, {:.1} MiB",
            stats.out_path.display(),
            stats.messages_copied,
            stats.extents_read,
            stats.bytes_copied as f64 / 1_048_576.0,
        );
        if stats.records_skipped > 0 || stats.chunks_dropped > 0 {
            warn!(
                "clip {} is missing data over damage in the recording: \
                 {} records skipped, {} chunks dropped",
                stats.out_path.display(),
                stats.records_skipped,
                stats.chunks_dropped,
            );
        }
    }
    Ok(())
}

/// The largest `preroll` or `postroll` a trigger may request, in nanoseconds
/// (30 minutes). A window wider than this is a malformed or runaway request, not
/// a real clip; the cap bounds how far a cut reaches back into the retained
/// recordings and how long a handler parks. The exact value is accepted.
const MAX_ROLL_NS: u64 = 1_800_000_000_000; // 30 * 60 * 1e9

/// The largest amount a resolved anchor may sit in the future of `now`, in
/// nanoseconds (30 minutes — the same horizon as [`MAX_ROLL_NS`], since both
/// bound how long one trigger can wedge a handler). The anchor drives the
/// postroll wall-floor sleep (`anchor + postroll`), so an anchor far in the
/// future parks a handler for that long; a wildly future anchor is a producer
/// clock fault or a hostile record stamp, never a real request. The guard is on
/// the *resolved* anchor, whatever cell produced it: `--interface ros
/// --time-source log` resolves it to `now` and always passes, while a
/// `ros`+`publish` `trigger_time` or a tail record's own stamp is exactly what it
/// bites on — and a tail record's stamp is the only one a build without the
/// `ros` feature can present.
const MAX_ANCHOR_FUTURE_SKEW_NS: u64 = 1_800_000_000_000; // 30 * 60 * 1e9

/// The largest trigger `name`, in bytes. The name is embedded in the clip
/// pathname `<anchor_ns>_<name>.mcap`, so it is bounded and kept filename-safe
/// (see [`validate_name`]).
const MAX_TRIGGER_NAME_LEN: usize = 128;

/// The trigger-admission gate: whether a resolved trigger is cut into a clip, or
/// the reason it is rejected. Every incoming trigger passes through here (in the
/// interface-fired callback) before any handler work; a rejection logs at
/// `error!` and produces no clip and no `Recorded`. `now_ns` is the current
/// system clock, passed in so the gate stays a pure function (the future-skew
/// guard is the only clock-relative check). The checks, any one of which
/// rejects:
///
/// - **`trigger_time` in a cell that ignores it.** At most one cell of the
///   interface × `--time-source` matrix reads `trigger_time` — `--interface ros
///   --time-source publish`, where it *is* the anchor ([`Anchor::from_trigger_time`]);
///   a build without the `ros` feature has no such cell, and every other cell
///   anchors on a transport stamp. Sending `trigger_time` where
///   it is ignored would silently anchor the window on the trigger's arrival
///   rather than the requested instant, so it is refused loudly. `trigger_time == 0`
///   is always accepted.
/// - **`preroll`/`postroll` past [`MAX_ROLL_NS`].**
/// - **A resolved anchor more than [`MAX_ANCHOR_FUTURE_SKEW_NS`] past `now`.**
///   The anchor — not `trigger_time` specifically — is the guarded value, since
///   it is what parks a handler through its postroll sleep whatever cell resolved
///   it.
/// - **A `name` that is empty, past [`MAX_TRIGGER_NAME_LEN`], or unsafe to embed
///   in the clip pathname** (see [`validate_name`]).
fn validate_trigger(trig: &Trigger, anchor: Anchor, now_ns: u64) -> Result<(), String> {
    if !anchor.from_trigger_time && trig.trigger_time.ns() != 0 {
        return Err(format!(
            "name={:?} sets trigger_time={} but the active interface and \
             --time-source anchor on a transport stamp and ignore it; send \
             trigger_time=0",
            trig.name,
            trig.trigger_time.ns(),
        ));
    }
    if trig.preroll > MAX_ROLL_NS {
        return Err(format!(
            "name={:?} preroll={} ns exceeds the {MAX_ROLL_NS} ns maximum",
            trig.name, trig.preroll,
        ));
    }
    if trig.postroll > MAX_ROLL_NS {
        return Err(format!(
            "name={:?} postroll={} ns exceeds the {MAX_ROLL_NS} ns maximum",
            trig.name, trig.postroll,
        ));
    }
    if anchor.ns > now_ns.saturating_add(MAX_ANCHOR_FUTURE_SKEW_NS) {
        return Err(format!(
            "name={:?} anchor {} ns is more than {MAX_ANCHOR_FUTURE_SKEW_NS} ns \
             past now ({now_ns} ns)",
            trig.name, anchor.ns,
        ));
    }
    if let Err(why) = validate_name(&trig.name) {
        return Err(format!("name={:?} {why}", trig.name));
    }
    Ok(())
}

/// Reject a trigger `name` that cannot be safely embedded in the clip pathname
/// `<anchor_ns>_<name>.mcap`. [`clip::segment::sanitize`] maps stray characters to `_`
/// at clip creation, but structural hazards — an empty name, a path separator or
/// NUL, a leading dot (a hidden file), or an embedded `..` (a parent-directory
/// escape) — are refused whole here rather than silently rewritten, so a
/// malformed request never reaches the filesystem in a surprising shape.
fn validate_name(name: &str) -> Result<(), &'static str> {
    if name.is_empty() {
        return Err("is empty");
    }
    if name.len() > MAX_TRIGGER_NAME_LEN {
        return Err("exceeds the name length limit");
    }
    if name.contains('\0') {
        return Err("contains a NUL byte");
    }
    if name.contains('/') || name.contains('\\') {
        return Err("contains a path separator");
    }
    if name.starts_with('.') {
        return Err("starts with a dot");
    }
    if name.contains("..") {
        return Err("contains '..'");
    }
    Ok(())
}

/// Wire one interface to the tail and run the recorder for the process's
/// lifetime, then supervise. Generic over the active [`Interface`] — static
/// dispatch, no `Box<dyn>`.
///
/// Spawns two long-lived companions over the supervision channels: the **tail**
/// thread (file scan feeding coverage and the extent index) and the
/// **interface** thread (`iface.run`, which drains its trigger source — a ROS
/// subscription or the MCAP tap — and fires the per-trigger callback). The
/// callback admits the trigger (a flood bound), then spawns one handler thread
/// that cuts the clip and announces through the interface's announcer; per-trigger
/// errors are isolated (logged, the permit returned on drop). The ROS interface
/// owns its own node spin internally, so supervision is uniform in either mode.
fn drive<I: Interface>(
    iface: I,
    cfg: Arc<Config>,
    tailer: Arc<Tailer>,
    coverage: Arc<Watch<Coverage>>,
    extract_tx: Sender<segment::StageJob>,
    admission: Arc<Admission>,
    producer: Producer,
) -> anyhow::Result<()> {
    let iface_name = iface.name();
    let announcer = iface.announcer();

    // The callback the interface fires per decoded Trigger. `Fn` + `Send`: it is
    // moved into the single interface thread and called from there, never shared.
    let fire = {
        // The seam: the handler half reads only these settings, so unpack them
        // here rather than handing the CLI parser's `Config` down. The interface
        // resolves each trigger's `anchor_ns` (the window centre) — the ROS
        // interface from `trigger_time`, the MCAP interface from the trigger
        // record's own stamp on the active `--time-source` — and the handler
        // takes that resolved anchor rather than re-deriving one.
        let out_dir = cfg.out_dir.clone();
        let grace = cfg.grace();
        let time_source = cfg.time_source;
        let tailer = tailer.clone();
        let coverage = coverage.clone();
        let extract_tx = extract_tx.clone();
        let admission = admission.clone();
        move |trig: Trigger, anchor: Anchor| {
            // The single validation gate every resolved trigger passes before a
            // handler is spawned. A rejected trigger cuts no clip and announces
            // nothing — the `error!` log is its only trace.
            if let Err(reason) = validate_trigger(&trig, anchor, now_ns()) {
                error!("trigger rejected: {reason}");
                return;
            }
            let anchor_ns = anchor.ns;
            let Some(permit) = admission.clone().try_acquire() else {
                error!(
                    "trigger rejected: all {MAX_ACTIVE_TRIGGERS} trigger handlers are busy; \
                     ignoring name={:?} anchor={anchor_ns}",
                    trig.name,
                );
                return;
            };
            let out_dir = out_dir.clone();
            let tailer = tailer.clone();
            let coverage = coverage.clone();
            let extract_tx = extract_tx.clone();
            let announcer = announcer.clone();
            // Per-trigger error isolation: a failed cut is logged and counted but
            // does not tear down the interface loop, and a panic dies with the
            // handler's own thread (its permit returns on drop either way).
            let spawned = thread::Builder::new()
                .name(format!("trigger-{anchor_ns}"))
                .spawn(move || {
                    let _permit = permit;
                    if let Err(e) = handler::handle_trigger(
                        trig,
                        anchor_ns,
                        &out_dir,
                        grace,
                        tailer,
                        coverage,
                        extract_tx,
                        announcer,
                        time_source,
                        producer,
                    ) {
                        error!("trigger handling failed: {e:#}");
                    }
                });
            if let Err(e) = spawned {
                error!("spawning a trigger handler failed: {e}");
            }
        }
    };

    // The tail thread: discovers and scans the recording for the process's
    // lifetime (blocking IO on its own thread). Supervised: with a dead tailer
    // every clip degrades to a grace-timeout cut, so the process exits rather
    // than limping on silently.
    let tail = {
        let tailer = tailer.clone();
        let record_dir = cfg.record_dir.clone();
        let watch = cfg.watch_old_files();
        let delete_old_files = cfg.delete_old_files;
        spawn_supervised("tail", move || {
            tailer.run(&record_dir, watch, delete_old_files)
        })
    };

    // The interface thread: drains its trigger source and fires the callback for
    // the process's lifetime. Supervised: a dead interface silently stops acting
    // on triggers, so the process exits rather than going quiet.
    let interface = spawn_supervised("interface", move || iface.run(fire));

    let signal_rx = signal_channel().context("signal handler failed to install")?;

    info!(
        "clipper tail up: {iface_name} interface, triggers on {TRIGGER_TOPIC}, \
         tailing {}, writing clips to {}",
        cfg.record_dir.display(),
        cfg.out_dir.display(),
    );

    supervise(tail, interface, signal_rx)
}

/// Watch the two critical long-lived threads and the shutdown signal; return
/// when any of them resolves.
///
/// Each supervised thread reports on its channel (see [`spawn_supervised`]): a
/// received value is its verdict, a disconnect without a value is a panic,
/// harvested through the join handle so the payload lands in the error chain.
///
/// Returns `Ok(())` when the signal channel delivers SIGINT or SIGTERM — the
/// requested, orderly stop path; the signal is logged here and the caller exits
/// zero. Every other arm returns `Err`: a thread exiting (clean or panic) is a
/// fault that a supervisor must respond to by restarting the process, and the
/// signal channel disconnecting (the forwarder thread died) must not be silent
/// either, since it means SIGINT could never trigger a clean shutdown.
///
/// Both threads carry a typed `anyhow::Result<()>` and loop for the process's
/// lifetime, so a clean `Ok(())` return is as unexpected as a fault; a fault
/// surfaces as the inner `Err`, wrapped so the operator sees the root cause. The
/// **tail** thread feeds coverage and the extent index (a dead tailer silently
/// degrades every clip to a grace-timeout cut); the **interface** thread drains
/// the trigger source and, for the ROS interface, owns the node spin (a dead
/// interface silently stops acting on triggers).
fn supervise(
    tail: Supervised<anyhow::Result<()>>,
    interface: Supervised<anyhow::Result<()>>,
    signal: Receiver<i32>,
) -> anyhow::Result<()> {
    let (tail_rx, tail_handle) = tail;
    let (interface_rx, interface_handle) = interface;
    select! {
        recv(tail_rx) -> res => match res {
            // run() loops for the process's lifetime, so a clean return is
            // as unexpected as a fault. A scan fault it could not retry past
            // comes back as the inner Err, wrapped so the operator sees the
            // root cause; a panic is the disconnect.
            Ok(Ok(())) => anyhow::bail!("tail thread exited unexpectedly"),
            Ok(Err(e)) => Err(e.context("tail thread failed")),
            Err(_) => Err(harvest_panic(tail_handle).context("tail thread exited unexpectedly")),
        },
        recv(interface_rx) -> res => match res {
            // The interface drains its trigger source for the process's
            // lifetime; a clean return or a fault both mean it stopped.
            Ok(Ok(())) => anyhow::bail!("interface thread exited unexpectedly"),
            Ok(Err(e)) => Err(e.context("interface thread failed")),
            Err(_) => Err(harvest_panic(interface_handle)
                .context("interface thread exited unexpectedly")),
        },
        recv(signal) -> res => match res {
            // Requested shutdown — not a fault; the caller exits zero.
            Ok(sig) => {
                info!("{} received; shutting down", signal_name(sig));
                Ok(())
            }
            Err(_) => anyhow::bail!("signal handler thread exited unexpectedly"),
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Parse a `Cli` from an explicit argv through the same env-prefixed
    /// command `load_cli` builds, so the tests exercise the real wiring.
    fn cli_from<I, T>(argv: I) -> Result<Cli, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        let matches = with_env_prefix(Cli::command()).try_get_matches_from(argv)?;
        Cli::from_arg_matches(&matches)
    }

    /// The recorder's `Config` out of an argv naming the `tail` mode.
    fn parse_from<I, T>(argv: I) -> Result<Config, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        cli_from(argv).map(|cli| match cli.mode {
            Mode::Tail(cfg) => cfg,
            Mode::Clip(_) => panic!("this argv names the tail mode"),
        })
    }

    /// The cutter's `ClipConfig` out of an argv naming the `clip` mode.
    fn clip_from<I, T>(argv: I) -> Result<ClipConfig, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        cli_from(argv).map(|cli| match cli.mode {
            Mode::Clip(cfg) => cfg,
            Mode::Tail(_) => panic!("this argv names the clip mode"),
        })
    }

    #[test]
    fn config_defaults_when_no_flags_given() {
        let cfg = parse_from(["clipper", "tail"]).unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("./record"));
        assert_eq!(cfg.out_dir, PathBuf::from("./clipped"));
        assert_eq!(cfg.grace(), Duration::from_secs(30));
        assert_eq!(cfg.extract_parallelism, 1);
        assert_eq!(cfg.clip_compression, ClipCompression::Zstd);
        assert_eq!(cfg.time_source, TimeSource::Log);
    }

    #[test]
    fn config_cli_flags_populate_every_field() {
        let cfg = parse_from([
            "clipper",
            "tail",
            "--record-dir",
            "/data/record",
            "--out-dir",
            "/data/clips",
            "--grace-secs",
            "7",
            "--extract-parallelism",
            "3",
            "--clip-compression",
            "lz4",
            "--interface",
            "mcap",
            "--time-source",
            "publish",
            "--watch-old-files-duration",
            "90",
            "--delete-old-files",
        ])
        .unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("/data/record"));
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.grace(), Duration::from_secs(7));
        assert_eq!(cfg.extract_parallelism, 3);
        assert_eq!(cfg.clip_compression, ClipCompression::Lz4);
        assert_eq!(cfg.interface, InterfaceKind::Mcap);
        assert_eq!(cfg.time_source, TimeSource::Publish);
        assert_eq!(cfg.watch_old_files(), Duration::from_secs(90));
        assert!(cfg.delete_old_files);
    }

    /// `with_env_prefix` binds every argument generically, so assert the
    /// invariant generically: each one carries `MOMENTEDGE_<FIELD>`, and the
    /// auto-generated `--help`/`--version` carry none. Written over
    /// `get_arguments()` rather than a hand-listed set so a field added to
    /// a mode's config is covered the moment it exists — a per-field list would
    /// silently leave the newest field, the one most likely to be mis-wired,
    /// untested. Every mode is walked, since the binding is applied per
    /// subcommand.
    #[test]
    fn env_prefix_binds_a_momentedge_name_to_every_field() {
        let cli = with_env_prefix(Cli::command());
        let bound = |mode: &str| -> usize {
            let cmd = cli
                .find_subcommand(mode)
                .unwrap_or_else(|| panic!("{mode} is a subcommand of clipper"));
            let mut bound = 0;
            for arg in cmd.get_arguments() {
                let id = arg.get_id().as_str();
                let env = arg.get_env().map(|e| e.to_string_lossy().into_owned());
                if matches!(id, "help" | "version") {
                    assert_eq!(env, None, "{id} must keep clap's own handling");
                    continue;
                }
                assert_eq!(
                    env.as_deref(),
                    Some(format!("MOMENTEDGE_{}", id.to_uppercase()).as_str()),
                    "{mode}: {id} must fall back to its MOMENTEDGE_* env var",
                );
                bound += 1;
            }
            bound
        };
        assert_eq!(
            bound("tail"),
            9,
            "every Config field is bound (update on a new field)"
        );
        assert_eq!(
            bound("clip"),
            7,
            "every ClipConfig field is bound (update on a new field)"
        );
    }

    /// `--interface mcap` selects the in-recording interface in every build, and
    /// an unknown value is rejected. (The `MOMENTEDGE_INTERFACE` env fallback is
    /// covered by `env_prefix_binds_a_momentedge_name_to_every_field`.)
    #[test]
    fn config_interface_parses_mcap_and_rejects_an_unknown_value() {
        assert_eq!(
            parse_from(["clipper", "tail", "--interface", "mcap"])
                .unwrap()
                .interface,
            InterfaceKind::Mcap
        );
        assert!(parse_from(["clipper", "tail", "--interface", "bogus"]).is_err());
    }

    /// The device build: the `ros` feature offers the ROS interface, and clipper
    /// takes it when `--interface` is absent — the deployed behaviour.
    #[cfg(feature = "ros")]
    #[test]
    fn config_interface_defaults_to_ros_under_the_ros_feature() {
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().interface,
            InterfaceKind::Ros
        );
        assert_eq!(
            parse_from(["clipper", "tail", "--interface", "ros"])
                .unwrap()
                .interface,
            InterfaceKind::Ros
        );
    }

    /// The ROS-free build has no ROS interface to select: `--interface ros` is
    /// refused like any other unknown value, and `mcap` is what an absent flag
    /// means.
    #[cfg(not(feature = "ros"))]
    #[test]
    fn config_interface_is_mcap_only_without_the_ros_feature() {
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().interface,
            InterfaceKind::Mcap
        );
        assert!(
            parse_from(["clipper", "tail", "--interface", "ros"]).is_err(),
            "a build that links no ROS must not accept --interface ros"
        );
    }

    #[test]
    fn config_time_source_defaults_to_log_and_parses_publish() {
        // Default is the log domain; --time-source selects publish; an unknown
        // value is rejected. (Its MOMENTEDGE_TIME_SOURCE env fallback is covered
        // by env_prefix_binds_a_momentedge_name_to_every_field.)
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().time_source,
            TimeSource::Log
        );
        assert_eq!(
            parse_from(["clipper", "tail", "--time-source", "publish"])
                .unwrap()
                .time_source,
            TimeSource::Publish
        );
        assert!(parse_from(["clipper", "tail", "--time-source", "bogus"]).is_err());
    }

    #[test]
    fn config_rejects_a_non_numeric_grace() {
        assert!(parse_from(["clipper", "tail", "--grace-secs", "soon"]).is_err());
    }

    // ── the command shape: one binary, the mode is a subcommand ────────────

    /// A recorder flag offered to `clipper` itself belongs to `clipper tail`,
    /// and the message says so. The process exits non-zero on it: `load_cli`
    /// exits with `Error::exit_code`, which is 2 for a parse failure.
    #[test]
    fn a_bare_recorder_flag_is_rejected_and_points_at_the_tail_mode() {
        let err = cli_from(["clipper", "--record-dir", "/data/record"])
            .expect_err("a recorder flag on the bare command is not a valid command line");
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        let hint = mode_hint(err.kind()).expect("the failure left the mode unnamed");
        assert!(
            hint.contains("clipper tail"),
            "the hint names the subcommand that owns the flag: {hint}"
        );
    }

    /// Naming no mode at all is the same rejection: `clipper` alone runs
    /// nothing.
    #[test]
    fn naming_no_mode_is_rejected_and_points_at_the_tail_mode() {
        let err = cli_from(["clipper"]).expect_err("clipper with no mode runs nothing");
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        assert!(mode_hint(err.kind()).is_some_and(|h| h.contains("clipper tail")));
    }

    /// `clipper --help` lists the modes. It is not a failure — exit code 0 and
    /// no hint, since nothing went wrong.
    #[test]
    fn top_level_help_lists_the_modes() {
        let err = cli_from(["clipper", "--help"]).expect_err("--help short-circuits the parse");
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp);
        assert_eq!(err.exit_code(), 0);
        assert_eq!(mode_hint(err.kind()), None);
        let help = err.to_string();
        assert!(help.contains("Commands:"), "{help}");
        for mode in ["tail", "clip"] {
            assert!(help.contains(mode), "the mode listing names {mode}: {help}");
        }
    }

    /// `clipper tail --help` is the recorder's own surface: its flags, and no
    /// list of modes to descend into.
    #[test]
    fn tail_help_lists_the_recorder_flags_and_no_modes() {
        let err =
            cli_from(["clipper", "tail", "--help"]).expect_err("--help short-circuits the parse");
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp);
        let help = err.to_string();
        for flag in [
            "--record-dir",
            "--out-dir",
            "--grace-secs",
            "--extract-parallelism",
            "--clip-compression",
            "--interface",
            "--time-source",
            "--watch-old-files-duration",
            "--delete-old-files",
        ] {
            assert!(
                help.contains(flag),
                "clipper tail --help lists {flag}: {help}"
            );
        }
        assert!(
            !help.contains("Commands:"),
            "the recorder mode has no submodes: {help}"
        );
    }

    /// `clipper tail --help` names the interfaces *this* build can select — the
    /// one place the `ros` cargo feature is visible on the command line. The
    /// feature build offers both values and defaults to `ros`; the ROS-free
    /// build offers `mcap` alone and says outright that the ROS interface was
    /// not built in, so an operator reading `--help` on a host with no ROS is
    /// told why rather than left guessing.
    #[test]
    fn tail_help_names_the_interfaces_this_build_offers() {
        let err =
            cli_from(["clipper", "tail", "--help"]).expect_err("--help short-circuits the parse");
        let help = err.to_string();
        // `--help` renders a `ValueEnum`'s accepted values as a `Possible
        // values:` list, one `- <name>:` line each.
        assert!(help.contains("- mcap:"), "every build offers mcap: {help}");
        #[cfg(feature = "ros")]
        {
            assert!(help.contains("- ros:"), "{help}");
            assert!(help.contains("[default: ros]"), "{help}");
        }
        #[cfg(not(feature = "ros"))]
        {
            assert!(
                !help.contains("- ros:"),
                "a build that links no ROS must not offer --interface ros: {help}"
            );
            assert!(help.contains("[default: mcap]"), "{help}");
            assert!(
                help.contains("this build was made without it"),
                "the help says the ros interface is absent from this build: {help}"
            );
        }
    }

    /// A failure *inside* a named mode is the mode's own problem, so it carries
    /// no subcommand hint — the caller already said `tail`.
    #[test]
    fn a_bad_flag_value_inside_the_mode_carries_no_hint() {
        let err = cli_from(["clipper", "tail", "--time-source", "bogus"])
            .expect_err("an unknown --time-source value is rejected");
        assert_eq!(mode_hint(err.kind()), None);
    }

    // ── `clipper clip`: one clip out of one finished recording ─────────────

    /// A full `clipper clip` command line populates every field of the cutter's
    /// config.
    #[test]
    fn clip_cli_flags_populate_every_field() {
        let cfg = clip_from([
            "clipper",
            "clip",
            "/data/record/rosbag2_0.mcap",
            "--out-dir",
            "/data/clips",
            "--trigger-time",
            "1738000000000000000",
            "--preroll",
            "5000000000",
            "--postroll",
            "2000000000",
            "--trigger-name",
            "brake-event",
            "--trigger-description",
            "hard brake over 0.8 g",
        ])
        .unwrap();
        assert_eq!(
            cfg.recording,
            PathBuf::from("/data/record/rosbag2_0.mcap"),
            "the recording is the positional argument"
        );
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.trigger_time, 1_738_000_000_000_000_000);
        assert_eq!(cfg.preroll, 5_000_000_000);
        assert_eq!(cfg.postroll, 2_000_000_000);
        assert_eq!(cfg.trigger_name, "brake-event");
        assert_eq!(cfg.trigger_description, "hard brake over 0.8 g");
    }

    /// The window is what only the caller knows, so leaving out the instant it
    /// centres on is refused by name rather than defaulted to something.
    ///
    /// The refusal is clap's, so it happens before `clip_mode` runs: no output
    /// directory is created and no clip is written.
    #[test]
    fn clip_without_a_trigger_time_is_refused_by_name() {
        let err = cli_from([
            "clipper",
            "clip",
            "rec.mcap",
            "--out-dir",
            "/data/clips",
            "--preroll",
            "1000",
            "--postroll",
            "1000",
        ])
        .expect_err("a clip with no trigger time names no window");
        assert_eq!(err.kind(), clap::error::ErrorKind::MissingRequiredArgument);
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        assert!(
            err.to_string().contains("--trigger-time"),
            "the refusal names the missing flag: {err}"
        );
    }

    /// The cutter has no clock-domain flag: a window over a finished recording
    /// lives on `log_time`, the clock its summary states message times on.
    /// Passing `--time-source` is refused rather than quietly ignored.
    #[test]
    fn clip_offers_no_clock_domain_flag() {
        assert!(
            clip_from([
                "clipper",
                "clip",
                "rec.mcap",
                "--out-dir",
                "/data/clips",
                "--trigger-time",
                "1000",
                "--preroll",
                "0",
                "--postroll",
                "0",
                "--time-source",
                "log",
            ])
            .is_err(),
            "the cutter takes no --time-source"
        );

        let err =
            cli_from(["clipper", "clip", "--help"]).expect_err("--help short-circuits the parse");
        let help = err.to_string();
        assert!(
            !help.contains("--time-source"),
            "clipper clip --help offers no clock domain: {help}"
        );
        for flag in [
            "--out-dir",
            "--trigger-time",
            "--preroll",
            "--postroll",
            "--trigger-name",
            "--trigger-description",
        ] {
            assert!(
                help.contains(flag),
                "clipper clip --help lists {flag}: {help}"
            );
        }
    }

    /// The end-to-end cut: a real chunked recording in, one clip out, stamped
    /// with the mode that produced it.
    ///
    /// `producer.mode` is what tells a clip cut here from one the recorder cut,
    /// and it is read off the parsed mode rather than spelled at the cut, so it
    /// is taken through `Mode::producer` exactly as `main` takes it.
    #[test]
    fn clip_mode_cuts_the_window_and_stamps_the_clip_as_its_own() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-mode")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(
            &rec,
            true,
            &[("/t", 1_000), ("/t", 2_000), ("/t", 3_000), ("/t", 4_000)],
        )?;
        let out_dir = root.join("clipped");

        let mode = Mode::Clip(ClipConfig {
            recording: rec.clone(),
            out_dir: out_dir.clone(),
            trigger_time: 3_000,
            preroll: 1_500,
            postroll: 500,
            trigger_name: "brake".to_string(),
            trigger_description: "hard brake".to_string(),
        });
        let producer = mode.producer();
        let Mode::Clip(cfg) = mode else {
            unreachable!("the mode was just built as Clip")
        };
        clip_mode(cfg, producer)?;

        let clip_path = out_dir.join("3000_brake.mcap");
        assert_eq!(
            clip::testing::read_clip(&clip_path)?,
            vec![("/t".to_string(), 2_000), ("/t".to_string(), 3_000)],
            "the window [1500, 3500] holds exactly these two messages"
        );

        let m = clip::manifest::read_manifest(&clip_path)?.expect("every clip carries a manifest");
        assert_eq!(m["producer.name"], "clipper");
        assert_eq!(
            m["producer.mode"], "clip",
            "a clip cut here is told from a recorder's without opening the recording"
        );
        assert_eq!(m["trigger.name"], "brake");
        assert_eq!(m["trigger.description"], "hard brake");
        assert_eq!(m["trigger.anchor_ns"], "3000");
        assert_eq!(m["trigger.preroll_ns"], "1500");
        assert_eq!(m["trigger.postroll_ns"], "500");
        assert_eq!(m["window.time_source"], "log");
        assert_eq!(m["window.start_ns"], "1500");
        assert_eq!(m["window.end_ns"], "3500");
        assert_eq!(m["source.path"], rec.display().to_string());
        assert_eq!(m["source.files_planned"], "1");
        assert_eq!(m["clip.messages"], "2");
        assert_eq!(
            m["clip.short"], "false",
            "the recording runs past the window end"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The cut runs with both of the recorder's waits absent.
    ///
    /// The window ends a minute in the wall-clock future and a minute past the
    /// last recorded message — the two things `tail::handler` sleeps for. The
    /// recording is finished, so there is nothing to wait for: the clip is
    /// written at once and says it stops short of what was asked for.
    #[test]
    fn clip_mode_over_a_finished_recording_does_not_wait() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-nowait")?;
        let rec = root.join("rec.mcap");
        let base = clip::trigger::now_ns();
        clip::testing::write_recording(&rec, true, &[("/t", base), ("/t", base + 1_000)])?;
        let out_dir = root.join("clipped");

        let postroll = 60_000_000_000;
        let began = std::time::Instant::now();
        clip_mode(
            ClipConfig {
                recording: rec,
                out_dir: out_dir.clone(),
                trigger_time: base,
                preroll: 0,
                postroll,
                trigger_name: "late".to_string(),
                trigger_description: String::new(),
            },
            Producer {
                program: PROGRAM,
                mode: "clip",
            },
        )?;
        let elapsed = began.elapsed();

        let clip_path = out_dir.join(format!("{base}_late.mcap"));
        let m = clip::manifest::read_manifest(&clip_path)?.expect("every clip carries a manifest");
        assert_eq!(m["clip.messages"], "2");
        assert_eq!(
            m["clip.short"], "true",
            "the recording stops well inside the window, and the clip says so"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "the cut must not sleep out the window's {postroll} ns of postroll: took {elapsed:?}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A trigger name that cannot be safely embedded in the clip pathname is
    /// refused here exactly as it is when it arrives on a topic, and nothing is
    /// written.
    #[test]
    fn clip_mode_refuses_an_unsafe_trigger_name() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-badname")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, true, &[("/t", 1_000)])?;
        let out_dir = root.join("clipped");

        let err = clip_mode(
            ClipConfig {
                recording: rec,
                out_dir: out_dir.clone(),
                trigger_time: 1_000,
                preroll: 0,
                postroll: 0,
                trigger_name: "../escape".to_string(),
                trigger_description: String::new(),
            },
            Producer {
                program: PROGRAM,
                mode: "clip",
            },
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("--trigger-name"),
            "the refusal names the flag: {err:#}"
        );
        assert!(
            !out_dir.exists(),
            "a refused command line writes nothing at all"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── supervise() tests ──────────────────────────────────────────────────

    /// A supervised arm that never resolves: the sender is leaked so the
    /// channel never disconnects, and the handle is a finished no-op thread
    /// ([`supervise`] joins a handle only after a disconnect).
    fn pending<T: Send + 'static>() -> Supervised<T> {
        let (tx, rx) = bounded::<T>(1);
        std::mem::forget(tx);
        (rx, thread::spawn(|| {}))
    }

    /// A signal channel that never fires (and never disconnects).
    fn no_signal() -> Receiver<i32> {
        let (tx, rx) = bounded::<i32>(1);
        std::mem::forget(tx);
        rx
    }

    #[test]
    fn supervise_carries_tail_failure_cause() {
        // The tail thread resolves a typed anyhow::Result. A scan fault it
        // could not retry past comes back as a received Err; supervise must
        // wrap it so the formatted chain names the tail thread AND carries
        // the scan-fault root cause for the operator.
        let tail = spawn_supervised("tail", || -> anyhow::Result<()> {
            Err(anyhow::anyhow!("scan of X faulted at offset 42"))
        });

        let err = supervise(tail, pending(), no_signal()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("tail thread"),
            "error must name the tail thread, got: {msg}"
        );
        assert!(
            msg.contains("faulted at offset 42"),
            "error must carry the scan-fault root cause, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_interface_end() {
        // An interface thread that exits cleanly (its trigger source ended) is
        // an error: the recorder stops acting on triggers with no noise. The
        // other arms are parked as "pending forever" to isolate it.
        let interface = spawn_supervised("interface", || -> anyhow::Result<()> { Ok(()) });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("interface thread"),
            "error must name the interface thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_carries_interface_failure_cause() {
        // The interface thread resolves a typed anyhow::Result; a fault comes
        // back as a received Err, wrapped so the chain names the interface
        // thread AND carries the root cause for the operator.
        let interface = spawn_supervised("interface", || -> anyhow::Result<()> {
            Err(anyhow::anyhow!("the trigger subscription stream ended"))
        });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("interface thread"),
            "error must name the interface thread, got: {msg}"
        );
        assert!(
            msg.contains("subscription stream ended"),
            "error must carry the interface fault root cause, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_interface_panic() {
        // A panicking interface drops its result sender without a send; the
        // disconnect routes through the join handle so the formatted chain
        // carries the panic payload and the operator knows what went wrong.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let interface: Supervised<anyhow::Result<()>> =
            spawn_supervised("interface", || -> anyhow::Result<()> { panic!("boom") });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        std::panic::set_hook(prev_hook);
        let msg = format!("{err:#}");
        assert!(
            msg.contains("interface thread"),
            "error must name the interface thread, got: {msg}"
        );
        assert!(
            msg.contains("boom") || msg.contains("panic"),
            "error must carry the panic context, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_tail_end() {
        // The tail loop never returns Ok on its own, so a clean Ok(()) return
        // is an unexpected exit: clips would degrade to grace-timeout cuts
        // silently if not caught.
        let tail = spawn_supervised("tail", || -> anyhow::Result<()> { Ok(()) });

        let err = supervise(tail, pending(), no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("tail thread"),
            "error must name the tail thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_returns_ok_on_shutdown_signal() {
        // A delivered shutdown signal (SIGINT / SIGTERM) is a requested,
        // orderly stop — not a fault. supervise() must return Ok(()) so main
        // can exit zero, distinguishing it from a dead thread.
        let (sig_tx, sig_rx) = bounded(1);
        sig_tx.send(SIGINT).unwrap();

        let result = supervise(pending(), pending(), sig_rx);
        assert!(
            result.is_ok(),
            "a signal must return Ok(()), got: {result:?}"
        );
    }

    #[test]
    fn supervise_reports_signal_handler_failure() {
        // The signal forwarder dying (its channel disconnecting) must surface
        // as an error naming the signal handler — losing it silently would
        // mean SIGINT/SIGTERM could never trigger a clean shutdown. (An
        // installation failure carries the same attribution, raised in main()
        // before supervision starts.)
        let (sig_tx, sig_rx) = bounded::<i32>(1);
        drop(sig_tx);

        let err = supervise(pending(), pending(), sig_rx).unwrap_err();
        assert!(
            format!("{err:#}").contains("signal handler"),
            "error must name the signal handler, got: {err:#}"
        );
    }

    /// Admission at the limit, rejection above it, and slot reuse — the
    /// acceptance test for MAX_ACTIVE_TRIGGERS. This is the exact scenario the
    /// recorder must handle: 16 concurrent trigger handlers admitted, the 17th
    /// rejected (flood-sanity bound), then a completed handler returns its
    /// permit and a later trigger is admitted. Mirrors the consumer loop: a
    /// permit is taken without waiting and rides in the handler thread until
    /// it finishes.
    #[test]
    fn admission_at_the_limit_rejection_above_and_slot_reuse() -> anyhow::Result<()> {
        let active = Admission::new(MAX_ACTIVE_TRIGGERS);
        // Each handler parks on a watch until released, as a real handler
        // does while it waits out its window and extracts.
        let release = Arc::new(Watch::new(false));

        let mut handlers = Vec::new();
        for i in 0..MAX_ACTIVE_TRIGGERS {
            let permit = active
                .clone()
                .try_acquire()
                .ok_or_else(|| anyhow::anyhow!("trigger {i} rejected; expected admission"))?;
            let release = release.clone();
            handlers.push(std::thread::spawn(move || {
                let _permit = permit;
                release.wait_timeout_for(Duration::from_secs(30), |&v| v);
            }));
        }

        // Trigger 17: every permit is held by a parked handler — admission
        // must fail immediately, without waiting.
        assert!(
            active.clone().try_acquire().is_none(),
            "trigger beyond the limit must be rejected"
        );

        // Handlers complete; their permits return, so a later trigger is
        // admitted as soon as the first one comes back.
        release.send_replace(true);
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let _readmitted = loop {
            if let Some(permit) = active.clone().try_acquire() {
                break permit;
            }
            if std::time::Instant::now() >= deadline {
                anyhow::bail!("a completed handler must free a slot");
            }
            std::thread::sleep(Duration::from_millis(5));
        };

        for h in handlers {
            h.join().unwrap();
        }
        Ok(())
    }

    /// `ClipCompression::to_mcap` maps each variant to the expected
    /// `mcap::Compression` option: `None` yields uncompressed, `Zstd` and `Lz4`
    /// yield their named codecs.
    #[test]
    fn clip_compression_to_mcap_maps_all_variants() {
        // `mcap::Compression` is not `PartialEq`, so match the option shape.
        assert!(ClipCompression::None.to_mcap().is_none());
        assert!(matches!(
            ClipCompression::Zstd.to_mcap(),
            Some(mcap::Compression::Zstd)
        ));
        assert!(matches!(
            ClipCompression::Lz4.to_mcap(),
            Some(mcap::Compression::Lz4)
        ));
    }

    /// A panicking handler must return its permit: the admission bound would
    /// otherwise ratchet down with every panic until every trigger is
    /// rejected. The permit is held by the handler thread and returns on
    /// drop, which unwinding covers.
    #[test]
    fn panicking_handler_returns_its_permit() -> anyhow::Result<()> {
        // Suppress the panic backtrace noise in test output.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));

        let active = Admission::new(1);
        let permit = active
            .clone()
            .try_acquire()
            .ok_or_else(|| anyhow::anyhow!("a fresh admission gate must admit"))?;
        let handler = std::thread::spawn(move || {
            let _permit = permit;
            panic!("boom");
        });
        let joined = handler.join();
        std::panic::set_hook(prev_hook);
        assert!(joined.is_err(), "the handler must have panicked");

        assert!(
            active.try_acquire().is_some(),
            "the panicked handler's permit must be free again"
        );
        Ok(())
    }

    /// A representative "now" for the pure-function gate tests — far past every
    /// small anchor the trigger_time tests use, so only the future-skew tests
    /// approach the horizon.
    const TEST_NOW: u64 = 1_000_000_000_000_000_000;

    /// An admissible anchor at `TEST_NOW` from a transport stamp (not
    /// `trigger_time`): the shape the field tests vary one axis away from.
    fn valid_anchor() -> Anchor {
        Anchor {
            ns: TEST_NOW,
            from_trigger_time: false,
        }
    }

    /// A fully valid domain `Trigger` — accepted by `validate_trigger` with
    /// `valid_anchor()` at `TEST_NOW`; a test mutates one field to probe a bound.
    fn valid_trigger() -> Trigger {
        Trigger {
            name: "evt".to_string(),
            description: String::new(),
            trigger_time: clip::trigger::Stamp { sec: 0, nanosec: 0 },
            preroll: 0,
            postroll: 0,
        }
    }

    /// [`valid_trigger`] with a chosen `trigger_time`, for the matrix cell tests.
    fn trigger_with_time(trigger_time_ns: u64) -> Trigger {
        Trigger {
            trigger_time: clip::trigger::Stamp {
                sec: (trigger_time_ns / 1_000_000_000) as i32,
                nanosec: (trigger_time_ns % 1_000_000_000) as u32,
            },
            ..valid_trigger()
        }
    }

    /// A cell that ignores `trigger_time` (the anchor is not `from_trigger_time`
    /// — ros+log, mcap+log, mcap+publish) rejects a non-zero `trigger_time` and
    /// accepts zero.
    #[test]
    fn validate_rejects_trigger_time_in_an_ignoring_cell() {
        let ignoring = Anchor {
            ns: 5,
            from_trigger_time: false,
        };
        assert!(
            validate_trigger(&trigger_with_time(0), ignoring, TEST_NOW).is_ok(),
            "trigger_time=0 is accepted where the field is ignored"
        );
        assert!(
            validate_trigger(&trigger_with_time(7_000_000_250), ignoring, TEST_NOW).is_err(),
            "a non-zero trigger_time is rejected where the field is ignored"
        );
    }

    /// The one reading cell (ros+publish, the anchor *is* `from_trigger_time`)
    /// accepts any `trigger_time` — it is the window anchor there.
    #[test]
    fn validate_accepts_trigger_time_in_the_reading_cell() {
        let reading = Anchor {
            ns: 7_000_000_250,
            from_trigger_time: true,
        };
        assert!(validate_trigger(&trigger_with_time(7_000_000_250), reading, TEST_NOW).is_ok());
        assert!(
            validate_trigger(&trigger_with_time(0), reading, TEST_NOW).is_ok(),
            "trigger_time=0 anchors the window at the epoch, but that is the \
             publisher's choice, not a rejected one"
        );
    }

    /// `preroll` and `postroll` are each accepted exactly at [`MAX_ROLL_NS`] and
    /// rejected one nanosecond above it.
    #[test]
    fn validate_bounds_preroll_and_postroll() {
        let ok = |trig: &Trigger| validate_trigger(trig, valid_anchor(), TEST_NOW).is_ok();

        let mut t = valid_trigger();
        t.preroll = MAX_ROLL_NS;
        assert!(ok(&t), "preroll exactly at the maximum is accepted");
        t.preroll = MAX_ROLL_NS + 1;
        assert!(!ok(&t), "preroll one ns over the maximum is rejected");

        let mut t = valid_trigger();
        t.postroll = MAX_ROLL_NS;
        assert!(ok(&t), "postroll exactly at the maximum is accepted");
        t.postroll = MAX_ROLL_NS + 1;
        assert!(!ok(&t), "postroll one ns over the maximum is rejected");
    }

    /// The future-skew guard is on the *resolved anchor*: accepted exactly at the
    /// horizon, rejected beyond it, whatever cell produced the anchor.
    #[test]
    fn validate_guards_the_resolved_anchor_against_future_skew() {
        let check = |anchor: Anchor| validate_trigger(&valid_trigger(), anchor, TEST_NOW);
        let at = |ns: u64, from_trigger_time: bool| Anchor {
            ns,
            from_trigger_time,
        };

        assert!(
            check(at(TEST_NOW + MAX_ANCHOR_FUTURE_SKEW_NS, false)).is_ok(),
            "an anchor exactly at the future horizon is accepted"
        );
        assert!(
            check(at(TEST_NOW + MAX_ANCHOR_FUTURE_SKEW_NS + 1, false)).is_err(),
            "an anchor one ns past the horizon is rejected"
        );
        // The guard bites whatever cell resolved the anchor — a pathological
        // far-future ros+publish `trigger_time` and a hostile mcap record stamp
        // alike.
        let far_future = TEST_NOW + 10 * MAX_ANCHOR_FUTURE_SKEW_NS;
        assert!(
            check(at(far_future, true)).is_err(),
            "a far-future ros+publish trigger_time anchor is rejected"
        );
        assert!(
            check(at(far_future, false)).is_err(),
            "a far-future mcap record-stamp anchor is rejected"
        );
        assert!(
            check(at(TEST_NOW - 1, false)).is_ok(),
            "a past anchor is fine"
        );
    }

    /// A trigger `name` is accepted plain and exactly at [`MAX_TRIGGER_NAME_LEN`],
    /// and rejected when over-length, empty, or carrying a filename hazard (a
    /// path separator, NUL, leading dot, or embedded `..`).
    #[test]
    fn validate_rejects_unsafe_and_oversized_names() {
        let with_name = |name: &str| {
            let mut t = valid_trigger();
            t.name = name.to_string();
            validate_trigger(&t, valid_anchor(), TEST_NOW)
        };

        assert!(with_name("evt-1").is_ok(), "a plain name is accepted");
        assert!(
            with_name(&"a".repeat(MAX_TRIGGER_NAME_LEN)).is_ok(),
            "a name exactly at the length limit is accepted"
        );
        assert!(
            with_name(&"a".repeat(MAX_TRIGGER_NAME_LEN + 1)).is_err(),
            "a name one byte over the limit is rejected"
        );
        assert!(with_name("").is_err(), "an empty name is rejected");
        assert!(with_name("a/b").is_err(), "a path separator is rejected");
        assert!(with_name("a\\b").is_err(), "a backslash is rejected");
        assert!(with_name("a\0b").is_err(), "a NUL byte is rejected");
        assert!(with_name(".hidden").is_err(), "a leading dot is rejected");
        assert!(with_name("..").is_err(), "'..' is rejected");
        assert!(
            with_name("../escape").is_err(),
            "path traversal is rejected"
        );
        assert!(with_name("a..b").is_err(), "an embedded '..' is rejected");
    }
}
