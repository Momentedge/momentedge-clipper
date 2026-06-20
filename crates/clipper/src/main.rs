//! Triggered clip recorder reading ONE continuous MCAP file.
//!
//! A continuous `ros2 bag record` (started separately — see the README and
//! `scripts/record.sh`) writes a single growing MCAP file. This
//! binary keeps that file open and tails it ([`tail`]): an incremental scan
//! over the record framing that maintains a byte-extent index, a
//! schema/channel registry, and a coverage watch (the highest `log_time` on
//! disk). There are no bag splits and no `/events/write_split` dependency.
//!
//! On `/events/momentedge/trigger` (`momentedge_msgs/Trigger`) it cuts the
//! window `[trigger_time - preroll, trigger_time + postroll]`: wait until the
//! wall clock passes the window end, wait until the tail's coverage reaches it
//! (the recording provably holds the window), then bulk-copy the in-window
//! messages out of the planned extents into a clip published at
//! `./clipped/<trigger_ns>_<name>.mcap` (see [`clip`] — a raw-bytes
//! copy, no CDR decode, finished with a proper summary + footer, assembled in
//! a capturing dir and moved atomically into place so observers never see a
//! footer-less file), and finally publish `/events/momentedge/recorded`
//! (`momentedge_msgs/Recorded`), which therefore always names a durable clip.
//!
//! Time base: MCAP `log_time`, the trigger stamp, and the wait clock are all
//! treated as nanoseconds on the system (ROS) clock — this assumes the default
//! (no `use_sim_time`). Each trigger is handled on its own thread, so
//! overlapping windows are cut concurrently against one shared tail — at most
//! [`MAX_ACTIVE_TRIGGERS`] at once; a trigger beyond that limit is rejected,
//! logged, and ignored.
//!
//! Everything runs on plain OS threads — there is no async runtime. The main
//! thread supervises ([`supervise`]) four long-lived companions over
//! crossbeam channels: the tail thread (file scan), the node spin thread
//! (ROS executor), the trigger consumer (drains the typed subscription with
//! `futures::executor::block_on`), and a signal forwarder (SIGINT/SIGTERM →
//! orderly exit 0). Clip copies run on a fixed pool of
//! `extract_parallelism` worker threads consuming one FIFO job channel.
//!
//! Configuration is parsed by clap (`Config`): each setting is a CLI flag that
//! falls back to a `MOMENTEDGE_<KEY>` environment variable, then to a built-in
//! default — a CLI flag wins over the env var, which wins over the default. The
//! `MOMENTEDGE_*` env names are derived from one prefix applied to every field
//! (`with_env_prefix`). `--help` lists the flags and `--version` prints the
//! version. Settings (all optional):
//!   --record-dir   bag directory of the continuous recording (default ./record)
//!   --out-dir      where clips are written                   (default ./clipped)
//!   --grace-secs   how long past the window end to wait for coverage
//!                  before cutting from what is on disk (default 30; must
//!                  exceed the recorder's flush latency — for a chunked
//!                  recording roughly chunk size / aggregate data rate)
//!   --extract-parallelism  extraction worker threads (default 1: copies
//!                  queue FIFO — the bulk copy competes with the recorder's
//!                  writes for disk bandwidth; waiting is always concurrent)
//!   --clip-compression  codec for written clips: none, lz4, or zstd
//!                  (default zstd) — the choice is explicit, not inherited
//!                  from the mcap crate default
//!
//! Logging uses the `log` facade with a pretty_env_logger backend; `RUST_LOG`
//! controls verbosity.

mod clip;
mod tail;
mod watch;

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context;
use clap::{CommandFactory, FromArgMatches, Parser, ValueEnum};
use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
use futures::executor::block_on;
use futures::stream::StreamExt;
use log::{error, info, warn};
use r2r::{Publisher, QosProfile};
use signal_hook::consts::{SIGINT, SIGTERM};
use tail::{Coverage, Tailer};
use watch::Watch;

const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";
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

/// Recorder configuration, parsed by clap from CLI flags with a `CLIPPER_*`
/// environment-variable fallback per field (see [`load_config`]). The field doc
/// comments are the `--help` text: the first line is the short help, the rest is
/// shown under `--help`.
#[derive(Debug, Parser)]
#[command(version, about = "Triggered MCAP clip recorder")]
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
}

impl Config {
    fn grace(&self) -> Duration {
        Duration::from_secs(self.grace_secs)
    }
}

/// Prefix shared by every `MOMENTEDGE_*` environment variable. [`with_env_prefix`]
/// applies it to all of [`Config`]'s arguments, so the env names track the field
/// names (`grace_secs` → `MOMENTEDGE_GRACE_SECS`) with no per-field wiring.
const ENV_PREFIX: &str = "MOMENTEDGE";

/// Give every argument an environment-variable fallback named `<ENV_PREFIX>_` +
/// the field name upper-cased (`record_dir` → `MOMENTEDGE_RECORD_DIR`). One place
/// defines the prefix; the auto-generated `--help`/`--version` flags are left
/// without an env binding.
fn with_env_prefix(cmd: clap::Command) -> clap::Command {
    cmd.mut_args(|arg| match arg.get_id().as_str() {
        "help" | "version" => arg,
        id => {
            let env = format!("{ENV_PREFIX}_{}", id.to_uppercase());
            arg.env(env)
        }
    })
}

/// Parse [`Config`] from the command line, each field falling back to its
/// `CLIPPER_*` environment variable and then its default (CLI > env > default).
/// clap prints `--help`/`--version` and any parse error, then exits the process,
/// so this returns only a fully-populated config.
fn load_config() -> Config {
    let matches = with_env_prefix(Config::command()).get_matches();
    Config::from_arg_matches(&matches).unwrap_or_else(|e| e.exit())
}

/// One supervised thread: the channel its closure's return value arrives on,
/// and the join handle [`supervise`] harvests a panic payload from after a
/// disconnect.
type Supervised<T> = (Receiver<T>, JoinHandle<()>);

/// Spawn a named thread whose return value arrives on the paired channel.
///
/// [`supervise`] selects on that channel: a received value is the thread's
/// verdict (clean return or typed error); a disconnect without a value means
/// the closure unwound (panicked) before it could send, and the join handle
/// then carries the payload.
fn spawn_supervised<T: Send + 'static>(
    name: &str,
    f: impl FnOnce() -> T + Send + 'static,
) -> Supervised<T> {
    let (tx, rx) = bounded(1);
    let handle = thread::Builder::new()
        .name(name.to_string())
        .spawn(move || {
            // The send fails only when supervision is already gone, and the
            // value is then moot.
            let _ = tx.send(f());
        })
        .expect("spawning thread");
    (rx, handle)
}

/// Render a panic payload (from [`JoinHandle::join`] or
/// [`std::panic::catch_unwind`]) as text: panics carry a `&str` or `String`
/// message in practice; anything else gets a placeholder.
fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "non-string panic payload".to_string())
}

/// The error for a supervised thread whose result channel disconnected
/// without a value: the closure unwound before it could send, so the join —
/// immediate, the disconnect proves the thread is already dead — carries the
/// panic payload.
fn harvest_panic(handle: JoinHandle<()>) -> anyhow::Error {
    match handle.join() {
        Err(payload) => anyhow::anyhow!("thread panicked: {}", panic_text(payload.as_ref())),
        // Unreachable for spawn_supervised threads (a returning closure always
        // sends first), but a sane shape regardless.
        Ok(()) => anyhow::anyhow!("thread exited without reporting a result"),
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
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| {
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

/// One queued clip extraction: the window, the destination, and the channel
/// the worker replies on. Queued by [`record_clip`]; dequeued FIFO by the
/// extraction workers.
struct ExtractJob {
    start_ns: u64,
    end_ns: u64,
    out_path: PathBuf,
    reply: Sender<anyhow::Result<clip::ClipStats>>,
}

/// Spawn the fixed extraction worker pool: `parallelism` threads consuming
/// one shared channel. The channel is FIFO, so with the default single worker
/// the bulk copies serialize in submission order — extraction reads compete
/// with the recorder's writes for disk bandwidth (see
/// [`Config::extract_parallelism`]).
///
/// The window plan is snapshotted in the worker, after dequeue: the plan is
/// taken at copy start, so a job that waited in the queue still cuts from the
/// freshest index. A panicking extraction is caught and replied as an error —
/// per-job isolation, the pool outlives it.
fn spawn_extract_workers(
    parallelism: usize,
    tailer: Arc<Tailer>,
    compression: Option<mcap::Compression>,
) -> Sender<ExtractJob> {
    let (tx, rx) = unbounded::<ExtractJob>();
    for i in 0..parallelism.max(1) {
        let rx = rx.clone();
        let tailer = tailer.clone();
        thread::Builder::new()
            .name(format!("extract-{i}"))
            .spawn(move || {
                for job in rx.iter() {
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let plan = tailer.plan_window(job.start_ns, job.end_ns);
                        clip::extract_clip(
                            &plan,
                            &job.out_path,
                            job.start_ns,
                            job.end_ns,
                            compression,
                        )
                    }))
                    .unwrap_or_else(|payload| {
                        Err(anyhow::anyhow!(
                            "extraction panicked: {}",
                            panic_text(payload.as_ref())
                        ))
                    });
                    // A send failure means the handler is gone (its thread
                    // died); there is no one left to care about this clip.
                    let _ = job.reply.send(result);
                }
            })
            .expect("spawning extraction worker");
    }
    tx
}

/// Entry point and supervisor. Spawns the long-lived threads — tail, node
/// spin, trigger consumer, the extraction worker pool, and the signal
/// forwarder — then blocks in [`supervise`] until a shutdown signal (exit 0)
/// or the first dead critical thread (exit non-zero, for a supervisor to
/// restart the process).
///
/// Returning ends the process, which kills the remaining threads: the
/// immortal spin/tail loops, parked handlers, and any in-flight extraction.
/// That is safe for clips by construction — the capturing-dir reset at
/// startup reclaims any stranded staged file, and `out_dir` only ever holds
/// complete clips.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let cfg = Arc::new(load_config());

    // Start each run with a clean capturing dir: a crash mid-publish can strand
    // a stale staged file there, and clearing it at startup bounds that clutter
    // to a single run. This also creates out_dir, so the first clip can be
    // published without further setup. Fatal if it fails — a recorder that
    // cannot prepare its output directory must not start.
    clip::reset_capturing_dir(&cfg.out_dir)?;

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "clipper", "")?;

    let mut trigger_sub =
        node.subscribe::<r2r::momentedge_msgs::msg::Trigger>(TRIGGER_TOPIC, QosProfile::default())?;
    let recorded_pub = node.create_publisher::<r2r::momentedge_msgs::msg::Recorded>(
        RECORDED_TOPIC,
        QosProfile::default(),
    )?;

    let (tailer, coverage) = Tailer::new();

    // The tail thread: discovers and scans the recording for the process's
    // lifetime (blocking IO on its own thread). Supervised: with a dead tailer
    // every clip degrades to a grace-timeout cut, so the process exits rather
    // than limping on silently.
    let tail = {
        let tailer = tailer.clone();
        let record_dir = cfg.record_dir.clone();
        spawn_supervised("tail", move || tailer.run(&record_dir))
    };

    // One worker per allowed concurrent clip copy; see Config::extract_parallelism.
    // The clip compression codec is process-global, captured in the workers.
    let extract_tx = spawn_extract_workers(
        cfg.extract_parallelism,
        tailer,
        cfg.clip_compression.to_mcap(),
    );

    // Admission gate for trigger handlers; see [`Admission`].
    let admission = Admission::new(MAX_ACTIVE_TRIGGERS);

    // Trigger consumer: drains the typed trigger stream for the process's
    // lifetime, spawning one handler thread per admitted trigger. A trigger
    // arriving while all MAX_ACTIVE_TRIGGERS handlers are active is rejected
    // with error! and ignored — no handler, no clip, no Recorded. Supervised:
    // a dead consumer (stream closed or panic) must exit the process rather
    // than silently stopping on triggers.
    let consumer = {
        let cfg = cfg.clone();
        let coverage = coverage.clone();
        spawn_supervised("trigger-consumer", move || {
            while let Some(trig) = block_on(trigger_sub.next()) {
                let Some(permit) = admission.clone().try_acquire() else {
                    error!(
                        "trigger rejected: all {MAX_ACTIVE_TRIGGERS} trigger handlers are busy; \
                         ignoring name={:?} trigger_time={}",
                        trig.name,
                        time_to_ns(&trig.trigger_time),
                    );
                    continue;
                };
                let cfg = cfg.clone();
                let recorded_pub = recorded_pub.clone();
                let coverage = coverage.clone();
                let extract_tx = extract_tx.clone();
                // Per-trigger error isolation: a failed extraction is logged
                // and counted but does not tear down the consumer loop, and a
                // panic dies with the handler's own thread.
                let spawned = thread::Builder::new()
                    .name(format!("trigger-{}", time_to_ns(&trig.trigger_time)))
                    .spawn(move || {
                        let _permit = permit;
                        if let Err(e) =
                            handle_trigger(trig, cfg, recorded_pub, coverage, extract_tx)
                        {
                            error!("trigger handling failed: {e:#}");
                        }
                    });
                if let Err(e) = spawned {
                    error!("spawning a trigger handler failed: {e}");
                }
            }
        })
    };

    info!(
        "clipper up: triggers on {TRIGGER_TOPIC}, tailing {}, writing clips to {}",
        cfg.record_dir.display(),
        cfg.out_dir.display(),
    );
    if !cfg.record_dir.is_dir() {
        warn!(
            "record dir {} does not exist; the tail idles until the continuous \
             recording (scripts/record.sh) creates it",
            cfg.record_dir.display()
        );
    }

    // The node's single owner: spin continuously to feed the streams.
    // Supervised alongside the tail and consumer; any of the three exiting is
    // an error.
    let spin: Supervised<()> = spawn_supervised("node-spin", move || {
        loop {
            node.spin_once(Duration::from_millis(10));
        }
    });

    let signal_rx = signal_channel().context("signal handler failed to install")?;

    match supervise(tail, spin, consumer, signal_rx) {
        Ok(()) => Ok(()),
        Err(e) => Err(e.into()),
    }
}

/// Watch the three critical long-lived threads and the shutdown signal;
/// return when any of them resolves.
///
/// Each supervised thread reports on its channel (see [`spawn_supervised`]):
/// a received value is its verdict, a disconnect without a value is a panic,
/// harvested through the join handle so the payload lands in the error chain.
///
/// Returns `Ok(())` when the signal channel delivers SIGINT or SIGTERM — the
/// requested, orderly stop path; the signal is logged here and the caller
/// exits zero. Every other arm returns `Err`: a thread exiting (clean or
/// panic) is a fault that a supervisor must respond to by restarting the
/// process, and the signal channel disconnecting (the forwarder thread died)
/// must not be silent either, since it means SIGINT could never trigger a
/// clean shutdown.
///
/// The tail thread carries a typed `anyhow::Result<()>`: its loop never
/// returns `Ok` on its own, so a clean return is treated as an unexpected
/// exit, while a scan fault it could not retry past surfaces as the inner
/// `Err`, wrapped so the operator sees the scan-fault root cause and the path
/// it named.
///
/// All three threads must run for the lifetime of the process: the tail
/// thread feeds coverage and the extent index (a dead tailer silently
/// degrades every clip to a grace-timeout cut); the spin thread pumps the ROS
/// node (a dead spin thread silently stops delivering triggers); the trigger
/// consumer drains the typed stream (a dead consumer silently stops acting on
/// triggers).
fn supervise(
    tail: Supervised<anyhow::Result<()>>,
    spin: Supervised<()>,
    consumer: Supervised<()>,
    signal: Receiver<i32>,
) -> anyhow::Result<()> {
    let (tail_rx, tail_handle) = tail;
    let (spin_rx, spin_handle) = spin;
    let (consumer_rx, consumer_handle) = consumer;
    select! {
        recv(tail_rx) -> res => match res {
            // run() loops for the process's lifetime, so a clean return is
            // as unexpected as the other threads ending. A scan fault it
            // could not retry past comes back as the inner Err, wrapped so
            // the operator sees the root cause; a panic is the disconnect.
            Ok(Ok(())) => anyhow::bail!("tail thread exited unexpectedly"),
            Ok(Err(e)) => Err(e.context("tail thread failed")),
            Err(_) => Err(harvest_panic(tail_handle).context("tail thread exited unexpectedly")),
        },
        recv(spin_rx) -> res => match res {
            Ok(()) => anyhow::bail!("node spin thread exited unexpectedly"),
            Err(_) => {
                Err(harvest_panic(spin_handle).context("node spin thread exited unexpectedly"))
            }
        },
        recv(consumer_rx) -> res => match res {
            Ok(()) => anyhow::bail!("trigger consumer exited unexpectedly"),
            Err(_) => {
                Err(harvest_panic(consumer_handle).context("trigger consumer exited unexpectedly"))
            }
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

/// Run one trigger's wait-then-extract-then-announce flow.
fn handle_trigger(
    trig: r2r::momentedge_msgs::msg::Trigger,
    cfg: Arc<Config>,
    recorded_pub: Publisher<r2r::momentedge_msgs::msg::Recorded>,
    coverage: Arc<Watch<Coverage>>,
    extract_tx: Sender<ExtractJob>,
) -> anyhow::Result<()> {
    let trigger_ns = time_to_ns(&trig.trigger_time);
    let start_ns = trigger_ns.saturating_sub(trig.preroll);
    let end_ns = trigger_ns.saturating_add(trig.postroll);
    info!(
        "trigger name={:?} window=[{start_ns}, {end_ns}] preroll={} postroll={}",
        trig.name, trig.preroll, trig.postroll
    );

    let out_path = cfg
        .out_dir
        .join(format!("{trigger_ns}_{}.mcap", sanitize(&trig.name)));
    let stats = record_clip(
        start_ns,
        end_ns,
        out_path,
        &coverage,
        cfg.grace(),
        &extract_tx,
    )?;
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

    let filename = stats.out_path.to_string_lossy().into_owned();
    let recorded = r2r::momentedge_msgs::msg::Recorded {
        name: trig.name.clone(),
        filename: filename.clone(),
        description: trig.description.clone(),
        trigger_time: trig.trigger_time.clone(),
        preroll: trig.preroll,
    };
    recorded_pub.publish(&recorded)?;
    info!(
        "emitted {RECORDED_TOPIC} name={:?} filename={filename}",
        trig.name
    );
    Ok(())
}

/// The decode-free, ROS-free core of [`handle_trigger`]: wait out the
/// postroll on the wall clock, wait for the tail's coverage to reach the
/// window end, then queue the extraction on the worker channel and block on
/// the reply. The workers dequeue FIFO and snapshot the window plan at copy
/// start, so a job that waited in the queue still cuts from the freshest
/// index. The extraction stages the clip and moves it atomically into
/// `out_dir`, so the returned [`clip::ClipStats`] already names a durable
/// file: the caller may announce it immediately.
fn record_clip(
    start_ns: u64,
    end_ns: u64,
    out_path: PathBuf,
    coverage: &Watch<Coverage>,
    grace: Duration,
    extract_tx: &Sender<ExtractJob>,
) -> anyhow::Result<clip::ClipStats> {
    // 1. Wait out the postroll: hold until the wall clock passes the window end.
    let now = now_ns();
    if end_ns > now {
        thread::sleep(Duration::from_nanos(end_ns - now));
    }

    // 2. Wait until the recording provably covers the window end: a message
    //    with log_time at/after it is on disk (or the recording ended). The
    //    grace timeout bounds the wait when the recorded topics go quiet —
    //    and bounds every other stall the same way (a dead tail thread
    //    already exits the process through supervision).
    if !coverage.wait_timeout_for(grace, |c| c.ended || c.high_water_ns >= end_ns) {
        warn!(
            "window end {end_ns} still uncovered after {grace:?}; \
             cutting the clip from what is on disk"
        );
    }

    // 3. Queue the copy on the extraction workers (FIFO; default one copy at
    //    a time — the bulk copy competes with the recorder's writes for disk
    //    bandwidth) and block on the reply.
    let (reply_tx, reply_rx) = bounded(1);
    extract_tx
        .send(ExtractJob {
            start_ns,
            end_ns,
            out_path,
            reply: reply_tx,
        })
        .map_err(|_| anyhow::anyhow!("the extraction workers are gone"))?;
    reply_rx
        .recv()
        .map_err(|_| anyhow::anyhow!("the extraction worker dropped the job"))?
}

/// Nanoseconds since the Unix epoch on the system clock.
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Flatten a `builtin_interfaces/Time` to nanoseconds since the epoch on the
/// system clock (no `use_sim_time`).
fn time_to_ns(t: &r2r::builtin_interfaces::msg::Time) -> u64 {
    t.sec.max(0) as u64 * 1_000_000_000 + t.nanosec as u64
}

/// Make a trigger name safe to embed in a filename: keep alphanumerics, `-`,
/// `_` and `.`; everything else (notably `/`) becomes `_`.
fn sanitize(name: &str) -> String {
    let s: String = name
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || matches!(c, '-' | '_' | '.') {
                c
            } else {
                '_'
            }
        })
        .collect();
    if s.is_empty() {
        "unnamed".to_string()
    } else {
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The clip compression the recorder's default (zstd) maps to; the unit
    /// tests drive the extraction worker pool through the same codec the
    /// recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// Parse a `Config` from an explicit argv through the same env-prefixed
    /// command `load_config` builds, so the tests exercise the real wiring.
    fn parse_from<I, T>(argv: I) -> Result<Config, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        let matches = with_env_prefix(Config::command()).try_get_matches_from(argv)?;
        Config::from_arg_matches(&matches)
    }

    #[test]
    fn config_defaults_when_no_flags_given() {
        let cfg = parse_from(["clipper"]).unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("./record"));
        assert_eq!(cfg.out_dir, PathBuf::from("./clipped"));
        assert_eq!(cfg.grace(), Duration::from_secs(30));
        assert_eq!(cfg.extract_parallelism, 1);
        assert_eq!(cfg.clip_compression, ClipCompression::Zstd);
    }

    #[test]
    fn config_cli_flags_populate_every_field() {
        let cfg = parse_from([
            "clipper",
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
        ])
        .unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("/data/record"));
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.grace(), Duration::from_secs(7));
        assert_eq!(cfg.extract_parallelism, 3);
        assert_eq!(cfg.clip_compression, ClipCompression::Lz4);
    }

    #[test]
    fn env_prefix_binds_clipper_names_to_every_field() {
        let cmd = with_env_prefix(Config::command());
        let env_of = |id: &str| {
            cmd.get_arguments()
                .find(|a| a.get_id().as_str() == id)
                .and_then(|a| a.get_env())
                .map(|e| e.to_string_lossy().into_owned())
        };
        assert_eq!(
            env_of("record_dir").as_deref(),
            Some("MOMENTEDGE_RECORD_DIR")
        );
        assert_eq!(env_of("out_dir").as_deref(), Some("MOMENTEDGE_OUT_DIR"));
        assert_eq!(
            env_of("grace_secs").as_deref(),
            Some("MOMENTEDGE_GRACE_SECS")
        );
        assert_eq!(
            env_of("extract_parallelism").as_deref(),
            Some("MOMENTEDGE_EXTRACT_PARALLELISM")
        );
        assert_eq!(
            env_of("clip_compression").as_deref(),
            Some("MOMENTEDGE_CLIP_COMPRESSION")
        );
    }

    #[test]
    fn sanitize_replaces_separators_and_whitespace() {
        // The slash replacement is the safety property: a trigger name can
        // never introduce a path component into <trigger_ns>_<name>.mcap.
        assert_eq!(sanitize("a/b c"), "a_b_c");
        assert_eq!(sanitize("../escape"), ".._escape");
        assert_eq!(sanitize(""), "unnamed");
    }

    #[test]
    fn time_to_ns_flattens_and_clamps() {
        let t = r2r::builtin_interfaces::msg::Time {
            sec: 2,
            nanosec: 500,
        };
        assert_eq!(time_to_ns(&t), 2_000_000_500);
        let neg = r2r::builtin_interfaces::msg::Time {
            sec: -5,
            nanosec: 250,
        };
        assert_eq!(time_to_ns(&neg), 250);
    }

    #[test]
    fn config_rejects_a_non_numeric_grace() {
        assert!(parse_from(["clipper", "--grace-secs", "soon"]).is_err());
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

        let err = supervise(tail, pending(), pending(), no_signal()).unwrap_err();
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
    fn supervise_reports_consumer_end() {
        // A consumer thread that exits cleanly (stream ended or explicit
        // return) is an error: the recorder stops receiving triggers with no
        // noise. The other arms are parked as "pending forever" to isolate
        // the consumer signal.
        let consumer = spawn_supervised("trigger-consumer", || {});

        let err = supervise(pending(), pending(), consumer, no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("trigger consumer"),
            "error must name the trigger consumer, got: {err:#}"
        );
    }

    #[test]
    fn supervise_reports_consumer_panic() {
        // A panicking consumer drops its result sender without a send; the
        // disconnect routes through the join handle so the formatted chain
        // carries the panic payload and the operator knows what went wrong.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let consumer: Supervised<()> = spawn_supervised("trigger-consumer", || panic!("boom"));

        let err = supervise(pending(), pending(), consumer, no_signal()).unwrap_err();
        std::panic::set_hook(prev_hook);
        let msg = format!("{err:#}");
        assert!(
            msg.contains("trigger consumer"),
            "error must name the trigger consumer, got: {msg}"
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

        let err = supervise(tail, pending(), pending(), no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("tail thread"),
            "error must name the tail thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_reports_spin_end() {
        // The spin thread exiting means the node is no longer pumping
        // messages: triggers silently stop arriving.
        let spin = spawn_supervised("node-spin", || {});

        let err = supervise(pending(), spin, pending(), no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("node spin thread"),
            "error must name the node spin thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_returns_ok_on_shutdown_signal() {
        // A delivered shutdown signal (SIGINT / SIGTERM) is a requested,
        // orderly stop — not a fault. supervise() must return Ok(()) so main
        // can exit zero, distinguishing it from a dead thread.
        let (sig_tx, sig_rx) = bounded(1);
        sig_tx.send(SIGINT).unwrap();

        let result = supervise(pending(), pending(), pending(), sig_rx);
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

        let err = supervise(pending(), pending(), pending(), sig_rx).unwrap_err();
        assert!(
            format!("{err:#}").contains("signal handler"),
            "error must name the signal handler, got: {err:#}"
        );
    }

    use crate::clip::tests::read_clip;
    use crate::tail::tests::{scan_to_end, test_dir, write_recording, write_unfinished_recording};

    #[test]
    fn record_clip_grace_timeout_cuts_what_is_on_disk() -> anyhow::Result<()> {
        let root = test_dir("grace")?;
        let (tailer, coverage) = Tailer::new();
        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);

        // The window end is far in the past on the wall clock (no postroll
        // sleep), but coverage never reaches it — no recording was ever
        // discovered. The grace timeout must fire and cut a valid empty clip
        // instead of hanging or erroring.
        let stats = record_clip(
            0,
            1_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_millis(50),
            &extract_tx,
        )?;

        assert_eq!(stats.messages_copied, 0);
        assert!(read_clip(&stats.out_path)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_clip_completes_once_coverage_arrives() -> anyhow::Result<()> {
        let root = test_dir("cov")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 900), ("/t", 2_000)])?;

        // The tail discovers and scans the recording a little later, as a
        // live tail would; record_clip must block on the coverage watch until
        // a message at/after the window end (1_000) is on disk.
        let (tailer, coverage) = Tailer::new();
        let scanner = tailer.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            let file = Arc::new(std::fs::File::open(&rec).unwrap());
            scanner.attach(file.clone());
            scan_to_end(&scanner, &file, 8).unwrap();
        });

        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let stats = record_clip(
            100,
            1_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_secs(10),
            &extract_tx,
        )?;

        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_clip(&stats.out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 900)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_clip_waits_out_the_postroll() -> anyhow::Result<()> {
        let root = test_dir("postroll")?;
        let rec = root.join("rec.mcap");
        let now = now_ns();
        // One message inside the window, one past the window end so coverage
        // is already satisfied — only the wall-clock wait holds the cut back.
        write_recording(&rec, false, &[("/t", now), ("/t", now + 300_000_000)])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let end_ns = now + 150_000_000; // 150 ms past the trigger stamp
        let started = std::time::Instant::now();
        let stats = record_clip(
            now.saturating_sub(1_000_000_000),
            end_ns,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_secs(10),
            &extract_tx,
        )?;

        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "the cut must wait for the wall clock to pass the window end"
        );
        assert_eq!(stats.messages_copied, 1, "the future message is outside");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_clip_cuts_immediately_when_the_recording_ended() -> anyhow::Result<()> {
        let root = test_dir("ended")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        // Footer scanned → ended. The high-water mark (200) stays far below
        // the window end, so only the ended flag can release the coverage
        // wait — it must short-circuit the 30 s grace, cutting what exists.
        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let started = std::time::Instant::now();
        let stats = record_clip(
            50,
            1_000_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_secs(30),
            &extract_tx,
        )?;

        assert!(
            started.elapsed() < Duration::from_secs(5),
            "ended must short-circuit the grace wait"
        );
        assert_eq!(stats.messages_copied, 2);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn coverage_exactly_at_the_window_end_releases_the_wait() -> anyhow::Result<()> {
        let root = test_dir("cov-eq")?;
        let rec = root.join("rec.mcap");
        // A live (unfinished) recording whose newest message sits EXACTLY at
        // the window end: `high_water >= end` must release the wait without
        // the ended flag and without burning the grace timeout.
        write_unfinished_recording(&rec, "/t", &[100, 1_000])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        {
            let cov = coverage.get();
            assert!(!cov.ended, "no footer was written");
            assert_eq!(cov.high_water_ns, 1_000);
        }

        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let started = std::time::Instant::now();
        let stats = record_clip(
            0,
            1_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_secs(30),
            &extract_tx,
        )?;

        assert!(
            started.elapsed() < Duration::from_secs(5),
            "high_water == end satisfies the wait (>=, not >)"
        );
        assert_eq!(stats.messages_copied, 2, "the boundary message is inside");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn concurrent_overlapping_triggers_serialize_and_take_distinct_paths() -> anyhow::Result<()> {
        let root = test_dir("overlap")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            false,
            &[("/t", 100), ("/t", 200), ("/t", 300), ("/t", 400)],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        // Two overlapping windows racing for the same out path and a single
        // extraction worker: the copies serialize FIFO, the second writer
        // lands on a `_1` sibling, and both clips come out complete.
        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let cut = |start_ns: u64, end_ns: u64| {
            let coverage = coverage.clone();
            let extract_tx = extract_tx.clone();
            let out = out.clone();
            std::thread::spawn(move || {
                record_clip(
                    start_ns,
                    end_ns,
                    out,
                    &coverage,
                    Duration::from_secs(30),
                    &extract_tx,
                )
            })
        };
        let (ha, hb) = (cut(100, 300), cut(200, 400));
        let a = ha.join().unwrap()?;
        let b = hb.join().unwrap()?;

        assert_ne!(
            a.out_path, b.out_path,
            "two writers must never share a file"
        );
        let mut paths = vec![a.out_path.clone(), b.out_path.clone()];
        paths.sort();
        assert_eq!(paths, vec![out, root.join("clip_1.mcap")]);
        assert_eq!(
            read_clip(&a.out_path)?,
            vec![
                ("/t".to_string(), 100),
                ("/t".to_string(), 200),
                ("/t".to_string(), 300),
            ]
        );
        assert_eq!(
            read_clip(&b.out_path)?,
            vec![
                ("/t".to_string(), 200),
                ("/t".to_string(), 300),
                ("/t".to_string(), 400),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// FIFO is a property of the worker channel, observed end to end: two
    /// jobs racing for the same desired out path are submitted in a known
    /// order, and the single worker dequeues them in exactly that order, so
    /// the first job claims the unsuffixed name and the second resolves to
    /// the `_1` sibling — deterministically, not as one of two race outcomes.
    #[test]
    fn extract_workers_dequeue_jobs_fifo() -> anyhow::Result<()> {
        let root = test_dir("fifo")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let (tailer, _coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx = spawn_extract_workers(1, tailer, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let (reply_a, recv_a) = bounded(1);
        let (reply_b, recv_b) = bounded(1);
        extract_tx.send(ExtractJob {
            start_ns: 0,
            end_ns: 300,
            out_path: out.clone(),
            reply: reply_a,
        })?;
        extract_tx.send(ExtractJob {
            start_ns: 0,
            end_ns: 300,
            out_path: out.clone(),
            reply: reply_b,
        })?;

        let a = recv_a.recv()??;
        let b = recv_b.recv()??;
        assert_eq!(a.out_path, out, "the first-submitted job claims the name");
        assert_eq!(
            b.out_path,
            root.join("clip_1.mcap"),
            "the second job resolves against the taken name"
        );
        assert_eq!(a.messages_copied, 2);
        assert_eq!(b.messages_copied, 2);

        std::fs::remove_dir_all(root)?;
        Ok(())
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
}
