//! Triggered clip recorder reading ONE continuous MCAP file.
//!
//! A continuous `ros2 bag record` (started separately — see the README and
//! `scripts/record-continuous.sh`) writes a single growing MCAP file. This
//! binary keeps that file open and tails it ([`tail`]): an incremental scan
//! over the record framing that maintains a byte-extent index, a
//! schema/channel registry, and a coverage watch (the highest `log_time` on
//! disk). There are no bag splits and no `/events/write_split` dependency.
//!
//! On `/events/edgestream/trigger` (`edgestream_msgs/Trigger`) it cuts the
//! window `[trigger_time - preroll, trigger_time + postroll]`: wait until the
//! wall clock passes the window end, wait until the tail's coverage reaches it
//! (the recording provably holds the window), then bulk-copy the in-window
//! messages out of the planned extents into
//! `./triggered-cont/<trigger_ns>_<name>.mcap` (see [`clip`] — a raw-bytes
//! copy, no CDR decode, finished with a proper summary + footer), and finally
//! publish `/events/edgestream/recorded` (`edgestream_msgs/Recorded`).
//!
//! Time base: MCAP `log_time`, the trigger stamp, and the wait clock are all
//! treated as nanoseconds on the system (ROS) clock — this assumes the default
//! (no `use_sim_time`). Each trigger is handled in its own task, so
//! overlapping windows are cut concurrently against one shared tail.
//!
//! Configuration is layered (defaults → TOML file → environment, via
//! config-rs); there are no CLI args. The TOML file is
//! `edgestream-rec-cont.toml` in the working directory, or the path in
//! `$EDGESTREAM_CONFIG`; each key also reads from an
//! `EDGESTREAM_<KEY>` environment variable. Keys (all optional):
//!   record_dir   bag directory of the continuous recording (default ./record-cont)
//!   out_dir      where clips are written                   (default ./triggered-cont)
//!   grace_secs   how long past the window end to wait for coverage
//!                before cutting from what is on disk (default 30; must
//!                exceed the recorder's flush latency — for a chunked
//!                recording roughly chunk size / aggregate data rate)
//!   extract_parallelism  concurrent clip copies (default 1: extractions
//!                queue FIFO — the bulk copy competes with the recorder's
//!                writes for disk bandwidth; waiting is always concurrent)
//!
//! Logging uses the `log` facade with a pretty_env_logger backend; `RUST_LOG`
//! controls verbosity.

mod clip;
mod tail;

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::Context;
use futures::stream::StreamExt;
use log::{error, info, warn};
use r2r::{Publisher, QosProfile};
use serde::Deserialize;
use tokio::sync::{Semaphore, watch};

use tail::{Coverage, Tailer};

const TRIGGER_TOPIC: &str = "/events/edgestream/trigger";
const RECORDED_TOPIC: &str = "/events/edgestream/recorded";

#[derive(Debug, Deserialize)]
struct Config {
    record_dir: PathBuf,
    out_dir: PathBuf,
    /// How long past the window end to keep waiting for the recording to
    /// cover `trigger_time + postroll` before cutting the clip from what is
    /// on disk. Coverage normally lags the wall clock by the recorder's flush
    /// latency only; the timeout fires when the recorded topics go quiet, and
    /// the clip then simply ends at the last data that exists.
    grace_secs: u64,
    /// How many clip extractions may run at once. The default of 1
    /// serializes the bulk copies FIFO: extraction reads compete with the
    /// recorder's writes on the same disk, and concurrent copies inflate the
    /// recorder's flush latency (rosbag2's cache drops messages when it
    /// cannot drain). Waiting — postroll, coverage — is always concurrent;
    /// only the copy is gated. Raise on storage with IO headroom.
    extract_parallelism: usize,
}

impl Config {
    fn grace(&self) -> Duration {
        Duration::from_secs(self.grace_secs)
    }
}

/// Layered load: defaults, then the TOML file (`edgestream-rec-cont.toml` in
/// the working directory, or `$EDGESTREAM_CONFIG`; missing is fine), then
/// `EDGESTREAM_<KEY>` environment variables.
fn load_config() -> Result<Config, config::ConfigError> {
    let file = std::env::var("EDGESTREAM_CONFIG")
        .unwrap_or_else(|_| "edgestream-rec-cont.toml".to_string());
    config::Config::builder()
        .set_default("record_dir", "./record-cont")?
        .set_default("out_dir", "./triggered-cont")?
        .set_default("grace_secs", 30_u64)?
        .set_default("extract_parallelism", 1_u64)?
        .add_source(config::File::with_name(&file).required(false))
        .add_source(config::Environment::with_prefix("EDGESTREAM"))
        .build()?
        .try_deserialize()
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let cfg = Arc::new(load_config()?);

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "edgestream_recorder_cont", "")?;

    let mut trigger_sub =
        node.subscribe::<r2r::edgestream_msgs::msg::Trigger>(TRIGGER_TOPIC, QosProfile::default())?;
    let recorded_pub = node.create_publisher::<r2r::edgestream_msgs::msg::Recorded>(
        RECORDED_TOPIC,
        QosProfile::default(),
    )?;

    let (tailer, coverage_rx) = Tailer::new();

    // The tail thread: discovers and scans the recording for the process's
    // lifetime (blocking IO, so off the async runtime). The handle is watched
    // at the bottom of main: with a dead tailer every clip degrades to a
    // grace-timeout cut, so the process exits instead.
    let tail_task = {
        let tailer = tailer.clone();
        let record_dir = cfg.record_dir.clone();
        tokio::task::spawn_blocking(move || tailer.run(&record_dir))
    };

    // One permit per allowed concurrent clip copy; see Config::extract_parallelism.
    let extract_permits = Arc::new(Semaphore::new(cfg.extract_parallelism.max(1)));

    // trigger consumer: spawn one handler per trigger.
    {
        let cfg = cfg.clone();
        let tailer = tailer.clone();
        tokio::spawn(async move {
            while let Some(trig) = trigger_sub.next().await {
                let cfg = cfg.clone();
                let recorded_pub = recorded_pub.clone();
                let tailer = tailer.clone();
                let coverage_rx = coverage_rx.clone();
                let permits = extract_permits.clone();
                tokio::spawn(async move {
                    if let Err(e) =
                        handle_trigger(trig, cfg, recorded_pub, tailer, coverage_rx, permits).await
                    {
                        error!("trigger handling failed: {e:#}");
                    }
                });
            }
        });
    }

    info!(
        "edgestream-rec-cont up: triggers on {TRIGGER_TOPIC}, tailing {}, writing clips to {}",
        cfg.record_dir.display(),
        cfg.out_dir.display(),
    );
    if !cfg.record_dir.is_dir() {
        warn!(
            "record dir {} does not exist; the tail idles until the continuous \
             recording (scripts/record-continuous.sh) creates it",
            cfg.record_dir.display()
        );
    }

    // The node's single owner: spin continuously to feed the streams.
    let spin_task = tokio::task::spawn_blocking(move || {
        loop {
            node.spin_once(Duration::from_millis(10));
        }
    });

    // Neither thread returns, so either handle resolving means a panic (or an
    // impossible clean exit). Exit non-zero so a supervisor restarts the
    // recorder rather than letting it limp on with a dead tail or node.
    tokio::select! {
        res = tail_task => {
            res?;
            Err("tail thread exited unexpectedly".into())
        }
        res = spin_task => {
            res?;
            Err("node spin thread exited unexpectedly".into())
        }
    }
}

/// Run one trigger's wait-then-extract-then-announce flow.
async fn handle_trigger(
    trig: r2r::edgestream_msgs::msg::Trigger,
    cfg: Arc<Config>,
    recorded_pub: Publisher<r2r::edgestream_msgs::msg::Recorded>,
    tailer: Arc<Tailer>,
    coverage_rx: watch::Receiver<Coverage>,
    permits: Arc<Semaphore>,
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
        tailer,
        coverage_rx,
        cfg.grace(),
        permits,
    )
    .await?;
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
    let recorded = r2r::edgestream_msgs::msg::Recorded {
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
/// window end, then — holding an extraction permit — snapshot the window
/// plan and run the blocking extraction.
async fn record_clip(
    start_ns: u64,
    end_ns: u64,
    out_path: PathBuf,
    tailer: Arc<Tailer>,
    mut coverage_rx: watch::Receiver<Coverage>,
    grace: Duration,
    permits: Arc<Semaphore>,
) -> anyhow::Result<clip::ClipStats> {
    // 1. Wait out the postroll: hold until the wall clock passes the window end.
    let now = now_ns();
    if end_ns > now {
        tokio::time::sleep(Duration::from_nanos(end_ns - now)).await;
    }

    // 2. Wait until the recording provably covers the window end: a message
    //    with log_time at/after it is on disk (or the recording ended). The
    //    grace timeout bounds the wait when the recorded topics go quiet.
    let covered = coverage_rx.wait_for(|c| c.ended || c.high_water_ns >= end_ns);
    match tokio::time::timeout(grace, covered).await {
        Ok(Ok(_)) => {}
        Ok(Err(_)) => anyhow::bail!("tail stopped before the window was covered"),
        Err(_) => warn!(
            "window end {end_ns} still uncovered after {grace:?}; \
             cutting the clip from what is on disk"
        ),
    }

    // 3. Acquire an extraction permit (FIFO; default one copy at a time —
    //    the bulk copy competes with the recorder's writes for disk
    //    bandwidth), then snapshot the plan and bulk-copy the window. The
    //    copy is blocking file IO, so it runs off the async runtime.
    let _permit = permits
        .acquire_owned()
        .await
        .context("extraction semaphore closed")?;
    let plan = tailer.plan_window(start_ns, end_ns);
    tokio::task::spawn_blocking(move || clip::extract_clip(&plan, &out_path, start_ns, end_ns))
        .await?
}

/// Nanoseconds since the Unix epoch on the system clock.
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Flatten a `builtin_interfaces/Time` to nanoseconds since the epoch.
/// Keep in step with the identical helper in `edgestream-rec`.
fn time_to_ns(t: &r2r::builtin_interfaces::msg::Time) -> u64 {
    t.sec.max(0) as u64 * 1_000_000_000 + t.nanosec as u64
}

/// Make a trigger name safe to embed in a filename: keep alphanumerics, `-`,
/// `_` and `.`; everything else (notably `/`) becomes `_`. Keep in step with
/// the identical helper in `edgestream-rec`.
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

    #[test]
    fn config_loads_every_key_from_toml() {
        let cfg: Config = config::Config::builder()
            .add_source(config::File::from_str(
                r#"
                record_dir = "/data/record"
                out_dir = "/data/clips"
                grace_secs = 7
                extract_parallelism = 3
                "#,
                config::FileFormat::Toml,
            ))
            .build()
            .unwrap()
            .try_deserialize()
            .unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("/data/record"));
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.grace(), Duration::from_secs(7));
        assert_eq!(cfg.extract_parallelism, 3);
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
        let res = config::Config::builder()
            .add_source(config::File::from_str(
                r#"
                record_dir = "/data/record"
                out_dir = "/data/clips"
                grace_secs = "soon"
                extract_parallelism = 1
                "#,
                config::FileFormat::Toml,
            ))
            .build()
            .unwrap()
            .try_deserialize::<Config>();
        assert!(res.is_err());
    }

    use crate::clip::tests::read_clip;
    use crate::tail::tests::{scan_to_end, test_dir, write_recording, write_unfinished_recording};

    #[tokio::test]
    async fn record_clip_grace_timeout_cuts_what_is_on_disk() -> anyhow::Result<()> {
        let root = test_dir("grace")?;
        let (tailer, coverage_rx) = Tailer::new();

        // The window end is far in the past on the wall clock (no postroll
        // sleep), but coverage never reaches it — no recording was ever
        // discovered. The grace timeout must fire and cut a valid empty clip
        // instead of hanging or erroring.
        let stats = record_clip(
            0,
            1_000,
            root.join("clip.mcap"),
            tailer,
            coverage_rx,
            Duration::from_millis(50),
            Arc::new(Semaphore::new(1)),
        )
        .await?;

        assert_eq!(stats.messages_copied, 0);
        assert!(read_clip(&stats.out_path)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn record_clip_completes_once_coverage_arrives() -> anyhow::Result<()> {
        let root = test_dir("cov")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 900), ("/t", 2_000)])?;

        // The tail discovers and scans the recording a little later, as a
        // live tail would; record_clip must block on the coverage watch until
        // a message at/after the window end (1_000) is on disk.
        let (tailer, coverage_rx) = Tailer::new();
        let scanner = tailer.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            let file = Arc::new(std::fs::File::open(&rec).unwrap());
            scanner.attach(file.clone());
            scan_to_end(&scanner, &file, 8).unwrap();
        });

        let stats = record_clip(
            100,
            1_000,
            root.join("clip.mcap"),
            tailer,
            coverage_rx,
            Duration::from_secs(10),
            Arc::new(Semaphore::new(1)),
        )
        .await?;

        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_clip(&stats.out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 900)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn record_clip_waits_out_the_postroll() -> anyhow::Result<()> {
        let root = test_dir("postroll")?;
        let rec = root.join("rec.mcap");
        let now = now_ns();
        // One message inside the window, one past the window end so coverage
        // is already satisfied — only the wall-clock wait holds the cut back.
        write_recording(&rec, false, &[("/t", now), ("/t", now + 300_000_000)])?;

        let (tailer, coverage_rx) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let end_ns = now + 150_000_000; // 150 ms past the trigger stamp
        let started = std::time::Instant::now();
        let stats = record_clip(
            now.saturating_sub(1_000_000_000),
            end_ns,
            root.join("clip.mcap"),
            tailer,
            coverage_rx,
            Duration::from_secs(10),
            Arc::new(Semaphore::new(1)),
        )
        .await?;

        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "the cut must wait for the wall clock to pass the window end"
        );
        assert_eq!(stats.messages_copied, 1, "the future message is outside");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn record_clip_cuts_immediately_when_the_recording_ended() -> anyhow::Result<()> {
        let root = test_dir("ended")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        // Footer scanned → ended. The high-water mark (200) stays far below
        // the window end, so only the ended flag can release the coverage
        // wait — it must short-circuit the 30 s grace, cutting what exists.
        let (tailer, coverage_rx) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let started = std::time::Instant::now();
        let stats = record_clip(
            50,
            1_000_000,
            root.join("clip.mcap"),
            tailer,
            coverage_rx,
            Duration::from_secs(30),
            Arc::new(Semaphore::new(1)),
        )
        .await?;

        assert!(
            started.elapsed() < Duration::from_secs(5),
            "ended must short-circuit the grace wait"
        );
        assert_eq!(stats.messages_copied, 2);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn coverage_exactly_at_the_window_end_releases_the_wait() -> anyhow::Result<()> {
        let root = test_dir("cov-eq")?;
        let rec = root.join("rec.mcap");
        // A live (unfinished) recording whose newest message sits EXACTLY at
        // the window end: `high_water >= end` must release the wait without
        // the ended flag and without burning the grace timeout.
        write_unfinished_recording(&rec, "/t", &[100, 1_000])?;

        let (tailer, coverage_rx) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        {
            let cov = coverage_rx.borrow();
            assert!(!cov.ended, "no footer was written");
            assert_eq!(cov.high_water_ns, 1_000);
        }

        let started = std::time::Instant::now();
        let stats = record_clip(
            0,
            1_000,
            root.join("clip.mcap"),
            tailer,
            coverage_rx,
            Duration::from_secs(30),
            Arc::new(Semaphore::new(1)),
        )
        .await?;

        assert!(
            started.elapsed() < Duration::from_secs(5),
            "high_water == end satisfies the wait (>=, not >)"
        );
        assert_eq!(stats.messages_copied, 2, "the boundary message is inside");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn concurrent_overlapping_triggers_serialize_and_take_distinct_paths()
    -> anyhow::Result<()> {
        let root = test_dir("overlap")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            false,
            &[("/t", 100), ("/t", 200), ("/t", 300), ("/t", 400)],
        )?;

        let (tailer, coverage_rx) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        // Two overlapping windows racing for the same out path and a single
        // extraction permit: the copies serialize FIFO, the second writer
        // lands on a `_1` sibling, and both clips come out complete.
        let permits = Arc::new(Semaphore::new(1));
        let out = root.join("clip.mcap");
        let (a, b) = tokio::join!(
            record_clip(
                100,
                300,
                out.clone(),
                tailer.clone(),
                coverage_rx.clone(),
                Duration::from_secs(30),
                permits.clone(),
            ),
            record_clip(
                200,
                400,
                out.clone(),
                tailer.clone(),
                coverage_rx.clone(),
                Duration::from_secs(30),
                permits.clone(),
            ),
        );
        let (a, b) = (a?, b?);

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
}
