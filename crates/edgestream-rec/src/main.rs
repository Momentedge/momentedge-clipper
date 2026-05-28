//! Triggered clip recorder.
//!
//! A continuous `ros2 bag record` (started separately — see the README) writes
//! 5 s MCAP splits into `./record`. This binary watches two ROS2 topics and,
//! on demand, cuts a clip out of those splits:
//!
//!   * `/events/edgestream/trigger` (`edgestream_msgs/Trigger`) — a request to
//!     keep the window `[trigger_time - preroll, trigger_time + postroll]`.
//!   * `/events/write_split` (`rosbag2_interfaces/WriteSplitEvent`) — rosbag2's
//!     signal that it has finalised a split file.
//!
//! For each trigger the flow is: wait until wall-clock time passes
//! `trigger_time + postroll`, then wait for the next `write_split` (so the split
//! covering the end of the window is closed and complete on disk), then
//! bulk-copy every in-window message into `./triggered/<trigger_ns>_<name>.mcap`
//! (see [`mcap_copy`] — a raw-bytes copy, no CDR decode), and finally publish
//! `/events/edgestream/recorded` (`edgestream_msgs/Recorded`).
//!
//! Time base: rosbag2 `log_time`, the trigger stamp, and the wait clock are all
//! treated as nanoseconds on the system (ROS) clock — this assumes the default
//! (no `use_sim_time`). Each trigger is handled in its own task, so overlapping
//! windows are cut concurrently.
//!
//! Flags (all optional):
//!   --record-dir <dir>   continuous rosbag2 splits to read (default ./record)
//!   --out-dir <dir>      where clips are written          (default ./triggered)
//!
//! Logging uses the `log` facade with a pretty_env_logger backend; `RUST_LOG`
//! controls verbosity (defaults to `info`; `debug` logs every write_split).

mod mcap_copy;

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use futures::stream::StreamExt;
use log::{debug, error, info};
use r2r::{Publisher, QosProfile};
use tokio::sync::broadcast;

const TRIGGER_TOPIC: &str = "/events/edgestream/trigger";
const SPLIT_TOPIC: &str = "/events/write_split";
const RECORDED_TOPIC: &str = "/events/edgestream/recorded";

#[derive(Clone, Debug)]
struct SplitEvent {
    observed_ns: u64,
    closed_file: PathBuf,
}

struct Config {
    record_dir: PathBuf,
    out_dir: PathBuf,
}

fn parse_args() -> Config {
    let mut cfg = Config {
        record_dir: PathBuf::from("./record"),
        out_dir: PathBuf::from("./triggered"),
    };
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().expect("flag needs a value");
        match flag.as_str() {
            "--record-dir" => cfg.record_dir = PathBuf::from(value()),
            "--out-dir" => cfg.out_dir = PathBuf::from(value()),
            other => panic!("unknown flag: {other}"),
        }
    }
    cfg
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let cfg = Arc::new(parse_args());

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "edgestream_recorder", "")?;

    // Typed endpoints. The streams are fed by the spin loop below; we move each
    // into its own consumer task. The publisher is Clone and is shared into the
    // per-trigger handlers.
    let mut trigger_sub =
        node.subscribe::<r2r::edgestream_msgs::msg::Trigger>(TRIGGER_TOPIC, QosProfile::default())?;
    let mut split_sub = node.subscribe::<r2r::rosbag2_interfaces::msg::WriteSplitEvent>(
        SPLIT_TOPIC,
        QosProfile::default(),
    )?;
    let recorded_pub = node.create_publisher::<r2r::edgestream_msgs::msg::Recorded>(
        RECORDED_TOPIC,
        QosProfile::default(),
    )?;

    // Fan write_split events out with their observation time (ns since epoch)
    // and the closed split path. A trigger handler subscribes before it starts
    // waiting, then consumes events until it sees one at/after its window end.
    let (split_tx, _) = broadcast::channel::<SplitEvent>(64);

    // write_split consumer.
    {
        let split_tx = split_tx.clone();
        tokio::spawn(async move {
            while let Some(ev) = split_sub.next().await {
                debug!(
                    "write_split: closed={} opened={}",
                    ev.closed_file, ev.opened_file
                );
                let _ = split_tx.send(SplitEvent {
                    observed_ns: now_ns(),
                    closed_file: PathBuf::from(ev.closed_file),
                });
            }
        });
    }

    // trigger consumer: spawn one handler per trigger.
    {
        let cfg = cfg.clone();
        tokio::spawn(async move {
            while let Some(trig) = trigger_sub.next().await {
                let cfg = cfg.clone();
                let recorded_pub = recorded_pub.clone();
                // Subscribe to splits now, so events arriving during the wait
                // are captured rather than missed.
                let split_rx = split_tx.subscribe();
                tokio::spawn(async move {
                    if let Err(e) = handle_trigger(trig, cfg, recorded_pub, split_rx).await {
                        error!("trigger handling failed: {e:#}");
                    }
                });
            }
        });
    }

    info!(
        "edgestream-rec up: triggers on {TRIGGER_TOPIC}, splits on {SPLIT_TOPIC}, \
         reading {}, writing clips to {}",
        cfg.record_dir.display(),
        cfg.out_dir.display(),
    );

    // The node's single owner: spin continuously to feed the streams.
    let worker = tokio::task::spawn_blocking(move || {
        loop {
            node.spin_once(Duration::from_millis(10));
        }
    });
    worker.await?;
    Ok(())
}

/// Run one trigger's wait-then-extract-then-announce flow.
async fn handle_trigger(
    trig: r2r::edgestream_msgs::msg::Trigger,
    cfg: Arc<Config>,
    recorded_pub: Publisher<r2r::edgestream_msgs::msg::Recorded>,
    mut split_rx: broadcast::Receiver<SplitEvent>,
) -> anyhow::Result<()> {
    let trigger_ns = time_to_ns(&trig.trigger_time);
    let start_ns = trigger_ns.saturating_sub(trig.preroll);
    let end_ns = trigger_ns.saturating_add(trig.postroll);
    info!(
        "trigger name={:?} window=[{start_ns}, {end_ns}] preroll={} postroll={}",
        trig.name, trig.preroll, trig.postroll
    );

    // 1. Wait out the postroll: hold until the wall clock passes the window end.
    let now = now_ns();
    if end_ns > now {
        tokio::time::sleep(Duration::from_nanos(end_ns - now)).await;
    }

    // 2. Wait for the first split finalised at/after the window end, so the file
    //    holding the tail of the window is closed and safe to read.
    let closed_through = loop {
        match split_rx.recv().await {
            Ok(ev) if ev.observed_ns >= end_ns => break ev.closed_file,
            Ok(_) => continue,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                debug!("split events lagged by {n}; continuing");
            }
            Err(broadcast::error::RecvError::Closed) => {
                anyhow::bail!("split event channel closed before a post-window split");
            }
        }
    };

    // 3. Bulk-copy the window into ./triggered/<trigger_ns>_<name>.mcap. The copy
    //    is blocking file IO, so it runs off the async runtime.
    let out_path = cfg
        .out_dir
        .join(format!("{trigger_ns}_{}.mcap", sanitize(&trig.name)));
    let stats = {
        let record_dir = cfg.record_dir.clone();
        let out_path = out_path.clone();
        let closed_through = closed_through.clone();
        tokio::task::spawn_blocking(move || {
            mcap_copy::extract_clip(
                &record_dir,
                &out_path,
                start_ns,
                end_ns,
                Some(&closed_through),
            )
        })
        .await??
    };
    info!(
        "clip {} written: {} msgs from {}/{} splits, {:.1} MiB",
        out_path.display(),
        stats.messages_copied,
        stats.files_used,
        stats.files_scanned,
        stats.bytes_copied as f64 / 1_048_576.0,
    );

    // 4. Announce the clip.
    let filename = out_path.to_string_lossy().into_owned();
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

/// Nanoseconds since the Unix epoch on the system clock.
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as u64)
        .unwrap_or(0)
}

/// Flatten a `builtin_interfaces/Time` to nanoseconds since the epoch.
fn time_to_ns(t: &r2r::builtin_interfaces::msg::Time) -> u64 {
    t.sec.max(0) as u64 * 1_000_000_000 + t.nanosec as u64
}

/// Make a trigger name safe to embed in a filename: keep alphanumerics, `-`, `_`
/// and `.`; everything else (notably `/`) becomes `_`.
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
    fn sanitize_keeps_filename_safe_characters() {
        assert_eq!(sanitize("clip-1_v.2"), "clip-1_v.2");
    }

    #[test]
    fn sanitize_replaces_separators_and_whitespace() {
        // The slash replacement is the safety property: a trigger name can never
        // introduce a path component into <trigger_ns>_<name>.mcap.
        assert_eq!(sanitize("a/b c"), "a_b_c");
        assert_eq!(sanitize("../escape"), ".._escape");
    }

    #[test]
    fn sanitize_falls_back_for_empty_names() {
        assert_eq!(sanitize(""), "unnamed");
        assert_eq!(sanitize("/"), "_");
    }

    #[test]
    fn time_to_ns_flattens_seconds_and_nanos() {
        let t = r2r::builtin_interfaces::msg::Time {
            sec: 2,
            nanosec: 500,
        };
        assert_eq!(time_to_ns(&t), 2_000_000_500);
    }

    #[test]
    fn time_to_ns_clamps_negative_seconds_to_zero() {
        let t = r2r::builtin_interfaces::msg::Time {
            sec: -5,
            nanosec: 250,
        };
        assert_eq!(time_to_ns(&t), 250);
    }
}
