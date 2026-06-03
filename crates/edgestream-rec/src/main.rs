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

    // Wait out the postroll, rendezvous with the closing split, and copy the
    // window. Each trigger runs this on its own task, so overlapping windows are
    // cut concurrently against the shared record dir and split channel.
    let out_path = cfg
        .out_dir
        .join(format!("{trigger_ns}_{}.mcap", sanitize(&trig.name)));
    let stats = record_clip(
        start_ns,
        end_ns,
        out_path.clone(),
        cfg.record_dir.clone(),
        &mut split_rx,
    )
    .await?;
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

/// The decode-free, ROS-free core of [`handle_trigger`]: wait out the postroll,
/// wait for the `write_split` that finalises the window's tail, then bulk-copy
/// the clip. It owns the postroll sleep, the split rendezvous and the blocking
/// extraction, but neither the `Trigger`/`Recorded` types nor the publisher —
/// so overlapping windows (each on its own task, sharing `record_dir` and one
/// split broadcast) can be exercised without a live ROS graph.
async fn record_clip(
    start_ns: u64,
    end_ns: u64,
    out_path: PathBuf,
    record_dir: PathBuf,
    split_rx: &mut broadcast::Receiver<SplitEvent>,
) -> anyhow::Result<mcap_copy::ClipStats> {
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

    // 3. Bulk-copy the window into the clip. The copy is blocking file IO, so it
    //    runs off the async runtime.
    tokio::task::spawn_blocking(move || {
        mcap_copy::extract_clip(&record_dir, &out_path, start_ns, end_ns, Some(&closed_through))
    })
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

    // --- Overlapping triggers ------------------------------------------------
    //
    // These drive `record_clip` directly (the ROS-free core), spawning one task
    // per trigger against a shared record dir and a single split broadcast — the
    // same wiring as the live consumer loop. They exercise the postroll sleep, so
    // they run on real (short) wall-clock time. Window bounds and message stamps
    // are absolute `now`-relative nanoseconds; assertions compare what each clip
    // captured against what its window should hold, independent of the clock.

    use std::collections::BTreeMap;
    use std::fs::File;
    use std::io::BufWriter;
    use std::path::Path;

    use memmap2::Mmap;
    use tokio::sync::mpsc;

    const MS: u64 = 1_000_000;

    #[tokio::test]
    async fn overlapping_triggers_overtake_each_write_correct_clips() -> anyhow::Result<()> {
        // trigger2 is "published" after trigger1 yet its window is fully nested in
        // trigger1's postroll: start1 < start2 and end2 < end1. Because each
        // trigger sleeps independently out to its own window end, trigger2
        // *overtakes* — it finishes first — and both clips must still hold exactly
        // their own window's messages.
        let root = temp_root("overtake")?;
        let record_dir = root.join("record");
        let out_dir = root.join("triggered");
        std::fs::create_dir_all(&record_dir)?;

        let now = now_ns();
        let (start1, end1) = (now, now + 500 * MS); // outer window
        let (start2, end2) = (now + 20 * MS, now + 150 * MS); // nested window
        assert!(
            start1 < start2 && end2 < end1,
            "window 2 must nest inside window 1"
        );

        // Two splits: rec_0 holds the earlier half, rec_1 the later half, so each
        // clip has to read across both files. Stamps are placed on/inside/outside
        // the two windows.
        let early = record_dir.join("rec_0.mcap");
        let late = record_dir.join("rec_1.mcap");
        write_split_file(
            &early,
            "/topic",
            &[
                now.saturating_sub(20 * MS), // before both
                now,                         // start1 only
                now + 20 * MS,               // start2: both
                now + 80 * MS,               // both
            ],
        )?;
        write_split_file(
            &late,
            "/topic",
            &[
                now + 150 * MS, // end2: both
                now + 300 * MS, // after end2: window 1 only
                now + 500 * MS, // end1: window 1 only
                now + 700 * MS, // after both
            ],
        )?;
        set_split_mtime(&early, UNIX_EPOCH + Duration::from_secs(1_000))?;
        set_split_mtime(&late, UNIX_EPOCH + Duration::from_secs(2_000))?;

        // One broadcast feeds both handlers; both subscribe before any send, just
        // as the live loop does.
        let (split_tx, _) = broadcast::channel::<SplitEvent>(64);
        let mut rx1 = split_tx.subscribe();
        let mut rx2 = split_tx.subscribe();

        // Completion order is observed here: whoever finishes first sends first.
        let (done_tx, mut done_rx) = mpsc::unbounded_channel::<&'static str>();

        let h1 = {
            let out = out_dir.join("clip1.mcap");
            let rec = record_dir.clone();
            let done = done_tx.clone();
            tokio::spawn(async move {
                let stats = record_clip(start1, end1, out, rec, &mut rx1).await?;
                done.send("clip1").ok();
                anyhow::Ok(stats)
            })
        };
        let h2 = {
            let out = out_dir.join("clip2.mcap");
            let rec = record_dir.clone();
            let done = done_tx.clone();
            tokio::spawn(async move {
                let stats = record_clip(start2, end2, out, rec, &mut rx2).await?;
                done.send("clip2").ok();
                anyhow::Ok(stats)
            })
        };
        drop(done_tx);

        // The split that closes both windows' tails: observed well after end1,
        // naming the newest split. Buffered now; each handler consumes it once its
        // own postroll sleep elapses.
        split_tx.send(SplitEvent {
            observed_ns: end1 + 50 * MS,
            closed_file: late.clone(),
        })?;

        let s1 = h1.await??;
        let s2 = h2.await??;

        // The overtake: the nested, earlier-ending trigger2 reported done first.
        assert_eq!(
            done_rx.recv().await,
            Some("clip2"),
            "trigger2 should overtake trigger1"
        );

        // Both clips hold exactly their window's messages — neither leaks the
        // other's.
        assert_eq!(
            sorted_clip_times(&out_dir.join("clip1.mcap"))?,
            vec![
                now,
                now + 20 * MS,
                now + 80 * MS,
                now + 150 * MS,
                now + 300 * MS,
                now + 500 * MS,
            ],
        );
        assert_eq!(s1.messages_copied, 6);

        assert_eq!(
            sorted_clip_times(&out_dir.join("clip2.mcap"))?,
            vec![now + 20 * MS, now + 80 * MS, now + 150 * MS],
        );
        assert_eq!(s2.messages_copied, 3);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[tokio::test]
    async fn overlapping_triggers_partial_overlap_each_write_correct_clips() -> anyhow::Result<()> {
        // Staggered windows that overlap without nesting: start1 < start2 < end1 <
        // end2. The shared region [start2, end1] lands in both clips; the
        // exclusive tails land in only one each.
        let root = temp_root("partial")?;
        let record_dir = root.join("record");
        let out_dir = root.join("triggered");
        std::fs::create_dir_all(&record_dir)?;

        let now = now_ns();
        let (start1, end1) = (now, now + 200 * MS);
        let (start2, end2) = (now + 100 * MS, now + 500 * MS);
        assert!(
            start1 < start2 && start2 < end1 && end1 < end2,
            "windows must overlap without either nesting"
        );

        let early = record_dir.join("rec_0.mcap");
        let late = record_dir.join("rec_1.mcap");
        write_split_file(
            &early,
            "/topic",
            &[
                now.saturating_sub(20 * MS), // before both
                now,                         // window 1 only (< start2)
                now + 50 * MS,               // window 1 only
                now + 100 * MS,              // start2: shared region
                now + 150 * MS,              // shared region
            ],
        )?;
        write_split_file(
            &late,
            "/topic",
            &[
                now + 200 * MS, // end1: shared region (last stamp in both)
                now + 300 * MS, // window 2 only (> end1)
                now + 500 * MS, // end2: window 2 only
                now + 700 * MS, // after both
            ],
        )?;
        set_split_mtime(&early, UNIX_EPOCH + Duration::from_secs(1_000))?;
        set_split_mtime(&late, UNIX_EPOCH + Duration::from_secs(2_000))?;

        let (split_tx, _) = broadcast::channel::<SplitEvent>(64);
        let mut rx1 = split_tx.subscribe();
        let mut rx2 = split_tx.subscribe();

        let h1 = {
            let out = out_dir.join("clip1.mcap");
            let rec = record_dir.clone();
            tokio::spawn(async move { record_clip(start1, end1, out, rec, &mut rx1).await })
        };
        let h2 = {
            let out = out_dir.join("clip2.mcap");
            let rec = record_dir.clone();
            tokio::spawn(async move { record_clip(start2, end2, out, rec, &mut rx2).await })
        };

        // Observed after the later window end so both handlers accept it.
        split_tx.send(SplitEvent {
            observed_ns: end2 + 50 * MS,
            closed_file: late.clone(),
        })?;

        let s1 = h1.await??;
        let s2 = h2.await??;

        assert_eq!(
            sorted_clip_times(&out_dir.join("clip1.mcap"))?,
            vec![
                now,
                now + 50 * MS,
                now + 100 * MS,
                now + 150 * MS,
                now + 200 * MS,
            ],
        );
        assert_eq!(s1.messages_copied, 5);

        assert_eq!(
            sorted_clip_times(&out_dir.join("clip2.mcap"))?,
            vec![
                now + 100 * MS,
                now + 150 * MS,
                now + 200 * MS,
                now + 300 * MS,
                now + 500 * MS,
            ],
        );
        assert_eq!(s2.messages_copied, 5);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    fn write_split_file(path: &Path, topic: &str, log_times: &[u64]) -> anyhow::Result<()> {
        let mut writer = mcap::Writer::new(BufWriter::new(File::create(path)?))?;
        let schema_id = writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data")?;
        let channel_id = writer.add_channel(schema_id, topic, "cdr", &BTreeMap::new())?;
        for (seq, &log_time) in log_times.iter().enumerate() {
            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence: seq as u32,
                    log_time,
                    publish_time: log_time,
                },
                b"payload",
            )?;
        }
        writer.finish()?;
        Ok(())
    }

    fn sorted_clip_times(path: &Path) -> anyhow::Result<Vec<u64>> {
        let file = File::open(path)?;
        let mmap = unsafe { Mmap::map(&file)? };
        let mut times = mcap::MessageStream::new(&mmap)?
            .map(|msg| Ok(msg?.log_time))
            .collect::<anyhow::Result<Vec<u64>>>()?;
        times.sort_unstable();
        Ok(times)
    }

    fn set_split_mtime(path: &Path, when: SystemTime) -> anyhow::Result<()> {
        File::options().write(true).open(path)?.set_modified(when)?;
        Ok(())
    }

    fn temp_root(name: &str) -> anyhow::Result<PathBuf> {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
        let path = std::env::temp_dir().join(format!(
            "edgestream-rec-overlap-{name}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&path)?;
        Ok(path)
    }
}
