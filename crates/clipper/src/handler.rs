//! The Handler: turn a decoded [`Trigger`] into durable clips, independent of
//! how the trigger arrived or how completion is announced.
//!
//! This half of the recorder knows nothing of ROS or wire encodings. It takes a
//! neutral [`Trigger`], waits out the postroll and coverage, stages one clip
//! segment per source recording over the collection tail ([`record_clip`]), and
//! reports the result through an [`Announce`] the interface supplies — a ROS
//! `Recorded` publish or an MCAP no-op. The clip extraction itself ([`crate::clip`])
//! is a raw-byte copy, untouched here.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crossbeam_channel::{Sender, bounded, unbounded};
use log::{info, warn};

use crate::clip;
use crate::supervision::panic_text;
use crate::tail::{Coverage, Tailer, WindowPlan};
use crate::trigger::{Announce, Completion, Trigger, now_ns};
use crate::watch::Watch;

/// One queued clip-segment staging: the window-plan snapshot the handler took
/// for one source recording, the window bounds for the message-time filter, the
/// base output path, and the reply channel. Queued by [`record_clip`]; dequeued
/// FIFO by the staging workers, which run the bulk copy into the capturing dir
/// and reply a [`clip::StagedClip`]. The handler publishes the staged segments
/// itself, once the window's segment count is known.
pub(crate) struct StageJob {
    plan: WindowPlan,
    start_ns: u64,
    end_ns: u64,
    out_path: PathBuf,
    reply: Sender<anyhow::Result<clip::StagedClip>>,
}

/// Spawn the fixed staging worker pool: `parallelism` threads consuming one
/// shared FIFO channel. With the default single worker the bulk copies serialize
/// in submission order — staging reads compete with the recorder's writes for
/// disk bandwidth (see the `--extract-parallelism` flag).
///
/// Each worker runs only [`clip::stage_clip`] — the bulk copy into the capturing
/// dir — and replies the [`clip::StagedClip`]; the handler publishes it once it
/// knows the window's segment count. The window plan rides in the job: the
/// handler snapshots it (one per source recording, pinning each file's
/// `Arc<File>`), so the worker never touches the tailer. The clip compression
/// codec is process-global, captured here. A panicking stage is caught and
/// replied as an error — per-job isolation, the pool outlives it.
pub(crate) fn spawn_stage_workers(
    parallelism: usize,
    compression: Option<mcap::Compression>,
) -> Sender<StageJob> {
    let (tx, rx) = unbounded::<StageJob>();
    for i in 0..parallelism.max(1) {
        let rx = rx.clone();
        thread::Builder::new()
            .name(format!("stage-{i}"))
            .spawn(move || {
                for job in rx.iter() {
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        clip::stage_clip(
                            &job.plan,
                            &job.out_path,
                            job.start_ns,
                            job.end_ns,
                            compression,
                        )
                    }))
                    .unwrap_or_else(|payload| {
                        Err(anyhow::anyhow!(
                            "staging panicked: {}",
                            panic_text(payload.as_ref())
                        ))
                    });
                    // A send failure means the handler is gone (its thread
                    // died); there is no one left to care about this clip.
                    let _ = job.reply.send(result);
                }
            })
            .expect("spawning staging worker");
    }
    tx
}

/// Run one trigger's wait-then-stage-then-announce flow. A window that stays in
/// one recording yields one clip; one that straddles a rollover yields one
/// segment per source file (`<base>_NN.mcap`). The single [`Completion`]
/// announces every segment in `filenames`.
///
/// Generic over the [`Announce`] the active interface supplies: the ROS
/// interface publishes a `Recorded`, the MCAP interface does nothing (the clip's
/// atomic move into `out_dir` is the signal). The handler is otherwise identical
/// either way — it knows only the neutral [`Trigger`]/[`Completion`] contract.
///
/// `out_dir` and `grace` are the only configuration this half reads; the driver
/// unpacks them at the seam so nothing below it depends on the CLI parser.
pub(crate) fn handle_trigger<A: Announce>(
    trig: Trigger,
    out_dir: &Path,
    grace: Duration,
    tailer: Arc<Tailer>,
    coverage: Arc<Watch<Coverage>>,
    extract_tx: Sender<StageJob>,
    announce: A,
) -> anyhow::Result<()> {
    let trigger_ns = trig.trigger_time.ns();
    let start_ns = trigger_ns.saturating_sub(trig.preroll);
    let end_ns = trigger_ns.saturating_add(trig.postroll);
    info!(
        "trigger name={:?} window=[{start_ns}, {end_ns}] preroll={} postroll={}",
        trig.name, trig.preroll, trig.postroll
    );

    let base_out_path = out_dir.join(format!("{trigger_ns}_{}.mcap", sanitize(&trig.name)));
    let segments = record_clip(
        &tailer,
        start_ns,
        end_ns,
        base_out_path,
        &coverage,
        grace,
        &extract_tx,
    )?;

    let mut filenames = Vec::with_capacity(segments.len());
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
        filenames.push(stats.out_path.to_string_lossy().into_owned());
    }
    if segments.len() > 1 {
        info!(
            "trigger name={:?} spanned a rollover into {} segments: {filenames:?}",
            trig.name,
            segments.len(),
        );
    }

    announce.announce(&Completion {
        name: trig.name,
        filenames,
        description: trig.description,
        trigger_time: trig.trigger_time,
        preroll: trig.preroll,
    });
    Ok(())
}

/// The decode-free, ROS-free core of [`handle_trigger`]: wait out the postroll
/// wall floor, wait for the tail's collection-wide coverage to reach the window
/// end (bounded by `grace`), then take one multi-file snapshot and stage one
/// segment per source recording.
///
/// A window inside one recording yields a single segment; one straddling a
/// rollover (a bag split or restart clipper indexed while running) yields one
/// segment per source file, recovered from the tail's retained collection.
/// Empty segments are dropped when the window produced real data elsewhere, but
/// one segment is always kept so an all-empty window (a rollover gap, all
/// relevant files pruned, or nothing recorded yet) still announces a valid clip.
/// Segments are named only once the count is known: a single segment keeps the
/// bare `<base>.mcap`, several get one `<base>_NN.mcap` per file. Every returned
/// [`clip::ClipStats`] names a durable file, so the caller may announce them all.
fn record_clip(
    tailer: &Arc<Tailer>,
    start_ns: u64,
    end_ns: u64,
    base_out_path: PathBuf,
    coverage: &Watch<Coverage>,
    grace: Duration,
    extract_tx: &Sender<StageJob>,
) -> anyhow::Result<Vec<clip::ClipStats>> {
    // 1. Postroll wall floor: never cut before the wall clock passes the window
    //    end. `checked_sub` reads the clock once per iteration, so a clock that
    //    crosses `end_ns` between the check and the sleep cannot underflow.
    while let Some(remaining) = end_ns.checked_sub(now_ns()).filter(|n| *n > 0) {
        thread::sleep(Duration::from_nanos(remaining));
    }

    // 2. Coverage: wait until the collection-wide high-water reaches the window
    //    end, bounded by `grace`. A window inside a recording is already covered
    //    (its high-water is past `end_ns` at the footer, or as soon as the scan
    //    reaches it); only a window whose end is past the last recorded message
    //    with no successor — a clean stop — waits out the full grace.
    if !coverage.wait_timeout_for(grace, |c| c.high_water_ns >= end_ns) {
        warn!(
            "window end {end_ns} still uncovered after {grace:?}; \
             cutting the clip from what is on disk"
        );
    }

    // 3. One multi-file snapshot — each plan pins its own recording's Arc<File>,
    //    so a retention prune or rollover after this cannot pull the bytes out.
    let plans = tailer.plan_window(start_ns, end_ns);

    // 4. Stage one segment per plan (FIFO worker pool), or one empty segment
    //    when no recording covers the window — the empty path needs no source
    //    file (a channelless MCAP is just magic + summary + footer).
    let mut staged: Vec<clip::StagedClip> = if plans.is_empty() {
        vec![stage_segment(
            extract_tx,
            WindowPlan::empty(),
            start_ns,
            end_ns,
            &base_out_path,
        )?]
    } else {
        let mut v = Vec::with_capacity(plans.len());
        for plan in plans {
            v.push(stage_segment(
                extract_tx,
                plan,
                start_ns,
                end_ns,
                &base_out_path,
            )?);
        }
        v
    };

    // 5. Drop empty segments when the window produced real data elsewhere, but
    //    keep one so an all-empty window still announces a valid clip.
    if staged.len() > 1 {
        if staged.iter().any(|c| !c.is_empty()) {
            staged.retain(|c| !c.is_empty());
        } else {
            staged.truncate(1);
        }
    }

    // 6. Publish the staged segments, naming them only now the count is known:
    //    one segment keeps the bare name, several get one `_NN` per source file.
    let n = staged.len();
    let mut stats = Vec::with_capacity(n);
    for (i, mut clip) in staged.into_iter().enumerate() {
        if n > 1 {
            clip.set_final_name(segment_name(&base_out_path, i));
        }
        stats.push(clip::publish_clip(clip)?);
    }
    Ok(stats)
}

/// Queue one segment's copy on the staging workers and block on the reply. The
/// plan is the handler's snapshot of one source recording, so a job that waits
/// in the FIFO queue still copies the recording it was taken from.
fn stage_segment(
    extract_tx: &Sender<StageJob>,
    plan: WindowPlan,
    start_ns: u64,
    end_ns: u64,
    out_path: &Path,
) -> anyhow::Result<clip::StagedClip> {
    let (reply_tx, reply_rx) = bounded(1);
    extract_tx
        .send(StageJob {
            plan,
            start_ns,
            end_ns,
            out_path: out_path.to_path_buf(),
            reply: reply_tx,
        })
        .map_err(|_| anyhow::anyhow!("the staging workers are gone"))?;
    reply_rx
        .recv()
        .map_err(|_| anyhow::anyhow!("the staging worker dropped the job"))?
}

/// `<base>` with a zero-padded `_NN` segment index inserted before the
/// extension (`clip.mcap` → `clip_00.mcap`), for a window that spanned a
/// rollover and writes one segment per source file.
fn segment_name(base: &Path, idx: usize) -> std::ffi::OsString {
    let stem = base.file_stem().unwrap_or_default().to_string_lossy();
    let ext = base
        .extension()
        .map(|e| format!(".{}", e.to_string_lossy()))
        .unwrap_or_default();
    std::ffi::OsString::from(format!("{stem}_{idx:02}{ext}"))
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
    use crate::clip::tests::read_clip;
    use crate::tail::tests::{
        drain, scan_to_end, test_dir, write_recording, write_unfinished_recording,
    };

    /// The clip compression the recorder's default (zstd) maps to; the unit
    /// tests drive the extraction worker pool through the same codec the
    /// recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    #[test]
    fn sanitize_replaces_separators_and_whitespace() {
        // The slash replacement is the safety property: a trigger name can
        // never introduce a path component into <trigger_ns>_<name>.mcap.
        assert_eq!(sanitize("a/b c"), "a_b_c");
        assert_eq!(sanitize("../escape"), ".._escape");
        assert_eq!(sanitize(""), "unnamed");
    }

    #[test]
    fn record_clip_grace_timeout_cuts_what_is_on_disk() -> anyhow::Result<()> {
        let root = test_dir("grace")?;
        let (tailer, coverage) = Tailer::new();
        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);

        // The window end is far in the past on the wall clock (no postroll
        // sleep), but coverage never reaches it — no recording was ever
        // discovered. The grace timeout must fire and cut a valid empty clip
        // instead of hanging or erroring.
        let stats = record_clip(
            &tailer,
            0,
            1_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_millis(50),
            &extract_tx,
        )?;

        assert_eq!(stats.len(), 1, "no recording yields a single empty segment");
        assert_eq!(stats[0].messages_copied, 0);
        assert!(read_clip(&stats[0].out_path)?.is_empty());

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

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let stats = record_clip(
            &tailer,
            100,
            1_000,
            root.join("clip.mcap"),
            &coverage,
            Duration::from_secs(10),
            &extract_tx,
        )?;

        assert_eq!(stats.len(), 1);
        assert_eq!(stats[0].messages_copied, 2);
        assert_eq!(
            read_clip(&stats[0].out_path)?,
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

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let end_ns = now + 150_000_000; // 150 ms past the trigger stamp
        let started = std::time::Instant::now();
        let stats = record_clip(
            &tailer,
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
        assert_eq!(stats.len(), 1);
        assert_eq!(stats[0].messages_copied, 1, "the future message is outside");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_clip_cuts_a_stopped_recorder_on_grace() -> anyhow::Result<()> {
        let root = test_dir("ended")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        // A stopped recorder (footer on disk) whose high-water (200) stays below
        // the window end: there is no ended short-circuit, so the coverage wait
        // runs out the (short) grace and then cuts what is on disk. The grace is
        // the only bound — the postroll floor is already in the past here.
        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let grace = Duration::from_millis(200);
        let started = std::time::Instant::now();
        let stats = record_clip(
            &tailer,
            50,
            1_000_000,
            root.join("clip.mcap"),
            &coverage,
            grace,
            &extract_tx,
        )?;

        assert!(
            started.elapsed() >= grace,
            "an uncovered window end waits out the grace before cutting"
        );
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "the grace is the bound — it does not hang"
        );
        assert_eq!(stats.len(), 1);
        assert_eq!(stats[0].messages_copied, 2);

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
        assert_eq!(coverage.get().high_water_ns, 1_000);

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let started = std::time::Instant::now();
        let stats = record_clip(
            &tailer,
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
        assert_eq!(stats.len(), 1);
        assert_eq!(
            stats[0].messages_copied, 2,
            "the boundary message is inside"
        );

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
        // staging worker: the copies serialize FIFO, the second writer lands on
        // a `_1` sibling at publish, and both clips come out complete. Neither
        // window straddles a rollover, so each is a single segment.
        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let cut = |start_ns: u64, end_ns: u64| {
            let tailer = tailer.clone();
            let coverage = coverage.clone();
            let extract_tx = extract_tx.clone();
            let out = out.clone();
            std::thread::spawn(move || {
                record_clip(
                    &tailer,
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
        assert_eq!((a.len(), b.len()), (1, 1), "each window is one segment");
        let (a, b) = (&a[0], &b[0]);

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

    /// Two segments staged through the worker pool for the same base name
    /// publish to distinct files: the first claims the bare name, the second
    /// resolves to the `_1` sibling. The staging copies run FIFO on the worker
    /// channel; the name collision is settled at publish, on the handler thread.
    #[test]
    fn staged_segments_publish_to_distinct_paths() -> anyhow::Result<()> {
        let root = test_dir("fifo")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let (tailer, _coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let plan = || {
            tailer
                .plan_window(0, 300)
                .into_iter()
                .next()
                .expect("the recording covers the window")
        };
        let first = stage_segment(&extract_tx, plan(), 0, 300, &out)?;
        let second = stage_segment(&extract_tx, plan(), 0, 300, &out)?;

        let a = clip::publish_clip(first)?;
        let b = clip::publish_clip(second)?;
        assert_eq!(a.out_path, out, "the first published claims the name");
        assert_eq!(
            b.out_path,
            root.join("clip_1.mcap"),
            "the second resolves against the taken name"
        );
        assert_eq!(a.messages_copied, 2);
        assert_eq!(b.messages_copied, 2);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_clip_recovers_across_a_rollover_into_two_segments() -> anyhow::Result<()> {
        // Two finished recordings clipper indexed while running (a split): one
        // `record_clip` over a window straddling the boundary stages one segment
        // per source file, published as `<base>_00.mcap` and `<base>_01.mcap`,
        // each a complete clip.
        let root = test_dir("two-seg")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let (tailer, coverage) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        crate::tail::tests::drain(&tailer)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = record_clip(
            &tailer,
            1_500,
            5_500,
            base,
            &coverage,
            Duration::from_secs(30),
            &extract_tx,
        )?;

        assert_eq!(stats.len(), 2, "a straddling window yields two segments");
        let mut paths: Vec<_> = stats.iter().map(|s| s.out_path.clone()).collect();
        paths.sort();
        assert_eq!(
            paths,
            vec![root.join("clip_00.mcap"), root.join("clip_01.mcap")]
        );
        // The segments tile the window: split0's tail, then split1's head.
        assert_eq!(
            read_clip(&root.join("clip_00.mcap"))?,
            vec![("/t".to_string(), 2_000)]
        );
        assert_eq!(
            read_clip(&root.join("clip_01.mcap"))?,
            vec![("/t".to_string(), 5_000)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A capturing [`Announce`] that records every completion it is handed, so a
    /// test can assert what `handle_trigger` announced.
    #[derive(Clone)]
    struct CapturingAnnouncer(Arc<std::sync::Mutex<Vec<Completion>>>);
    impl Announce for CapturingAnnouncer {
        fn announce(&self, completion: &Completion) {
            self.0.lock().unwrap().push(completion.clone());
        }
    }

    /// End to end through the public handler entry point: `handle_trigger` turns a
    /// neutral [`Trigger`] into a clip on disk and announces one [`Completion`]
    /// naming it. Exercises the whole flow the interfaces share — window math,
    /// staging, naming, and the announce hand-off — independent of ROS/encoding.
    #[test]
    fn handle_trigger_cuts_a_clip_and_announces_it() -> anyhow::Result<()> {
        let root = test_dir("handle-trigger")?;
        let out_dir = root.join("out");
        let rec = root.join("rec.mcap");
        // Two messages inside the window [0, 1000], one past it so coverage is
        // already satisfied and the cut does not wait out the grace.
        write_recording(&rec, false, &[("/t", 100), ("/t", 900), ("/t", 2_000)])?;

        clip::reset_capturing_dir(&out_dir)?;
        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);

        let captured = Arc::new(std::sync::Mutex::new(Vec::new()));
        let announcer = CapturingAnnouncer(captured.clone());

        let trig = Trigger {
            name: "evt".to_string(),
            description: "hi".to_string(),
            trigger_time: crate::trigger::Stamp {
                sec: 0,
                nanosec: 500,
            },
            preroll: 500,
            postroll: 500,
        };
        handle_trigger(
            trig,
            &out_dir,
            Duration::from_secs(5),
            tailer,
            coverage,
            extract_tx,
            announcer,
        )?;

        let done = captured.lock().unwrap();
        assert_eq!(done.len(), 1, "exactly one completion is announced");
        assert_eq!(done[0].name, "evt");
        assert_eq!(
            done[0].filenames.len(),
            1,
            "one segment — window in one file"
        );
        let clip_path = std::path::Path::new(&done[0].filenames[0]);
        assert!(clip_path.is_file(), "the announced clip exists on disk");
        assert_eq!(
            read_clip(clip_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 900)],
            "the clip holds exactly the in-window messages"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `handle_trigger` with a window spanning two source recordings logs the
    /// rollover info line (lines 144-149) and announces a `Completion` carrying
    /// both segment filenames. Models the same two-file setup as
    /// `record_clip_recovers_across_a_rollover_into_two_segments` but drives
    /// the full `handle_trigger` entry point so the per-segment `info!` loop
    /// and the `segments.len() > 1` branch are both exercised.
    #[test]
    fn handle_trigger_announces_two_segments_across_a_rollover() -> anyhow::Result<()> {
        let root = test_dir("ht-rollover")?;
        let out_dir = root.join("out");
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // Two finished recordings: split0 ends at 2_000, split1 starts at 5_000.
        // The trigger window [1_500, 5_500] straddles the gap.
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        clip::reset_capturing_dir(&out_dir)?;
        let (tailer, coverage) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let captured = Arc::new(std::sync::Mutex::new(Vec::new()));
        let announcer = CapturingAnnouncer(captured.clone());

        // trigger_time = 3_500 ns, preroll = 2_000 ns, postroll = 2_000 ns
        // → window [1_500, 5_500]
        let trig = Trigger {
            name: "rollover".to_string(),
            description: String::new(),
            trigger_time: crate::trigger::Stamp {
                sec: 0,
                nanosec: 3_500,
            },
            preroll: 2_000,
            postroll: 2_000,
        };
        handle_trigger(
            trig,
            &out_dir,
            Duration::from_secs(5),
            tailer,
            coverage,
            extract_tx,
            announcer,
        )?;

        let done = captured.lock().unwrap();
        assert_eq!(done.len(), 1, "one completion per trigger");
        assert_eq!(done[0].name, "rollover");
        assert_eq!(
            done[0].filenames.len(),
            2,
            "a rollover window must announce two segments"
        );
        // Both announced paths must exist on disk and be valid clips.
        for name in &done[0].filenames {
            let p = std::path::Path::new(name);
            assert!(p.is_file(), "announced segment {name} must be on disk");
            assert!(
                !read_clip(p)?.is_empty(),
                "each segment must hold at least one message"
            );
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `record_clip` with two source recordings where BOTH segments are empty
    /// keeps exactly one of them (lines 238-240): truncating to 1 rather than
    /// dropping all so an all-empty window still announces a valid clip.
    #[test]
    fn record_clip_all_empty_multi_segment_keeps_one() -> anyhow::Result<()> {
        let root = test_dir("all-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // Two recordings whose messages all fall far outside the narrow window
        // [500, 600]: both staged segments will be empty (messages_copied == 0).
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let (tailer, coverage) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = record_clip(
            &tailer,
            500,
            600,
            base.clone(),
            &coverage,
            Duration::from_secs(30),
            &extract_tx,
        )?;

        // Both segments are empty, so truncate(1) keeps exactly one.
        assert_eq!(
            stats.len(),
            1,
            "an all-empty multi-segment window keeps exactly one segment"
        );
        assert_eq!(stats[0].messages_copied, 0, "the kept segment is empty");
        assert!(read_clip(&stats[0].out_path)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `record_clip` with two source recordings where only one segment carries
    /// data drops the empty segment (lines 236-237): the `retain(!is_empty)`
    /// branch fires so the announced clip list contains only the non-empty one.
    #[test]
    fn record_clip_drops_empty_segment_when_other_has_data() -> anyhow::Result<()> {
        let root = test_dir("drop-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // split0 has messages inside the window [1_500, 5_500]; split1 does not.
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 8_000), ("/t", 9_000)])?;

        let (tailer, coverage) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        let extract_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = record_clip(
            &tailer,
            1_500,
            5_500,
            base.clone(),
            &coverage,
            Duration::from_secs(30),
            &extract_tx,
        )?;

        // split0 contributes message at 2_000; split1's messages are outside.
        // The empty split1 segment is dropped; only the data-carrying segment remains.
        assert_eq!(
            stats.len(),
            1,
            "the empty trailing segment is dropped when another carries data"
        );
        assert_eq!(stats[0].messages_copied, 1);
        assert_eq!(
            read_clip(&stats[0].out_path)?,
            vec![("/t".to_string(), 2_000)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
