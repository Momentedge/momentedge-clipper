//! Turning a decoded [`Trigger`] into durable clips, independent of how the
//! trigger arrived or how completion is announced.
//!
//! [`handle_trigger`] knows nothing of ROS or wire encodings. It takes a neutral
//! [`Trigger`], waits out the postroll and the tail's coverage, hands the window
//! to [`clip::segment::cut_window`] — which claims the clip's directory, copies
//! one file per source recording into it and completes it — and reports the
//! result through an [`Announce`] its caller supplies: a ROS `Recorded` publish,
//! or nothing at all when the clip directory's own metadata file is the signal.
//!
//! **A window whose clip is already there is skipped**, and that is not an
//! error: a second trigger for one window, or a restart meeting its own earlier
//! clips, writes nothing and announces nothing. The cut says which happened
//! ([`clip::segment::CutOutcome`]), so this cannot announce a clip it did not
//! cut.
//!
//! **The two waits are the whole of what this module is.** Everything past them
//! — planning the window, staging, dropping empty segments, publishing — is
//! [`clip::segment`], the same code that cuts from a recording nobody is
//! writing. Waiting is what makes this the *live* path: a window may reach past
//! the last byte on disk, so a cut blocks until the wall clock passes the window
//! end and the tail's coverage catches up (bounded by the caller's grace,
//! `--grace-secs` in the recorder) before there is anything worth cutting.
//!
//! One consequence of being the live path shows up on the way out rather than on
//! the way in: the same recording is cut from again and again, so a fault that
//! belongs to the *recording* rather than to the window repeats for every
//! trigger. `report_refusal` is where that repetition is turned into a signal
//! — announced in full the first time it costs a clip, counted every time after
//! — against the [`CutFaults`] tally the recorder shares between handlers.

use std::path::Path;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use clip::TimeSource;
use clip::cut::FramingDesync;
use clip::index::EXTENT_CAP_BYTES;
use clip::manifest::{CutRequest, Producer, WindowCoverage};
use clip::segment::{self, CutOutcome, StageJob};
use clip::trigger::{Announce, Completion, Trigger, now_ns};
use crossbeam_channel::Sender;
use log::{error, info, warn};

use crate::faults::CutFaults;
use crate::tailer::Tailer;

/// Run one trigger's wait-then-cut-then-announce flow. The clip is one directory
/// under `out_dir` holding one `<id>_N.mcap` per contributing source recording;
/// the single [`Completion`] names that directory in `filenames`. A window whose
/// directory is already there is skipped, and announces nothing.
///
/// Generic over the [`Announce`] the active interface supplies: the ROS
/// interface publishes a `Recorded`, the MCAP interface does nothing (the clip
/// directory's metadata file is the signal). The handler is otherwise identical
/// either way — it knows only the neutral [`Trigger`]/[`Completion`] contract.
///
/// `out_dir`, `grace`, and `time_source` are the only configuration this half
/// reads; the driver unpacks them at the seam so nothing below it depends on the
/// CLI parser. The window centres on `anchor_ns`, resolved by the interface per
/// the interface × time-source anchor matrix (the ROS interface from the
/// subscription instant under `log` or `trigger_time` under `publish`, the MCAP
/// interface from the trigger record's own stamp on the active `time_source`);
/// the same instant leads the clip's id, which names its directory. `time_source` is the clock domain the window lives in — it
/// selects which extents are read, which messages fall inside, and which
/// coverage the wait blocks on. `producer` is the binary and mode each clip's
/// document names as having cut it; the driver supplies it because only the
/// binary knows which of its subcommands is running. `faults` is the
/// recorder-wide tally of clips refused because a recording's bytes changed
/// under the tail, shared by every handler so `report_refusal` announces that
/// once per recording rather than once per trigger.
#[expect(
    clippy::too_many_arguments,
    reason = "the arguments are the recorder's cohesive per-trigger inputs — the \
              resolved anchor, the neutral trigger, the shared tail and staging \
              handles, the announcer, and the settings the seam unpacks. Bundling \
              them into a struct purely to satisfy the argument-count heuristic \
              would add indirection without making the seam clearer"
)]
#[expect(
    clippy::needless_pass_by_value,
    reason = "each caller runs this on its own per-trigger thread and hands over \
              the clones it made for it; borrowing would push the lifetime problem \
              back onto every caller for no saved allocation"
)]
pub fn handle_trigger<A: Announce>(
    trig: Trigger,
    anchor_ns: u64,
    out_dir: &Path,
    grace: Duration,
    tailer: Arc<Tailer>,
    extract_tx: Sender<StageJob>,
    faults: Arc<CutFaults>,
    announce: A,
    time_source: TimeSource,
    producer: Producer,
) -> anyhow::Result<()> {
    // The request is the window: `CutRequest` derives the bounds from the
    // trigger, so the window logged here, the window every message is tested
    // against, and the window the clip's document states are one value.
    let request = Arc::new(CutRequest::new(
        producer,
        trig.clone(),
        anchor_ns,
        time_source,
    ));
    info!(
        "trigger name={:?} source={time_source} window=[{}, {}] preroll={} postroll={}",
        trig.name,
        request.start_ns(),
        request.end_ns(),
        trig.preroll,
        trig.postroll
    );

    let outcome = record_clip(&tailer, &request, out_dir, grace, &extract_tx)
        .map_err(|e| report_refusal(&faults, e))?;
    // A skipped window wrote nothing, so there is nothing to announce: the
    // warning the cut already logged, naming the directory, is its whole trace.
    let CutOutcome::Cut(clip) = outcome else {
        return Ok(());
    };

    clip::cut::report_clips(&clip.files);
    if clip.files.len() > 1 {
        info!(
            "trigger name={:?} spanned a rollover into {} files in {}",
            trig.name,
            clip.files.len(),
            clip.dir.display(),
        );
    }

    // One entry: the clip is the directory, so a subscriber opens one handle per
    // clip rather than learning how its files are named.
    announce.announce(&Completion {
        name: trig.name,
        filenames: vec![clip.dir.to_string_lossy().into_owned()],
        description: trig.description,
        trigger_time: trig.trigger_time,
        preroll: trig.preroll,
    });
    Ok(())
}

/// Turn a failed cut into the error the caller logs, escalating the one fault
/// that repeats: a recording whose bytes changed under the tail after they were
/// indexed.
///
/// **Only a [`FramingDesync`] is escalated**, because it is the only cut failure
/// that is permanent for the recording it names — the scan is long past those
/// bytes and never re-reads them, and a length past `MAX_RECORD_LEN` is a value
/// no valid record reaches. Every other way a cut fails (an IO error, a full
/// disk, an output failure, a staging panic) is transient or is fixed somewhere
/// else, keeps its own per-trigger error, and is deliberately left out of the
/// tally so that "this recording is damaged" cannot be read off a full disk.
///
/// The first refusal against a recording is announced in full: what changed,
/// which recording, how much of it the refusal covers, and the one thing an
/// operator can do about it. Every refusal after it carries the running count
/// instead, so the log shows a tally climbing rather than one indistinguishable
/// line per trigger.
///
/// **It does not exit the process**, which is where this parts company with the
/// [scan-fault budget](crate::faults). The reasoning is in
/// [`crate::faults`]: the recorder is still functional here, and a restart makes
/// things strictly worse — the fresh scan meets the same damage ahead of it and
/// dies on the budget instead.
fn report_refusal(faults: &CutFaults, err: anyhow::Error) -> anyhow::Error {
    let Some(desync) = err.downcast_ref::<FramingDesync>() else {
        // Not file damage: a different fault with a different remedy, reported
        // as it is and counted toward nothing.
        return err;
    };
    let recording = desync.recording().display().to_string();
    let refusals = faults.refused(desync);
    if refusals == 1 {
        error!(
            "recording {recording} changed under the tail after it was indexed: {desync}. \
             Every clip whose window plans the extent at {} is refused with it — up to \
             {} MiB of recording, data written after the damage included — for as long \
             as this recording is tailed, and this recorder goes on cutting every window \
             that reads elsewhere. Rolling the recording over (a bag split, or \
             restarting `ros2 bag record`) is what clears it; restarting clipper does \
             not — a fresh scan meets these bytes ahead of it and exits on the \
             scan-fault budget instead.",
            desync.extent_offset(),
            EXTENT_CAP_BYTES / (1024 * 1024),
        );
    }
    err.context(format!(
        "clip {refusals} refused against {recording} since its framing desynced"
    ))
}

/// The live half of one trigger's cut: wait out the postroll wall floor, wait
/// for the tail's collection-wide coverage to reach the window end (bounded by
/// `grace`), then hand the window and `out_dir` to
/// [`clip::segment::cut_window`], which decides where the clip under it goes.
///
/// The two waits are the only reason this function exists. A window may reach
/// past the last byte on disk, and cutting one before the data lands would
/// silently truncate the clip; everything after the waits is the same shared
/// code a consumer cutting from a finished recording runs, and it does not wait
/// at all. A returned [`clip::segment::Clip`] is complete on disk — its metadata
/// file written and its directory fsynced — so the caller may announce it.
///
/// The coverage wait's verdict is not just a log line: it is the one thing a
/// finished clip cannot show from its own contents, so it travels into the cut
/// as a [`WindowCoverage`] and the clip's document reports it.
fn record_clip(
    tailer: &Arc<Tailer>,
    request: &Arc<CutRequest>,
    out_dir: &Path,
    grace: Duration,
    extract_tx: &Sender<StageJob>,
) -> anyhow::Result<CutOutcome> {
    // The watch the tailer itself feeds, so the wait cannot be pointed at a
    // different collection's coverage than the plan is taken from.
    let coverage = tailer.coverage();
    let (end_ns, time_source) = (request.end_ns(), request.time_source());

    // 1. Postroll wall floor: never cut before the wall clock passes the window
    //    end. `checked_sub` reads the clock once per iteration, so a clock that
    //    crosses `end_ns` between the check and the sleep cannot underflow.
    while let Some(remaining) = end_ns.checked_sub(now_ns()).filter(|n| *n > 0) {
        thread::sleep(Duration::from_nanos(remaining));
    }

    // 2. Coverage: wait until the collection-wide high-water on the window's
    //    time source reaches the window end, bounded by `grace`. On `log` this
    //    is a completeness proof — a window inside a recording is already
    //    covered; only a window whose end is past the last recorded message with
    //    no successor waits out the full grace. On `publish` it is a liveness
    //    signal only: publish times may arrive out of order, so a message can
    //    still land after the wait releases with an in-window `publish_time` and
    //    be lost from the cut — `grace` bounds the wait either way.
    let covered = if coverage.wait_timeout_for(grace, |c| c.for_source(time_source) >= end_ns) {
        WindowCoverage::Covered
    } else {
        warn!(
            "window end {end_ns} still uncovered after {grace:?}; \
             cutting the clip from what is on disk"
        );
        WindowCoverage::Short
    };

    // The data is as complete as it is going to get: claim the clip's directory,
    // plan, copy one file per source recording, drop the empties and complete
    // it. The tailer is the window planner — it serves plans out of its live
    // collection, each pinning its own recording's `Arc<File>` so a prune or
    // rollover after this cannot pull the bytes out from under the copy.
    segment::cut_window(tailer.as_ref(), request, covered, out_dir, extract_tx)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::assert_is_empty,
        clippy::too_many_arguments,
        clippy::too_many_lines,
        reason = "a failed unwrap or a panicking index is a failing test, and \
                  `assert!(x.is_empty())` names the claim better than the \
                  empty-array `assert_eq!` the lint asks for, \
                  and a test that builds a fixture, drives it and asserts on the \
                  whole result is long, nested and argument-heavy by \
                  construction — splitting one would scatter the case it states"
    )]

    use std::path::PathBuf;

    use clip::ChannelSelection;
    use clip::index::op;
    use clip::layout::read_metadata;
    use clip::segment::Clip;
    use clip::testing::{
        TEST_PRODUCER, channel_body, desync_record_framing, message_body_pub, raw_record,
        read_clip, test_dir, window_request, write_raw, write_recording,
        write_unfinished_recording,
    };

    use super::*;
    use crate::tailer::tests::{drain, scan_to_end};

    /// The request a test cuts `[start_ns, end_ns]` on `source` with, in the
    /// `Arc` the cut path shares between one window's segments.
    fn window((start_ns, end_ns): (u64, u64), source: TimeSource) -> Arc<CutRequest> {
        Arc::new(window_request(start_ns, end_ns, source))
    }

    /// The clip compression the recorder's default (zstd) maps to; the unit
    /// tests drive the extraction worker pool through the same codec the
    /// recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// The clip a cut wrote, or a failure naming what it did instead. A test
    /// about the waits cuts into an output directory of its own, so a skip means
    /// the test set itself up wrong.
    fn cut(outcome: CutOutcome) -> Clip {
        match outcome {
            CutOutcome::Cut(clip) => clip,
            CutOutcome::Skipped(dir) => panic!("expected a clip, got a skip of {}", dir.display()),
        }
    }

    /// The directory a cut skipped, or a failure naming what it did instead.
    fn skipped(outcome: CutOutcome) -> PathBuf {
        match outcome {
            CutOutcome::Skipped(dir) => dir,
            CutOutcome::Cut(clip) => {
                panic!("expected a skip, got the clip {}", clip.dir.display())
            }
        }
    }

    #[test]
    fn record_clip_grace_timeout_cuts_what_is_on_disk() -> anyhow::Result<()> {
        let root = test_dir("grace")?;
        let (tailer, _) = Tailer::new();
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        // The window end is far in the past on the wall clock (no postroll
        // sleep), but coverage never reaches it — no recording was ever
        // discovered. The grace timeout must fire and cut a valid empty clip
        // instead of hanging or erroring.
        let stats = cut(record_clip(
            &tailer,
            &window((0, 1_000), TimeSource::Log),
            &root,
            Duration::from_millis(50),
            &extract_tx,
        )?);

        assert_eq!(
            stats.files.len(),
            1,
            "no recording yields a single empty file"
        );
        assert_eq!(stats.files[0].messages_copied, 0);
        assert!(read_clip(&stats.files[0].out_path)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A second trigger for a window already cut is skipped, and the clip that
    /// is there is left exactly as it was.
    ///
    /// A repeated trigger is the ordinary case on a vehicle — a detector that
    /// fires twice on one event resolves to one window and one id — and a clip
    /// is written once. The directory's existence is the whole rule: the second
    /// cut's `mkdir` loses, so it never plans, never copies and never announces.
    #[test]
    fn record_clip_skips_a_trigger_whose_clip_is_already_there() -> anyhow::Result<()> {
        let root = test_dir("dup-trigger")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 900)])?;

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let out_dir = root.join("out");
        let record = || {
            record_clip(
                &tailer,
                &window((100, 900), TimeSource::Log),
                &out_dir,
                Duration::from_secs(10),
                &extract_tx,
            )
        };

        let first = cut(record()?);
        assert_eq!(
            first.dir.parent(),
            Some(out_dir.as_path()),
            "the cut puts the clip's directory under the output directory it was given"
        );
        let before = std::fs::read(&first.files[0].out_path)?;

        assert_eq!(
            skipped(record()?),
            first.dir,
            "the second trigger for the same window writes nothing"
        );
        assert_eq!(
            std::fs::read_dir(&out_dir)?.count(),
            1,
            "and the output directory still holds exactly the one clip"
        );
        assert_eq!(
            std::fs::read(&first.files[0].out_path)?,
            before,
            "not a byte of which changed"
        );

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
        let (tailer, _) = Tailer::new();
        let scanner = tailer.clone();
        std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            let file = Arc::new(std::fs::File::open(&rec).unwrap());
            scanner.attach(file.clone());
            scan_to_end(&scanner, &file, 8).unwrap();
        });

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let stats = cut(record_clip(
            &tailer,
            &window((100, 1_000), TimeSource::Log),
            &root,
            Duration::from_secs(10),
            &extract_tx,
        )?);

        assert_eq!(stats.files.len(), 1);
        assert_eq!(stats.files[0].messages_copied, 2);
        assert_eq!(
            read_clip(&stats.files[0].out_path)?,
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

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let end_ns = now + 150_000_000; // 150 ms past the trigger stamp
        let started = std::time::Instant::now();
        let stats = cut(record_clip(
            &tailer,
            &window((now.saturating_sub(1_000_000_000), end_ns), TimeSource::Log),
            &root,
            Duration::from_secs(10),
            &extract_tx,
        )?);

        assert!(
            started.elapsed() >= Duration::from_millis(50),
            "the cut must wait for the wall clock to pass the window end"
        );
        assert_eq!(stats.files.len(), 1);
        assert_eq!(
            stats.files[0].messages_copied, 1,
            "the future message is outside"
        );

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
        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let grace = Duration::from_millis(200);
        let started = std::time::Instant::now();
        let stats = cut(record_clip(
            &tailer,
            &window((50, 1_000_000), TimeSource::Log),
            &root,
            grace,
            &extract_tx,
        )?);

        assert!(
            started.elapsed() >= grace,
            "an uncovered window end waits out the grace before cutting"
        );
        assert!(
            started.elapsed() < Duration::from_secs(5),
            "the grace is the bound — it does not hang"
        );
        assert_eq!(stats.files.len(), 1);
        assert_eq!(stats.files[0].messages_copied, 2);

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

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let started = std::time::Instant::now();
        let stats = cut(record_clip(
            &tailer,
            &window((0, 1_000), TimeSource::Log),
            &root,
            Duration::from_secs(30),
            &extract_tx,
        )?);

        assert!(
            started.elapsed() < Duration::from_secs(5),
            "high_water == end satisfies the wait (>=, not >)"
        );
        assert_eq!(stats.files.len(), 1);
        assert_eq!(
            stats.files[0].messages_copied, 2,
            "the boundary message is inside"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// One announcement as a subscriber saw it: the [`Completion`] itself, and
    /// whether the clip it named was already complete — its metadata file on
    /// disk — at the instant `announce` was called.
    ///
    /// The second field is why the announcer captures rather than the test
    /// asserting afterwards. "A completion goes out only after the metadata file
    /// is durable" is an *ordering* claim, and by the time `handle_trigger` has
    /// returned the file is there whichever order the two happened in — so an
    /// assertion made then holds equally for a handler that announced first, and
    /// proves nothing.
    #[derive(Clone, Debug)]
    struct Announced {
        completion: Completion,
        complete_on_arrival: bool,
    }

    /// A capturing [`Announce`] that records every completion it is handed, so a
    /// test can assert what `handle_trigger` announced — and when.
    #[derive(Clone)]
    struct CapturingAnnouncer(Arc<std::sync::Mutex<Vec<Announced>>>);

    impl CapturingAnnouncer {
        fn new() -> Self {
            CapturingAnnouncer(Arc::new(std::sync::Mutex::new(Vec::new())))
        }

        /// Everything it has been handed, in the order it arrived.
        fn announced(&self) -> Vec<Announced> {
            self.0.lock().unwrap().clone()
        }
    }

    impl Announce for CapturingAnnouncer {
        fn announce(&self, completion: &Completion) {
            // Read the clip's directory the way a subscriber would: out of the
            // announcement's one `filenames` entry.
            let complete_on_arrival = completion
                .filenames
                .first()
                .is_some_and(|dir| clip::layout::metadata_path(Path::new(dir)).is_file());
            self.0.lock().unwrap().push(Announced {
                completion: completion.clone(),
                complete_on_arrival,
            });
        }
    }

    /// The trigger a window `[start_ns, end_ns]` is cut for, through the public
    /// entry point, returning what the handler made of it. The same `name` and
    /// window twice is the same trigger twice — one `CutRequest`, one id, and so
    /// the second call is the skip path.
    fn fire(
        tailer: &Arc<Tailer>,
        extract_tx: &Sender<StageJob>,
        faults: &Arc<CutFaults>,
        out_dir: &Path,
        announce: &CapturingAnnouncer,
        name: &str,
        (start_ns, end_ns): (u64, u64),
    ) -> anyhow::Result<()> {
        let half = (end_ns - start_ns) / 2;
        let anchor_ns = start_ns + half;
        handle_trigger(
            Trigger {
                name: name.to_string(),
                description: String::new(),
                trigger_time: clip::trigger::Stamp { sec: 0, nanosec: 0 },
                preroll: half,
                postroll: half,
            },
            anchor_ns,
            out_dir,
            Duration::from_millis(100),
            tailer.clone(),
            extract_tx.clone(),
            faults.clone(),
            announce.clone(),
            TimeSource::Log,
            TEST_PRODUCER,
        )
    }

    /// End to end through the public handler entry point: `handle_trigger` turns
    /// a neutral [`Trigger`] into a clip on disk and announces one
    /// [`Completion`] naming it. Exercises the whole flow the interfaces share —
    /// window math, the claim, the copy, naming, and the announce hand-off —
    /// independent of ROS/encoding.
    ///
    /// Three things about that announcement, which together are what a
    /// subscriber may rely on:
    ///
    /// - it names the clip's **directory**, one entry however many files the
    ///   clip took, so a subscriber opens one handle per clip and never learns
    ///   the naming scheme inside it;
    /// - the clip is **already complete when it arrives** — its metadata file is
    ///   on disk, checked inside `announce` rather than after the fact, so a
    ///   handler that announced ahead of the document would fail here;
    /// - and the directory it names holds exactly the in-window messages.
    #[test]
    fn handle_trigger_cuts_a_clip_and_announces_it() -> anyhow::Result<()> {
        let root = test_dir("handle-trigger")?;
        let out_dir = root.join("out");
        let rec = root.join("rec.mcap");
        // Two messages inside the window [0, 1000], one past it so coverage is
        // already satisfied and the cut does not wait out the grace.
        write_recording(&rec, false, &[("/t", 100), ("/t", 900), ("/t", 2_000)])?;

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let announcer = CapturingAnnouncer::new();

        let trig = Trigger {
            name: "evt".to_string(),
            description: "hi".to_string(),
            trigger_time: clip::trigger::Stamp {
                sec: 0,
                nanosec: 500,
            },
            preroll: 500,
            postroll: 500,
        };
        // The generic handler takes the anchor the interface resolved; here that
        // is the trigger's own stamp, so the window is [0, 1000].
        let anchor_ns = trig.trigger_time.ns();
        handle_trigger(
            trig,
            anchor_ns,
            &out_dir,
            Duration::from_secs(5),
            tailer,
            extract_tx,
            Arc::new(CutFaults::new()),
            announcer.clone(),
            TimeSource::Log,
            TEST_PRODUCER,
        )?;

        let done = announcer.announced();
        assert_eq!(done.len(), 1, "exactly one completion is announced");
        assert_eq!(done[0].completion.name, "evt");
        assert_eq!(
            done[0].completion.filenames.len(),
            1,
            "one entry: the clip is its directory"
        );
        assert!(
            done[0].complete_on_arrival,
            "the clip's metadata file is already on disk when the completion goes out"
        );
        let clip_dir = PathBuf::from(&done[0].completion.filenames[0]);
        assert!(clip_dir.is_dir(), "the announced clip exists on disk");
        let metadata = read_metadata(&clip_dir)?;
        assert_eq!(
            metadata.sources.len(),
            1,
            "one file — the window fell inside one recording"
        );
        assert_eq!(
            read_clip(&clip_dir.join(&metadata.sources[0].file))?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 900)],
            "the clip holds exactly the in-window messages"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `handle_trigger` over a window spanning two source recordings announces
    /// the one directory, and the two files are inside it.
    ///
    /// The announcement is one entry whatever the window straddled — that is the
    /// point of the clip being a directory — so what a subscriber has to handle
    /// does not change with the recorder's split schedule. The clip's document
    /// is where the two files are enumerated.
    #[test]
    fn handle_trigger_announces_one_directory_across_a_rollover() -> anyhow::Result<()> {
        let root = test_dir("ht-rollover")?;
        let out_dir = root.join("out");
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // Two finished recordings: split0 ends at 2_000, split1 starts at 5_000.
        // The trigger window [1_500, 5_500] straddles the gap.
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let (tailer, _) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let announcer = CapturingAnnouncer::new();

        // trigger_time = 3_500 ns, preroll = 2_000 ns, postroll = 2_000 ns
        // → window [1_500, 5_500]
        let trig = Trigger {
            name: "rollover".to_string(),
            description: String::new(),
            trigger_time: clip::trigger::Stamp {
                sec: 0,
                nanosec: 3_500,
            },
            preroll: 2_000,
            postroll: 2_000,
        };
        // trigger_time = 3_500 ns is the resolved anchor → window [1_500, 5_500].
        let anchor_ns = trig.trigger_time.ns();
        handle_trigger(
            trig,
            anchor_ns,
            &out_dir,
            Duration::from_secs(5),
            tailer,
            extract_tx,
            Arc::new(CutFaults::new()),
            announcer.clone(),
            TimeSource::Log,
            TEST_PRODUCER,
        )?;

        let done = announcer.announced();
        assert_eq!(done.len(), 1, "one completion per trigger");
        assert_eq!(done[0].completion.name, "rollover");
        assert_eq!(
            done[0].completion.filenames.len(),
            1,
            "a rollover window still announces one clip: its directory"
        );
        assert!(
            done[0].complete_on_arrival,
            "and it is complete on arrival however many files it took"
        );
        let clip_dir = PathBuf::from(&done[0].completion.filenames[0]);
        let metadata = read_metadata(&clip_dir)?;
        assert_eq!(
            metadata.sources.len(),
            2,
            "and the document enumerates the two files it took"
        );
        for source in &metadata.sources {
            let file = clip_dir.join(&source.file);
            assert!(file.is_file(), "{} must be on disk", file.display());
            assert!(
                !read_clip(&file)?.is_empty(),
                "each file must hold at least one message"
            );
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// **A skipped window announces nothing.** The same trigger twice resolves to
    /// one window, one id and one clip, and the second pass through
    /// `handle_trigger` writes nothing and tells nobody: nothing was recorded, so
    /// there is no completion to make, and the warning the cut logged naming the
    /// directory is the skip's whole trace.
    ///
    /// Its sibling `record_clip_skips_a_trigger_whose_clip_is_already_there` is
    /// the same case one layer down, where the [`CutOutcome`] names the skip and
    /// the bytes already on disk are shown to be untouched. This is the half only
    /// the public entry point can see — what a subscriber is told — and the one
    /// that would break if the announcement moved out from under the `Cut` arm.
    #[test]
    fn handle_trigger_announces_nothing_for_a_window_already_cut() -> anyhow::Result<()> {
        let root = test_dir("ht-skip")?;
        let out_dir = root.join("out");
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 900)])?;

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let faults = Arc::new(CutFaults::new());
        let announcer = CapturingAnnouncer::new();

        for _ in 0..2 {
            fire(
                &tailer,
                &extract_tx,
                &faults,
                &out_dir,
                &announcer,
                "evt",
                (100, 900),
            )?;
        }

        let done = announcer.announced();
        assert_eq!(
            done.len(),
            1,
            "the repeated trigger is answered with silence, not a second completion"
        );
        assert!(done[0].complete_on_arrival);
        assert_eq!(
            PathBuf::from(&done[0].completion.filenames[0]).parent(),
            Some(out_dir.as_path()),
            "and the one completion names the clip under the output directory"
        );
        assert_eq!(
            std::fs::read_dir(&out_dir)?.count(),
            1,
            "which is the only thing in it"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Coverage wait, extent overlap, and message selection all live on the
    /// window's [`TimeSource`]. A recording whose publish times run far ahead of
    /// its log times, windowed on `publish`: the handler blocks on the publish
    /// high-water, plans the publish-overlapping extent, and copies exactly the
    /// message whose `publish_time` is inside the window (retention is untouched
    /// — that stays on `log_time`).
    #[test]
    fn record_clip_windows_and_waits_on_the_configured_time_source() -> anyhow::Result<()> {
        let root = test_dir("cov-domain")?;
        let rec = root.join("rec.mcap");
        // log_time 100/200; publish_time 1_000/2_000 — the domains disagree.
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 100, 1_000, b"a")),
                raw_record(op::MESSAGE, &message_body_pub(1, 1, 200, 2_000, b"b")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        // Both high-waters are published independently: log 200, publish 2_000.
        assert_eq!(coverage.get().high_water_ns, 200);
        assert_eq!(coverage.get().publish_high_water_ns, 2_000);

        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        // Window [900, 1_500] on `publish`: the publish high-water (2_000)
        // satisfies the coverage wait, and only the message published at 1_000
        // (log_time 100) falls inside — the log high-water (200) is nowhere near
        // 1_500, so a log-domain wait would have timed out on the grace instead.
        let stats = cut(record_clip(
            &tailer,
            &window((900, 1_500), TimeSource::Publish),
            &root,
            Duration::from_secs(10),
            &extract_tx,
        )?);
        assert_eq!(stats.files.len(), 1);
        assert_eq!(stats.files[0].messages_copied, 1);
        assert_eq!(
            read_clip(&stats.files[0].out_path)?,
            vec![("/t".to_string(), 100)],
            "the publish window holds the message published at 1_000"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The coverage wait's verdict reaches the clip: a window the recording
    /// never reached is marked short, and one it did reach is not.
    ///
    /// This is the document's one caller-supplied fact, and the only one a clip
    /// cannot show from its own contents — both clips here end at the last
    /// message on disk and look identical. The two arms run against the same
    /// recording and differ only in the window, so nothing but the wait's
    /// outcome can be what moves the field.
    #[test]
    fn the_coverage_wait_marks_the_clip_short_or_not() -> anyhow::Result<()> {
        let root = test_dir("short")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let is_short =
            |clip: &Clip| -> anyhow::Result<bool> { Ok(read_metadata(&clip.dir)?.clip.short) };

        // The window ends at 200, exactly the recording's high-water: the wait
        // releases and the clip is complete.
        let covered = cut(record_clip(
            &tailer,
            &window((0, 200), TimeSource::Log),
            &root.join("covered"),
            Duration::from_secs(10),
            &extract_tx,
        )?);
        assert_eq!(covered.files[0].messages_copied, 2);
        assert!(!is_short(&covered)?);

        // The same recording, a window ending past everything it holds: the wait
        // burns its (short) grace and the clip is cut anyway, holding the same
        // two messages — the document is the only thing that says so.
        let short = cut(record_clip(
            &tailer,
            &window((0, 1_000_000), TimeSource::Log),
            &root.join("short"),
            Duration::from_millis(100),
            &extract_tx,
        )?);
        assert_eq!(short.files[0].messages_copied, 2);
        assert!(is_short(&short)?);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording a trigger handler has already scanned, then damaged behind
    /// that scan: the index still plans its extents, the bytes no longer frame
    /// the way it says they do, and every cut from it is refused.
    ///
    /// `stamps` are the message log times; `desync_at` is which of them loses
    /// its length prefix. The tailer is returned already drained, so the
    /// recordings are indexed and nothing rescans the broken bytes.
    fn damaged_recording(
        root: &Path,
        recordings: &[(&str, &[(&str, u64)])],
        desync: (&str, usize),
    ) -> anyhow::Result<Arc<Tailer>> {
        let (tailer, _) = Tailer::new();
        for (name, stamps) in recordings {
            let path = root.join(name);
            write_recording(&path, false, stamps)?;
            tailer.index_recording(&path);
        }
        drain(&tailer)?;
        desync_record_framing(&root.join(desync.0), desync.1)?;
        Ok(tailer)
    }

    /// A window over the damaged recording of [`damaged_recording`], and one
    /// over the clean one: disjoint in time, so each plans exactly one of them.
    const OVER_DAMAGE: (u64, u64) = (900, 3_100);
    const CLEAN_WINDOW: (u64, u64) = (50, 250);

    /// The escalation contract, driven through the handler over real damaged
    /// bytes: the first clip a recording's desync costs says so as clip **1**,
    /// and the cost of every later one is a number that climbs.
    ///
    /// The count riding in the error is what the recorder logs, so this is also
    /// the assertion that an operator reading the log sees a tally rather than
    /// one indistinguishable line per trigger.
    #[test]
    fn a_recordings_refusals_are_counted_and_the_count_climbs() -> anyhow::Result<()> {
        let root = test_dir("refusal-count")?;
        let out_dir = root.join("out");
        let tailer = damaged_recording(
            &root,
            &[("rec.mcap", &[("/t", 100), ("/t", 200), ("/t", 300)])],
            ("rec.mcap", 1),
        )?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let faults = Arc::new(CutFaults::new());
        let announcer = CapturingAnnouncer::new();

        for expected in 1..=3u64 {
            let err = fire(
                &tailer,
                &extract_tx,
                &faults,
                &out_dir,
                &announcer,
                "over-damage",
                (0, 400),
            )
            .expect_err("a desynced extent refuses the clip");
            let text = format!("{err:#}");
            assert!(
                text.contains(&format!("clip {expected} refused against")),
                "refusal {expected} must carry its own count: {text}"
            );
            assert!(
                text.contains("extent framing inconsistent with the tail's scan"),
                "the refusal still names the fault itself: {text}"
            );
        }
        assert_eq!(
            std::fs::read_dir(&out_dir)?.count(),
            0,
            "a refused cut leaves nothing behind: its clip directory goes with it"
        );
        assert!(
            announcer.announced().is_empty(),
            "and nothing is announced: a completion names a clip, and there is none"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A clip cut successfully from the damaged recording between two refusals
    /// does **not** reset the tally.
    ///
    /// This is the deliberate difference from the scan-fault budget, which does
    /// reset on a clean pass because a scan fault can be a record still being
    /// appended. A framing desync cannot heal: the scan is long past those bytes
    /// and never re-reads them, so a cut that succeeds only proves its window
    /// planned a different extent. Resetting here would re-announce in full
    /// every time windows alternated — exactly the per-trigger noise the
    /// escalation exists to replace.
    #[test]
    fn a_successful_cut_does_not_reset_the_tally() -> anyhow::Result<()> {
        let root = test_dir("refusal-no-reset")?;
        let out_dir = root.join("out");
        // Two recordings: one clean, one damaged, disjoint in time so each
        // window plans exactly one of them.
        let tailer = damaged_recording(
            &root,
            &[
                ("clean.mcap", &[("/t", 100), ("/t", 200)]),
                (
                    "damaged.mcap",
                    &[("/t", 1_000), ("/t", 2_000), ("/t", 3_000)],
                ),
            ],
            ("damaged.mcap", 1),
        )?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let faults = Arc::new(CutFaults::new());
        let announcer = CapturingAnnouncer::new();
        let err = fire(
            &tailer,
            &extract_tx,
            &faults,
            &out_dir,
            &announcer,
            "first",
            OVER_DAMAGE,
        )
        .expect_err("the damaged recording refuses");
        assert!(format!("{err:#}").contains("clip 1 refused against"));

        fire(
            &tailer,
            &extract_tx,
            &faults,
            &out_dir,
            &announcer,
            "between",
            CLEAN_WINDOW,
        )
        .expect("the clean recording still cuts — the recorder is not wedged");

        let err = fire(
            &tailer,
            &extract_tx,
            &faults,
            &out_dir,
            &announcer,
            "second",
            OVER_DAMAGE,
        )
        .expect_err("the damaged recording refuses again");
        assert!(
            format!("{err:#}").contains("clip 2 refused against"),
            "a clip cut elsewhere does not heal the damage: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Everything that is not a framing desync stays out of the tally and out of
    /// the escalation: it keeps its own error, unannotated.
    ///
    /// A full disk, an IO error on the recording and an output failure are all
    /// transient or fixed elsewhere, and folding them in would let "this
    /// recording is damaged" be read off a disk that filled up.
    #[test]
    fn only_a_framing_desync_is_counted() {
        let faults = CutFaults::new();
        let other = report_refusal(
            &faults,
            anyhow::anyhow!("writing clip: No space left on device (os error 28)"),
        );
        assert_eq!(
            format!("{other:#}"),
            "writing clip: No space left on device (os error 28)",
            "a fault that is not file damage is reported exactly as it came"
        );

        // The tally is untouched by it: the next desync is still the first.
        let desync = clip::cut::FramingDesync::RecordLength {
            recording: std::path::PathBuf::from("/rec/record_0.mcap"),
            extent_offset: 0,
            offset: 19_773,
            declared: u64::MAX,
        };
        let counted = report_refusal(&faults, anyhow::Error::new(desync));
        assert!(
            format!("{counted:#}").contains("clip 1 refused against /rec/record_0.mcap"),
            "the first desync is clip 1: {counted:#}"
        );
    }

    /// Through the public entry point: the clip a trigger produces states that
    /// trigger and the producer that cut it.
    ///
    /// The handler is where a real trigger exists, so this is the only place the
    /// name, description and rolls a requester actually sent can be checked
    /// against what came out the far end of the staging pool.
    #[test]
    fn handle_trigger_stamps_the_clip_with_the_trigger_that_asked_for_it() -> anyhow::Result<()> {
        let root = test_dir("ht-manifest")?;
        let out_dir = root.join("out");
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 900), ("/t", 2_000)])?;

        let (tailer, _) = Tailer::new();
        let file = Arc::new(std::fs::File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        let extract_tx =
            segment::spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let announcer = CapturingAnnouncer::new();
        let trig = Trigger {
            name: "evt".to_string(),
            description: "why this clip exists".to_string(),
            trigger_time: clip::trigger::Stamp {
                sec: 0,
                nanosec: 500,
            },
            preroll: 500,
            postroll: 500,
        };
        let anchor_ns = trig.trigger_time.ns();
        handle_trigger(
            trig,
            anchor_ns,
            &out_dir,
            Duration::from_secs(5),
            tailer,
            extract_tx,
            Arc::new(CutFaults::new()),
            announcer.clone(),
            TimeSource::Log,
            TEST_PRODUCER,
        )?;

        let done = announcer.announced();
        let clip_dir = PathBuf::from(&done[0].completion.filenames[0]);
        let metadata = read_metadata(&clip_dir)?;
        assert_eq!(metadata.producer.name, TEST_PRODUCER.program);
        assert_eq!(metadata.producer.mode, TEST_PRODUCER.mode);
        assert_eq!(metadata.trigger.name, "evt");
        assert_eq!(metadata.trigger.description, "why this clip exists");
        assert_eq!(metadata.trigger.anchor_ns, 500);
        assert_eq!(metadata.trigger.preroll_ns, 500);
        assert_eq!(metadata.trigger.postroll_ns, 500);
        // The window the handler resolved from that trigger, and the messages it
        // actually took: [500 - 500, 500 + 500].
        assert_eq!(metadata.window.start_ns, 0);
        assert_eq!(metadata.window.end_ns, 1_000);
        assert_eq!(metadata.clip.messages, 2);
        assert_eq!(metadata.sources.len(), 1);
        assert_eq!(
            read_clip(&clip_dir.join(&metadata.sources[0].file))?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 900)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
