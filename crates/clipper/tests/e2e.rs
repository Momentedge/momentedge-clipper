//! Live ROS2 end-to-end tests for the continuous extractor: a real
//! `ros2 bag record` (the production recording invocation, driven directly
//! by the harness), real CLI-published triggers, and real `Recorded`
//! announcements, against the deployed storage profiles.
//!
//! Gated on `MOMENTEDGE_E2E` (see [`harness::require_e2e`]); cargo-nextest is
//! the required runner. Inside the dev shell:
//!
//! ```text
//! CLIPPER_E2E=1 cargo nextest run -p clipper --profile e2e -E 'binary(e2e)'
//! ```
//!
//! Each test brings up its own recorder/source/extractor stack in its own
//! DDS domain and temp tree; the suite is serialized by the `ros-e2e` nextest
//! test group (`.config/nextest.toml`). Lifecycle-mutating tests (restart,
//! corruption) drive the recorder's start/kill/damage themselves — the
//! lifecycle is exactly what they exercise.

mod harness;

use std::path::Path;
use std::time::Duration;

use harness::*;
use rstest::rstest;

const SRC_TOPIC: &str = "/e2e/chatter";
const SRC_RATE: u32 = 20;
const SEC: u64 = 1_000_000_000;

/// The full happy path per storage profile: real append/flush, trigger
/// publication, `Recorded` semantics, and final-path visibility.
///
/// The fastwrite case (unchunked, write-through) covers the deployed default;
/// the zstd_fast case covers a chunked profile, where on-disk visibility lags
/// by a chunk fill — the test stops the recorder cleanly after the window so
/// the flushed footer (`ended`) releases the coverage wait deterministically.
#[rstest]
#[case::fastwrite("fastwrite", 0, 30, false)]
#[case::zstd_fast("zstd_fast", 0, 15, true)]
fn trigger_produces_clip_and_announcement(
    #[case] preset: &str,
    #[case] cache: u64,
    #[case] grace_secs: u64,
    #[case] stop_recorder_to_flush: bool,
) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder(preset, cache);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(grace_secs);

    // Lay down at least a preroll's worth of data before triggering.
    std::thread::sleep(Duration::from_secs(3));

    let mut listener = env.start_recorded_listener("clip");
    let trigger_ns = now_ns();
    let (preroll, postroll) = (2 * SEC, 3 * SEC);
    env.fire_trigger("e2e-clip", trigger_ns, preroll, postroll);

    if stop_recorder_to_flush {
        let end_ns = trigger_ns + postroll;
        let now = now_ns();
        if end_ns > now {
            std::thread::sleep(Duration::from_nanos(end_ns - now) + Duration::from_secs(1));
        }
        recorder.stop(libc::SIGINT, Duration::from_secs(30));
    }

    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(grace_secs + 40));

    // The announcement echoes the trigger and names exactly the clip path the
    // contract promises.
    assert_eq!(recorded.name, "e2e-clip");
    let expected = env.out_dir().join(format!("{trigger_ns}_e2e-clip.mcap"));
    assert_eq!(
        Path::new(&recorded.filename),
        expected,
        "announced filename must be <out_dir>/<trigger_ns>_<name>.mcap"
    );

    // Final-path visibility: the announced file already exists, is a
    // complete MCAP (read_clip parses through the footer), holds only
    // in-window messages, and includes the source topic.
    let msgs = read_clip(Path::new(&recorded.filename));
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, trigger_ns - preroll, trigger_ns + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip, got topics: {:?}",
        msgs.iter()
            .map(|(t, _)| t)
            .collect::<std::collections::HashSet<_>>(),
    );
    env.assert_capturing_drained();
    assert!(extractor.is_running(), "the extractor must outlive the cut");
}

/// Restart during operation: the recorder is stopped and relaunched (the
/// record script wipes the bag dir), the extractor must re-discover the new
/// recording and keep cutting clips for later triggers.
#[rstest]
fn recorder_restart_recovers_and_keeps_extracting() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    std::thread::sleep(Duration::from_secs(3));

    // Trigger #1 against the first recording.
    let mut listener1 = env.start_recorded_listener("first");
    let t1 = now_ns();
    env.fire_trigger("restart-1", t1, 2 * SEC, 2 * SEC);
    let r1 = wait_for_recorded(&mut listener1, Duration::from_secs(60));
    assert!(!read_clip(Path::new(&r1.filename)).is_empty());

    // Restart: clean stop, relaunch; the script wipes record/ and starts a
    // fresh bag, which the tail must notice as a replacement.
    let (_recorder2, _) = env.restart_recorder(&mut recorder, &extractor, "fastwrite", 0);
    std::thread::sleep(Duration::from_secs(3));
    assert!(
        extractor.is_running(),
        "the extractor must survive a recorder restart"
    );

    // Trigger #2 against the new recording.
    let mut listener2 = env.start_recorded_listener("second");
    let t2 = now_ns();
    env.fire_trigger("restart-2", t2, 2 * SEC, 2 * SEC);
    let r2 = wait_for_recorded(&mut listener2, Duration::from_secs(60));
    assert_eq!(r2.name, "restart-2");
    let msgs = read_clip(Path::new(&r2.filename));
    assert!(!msgs.is_empty(), "the post-restart clip must hold data");
    assert_clip_within_window(&msgs, t2 - 2 * SEC, t2 + 2 * SEC);
    assert!(msgs.iter().any(|(topic, _)| topic == SRC_TOPIC));
    env.assert_capturing_drained();
    assert!(extractor.is_running());
}

/// The recorder is restarted inside an open trigger window: the tail attaches
/// the replacement recording and the index resets, so the clip is cut from
/// the most recent recording only — the pre-restart data is accepted as lost
/// (recovering it from a still-readable old file is explicitly out of scope:
/// beads clipper-gl2). The announcement must still go out and every
/// message in the clip must postdate the restart.
///
/// The cases vary when the recording file is deleted relative to the trigger
/// and the relaunch (the record script's wipe deletes it at relaunch anyway):
/// a clean stop+start inside the window; an explicit deletion inside the
/// window before the restart; a deletion before the trigger even fires, with
/// the restart landing inside the window (the tail is idle re-discovering
/// when the trigger arrives).
/// When the recording file is deleted, relative to the trigger and the
/// restart, in [`recorder_restart_inside_the_window_cuts_only_the_new_recording`].
/// One axis with three points rather than two booleans: deleting both before
/// the trigger and again mid-window is no fourth case — the file is already
/// gone.
#[derive(Clone, Copy, PartialEq)]
enum Deletion {
    /// No explicit deletion — the relaunch's bag-dir wipe is the only one.
    Never,
    /// Deleted inside the open window, just before the restart.
    MidWindow,
    /// Deleted before the trigger even fires.
    BeforeTrigger,
}

#[rstest]
#[case::clean_restart(Deletion::Never)]
#[case::deleted_then_restarted(Deletion::MidWindow)]
#[case::deleted_before_the_trigger(Deletion::BeforeTrigger)]
fn recorder_restart_inside_the_window_cuts_only_the_new_recording(#[case] deletion: Deletion) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    std::thread::sleep(Duration::from_secs(3));

    // Deleted before the trigger: the recorder keeps appending to the
    // unlinked inode, the tail notices the replacement and idles
    // re-discovering, and the coverage high-water freezes.
    if deletion == Deletion::BeforeTrigger {
        env.delete_recording();
        std::thread::sleep(Duration::from_secs(1));
    }

    let mut listener = env.start_recorded_listener("restart-inside");
    let trigger_ns = now_ns();
    // The postroll must outlast the restart sequence with data time to
    // spare; the precondition after the relaunch checks that it did.
    let (preroll, postroll) = (2 * SEC, 15 * SEC);
    env.fire_trigger("restart-inside", trigger_ns, preroll, postroll);

    // Two seconds of the window lie down before the restart; that data is
    // the part the cut must not contain afterwards.
    std::thread::sleep(Duration::from_secs(2));
    if deletion == Deletion::MidWindow {
        env.delete_recording();
    }
    let (_recorder2, restart_ns) = env.restart_recorder(&mut recorder, &extractor, "fastwrite", 0);
    // On a machine slow enough that the restart ate the whole postroll, the
    // empty clip below would misread as a semantics regression — fail it as
    // the timing precondition it is.
    assert!(
        now_ns() + 3 * SEC < trigger_ns + postroll,
        "precondition: the restart must finish at least 3 s before the window end"
    );

    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert_eq!(recorded.name, "restart-inside");
    let msgs = read_clip(Path::new(&recorded.filename));
    assert!(
        !msgs.is_empty(),
        "the window tail past the restart must be in the clip"
    );
    assert_clip_within_window(&msgs, trigger_ns - preroll, trigger_ns + postroll);
    assert!(
        msgs.iter().all(|(_, log_time)| *log_time >= restart_ns),
        "the clip must hold only the new recording's data: a message predates \
         the restart at {restart_ns}, oldest stamp {:?}",
        msgs.iter().map(|(_, t)| t).min(),
    );
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "the extractor must survive a restart inside an open window"
    );
}

/// The recorder dies (hard) while a trigger is waiting for coverage: the
/// window end can never be covered, so the grace timeout must cut a valid
/// clip from what is on disk and the announcement must still go out.
#[rstest]
fn recorder_killed_mid_trigger_still_announces_via_grace_cut() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(5);
    std::thread::sleep(Duration::from_secs(2));

    let mut listener = env.start_recorded_listener("grace");
    let t = now_ns();
    env.fire_trigger("mid-kill", t, SEC, 8 * SEC);

    // Kill the recorder inside the postroll: the file freezes mid-window, no
    // footer is ever written, coverage stalls below the window end.
    std::thread::sleep(Duration::from_secs(2));
    recorder.signal_group(libc::SIGKILL);
    recorder
        .wait_exit(Duration::from_secs(10))
        .expect("SIGKILL must end the recorder");

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the cut must have come from the grace timeout"
    );
    let msgs = read_clip(Path::new(&r.filename));
    assert!(
        !msgs.is_empty(),
        "data recorded before the kill lies in the window"
    );
    assert_clip_within_window(&msgs, t - SEC, t + 8 * SEC);
    assert!(
        extractor.is_running(),
        "a dead recorder mid-trigger must not take the extractor down"
    );
}

/// The recording file is deleted and no replacement ever appears: the
/// recorder keeps appending to the unlinked inode, but the tail treats the
/// vanished path as a replacement, stops scanning, and idles re-discovering,
/// so the coverage high-water freezes at the deletion point. A window
/// reaching past the freeze can never be covered — the grace timeout cuts
/// the clip from the still-attached index of the deleted file, read through
/// the file handle the plan holds (a deleted recording is not an error), so
/// the data scanned before the deletion still makes it into the clip.
///
/// The cases vary where the deletion lands: inside the open window, or
/// before the trigger even fires (the trigger then arrives against an
/// already-frozen tail). No second recorder may ever start in this test:
/// the cut recovering pre-deletion data rests on nothing replacing the
/// deleted recording — a replacement would attach and wipe the index.
#[rstest]
#[case::deleted_mid_window(false)]
#[case::deleted_before_the_trigger(true)]
fn recording_deleted_without_restart_grace_cuts_the_old_data(
    #[case] delete_before_the_trigger: bool,
) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(5);
    std::thread::sleep(Duration::from_secs(3));

    if delete_before_the_trigger {
        env.delete_recording();
        std::thread::sleep(Duration::from_secs(1));
    }

    // The preroll must reach back past the deletion in the pre-trigger case:
    // between the deletion and the trigger stamp lie the settle sleep, the
    // listener's discovery head start, and the publish itself — several
    // seconds the window has to span to still cover pre-deletion data.
    let (preroll, postroll) = (8 * SEC, 6 * SEC);
    let mut listener = env.start_recorded_listener("deleted");
    let t = now_ns();
    env.fire_trigger("deleted", t, preroll, postroll);

    if !delete_before_the_trigger {
        std::thread::sleep(Duration::from_secs(2));
        env.delete_recording();
    }

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the frozen coverage must have forced a grace-timeout cut"
    );
    extractor.expect_log("replaced; re-discovering", Duration::from_secs(60));
    let msgs = read_clip(Path::new(&r.filename));
    assert!(
        !msgs.is_empty(),
        "the data scanned before the deletion lies in the window"
    );
    assert_clip_within_window(&msgs, t - preroll, t + postroll);
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "a deleted recording must not take the extractor down"
    );
}

/// The recording is deleted inside the window and the recorder only comes
/// back after the window has ended: the handler is still in its grace wait
/// when the replacement attaches and wipes the index, and the new
/// recording's first messages — stamped past the window end — release the
/// coverage wait. The plan then holds no extent overlapping the window, so
/// the cut is a complete, valid, empty clip, announced all the same: the
/// window's data is accepted as lost with the file it lived in.
#[rstest]
fn restart_after_the_window_ended_announces_an_empty_clip() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    std::thread::sleep(Duration::from_secs(3));

    let mut listener = env.start_recorded_listener("post-window");
    let t = now_ns();
    let (preroll, postroll) = (2 * SEC, 4 * SEC);
    env.fire_trigger("post-window", t, preroll, postroll);

    // Freeze coverage inside the window. Positive control, read before the
    // deletion: the recording demonstrably holds data inside the window, so
    // the empty clip below is the accept-loss semantics at work, not a
    // mistimed window quietly passing.
    std::thread::sleep(Duration::from_secs(1));
    let deleted = env.newest_recording().expect("the recording exists");
    assert!(
        partial_recording_stamps(&deleted)
            .iter()
            .any(|stamp| (t - preroll..=t + postroll).contains(stamp)),
        "precondition: the recording held the window's data before the deletion"
    );
    std::fs::remove_file(&deleted).expect("deleting the recording");

    // Let the window end while the tail idles re-discovering — the handler
    // enters its grace wait — then restart: the relaunch lands after the
    // window end but well inside the grace.
    let end_ns = t + postroll;
    let now = now_ns();
    if end_ns > now {
        std::thread::sleep(Duration::from_nanos(end_ns - now) + Duration::from_secs(1));
    }
    let (_recorder2, _) = env.restart_recorder(&mut recorder, &extractor, "fastwrite", 0);

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    let msgs = read_clip(Path::new(&r.filename));
    assert!(
        msgs.is_empty(),
        "the new recording holds nothing inside the window, so the clip must \
         be empty — the old file's data is lost with it, got {} messages",
        msgs.len()
    );
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "an empty cut is a valid outcome, not a failure"
    );
}

/// A finished recording left on disk next to the live one is never read
/// again: the tail follows the newest file only, and a window lying entirely
/// inside the old file's data cuts an empty clip even though that data is
/// still on disk and readable. Recovering a previous recording's data is an
/// explicit non-feature (beads clipper-gl2): a rotation means the old
/// file's data is gone as far as clips are concerned — later windows over
/// the live recording keep working.
#[rstest]
fn old_recording_on_disk_is_not_recovered_after_restart() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    let bag = env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    std::thread::sleep(Duration::from_secs(3));

    // Stop cleanly: a finished recording whose data runs up to t_old, moved
    // outside record/ before the relaunch wipes the bag dir.
    let t_old = now_ns();
    recorder.stop(libc::SIGINT, Duration::from_secs(30));
    let saved = env.root().join("previous.saved");
    std::fs::rename(&bag, &saved).expect("saving the finished recording");

    let _recorder2 = env.start_recorder("fastwrite", 0);
    env.wait_for_recording(Duration::from_secs(60));
    // The new recording must be attached before the old file reappears in
    // record/ — a fresh-mtime mcap there could otherwise win the discovery.
    // The needle carries the bag path (identical across the restart): the
    // second "tailing <bag>" line is the re-attach. A path-free needle would
    // already be satisfied by the startup banner, which names the record dir
    // in the same words.
    extractor.expect_log_count(
        &format!("tailing {}", bag.display()),
        2,
        Duration::from_secs(60),
    );

    // Put the finished old recording back next to the live one. Staged
    // outside the *.mcap glob and renamed in, with its mtime predated first,
    // so newest-by-mtime discovery keeps following the live file from the
    // moment the old one becomes discoverable at all.
    let staged = env.record_dir().join("previous.staged");
    std::fs::rename(&saved, &staged).expect("restoring the old recording");
    std::fs::File::options()
        .write(true)
        .open(&staged)
        .and_then(|f| f.set_modified(std::time::SystemTime::now() - Duration::from_secs(60)))
        .expect("predating the old recording's mtime");
    let restored = env.record_dir().join("previous_0.mcap");
    std::fs::rename(&staged, &restored).expect("renaming the old recording into place");

    // The window lies entirely inside the old recording's data — and that
    // data demonstrably sits on disk, readable, in record/.
    let (start, end) = (t_old - 2 * SEC, t_old);
    let old_msgs = read_clip(&restored);
    assert!(
        old_msgs
            .iter()
            .any(|(_, log_time)| (start..=end).contains(log_time)),
        "precondition: the restored old recording holds the window's data"
    );

    let mut listener = env.start_recorded_listener("old-window");
    env.fire_trigger("old-window", t_old, 2 * SEC, 0);
    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    let msgs = read_clip(Path::new(&r.filename));
    assert!(
        msgs.is_empty(),
        "the old file's data must not be recovered into the clip, got {} messages",
        msgs.len()
    );

    // Health proof: a window over the live recording still cuts real data.
    std::thread::sleep(Duration::from_secs(2));
    let mut listener2 = env.start_recorded_listener("fresh");
    let t2 = now_ns();
    env.fire_trigger("fresh", t2, SEC, 2 * SEC);
    let r2 = wait_for_recorded(&mut listener2, Duration::from_secs(60));
    let msgs2 = read_clip(Path::new(&r2.filename));
    assert!(!msgs2.is_empty(), "the live recording must still cut clips");
    assert_clip_within_window(&msgs2, t2 - SEC, t2 + 2 * SEC);
    env.assert_capturing_drained();
    assert!(extractor.is_running());
}

/// Quiet topics: the recording's topics go silent, so coverage never reaches
/// the window end. The grace timeout cuts a valid, possibly short clip. The
/// recording is restricted to the source topic so no ambient topic (/rosout)
/// can cover the window by accident.
#[rstest]
fn quiet_topics_grace_timeout_cut() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder_topics(&[SRC_TOPIC], "fastwrite", 0);
    let mut source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(5);
    std::thread::sleep(Duration::from_secs(3));

    let mut listener = env.start_recorded_listener("quiet");
    let t = now_ns();
    // Stop the source, then fire a trigger whose window extends past the
    // last data: coverage can never reach t + postroll.
    source.stop(libc::SIGTERM, Duration::from_secs(10));
    env.fire_trigger("quiet", t, 2 * SEC, 6 * SEC);

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the cut must have come from the grace timeout"
    );
    let msgs = read_clip(Path::new(&r.filename));
    assert!(
        !msgs.is_empty(),
        "the preroll data recorded before the quiet period lies in the window"
    );
    assert_clip_within_window(&msgs, t - 2 * SEC, t + 6 * SEC);
    env.assert_capturing_drained();
    assert!(extractor.is_running());
}

/// Corrupt tail, offline and deterministic: a framing fault (oversized
/// declared record length — no resync point) planted at a known record
/// boundary. The extractor must fail fast with a non-zero exit for a
/// supervisor, not limp on cutting silently degraded clips.
#[rstest]
fn corrupt_tail_fails_fast_offline() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    std::thread::sleep(Duration::from_secs(3));
    // Clean stop: a complete, valid recording to plant precise damage in.
    recorder.stop(libc::SIGINT, Duration::from_secs(30));

    let bag = env.newest_recording().expect("the recording exists");
    inject_framing_fault(&bag);

    let mut extractor = env.start_extractor(30);
    let status = extractor
        .wait_exit(Duration::from_secs(60))
        .unwrap_or_else(|| {
            extractor.dump_log();
            panic!("the extractor must fail fast on a framing fault, not limp on");
        });
    assert!(
        !status.success(),
        "a framing fault must exit non-zero for a supervisor, got {status}"
    );
    assert!(
        extractor.log_text().contains("faulted at offset"),
        "the exit must name the scan fault: see extractor log"
    );
}

/// Corrupt tail, live: damage injected into the growing file mid-record.
/// Racy by design (nextest retries cover it): depending on where the scan
/// was, the extractor either tolerates localized damage — stays up, and a
/// later trigger over undamaged data still announces — or fail-fast exits
/// non-zero on a framing desync. It must never hang or die silently.
#[rstest]
fn corrupt_tail_health_live() {
    if !require_e2e() || skip_flaky() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, 50);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(10);
    std::thread::sleep(Duration::from_secs(3));

    // Damage a run of bytes halfway into the recorded region, under the live
    // writer. The tail has typically consumed those bytes already, so the
    // damage surfaces at extraction; if the scan was still behind it, it
    // surfaces as a scan fault.
    let bag = env.newest_recording().expect("the recording exists");
    let len = std::fs::metadata(&bag).expect("recording metadata").len();
    overwrite_bytes(&bag, len / 2, 64);

    let fail_fast = |extractor: &mut Proc| {
        let status = extractor
            .wait_exit(Duration::from_secs(30))
            .expect("an extractor that stopped must fully exit");
        assert!(
            !status.success(),
            "a framing desync must exit non-zero, got {status}"
        );
        assert!(
            extractor.log_text().contains("faulted at offset"),
            "the exit must name the scan fault"
        );
    };

    // Trigger A spans the damaged region (preroll reaches the file's start).
    // Legal outcomes: a degraded-but-complete clip is announced, or the
    // extraction aborts per-trigger (no announcement) while the process
    // stays up, or the scan faulted and the process fail-fast exited.
    let mut listener_a = env.start_recorded_listener("over-damage");
    let ta = now_ns();
    if !extractor.is_running() {
        fail_fast(&mut extractor);
        return;
    }
    env.fire_trigger("over-damage", ta, 60 * SEC, SEC);
    if let Some(a) = try_wait_for_recorded(&mut listener_a, Duration::from_secs(30)) {
        // Whatever the damage did, an announced file is a complete MCAP.
        let msgs = read_clip(Path::new(&a.filename));
        assert_clip_within_window(&msgs, ta - 60 * SEC, ta + SEC);
    }
    if !extractor.is_running() {
        fail_fast(&mut extractor);
        return;
    }

    // Health proof: a fresh window past the damage must still announce.
    std::thread::sleep(Duration::from_secs(2));
    let mut listener_b = env.start_recorded_listener("post-damage");
    let tb = now_ns();
    env.fire_trigger("post-damage", tb, SEC, SEC);
    let b = wait_for_recorded(&mut listener_b, Duration::from_secs(60));
    let msgs = read_clip(Path::new(&b.filename));
    assert!(
        !msgs.is_empty(),
        "a window over undamaged data must still produce a full clip"
    );
    assert_clip_within_window(&msgs, tb - SEC, tb + SEC);
    assert!(
        extractor.is_running(),
        "localized damage must not take the extractor down"
    );
}
