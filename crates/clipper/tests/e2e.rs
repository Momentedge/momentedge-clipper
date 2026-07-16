//! Live ROS2 end-to-end tests for the continuous extractor: a real
//! `ros2 bag record` (the production recording invocation, driven directly
//! by the harness), real CLI-published triggers, and real `Recorded`
//! announcements, against the deployed storage profiles.
//!
//! Gated on `CLIPPER_E2E` (see [`harness::require_e2e`]); cargo-nextest is
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

/// Read every announced segment and concatenate their `(topic, log_time)` pairs
/// in announcement order — the window's full content across a multi-file cut. A
/// window straddling a rollover is published as one segment per source file, so
/// the recovered window is the union of the segments.
fn read_all(recorded: &Recorded) -> Vec<(String, u64)> {
    recorded
        .filenames
        .iter()
        .flat_map(|f| read_clip(Path::new(f)))
        .collect()
}

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
    // ros+log (the default) anchors the window on clipper's own subscription
    // instant and rejects a non-zero trigger_time, so publish zero and read the
    // resolved anchor back out of the announced clip name.
    let fired_ns = now_ns();
    let (preroll, postroll) = (2 * SEC, 3 * SEC);
    env.fire_trigger("e2e-clip", preroll, postroll);

    if stop_recorder_to_flush {
        // The real window end is the subscription-instant anchor plus postroll —
        // a second or so past `fired_ns + postroll` — so stop with extra margin.
        let end_ns = fired_ns + postroll;
        let now = now_ns();
        if end_ns > now {
            std::thread::sleep(Duration::from_nanos(end_ns - now) + Duration::from_secs(2));
        }
        recorder.stop(libc::SIGINT, Duration::from_secs(30));
    }

    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(grace_secs + 40));

    // The announcement echoes the trigger and names the clip on the resolved
    // anchor (clipper's subscription instant, encoded in the filename).
    assert_eq!(recorded.name, "e2e-clip");
    let anchor = anchor_from_clip(Path::new(recorded.only()));
    let expected = env.out_dir().join(format!("{anchor}_e2e-clip.mcap"));
    assert_eq!(
        Path::new(recorded.only()),
        expected,
        "announced filename must be <out_dir>/<anchor_ns>_<name>.mcap"
    );

    // Final-path visibility: the announced file already exists, is a
    // complete MCAP (read_clip parses through the footer), holds only
    // in-window messages, and includes the source topic.
    let msgs = read_clip(Path::new(recorded.only()));
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    let (ws, we) = announced_window(&recorded, preroll, postroll);
    assert_clip_within_window(&msgs, ws, we);
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

/// The ROS interface on `--time-source publish` anchors the window on the
/// trigger's own `trigger_time` — the one cell of the matrix that reads it — so a
/// publisher can request a clip around an instant in the recent past. The clip's
/// name carries exactly that anchor, and the cut holds the recorded data around
/// it. (The other three cells reject a non-zero `trigger_time`; here it is the
/// anchor, so it is accepted.)
#[rstest]
fn trigger_produces_clip_ros_publish_anchors_on_trigger_time() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor_src(30, "publish");

    // Lay down several seconds of data, then request a window anchored a few
    // seconds in the past — entirely over data already on disk.
    std::thread::sleep(Duration::from_secs(5));

    let mut listener = env.start_recorded_listener("ros-pub");
    let anchor = now_ns() - 3 * SEC;
    let (preroll, postroll) = (2 * SEC, 2 * SEC);
    env.fire_trigger_stamped("ros-pub", anchor, preroll, postroll);

    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert_eq!(recorded.name, "ros-pub");
    // ros+publish anchors on the payload trigger_time, so the clip name carries
    // exactly the requested anchor — not clipper's subscription instant.
    assert_eq!(
        anchor_from_clip(Path::new(recorded.only())),
        anchor,
        "the clip name must carry the requested trigger_time as its anchor"
    );
    let msgs = read_clip(Path::new(recorded.only()));
    assert!(
        !msgs.is_empty(),
        "the past-anchored window must hold recorded data"
    );
    // The window selects on publish_time; read_clip reports log_time (which
    // rosbag2 stamps a transport hop later), so a tight log-time window
    // assertion would race that gap — the anchor-name check above is the
    // publish-domain proof.
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip"
    );
    env.assert_capturing_drained();
    assert!(extractor.is_running(), "the extractor must outlive the cut");
}

/// The MCAP interface end to end (clipper-535): a ROS-published `Trigger` is
/// captured into the continuous recording, and clipper — running ROS-free on
/// `--interface mcap` — reads it back out of the MCAP, decodes it (CDR, as
/// rosbag2 writes it), and cuts the clip. No `Recorded` is published; the clip's
/// appearance in `out_dir` is the completion signal. The recorder, not clipper,
/// subscribes to the trigger topic — clipper learns of the trigger only from the
/// file it tails.
#[rstest]
fn mcap_interface_reads_trigger_from_the_recording() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // `--all` records every topic, the trigger topic included, into one unchunked
    // write-through bag (fastwrite), so the trigger lands on disk at once and the
    // tail's tap lifts it within a scan poll.
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor_mcap(15);

    // Lay down at least a preroll's worth of data before triggering.
    std::thread::sleep(Duration::from_secs(3));

    let (preroll, postroll) = (2 * SEC, 3 * SEC);
    env.fire_trigger_into_bag("mcap-clip", preroll, postroll);

    // No Recorded on the mcap interface — wait for the clip itself to appear in
    // out_dir. The MCAP interface anchors the window on the trigger record's own
    // log_time (the default --time-source), not the publisher's trigger_time, so
    // the clip's `<anchor_ns>_<name>.mcap` name carries the record's log_time
    // (which sits a hair after `trigger_ns`); locate it by name suffix.
    let clip = env.wait_for_clip_matching("_mcap-clip.mcap", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);

    // The clip is a complete MCAP, holds only in-window data, and includes the
    // source topic — clipper cut the exact window the in-bag trigger asked for.
    let msgs = read_clip(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, anchor - preroll, anchor + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip"
    );
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "the ROS-free extractor must outlive the cut"
    );
}

/// The MCAP interface over a chunked recording (clipper-535): with the deployed
/// `zstd_fast` profile the trigger lands inside a chunk that is only visible once
/// flushed, so the test stops the recorder after the window to flush the footer.
/// clipper — ROS-free on `--interface mcap` — then reads the trigger out of the
/// chunk, decodes it (CDR), and cuts the clip. Exercises the chunk-interior tap
/// path end to end, the path a write-through bag never takes.
#[rstest]
fn mcap_interface_reads_a_chunk_interior_trigger() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("zstd_fast", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor_mcap(15);

    // Lay down at least a preroll's worth of data before triggering.
    std::thread::sleep(Duration::from_secs(3));

    let fired_ns = now_ns();
    let (preroll, postroll) = (2 * SEC, 3 * SEC);
    // Publish without the inline receipt wait — the chunk holding the trigger is
    // not on disk yet, so clipper cannot read it until the flush below.
    env.publish_trigger_into_bag("mcap-chunk", preroll, postroll);

    // Stop the recorder after the window so its footer flushes; clipper then
    // reads the trigger (and the window data) out of the now-complete file and
    // the `ended` recording releases the coverage wait deterministically. The
    // real window end is the record's log_time anchor plus postroll — a little
    // past `fired_ns + postroll` — so stop with extra margin.
    let end_ns = fired_ns + postroll;
    let now = now_ns();
    if end_ns > now {
        std::thread::sleep(Duration::from_nanos(end_ns - now) + Duration::from_secs(2));
    }
    recorder.stop(libc::SIGINT, Duration::from_secs(30));

    // No Recorded on the mcap interface — wait for the clip itself to appear.
    // The anchor is the trigger record's own log_time (default --time-source),
    // encoded in the clip name; locate the clip by name suffix.
    let clip = env.wait_for_clip_matching("_mcap-chunk.mcap", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);

    let msgs = read_clip(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, anchor - preroll, anchor + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip"
    );
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "the ROS-free extractor must outlive the cut"
    );
}

/// `--time-source` selects the clip window's clock domain end to end, ROS-free.
/// A synthetic recording (written with the mcap crate — no `ros2 bag record`)
/// carries source messages whose `publish_time` runs 3 s ahead of their
/// `log_time`, plus a `json` trigger anchored at `now` on both its stamps, so the
/// window is `[now − 2 s, now + 2 s]` either way. Windowed on `log` the clip
/// holds the messages whose `log_time` is in the window; on `publish` a different
/// set — the ones whose `publish_time` is in the window. clipper reads the json
/// trigger out of the file it tails; no ros2 stack runs.
///
/// The `_secs` cases carry the per-domain expected source-message offsets from
/// the trigger (in seconds): on `log`, `log_time` in `[−2, +2]`; on `publish`,
/// `publish_time` (`log + 3 s`) in `[−2, +2]`, i.e. `log_time` in `[−5, −1]`.
#[rstest]
#[case::log("log", &[-2, -1, 0, 1, 2])]
#[case::publish("publish", &[-4, -3, -2, -1])]
fn time_source_selects_the_window_clock_domain(
    #[case] source: &str,
    #[case] expected_offsets: &[i64],
) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // An empty record dir at startup, so the tail discovers the synthetic file as
    // a live new recording — the trigger tap fires only for recordings indexed
    // live — then reads the trigger out of it.
    std::fs::create_dir_all(env.record_dir()).expect("creating the record dir");
    let mut extractor = env.start_extractor_mcap_src(15, source);

    // Timestamps sit near `now` so retention (floor = now − watch) never ages the
    // recording out. The trigger anchors at `now` on both stamps; source messages
    // straddle it at log_time = now + k s for k in −4..=3, publish = log + 3 s.
    let now = now_ns() as i64;
    let at = |k: i64| (now + k * SEC as i64) as u64;
    let src: Vec<(u64, u64)> = (-4..=3).map(|k| (at(k), at(k) + 3 * SEC)).collect();
    let staged = env.record_dir().join(".synthetic.tmp");
    let recording = env.record_dir().join("synthetic_0.mcap");
    write_time_source_recording(
        &staged,
        SRC_TOPIC,
        &src,
        "ts",
        at(0),
        at(0),
        2 * SEC,
        2 * SEC,
    );
    // Rename so the tail only ever sees a complete file (its `.tmp` extension
    // also keeps discovery from picking it up mid-write).
    std::fs::rename(&staged, &recording).expect("publishing the synthetic recording");

    let clip = env.wait_for_clip_matching("_ts.mcap", Duration::from_secs(60));
    let mut got: Vec<u64> = read_clip(&clip)
        .into_iter()
        .filter(|(topic, _)| topic == SRC_TOPIC)
        .map(|(_, log_time)| log_time)
        .collect();
    got.sort_unstable();
    got.dedup();
    let expected: Vec<u64> = expected_offsets.iter().map(|&k| at(k)).collect();
    assert_eq!(
        got, expected,
        "--time-source {source} selected the wrong source messages"
    );
    assert!(
        extractor.is_running(),
        "the ROS-free extractor must outlive the cut"
    );
}

/// Live capture-time windowing end to end (clipper-7jg): the momentedge
/// `custom-mcap-writer` appends a growing, unchunked recording while clipper
/// tails it `--interface mcap`, and the trigger clipper lifts back out of that
/// file drives the cut. Every data message's `publish_time` trails its
/// `log_time` by the writer's 3 s `--publish-offset-ms`, and the trigger's
/// ±2 s window is anchored on the trigger record's own stamp per `--time-source`:
/// its `log_time` under `log`, its `publish_time` (3 s earlier) under `publish`.
/// The 3 s offset dwarfs the 2 s window half-width, so the two domains window
/// provably differently over the same data — under `publish` every clip
/// message's `publish_time` is in the window while some message's `log_time` is
/// not, and symmetrically under `log`. This is the live-writer sibling of
/// `time_source_selects_the_window_clock_domain` (which windows a synthetic
/// pre-written file); both run ROS-free at runtime — no ros2 stack, clipper
/// reads the `json` trigger straight out of the file it tails.
#[rstest]
#[case::log("log")]
#[case::publish("publish")]
fn live_writer_capture_time_windowing(#[case] time_source: &str) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // An empty record dir at startup so the tail discovers the writer's growing
    // file as a live new recording — the trigger tap fires only for recordings
    // indexed live — then lifts the json trigger out of it.
    std::fs::create_dir_all(env.record_dir()).expect("creating the record dir");
    let mut extractor = env.start_extractor_mcap_src(6, time_source);

    // The writer appends rec_0.mcap for 5 s: 50 Hz data whose publish_time
    // trails log_time by 3 s, and one json trigger 1.5 s in. The 3 s offset is
    // comfortably larger than the ±2 s window, so the log and publish domains
    // select provably different windows over the same messages. clipper cuts as
    // the file grows under it; grace (6 s) is a backstop the natural coverage
    // (reached ~3.5 s in, well inside the 5 s run) clears first.
    let (offset_ns, half_window_ns) = (3 * SEC, 2 * SEC);
    let _writer = env.start_writer("rec_0.mcap", 5.0, 1500, offset_ns / 1_000_000);

    // No Recorded on the mcap interface — wait for the clip itself. Its name
    // carries the resolved anchor (the trigger record's log_time or publish_time
    // per --time-source); the writer names its trigger "custom-mcap-writer-example".
    let clip =
        env.wait_for_clip_matching("_custom-mcap-writer-example.mcap", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);
    let (ws, we) = (anchor - half_window_ns, anchor + half_window_ns);

    // Read both stamps per message: the discriminator is which one the window
    // was applied to. The clip must be a complete MCAP holding captured data.
    let msgs = read_clip_stamps(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the windowed data");
    let data: Vec<&(String, u64, u64)> = msgs
        .iter()
        .filter(|(topic, _, _)| topic != TRIGGER_TOPIC)
        .collect();
    assert!(
        !data.is_empty(),
        "the clip must hold captured data messages"
    );

    // `selected` is the stamp --time-source windowed on; `other` is the
    // contrasting stamp, offset 3 s away and so out of the same numeric window
    // for the data straddling the anchor. Tuple layout: (topic, log_time, publish_time).
    let windowed_on_publish = time_source == "publish";
    let selected = |m: &(String, u64, u64)| if windowed_on_publish { m.2 } else { m.1 };
    let other = |m: &(String, u64, u64)| if windowed_on_publish { m.1 } else { m.2 };

    // Every message the cut kept lies inside the window on the SELECTED stamp —
    // clipper windowed on the right clock domain (a cut on the wrong stamp would
    // leak the offset-shifted messages out of this bound).
    for m in &msgs {
        let s = selected(m);
        assert!(
            (ws..=we).contains(&s),
            "--time-source {time_source}: message on {} at selected stamp {s} \
             outside window [{ws}, {we}]",
            m.0,
        );
    }
    // ... and the 3 s offset put real captured data outside that same numeric
    // window on the OTHER stamp — proof the two domains genuinely differ (offset
    // ≫ window slack), not merely relabel the same set.
    assert!(
        data.iter().any(|m| !(ws..=we).contains(&other(m))),
        "--time-source {time_source}: no data message's contrasting stamp fell \
         outside the window [{ws}, {we}] — the log/publish domains must differ by \
         the 3 s offset, stamps {:?}",
        data.iter()
            .map(|m| (other(m), selected(m)))
            .collect::<Vec<_>>(),
    );

    assert!(
        extractor.is_running(),
        "the ROS-free extractor must outlive the cut"
    );
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
    env.fire_trigger("restart-1", 2 * SEC, 2 * SEC);
    let r1 = wait_for_recorded(&mut listener1, Duration::from_secs(60));
    assert!(!read_clip(Path::new(r1.only())).is_empty());

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
    env.fire_trigger("restart-2", 2 * SEC, 2 * SEC);
    let r2 = wait_for_recorded(&mut listener2, Duration::from_secs(60));
    assert_eq!(r2.name, "restart-2");
    let msgs = read_clip(Path::new(r2.only()));
    assert!(!msgs.is_empty(), "the post-restart clip must hold data");
    let (ws, we) = announced_window(&r2, 2 * SEC, 2 * SEC);
    assert_clip_within_window(&msgs, ws, we);
    assert!(msgs.iter().any(|(topic, _)| topic == SRC_TOPIC));
    env.assert_capturing_drained();
    assert!(extractor.is_running());
}

/// The recorder is restarted inside an open trigger window: the tail keeps the
/// closing recording in its collection (its data was indexed live, the open
/// file handle keeps it readable) and indexes the replacement, so the clip
/// recovers data from **both** sides of the boundary — one segment per source
/// file (beads clipper-gl2). The announcement still goes out and the recovered
/// window spans the restart.
///
/// The cases vary when the recording file is deleted relative to the trigger
/// and the relaunch (the record script's wipe deletes it at relaunch anyway):
/// a clean stop+start inside the window; an explicit deletion inside the
/// window before the restart; a deletion before the trigger even fires, with
/// the restart landing inside the window (the tail is idle re-discovering
/// when the trigger arrives). One axis with three points rather than two
/// booleans: deleting both before the trigger and again mid-window is no fourth
/// case — the file is already gone.
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
fn recorder_restart_inside_the_window_recovers_across_the_boundary(#[case] deletion: Deletion) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let mut recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    std::thread::sleep(Duration::from_secs(3));

    // Deleted before the trigger: the recorder keeps appending to the unlinked
    // inode, but the tail notices its file's inode vanish, ends the recording
    // into the collection (the open handle keeps it readable), and idles with no
    // current; the coverage high-water freezes until the relaunch.
    if deletion == Deletion::BeforeTrigger {
        env.delete_recording();
        std::thread::sleep(Duration::from_secs(1));
    }

    let mut listener = env.start_recorded_listener("restart-inside");
    let trigger_ns = now_ns();
    // The preroll reaches back into the closing recording's data — far enough to
    // clear the `BeforeTrigger` case's delete→trigger gap (the deletion, a 1 s
    // settle, and the listener's 2 s head start), so the retained closing
    // recording always holds in-window data. The postroll outlasts the restart
    // sequence with data time to spare; the precondition after the relaunch
    // checks that it did.
    let (preroll, postroll) = (8 * SEC, 15 * SEC);
    env.fire_trigger("restart-inside", preroll, postroll);

    // Two seconds of the window lie down before the restart; that data is the
    // closing recording's part the cut recovers across the boundary.
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
    // The window straddles the restart, so it is recovered across the boundary:
    // the closing recording's pre-restart data and the replacement's post-restart
    // data both land in the clip — read every announced segment.
    let msgs = read_all(&recorded);
    assert!(
        !msgs.is_empty(),
        "the recovered window must hold data from both sides of the restart"
    );
    let (ws, we) = announced_window(&recorded, preroll, postroll);
    assert_clip_within_window(&msgs, ws, we);
    assert!(
        msgs.iter().any(|(_, log_time)| *log_time < restart_ns),
        "the closing recording's pre-restart data must be recovered: no stamp \
         before the restart at {restart_ns}, stamps {:?}",
        msgs.iter().map(|(_, t)| t).collect::<Vec<_>>(),
    );
    assert!(
        msgs.iter().any(|(_, log_time)| *log_time >= restart_ns),
        "the replacement recording's post-restart data must be in the clip"
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
    env.fire_trigger("mid-kill", SEC, 8 * SEC);

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
    let msgs = read_clip(Path::new(r.only()));
    assert!(
        !msgs.is_empty(),
        "data recorded before the kill lies in the window"
    );
    let (ws, we) = announced_window(&r, SEC, 8 * SEC);
    assert_clip_within_window(&msgs, ws, we);
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
    env.fire_trigger("deleted", preroll, postroll);

    if !delete_before_the_trigger {
        std::thread::sleep(Duration::from_secs(2));
        env.delete_recording();
    }

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the frozen coverage must have forced a grace-timeout cut"
    );
    // The tail notices its own file's inode vanish and ends the recording,
    // keeping it in the collection so the grace cut still reads the data it
    // scanned before the deletion (through the open handle).
    extractor.expect_log("inode vanished", Duration::from_secs(60));
    // rosbag2 rolls over into fresh split files after its active recording is
    // deleted; the tail indexes each and the grace cut recovers every segment
    // overlapping the window, so the clip may come back as several files. The
    // data scanned before the deletion is among them.
    let msgs = read_all(&r);
    assert!(
        !msgs.is_empty(),
        "the data scanned before the deletion lies in the window"
    );
    let (ws, we) = announced_window(&r, preroll, postroll);
    assert_clip_within_window(&msgs, ws, we);
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "a deleted recording must not take the extractor down"
    );
}

/// The recording is deleted inside the window and the recorder only comes back
/// after the window has ended: the handler is still in its grace wait when the
/// replacement's first messages — stamped past the window end — release the
/// coverage wait. The closing recording stays in the collection (its data was
/// indexed live, the open handle keeps the unlinked inode readable), so the cut
/// recovers the window's data from it even though the file is gone from disk and
/// the replacement holds nothing inside the window.
#[rstest]
fn restart_after_the_window_ended_recovers_the_closing_recording() {
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
    // `t` is the pre-publish instant, a conservative floor for the resolved
    // anchor (clipper's subscription instant, a second or so later): the
    // positive control and the end-of-window sleep below use it, and the clip's
    // window is read back from its announced name.
    let t = now_ns();
    let (preroll, postroll) = (2 * SEC, 4 * SEC);
    env.fire_trigger("post-window", preroll, postroll);

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
    // window end but well inside the grace. The extra margin clears the gap
    // between `t` and the later resolved anchor (so the real window end has
    // passed before the restart).
    let end_ns = t + postroll;
    let now = now_ns();
    if end_ns > now {
        std::thread::sleep(Duration::from_nanos(end_ns - now) + Duration::from_secs(3));
    }
    let (_recorder2, _) = env.restart_recorder(&mut recorder, &extractor, "fastwrite", 0);

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    let msgs = read_all(&r);
    assert!(
        !msgs.is_empty(),
        "the closing recording's window data must be recovered from the \
         retained index, even though the replacement holds nothing in-window"
    );
    let (ws, we) = announced_window(&r, preroll, postroll);
    assert_clip_within_window(&msgs, ws, we);
    env.assert_capturing_drained();
    assert!(
        extractor.is_running(),
        "the extractor must survive a restart after the window ended"
    );
}

/// The headline cross-file recovery: a rosbag2 bag split rolls the recording
/// over into numbered files while clipper runs, and a trigger whose window
/// straddles a split boundary recovers data from both sides — one announced
/// segment per source file, together tiling the window (beads clipper-gl2).
#[rstest]
fn window_straddling_an_in_run_split_recovers_both_sides() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // Roll the bag over every 3 s, keeping each finished split on disk, so a
    // window a few seconds wide straddles at least one split boundary.
    let _recorder = env.start_recorder_split("fastwrite", 0, 3);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(30);
    // Let at least one rollover happen so the tail has indexed two recordings.
    std::thread::sleep(Duration::from_secs(7));

    let mut listener = env.start_recorded_listener("straddle");
    // ±4 s spans at least one 3 s split boundary on each side of the anchor.
    let (preroll, postroll) = (4 * SEC, 4 * SEC);
    env.fire_trigger("straddle", preroll, postroll);

    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert_eq!(recorded.name, "straddle");
    assert!(
        recorded.filenames.len() >= 2,
        "a window straddling a split must recover one segment per source file, \
         got {:?}",
        recorded.filenames,
    );
    // Every segment is published under the `<anchor_ns>_<name>_NN.mcap` naming
    // (the anchor is clipper's subscription instant, encoded in the name) and is
    // a complete, in-window MCAP.
    let anchor = anchor_from_clip(Path::new(&recorded.filenames[0]));
    for f in &recorded.filenames {
        let name = Path::new(f)
            .file_name()
            .expect("segment has a file name")
            .to_string_lossy()
            .into_owned();
        assert!(
            name.starts_with(&format!("{anchor}_straddle_")),
            "segment {name} must carry the <anchor_ns>_<name>_NN naming"
        );
    }
    let msgs = read_all(&recorded);
    assert!(!msgs.is_empty(), "the recovered window must hold data");
    let (ws, we) = announced_window(&recorded, preroll, postroll);
    assert_clip_within_window(&msgs, ws, we);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the recovered window"
    );
    // The segments must tile the window, not overlap: each source recording is
    // indexed exactly once, so no source message appears in two segments. A
    // duplicated `(topic, log_time)` would mean the tail re-indexed a recording
    // (the phantom-duplicate failure the identity-based discovery prevents). The
    // 20 Hz source guarantees distinct stamps, so any repeat is a regression.
    let mut src_stamps: Vec<u64> = msgs
        .iter()
        .filter(|(topic, _)| topic == SRC_TOPIC)
        .map(|(_, t)| *t)
        .collect();
    let total = src_stamps.len();
    src_stamps.sort_unstable();
    src_stamps.dedup();
    assert_eq!(
        src_stamps.len(),
        total,
        "recovered segments must not overlap — a duplicated source stamp means \
         a recording was indexed more than once"
    );
    env.assert_capturing_drained();
    assert!(extractor.is_running(), "the extractor must outlive the cut");
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
    // Stop the source, then fire a trigger whose window extends past the
    // last data: coverage can never reach the window end.
    source.stop(libc::SIGTERM, Duration::from_secs(10));
    env.fire_trigger("quiet", 2 * SEC, 6 * SEC);

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the cut must have come from the grace timeout"
    );
    let msgs = read_clip(Path::new(r.only()));
    assert!(
        !msgs.is_empty(),
        "the preroll data recorded before the quiet period lies in the window"
    );
    let (ws, we) = announced_window(&r, 2 * SEC, 6 * SEC);
    assert_clip_within_window(&msgs, ws, we);
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
    if !extractor.is_running() {
        fail_fast(&mut extractor);
        return;
    }
    env.fire_trigger("over-damage", 60 * SEC, SEC);
    if let Some(a) = try_wait_for_recorded(&mut listener_a, Duration::from_secs(30)) {
        // Whatever the damage did, an announced file is a complete MCAP.
        let msgs = read_clip(Path::new(a.only()));
        let (ws, we) = announced_window(&a, 60 * SEC, SEC);
        assert_clip_within_window(&msgs, ws, we);
    }
    if !extractor.is_running() {
        fail_fast(&mut extractor);
        return;
    }

    // Health proof: a fresh window past the damage must still announce.
    std::thread::sleep(Duration::from_secs(2));
    let mut listener_b = env.start_recorded_listener("post-damage");
    env.fire_trigger("post-damage", SEC, SEC);
    let b = wait_for_recorded(&mut listener_b, Duration::from_secs(60));
    let msgs = read_clip(Path::new(b.only()));
    assert!(
        !msgs.is_empty(),
        "a window over undamaged data must still produce a full clip"
    );
    let (ws, we) = announced_window(&b, SEC, SEC);
    assert_clip_within_window(&msgs, ws, we);
    assert!(
        extractor.is_running(),
        "localized damage must not take the extractor down"
    );
}
