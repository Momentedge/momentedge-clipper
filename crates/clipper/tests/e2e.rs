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

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::assert_is_empty,
    clippy::cast_possible_wrap,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::too_many_arguments,
    clippy::excessive_nesting,
    reason = "a failed unwrap, a panicking index or a wrapped cast is a failing \
              test, and `assert!(x.is_empty())` names the claim better than the \
              empty-array `assert_eq!` the lint asks for; the harness brings a \
              whole process stack up per test, so its bring-up takes the \
              arguments and the nesting that stack has"
)]

mod harness;

use std::path::{Path, PathBuf};
use std::time::Duration;

use harness::*;
use rstest::rstest;

const SRC_TOPIC: &str = "/e2e/chatter";
const SRC_RATE: u32 = 20;
const SEC: u64 = 1_000_000_000;

/// The status a fatal fault ends the recorder with, as
/// [the operating guide](../../../docs/operating.md) promises a supervisor.
///
/// Asserted as an exact code rather than "non-zero": a process killed by a
/// signal is non-zero too, so `!success()` would accept a death that says
/// nothing about whether clipper decided to stop.
const FATAL_EXIT_CODE: i32 = 1;

/// The status an orderly stop ends with — SIGINT and SIGTERM alike.
const CLEAN_EXIT_CODE: i32 = 0;

/// Read every announced clip and concatenate its `(topic, log_time)` pairs —
/// the window's full content. A `Recorded` names one directory per clip, and a
/// clip holds one file per source recording it was cut from, so the recovered
/// window is the union of them all.
fn read_all(recorded: &Recorded) -> Vec<(String, u64)> {
    recorded
        .filenames
        .iter()
        .flat_map(|f| read_clip_dir(Path::new(f)))
        .collect()
}

/// The full happy path per storage profile: real append/flush, trigger
/// publication, `Recorded` semantics, and final-path visibility.
///
/// The fastwrite case covers an unchunked, write-through recording — the shape
/// the tail reads while it grows, and the one
/// [`examples/continuous`](../../../examples/continuous/README.md) selects; the
/// zstd_fast case covers a chunked profile, where on-disk visibility lags by a
/// chunk fill — the test stops the recorder cleanly after the window so the
/// flushed footer (`ended`) releases the coverage wait deterministically.
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

    // The announcement echoes the trigger and names the clip by its id, which
    // opens with the resolved anchor (clipper's subscription instant) and
    // carries none of the trigger's text.
    assert_eq!(recorded.name, "e2e-clip");
    let clip = Path::new(recorded.only());
    let anchor = anchor_from_clip(clip);
    assert_eq!(
        clip.parent(),
        Some(env.out_dir().as_path()),
        "the announced clip is in out_dir: {}",
        clip.display()
    );
    let name = clip
        .file_name()
        .expect("the clip has a name")
        .to_string_lossy();
    assert!(
        is_clip_name(&name, anchor),
        "the announced clip must be <out_dir>/<anchor_ns>_<hash>, got {name}"
    );
    assert!(
        !name.contains("e2e-clip"),
        "no trigger text reaches the clip's name: {name}"
    );

    // Final-path visibility: the announced file already exists, is a
    // complete MCAP (read_clip parses through the footer), holds only
    // in-window messages, and includes the source topic.
    let msgs = read_clip_dir(Path::new(recorded.only()));
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
    assert_clip_metadata(Path::new(recorded.only()), "tail", preroll, postroll);
    env.assert_out_dir_holds_only_clips();
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
    let msgs = read_clip_dir(Path::new(recorded.only()));
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
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running(), "the extractor must outlive the cut");
}

/// The MCAP interface end to end (clipper-535): a ROS-published `Trigger` is
/// captured into the continuous recording, and clipper — running ROS-free on
/// `--trigger-source mcap` — reads it back out of the MCAP, decodes it (CDR, as
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
    // the clip's `<anchor_ns>_<hash>.mcap` name opens with the record's log_time
    // (which sits a hair after `trigger_ns`); locate it by what it says it is.
    let clip = env.wait_for_clip_named("mcap-clip", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);

    // The clip is a complete MCAP, holds only in-window data, and includes the
    // source topic — clipper cut the exact window the in-bag trigger asked for.
    let msgs = read_clip_dir(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, anchor - preroll, anchor + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip"
    );
    // The manifest is written by the shared cut, so the ROS-free interface's
    // clips carry the same record the ros interface's do.
    assert_clip_metadata(&clip, "tail", preroll, postroll);
    env.assert_out_dir_holds_only_clips();
    assert!(
        extractor.is_running(),
        "the ROS-free extractor must outlive the cut"
    );
}

/// The MCAP interface over a chunked recording (clipper-535): with the deployed
/// `zstd_fast` profile the trigger lands inside a chunk that is only visible once
/// flushed, so the test stops the recorder after the window to flush the footer.
/// clipper — ROS-free on `--trigger-source mcap` — then reads the trigger out of the
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
    let clip = env.wait_for_clip_named("mcap-chunk", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);

    let msgs = read_clip_dir(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, anchor - preroll, anchor + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == SRC_TOPIC),
        "the source topic must be in the clip"
    );
    env.assert_out_dir_holds_only_clips();
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

    let clip = env.wait_for_clip_named("ts", Duration::from_secs(60));
    let mut got: Vec<u64> = read_clip_dir(&clip)
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
/// tails it `--trigger-source mcap`, and the trigger clipper lifts back out of that
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
    let clip = env.wait_for_clip_named("custom-mcap-writer-example", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);
    let (ws, we) = (anchor - half_window_ns, anchor + half_window_ns);

    // Read both stamps per message: the discriminator is which one the window
    // was applied to. The clip must be a complete MCAP holding captured data.
    let msgs = read_clip_dir_stamps(&clip);
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

/// A copper (cu29) app produces the Recording, clipper cuts the clip (clipper-a6q).
/// The `examples/cu-mcap-record` binary — a copper `CuSinkTask` — appends a
/// growing, unchunked, epoch-stamped Recording while clipper tails it
/// `--trigger-source mcap`, entirely ROS-free at runtime: no ros2 stack, no
/// `ros2 bag record`. The copper app writes its own in-Recording `json`
/// `Trigger` (`periodic-1`) on its first iteration with a fixed ±3 s window, and
/// clipper lifts that Trigger back out of the file it tails and cuts the clip;
/// the clip's appearance in `out_dir` is the only completion signal. This is the
/// live copper-Producer sibling of `live_writer_capture_time_windowing` (which
/// drives the plain-Rust `custom-mcap-writer`): it proves a copper-rs robot with
/// no ROS surface reaches clipper through the Recording alone.
///
/// The binary is provisioned by [`cu_mcap_record_bin`] (a workspace member built
/// beside the recorder under test, prebuilt in CI). Anchored on the Trigger
/// record's own `log_time` (default `--time-source`), the window closes ~3 s
/// after the app starts, which the app's continuous ~50 Hz `/sensor` stream
/// covers naturally, well inside the wait; grace is only a backstop.
/// Assertions stay robust to scheduling — a parseable clip, in-window stamps,
/// `/sensor` present, non-empty — with no exact counts or tight latency bounds.
#[rstest]
fn copper_sink_recording_produces_clip() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // Empty record dir at startup so the tail discovers the copper app's growing
    // file as a live new Recording — the trigger tap fires only for Recordings
    // indexed live — then lifts the json Trigger out of it. Bring clipper up
    // first (it waits for its "clipper tail up" line) so it is already tailing
    // the dir when the app creates the file and writes the near-immediate
    // Trigger.
    std::fs::create_dir_all(env.record_dir()).expect("creating the record dir");
    let mut extractor = env.start_extractor_mcap(10);

    let _producer = env.start_cu_recorder();

    // No Recorded on the mcap interface — wait for the clip itself. The copper
    // app names its first Trigger "periodic-1"; the clip name carries the
    // resolved anchor (the Trigger record's own log_time under the default
    // --time-source log). Locate it by name suffix.
    let clip = env.wait_for_clip_named("periodic-1", Duration::from_secs(60));
    let anchor = anchor_from_clip(&clip);
    // The window bounds live in the producer's trigger JSON, and clipper anchors
    // on the trigger record's own stamp so that record is inside its own window
    // and copied into the clip — recover the bounds from it rather than
    // re-hardcoding, so a producer-side window change cannot silently desync this
    // assertion. Fall back to the producer's constants only if the trigger record
    // is somehow absent (examples/cu-mcap-record's TRIGGER_PREROLL_NS /
    // TRIGGER_POSTROLL_NS are the source of truth: 3 s each).
    let (preroll, postroll) = clip_trigger_window(&clip).unwrap_or((3 * SEC, 3 * SEC));

    // read_clip requires a complete summary/footer/magic, so it is also the MCAP
    // completeness check on the cut clip.
    let msgs = read_clip_dir(&clip);
    assert!(!msgs.is_empty(), "the clip must hold the recorded window");
    assert_clip_within_window(&msgs, anchor - preroll, anchor + postroll);
    assert!(
        msgs.iter().any(|(topic, _)| topic == "/sensor"),
        "the copper /sensor topic must be in the clip, got topics: {:?}",
        msgs.iter()
            .map(|(t, _)| t)
            .collect::<std::collections::HashSet<_>>(),
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
    assert!(!read_clip_dir(Path::new(r1.only())).is_empty());

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
    let msgs = read_clip_dir(Path::new(r2.only()));
    assert!(!msgs.is_empty(), "the post-restart clip must hold data");
    let (ws, we) = announced_window(&r2, 2 * SEC, 2 * SEC);
    assert_clip_within_window(&msgs, ws, we);
    assert!(msgs.iter().any(|(topic, _)| topic == SRC_TOPIC));
    env.assert_out_dir_holds_only_clips();
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

    // One clip, one file per contributing recording, and a document naming each
    // — the shape a window straddling a recorder restart takes on disk. At least
    // two, rather than exactly two: rosbag2 rolls over into further split files
    // after its active recording is deleted, so the deletion cases can contribute
    // more than the closing recording and its replacement.
    let clip = Path::new(recorded.only());
    let id = clip
        .file_name()
        .expect("the clip has a name")
        .to_string_lossy()
        .into_owned();
    let files = clip_files(clip);
    assert!(
        files.len() >= 2,
        "the window reached across the restart, so the clip holds one file per \
         contributing recording: {files:?}"
    );
    for (n, file) in files.iter().enumerate() {
        assert_eq!(
            file.file_name().unwrap_or_default().to_string_lossy(),
            format!("{id}_{n}.mcap"),
            "every file of a clip is <id>_N.mcap, numbered from 0"
        );
    }
    let metadata = clip::layout::read_document(clip).expect("reading the clip's document");
    assert_eq!(
        metadata.sources.len(),
        files.len(),
        "the document lists every file the clip holds"
    );
    for source in &metadata.sources {
        assert!(
            source.path.is_some(),
            "each file names the recording it came from"
        );
    }
    // Those names repeat, and that is the restart: the record script wipes the
    // bag directory and the replacement is written at the same path, so the
    // paths cannot tell the two recordings apart. What does is the data — the
    // first file carries what was recorded before the restart and the last what
    // was recorded after, so the files are the two sides of the boundary in
    // source order rather than an arbitrary split of one stream.
    assert!(
        read_clip(&files[0])
            .iter()
            .any(|(_, log_time)| *log_time < restart_ns),
        "the first file is the closing recording's part of the window"
    );
    assert!(
        read_clip(&files[files.len() - 1])
            .iter()
            .any(|(_, log_time)| *log_time >= restart_ns),
        "and the last is the replacement's"
    );

    env.assert_out_dir_holds_only_clips();
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
    let msgs = read_clip_dir(Path::new(r.only()));
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
    env.assert_out_dir_holds_only_clips();
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
    env.assert_out_dir_holds_only_clips();
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
    // One clip whatever the window straddled: the announcement names the
    // directory — one entry, so a subscriber opens one handle per clip however
    // many recordings the window reached across — and the two source recordings
    // are two `<id>_N.mcap` files inside it, each a complete, in-window MCAP.
    assert_eq!(
        recorded.filenames.len(),
        1,
        "a clip is announced as the one directory it is, got {:?}",
        recorded.filenames,
    );
    let clip = Path::new(recorded.only());
    let id = clip
        .file_name()
        .expect("the clip has a name")
        .to_string_lossy()
        .into_owned();
    let anchor = anchor_from_clip(clip);
    assert!(
        is_clip_name(&id, anchor),
        "the clip is named by its id: {id}"
    );
    // One file per contributing recording, however many the window reached
    // across. How many that is, this test cannot say: the anchor is clipper's
    // own subscription instant, so a window eight seconds wide over three-second
    // splits spans two boundaries or three depending on where the anchor falls.
    // The exact count belongs to the case that can choose its anchor,
    // [`a_window_straddling_a_split_keeps_one_file_per_source_recording`]; what
    // is a fact here is that the window was recovered from more than one.
    let files = clip_files(clip);
    assert!(
        files.len() >= 2,
        "a window straddling a split recovers one file per source recording: {files:?}"
    );
    for (n, file) in files.iter().enumerate() {
        assert_eq!(
            file.file_name().unwrap_or_default().to_string_lossy(),
            format!("{id}_{n}.mcap"),
            "every file of a clip is <id>_N.mcap, numbered from 0"
        );
    }
    // The document is the only place the clip says which recordings it came
    // from, and it has to name each — a consumer reading a several-file clip
    // cannot otherwise tell a straddled window from a re-read one.
    let metadata = clip::layout::read_document(clip).expect("reading the clip's document");
    let sources: Vec<&String> = metadata
        .sources
        .iter()
        .map(|s| {
            s.path
                .as_ref()
                .expect("each file names the recording it came from")
        })
        .collect();
    assert_eq!(
        sources.len(),
        files.len(),
        "the document lists every file the clip holds"
    );
    assert_eq!(
        sources
            .iter()
            .collect::<std::collections::HashSet<_>>()
            .len(),
        sources.len(),
        "and each names a different split: {sources:?}"
    );
    assert!(
        metadata.window.files_planned >= sources.len(),
        "every contributing split was planned, and possibly one that held nothing"
    );
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
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running(), "the extractor must outlive the cut");
}

/// Quiet topics: the recording's topics fall silent inside an open window, so
/// the tail's coverage high-water freezes short of the window end and only the
/// grace timeout can release the cut. The recorder stays alive throughout — no
/// footer, no rollover, no vanished inode — so a stream that stopped arriving is
/// the sole reason coverage stalls, which is what separates this case from the
/// other grace-cut tests. The recording is restricted to the source topic so no
/// ambient topic (`/rosout`) can cover the window by accident.
///
/// **The source is stopped after the trigger is received, not before.** The
/// window is anchored on clipper's own subscription instant, which trails the
/// harness's `ros2 topic pub` by a second or more, and the source's teardown
/// trails its own last recorded message by several hundred milliseconds
/// further. Going quiet first spends that whole unbounded, load-dependent sum
/// out of the preroll, and a window anchored far enough past the last message
/// holds no data at all — a legitimately empty clip the assertions below would
/// read as a lost one. Going quiet after fixes the preroll over a stream that
/// was live when the trigger arrived, and still leaves most of the postroll
/// with nothing to cover it.
#[rstest]
fn quiet_topics_grace_timeout_cut() {
    if !require_e2e() {
        return;
    }
    let (preroll, postroll) = (2 * SEC, 6 * SEC);
    let env = TestEnv::new();
    let _recorder = env.start_recorder_topics(&[SRC_TOPIC], "fastwrite", 0);
    let mut source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(5);
    // The preroll must reach back over recorded data, so wait for the recording
    // to hold a preroll's worth. The listener's head start and the trigger
    // publish only add to it — both run while the source is still going.
    env.wait_for_recording_span(Duration::from_nanos(preroll), Duration::from_secs(60));

    let mut listener = env.start_recorded_listener("quiet");
    env.fire_trigger("quiet", preroll, postroll);
    // The topics go quiet a moment into the window: the coverage high-water
    // freezes there, seconds short of the window end, and can never reach it.
    source.stop(libc::SIGTERM, Duration::from_secs(10));

    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    assert!(
        extractor.log_text().contains("still uncovered after"),
        "the cut must have come from the grace timeout"
    );
    let (ws, we) = announced_window(&r, preroll, postroll);
    let msgs = read_clip_dir(Path::new(r.only()));
    assert_clip_within_window(&msgs, ws, we);

    // What a grace cut owes its window is every message the recording holds
    // inside it — a short clip, never a lossy one. The recording has been static
    // since the source died a second into the window, ten seconds before the
    // grace timeout released the cut, so the file read here is byte for byte the
    // one the cut read; and it carries the source topic alone, so its stamps and
    // the clip's compare directly.
    let recording = env.newest_recording().expect("the recording exists");
    let mut in_window: Vec<u64> = partial_recording_stamps(&recording)
        .into_iter()
        .filter(|t| (ws..=we).contains(t))
        .collect();
    in_window.sort_unstable();
    assert!(
        !in_window.is_empty(),
        "the trigger fired while the source was still publishing, so the window \
         must lie over recorded data — an empty one means the setup failed, not \
         the cut"
    );
    let mut cut: Vec<u64> = msgs.iter().map(|(_, log_time)| *log_time).collect();
    cut.sort_unstable();
    assert_eq!(
        cut, in_window,
        "the grace cut must carry every recorded message inside the window"
    );

    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running());
}

/// A window lying entirely past the last recorded message: the empty clip that
/// is correct, cut deliberately (clipper-vfi).
///
/// The source stops and the recording falls silent; the trigger fires only once
/// that silence has outlasted the preroll, so the whole window — reach-back
/// included — sits after every message the recording holds. `plan_window` finds
/// no recording overlapping it, the grace expires on coverage that can never
/// arrive, and clipper stages, publishes and announces one empty segment. On
/// disk that clip is byte-identical to one that lost its data, so the assertions
/// here are the two things that tell them apart: the manifest's
/// `source.files_planned = 0` with `clip.short = true` — nothing covered the
/// window, as against the gap-between-splits empty (`0`/`false`) and the
/// nothing-matched empty (`>= 1`/`false`) — and the extractor's `0 msgs from 0
/// extents`, where a coverage shortfall would plan one extent and copy fewer
/// messages out of it.
///
/// **Waiting the window clear of the data before firing is sound here, where
/// waiting is wrong elsewhere in this suite.** A test that needs data *inside*
/// its window must bound the harness's own latency — the ros2 CLI startup the
/// anchor trails — which nobody can state, so those tests keep their source
/// publishing across the trigger instead. This one needs the window *empty*, and
/// every source of latency pushes the anchor further past the last recorded
/// message: a longer wait can only make the precondition more true, never less.
/// The wait is on the recording's own stamps rather than on a fixed sleep
/// because a source's last message reaches the recorder after the source process
/// is gone. Its length is the preroll plus a second — the shortest gap that puts
/// `anchor − preroll` past the last message without leaning on the anchor's lag
/// at all — and preroll, postroll and grace are the shortest values that still
/// exercise a two-sided window released by a coverage wait that ran out.
#[rstest]
fn window_past_the_last_recorded_message_cuts_an_empty_clip() {
    if !require_e2e() {
        return;
    }
    let (preroll, postroll) = (2 * SEC, 2 * SEC);
    let env = TestEnv::new();
    // Restricted to the source topic, so no ambient topic (`/rosout`, the
    // trigger itself) can put a message inside the window behind the test's
    // back — the recording must fall silent for good when the source dies.
    let _recorder = env.start_recorder_topics(&[SRC_TOPIC], "fastwrite", 0);
    let mut source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(3);
    // Record data first, so the clip is empty because the window misses the
    // recording rather than because nothing was ever recorded.
    env.wait_for_recording_span(Duration::from_nanos(preroll), Duration::from_secs(60));

    source.stop(libc::SIGTERM, Duration::from_secs(10));
    let mut listener = env.start_recorded_listener("past");
    env.wait_for_recording_quiet(Duration::from_nanos(preroll + SEC), Duration::from_secs(60));
    env.fire_trigger("past", preroll, postroll);

    // Every trigger produces a clip, this one included: it is announced, it is a
    // complete MCAP, and it holds nothing.
    let r = wait_for_recorded(&mut listener, Duration::from_secs(60));
    let clip = Path::new(r.only());
    let (ws, we) = announced_window(&r, preroll, postroll);
    let msgs = read_clip_dir(clip);
    assert!(
        msgs.is_empty(),
        "the window lies past every recorded message, so the clip holds none: {msgs:?}"
    );

    // The precondition, restated against the window the cut actually used: the
    // recording holds data, and all of it lies before the window starts.
    let recording = env.newest_recording().expect("the recording exists");
    let stamps = partial_recording_stamps(&recording);
    let last =
        stamps.iter().max().copied().unwrap_or_else(|| {
            panic!("the recording holds the data this window deliberately misses")
        });
    assert!(
        last < ws,
        "the window [{ws}, {we}] must start after the recording's last message \
         at {last} — an overlap means the wait failed, not the cut"
    );

    // Which kind of empty. `files_planned = 0` says no recording held a byte of
    // the window; `short = true` says the coverage it waited for never arrived.
    assert_clip_metadata(clip, "tail", preroll, postroll);
    let metadata = clip::layout::read_document(clip).expect("reading the clip's document");
    assert_eq!(metadata.clip.messages, 0);
    assert_eq!(
        metadata.window.files_planned, 0,
        "no recording overlapped the window"
    );
    assert!(
        metadata.clip.short,
        "the recording never covered the window end"
    );

    // The extractor's own account of the same cut: released by the grace
    // timeout, and copied from no extent at all.
    let log = extractor.log_text();
    assert!(
        log.contains("still uncovered after"),
        "the cut must have come from the grace timeout"
    );
    assert!(
        log.contains(&format!(
            "clip {} written: 0 msgs from 0 extents",
            clip_files(clip)[0].display()
        )),
        "the extractor must log this clip as copied from no extent; \
         a coverage shortfall would name one"
    );

    env.assert_out_dir_holds_only_clips();
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
    assert_eq!(
        status.code(),
        Some(FATAL_EXIT_CODE),
        "a framing fault must exit {FATAL_EXIT_CODE} for a supervisor, got {status}"
    );
    assert!(
        extractor.log_text().contains("faulted at offset"),
        "the exit must name the scan fault: see extractor log"
    );
}

/// Corrupt tail, live: a run of bytes overwritten inside one message's payload
/// while the recorder keeps appending — the damage class the cut is built to
/// absorb. The record's framing header, its channel and both its stamps survive,
/// so the tail's framing walk is indifferent to whether it had already consumed
/// those bytes, and the cut — which copies message payloads through without ever
/// decoding one — carries the damage into the clip and finishes normally.
///
/// Placing the run inside a record chosen from the recording's own framing is
/// what makes that a fact rather than a coin toss. A run dropped at an offset
/// picked without reading the framing straddles a record header whenever the
/// grid happens to fall that way — about one time in three at this suite's
/// 64-byte run and ~240-byte records — and that is a different failure
/// entirely: see [`corrupt_tail_framing_damage_live`].
#[rstest]
fn corrupt_tail_payload_damage_live() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, 50);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(10);
    env.wait_for_recording_span(Duration::from_secs(2), Duration::from_secs(60));

    let bag = env.newest_recording().expect("the recording exists");
    let records = message_records(&bag);
    assert!(
        records.len() >= 4,
        "recording too short to damage mid-file ({} messages)",
        records.len()
    );
    let (target, _) = records[records.len() / 2];
    let damage = overwrite_message_payload(&bag, &target);

    // The preroll reaches the recording's start, so the damaged record lies
    // inside the window and the clip has to account for it.
    let mut listener = env.start_recorded_listener("over-damage");
    env.fire_trigger("over-damage", 60 * SEC, SEC);
    let recorded = wait_for_recorded(&mut listener, Duration::from_secs(60));
    let clip = Path::new(recorded.only());
    // `read_clip` insists on a complete summary/footer/magic, so this is also
    // the proof that the announced file is a whole MCAP, damage and all.
    let msgs = read_clip_dir(clip);
    assert!(
        !msgs.is_empty(),
        "a window over the recording must produce a full clip"
    );
    let (ws, we) = announced_window(&recorded, 60 * SEC, SEC);
    assert_clip_within_window(&msgs, ws, we);
    assert!(
        clip_files(clip)
            .iter()
            .any(|file| clip_holds_payload(file, &damage)),
        "the damaged record is copied through, not skipped"
    );
    assert!(
        extractor.is_running(),
        "payload damage must not take the extractor down"
    );
    env.assert_out_dir_holds_only_clips();
}

/// Corrupt tail, live: a run of bytes overwritten across a record's framing
/// header, in a region the tail has already indexed. The scan never returns to
/// consumed bytes, so the recorder stays up — and the copy, walking that same
/// framing afresh, finds a length no record can have and refuses the clip
/// rather than assembling one out of bytes that changed since the scan.
///
/// The refusal is per trigger, is named in the log, and covers the whole extent
/// the damage sits in: extents close at 4 MiB, so a later window over data
/// recorded after the damage reads those bytes too and is refused with them.
/// That blast radius is the case's point — the recorder answers such a trigger
/// with the fault named, and never with a hang, a silent stop, or a clip built
/// on the changed bytes.
///
/// Which of the two faults the damage causes turns on whether the scan is past
/// it, so the test establishes that rather than assuming it: a clip cut before
/// the damage names the newest message the scan had indexed, and the damaged
/// record is taken from the first half of what the scan had therefore already
/// walked. Damage the scan has yet to reach is the offline case
/// ([`corrupt_tail_fails_fast_offline`]), where the fault is fatal instead.
///
/// Because the refusal repeats for every trigger, the recorder's answer to it is
/// an **escalation with a count**, and the rest of the case is that contract:
/// the first clip a recording's desync costs is announced in full — the
/// recording, the extent, the blast radius, and the one thing that clears it —
/// and every clip after it carries a climbing tally instead of the same line
/// again. Three facts have to hold together for that to be worth reading: the
/// announcement is exactly once, the count is per clip, and the recorder is
/// still up.
///
/// The announcement tells an operator not to restart clipper, and the last step
/// is why: restarted against the same still-growing recording, the fresh scan
/// meets those bytes *ahead* of it and dies on the scan-fault budget within
/// seconds. That is the whole reason this fault is counted rather than made
/// fatal — a fatal cut-side budget would trade a recorder that refuses some
/// clips for a supervisor loop that produces none.
#[rstest]
fn corrupt_tail_framing_damage_live() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, 50);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor(10);
    env.wait_for_recording_span(Duration::from_secs(2), Duration::from_secs(60));

    // One clean cut first, purely to establish how far the scan has read: a
    // clip can only carry a message the scan had indexed, so every record
    // ahead of that one in the file is behind the scan as well.
    env.fire_trigger("probe", 60 * SEC, SEC);
    let probe = env.wait_for_clip_named("probe", Duration::from_secs(60));
    let indexed_through = read_clip_dir(&probe)
        .iter()
        .map(|(_, log_time)| *log_time)
        .max()
        .expect("the probe clip carries the data it proves was indexed");

    let bag = env.newest_recording().expect("the recording exists");
    let records = message_records(&bag);
    let reach = records
        .iter()
        .position(|(_, log_time)| *log_time == indexed_through)
        .expect("the clip's newest message is a record of the recording");
    assert!(
        reach >= 4,
        "recording too short to damage behind the scan ({reach} indexed messages)"
    );
    let (target, _) = records[reach / 2];
    overwrite_record_header(&bag, &target);

    // A window over data recorded seconds after the damaged record — over
    // nothing the damage touched, except the extent it shares with it.
    env.fire_trigger("over-damage", SEC, SEC);
    extractor.expect_log(
        "extent framing inconsistent with the tail's scan",
        Duration::from_secs(60),
    );
    extractor.expect_log("clip 1 refused against", Duration::from_secs(60));
    let announcement = extractor.log_text();
    assert!(
        announcement.contains("changed under the tail after it was indexed"),
        "the first refusal is announced in full: see extractor log"
    );
    assert!(
        announcement.contains(&bag.display().to_string()),
        "the announcement must name the damaged recording: see extractor log"
    );
    assert!(
        extractor.is_running(),
        "a refused cut must not take the extractor down"
    );
    assert_eq!(
        env.published_clips(),
        vec![
            probe
                .file_name()
                .expect("the probe clip has a name")
                .to_string_lossy()
                .into_owned(),
        ],
        "a refused cut publishes nothing"
    );
    env.assert_out_dir_holds_only_clips();

    // A second window over the same extent: the cost is a number that climbs,
    // and the full announcement is not repeated. One line per trigger is what
    // the escalation replaces.
    env.fire_trigger("over-damage-again", SEC, SEC);
    extractor.expect_log("clip 2 refused against", Duration::from_secs(60));
    assert_eq!(
        extractor
            .log_text()
            .matches("changed under the tail after it was indexed")
            .count(),
        1,
        "the recording is announced once, not once per trigger: see extractor log"
    );
    assert!(extractor.is_running(), "still up after the second refusal");

    // What the announcement tells the operator *not* to do, tested: restarted
    // against the same damaged recording, the fresh scan meets those bytes
    // ahead of it and exits on the scan-fault budget. A recording that is not
    // being split has no successor for the startup adopt to pick up instead, so
    // a supervisor restarting this gets a loop that publishes nothing at all —
    // which is why the cut side counts where the scan side exits.
    let stopped = extractor.stop(libc::SIGINT, Duration::from_secs(30));
    assert_eq!(
        stopped.code(),
        Some(CLEAN_EXIT_CODE),
        "a requested stop is the orderly one, whatever the run met on the way, \
         got {stopped}"
    );
    let mut restarted = env.start_extractor(10);
    let status = restarted
        .wait_exit(Duration::from_secs(60))
        .unwrap_or_else(|| {
            restarted.dump_log();
            panic!("a restart must meet the damage ahead of its scan, not survive it");
        });
    assert_eq!(
        status.code(),
        Some(FATAL_EXIT_CODE),
        "the restart exits {FATAL_EXIT_CODE} for the supervisor, got {status}"
    );
    assert!(
        restarted.log_text().contains("giving up"),
        "the restart's exit must name the exhausted scan-fault budget: see extractor log"
    );
}

// ─────────────────────────────────────────────────────────────────────────────
// Cutting from a recording nobody is writing
//
// `clipper clip` is the other half of the live suite: a real clipper process
// over a real `ros2 bag record` output, observed from outside, with none of the
// waits a growing file costs. Every window's anchor is a value the test chose
// (`--trigger-time`, or the recording's own trigger record), so the clip's id —
// and therefore the directory it lands in — is known before the run starts.
// That is what lets these scenarios assert the output directory's contents
// exactly rather than by shape, and it is why the shape, skip and resume rules
// live here rather than behind the recorder's own clock.
// ─────────────────────────────────────────────────────────────────────────────

/// A recording nobody is writing any more: the bag directory, its splits in
/// recording order, and the `log_time` of every message in each.
struct Finished {
    dir: PathBuf,
    splits: Vec<PathBuf>,
    stamps: Vec<Vec<u64>>,
}

impl Finished {
    /// Read a stopped recorder's bag directory: the splits in the order
    /// `clip::bag` reads them (modification time, since `ros2 bag record`'s own
    /// `metadata.yaml` names them in that order too) and each one's stamps.
    fn of(dir: &Path) -> Self {
        let splits = clip::bag::splits(dir).expect("listing the bag directory's splits");
        assert!(
            !splits.is_empty(),
            "the recorder left no recording in {}",
            dir.display()
        );
        let stamps = splits
            .iter()
            .map(|s| finished_recording_stamps(s))
            .collect();
        Finished {
            dir: dir.to_path_buf(),
            splits,
            stamps,
        }
    }

    /// The one split of a recording that never rolled over — the single-file
    /// input `clipper clip` takes beside a bag directory.
    fn file(&self) -> &Path {
        assert_eq!(
            self.splits.len(),
            1,
            "this recording rolled over into {} splits",
            self.splits.len()
        );
        &self.splits[0]
    }

    /// Every stamp of the whole collection, ascending.
    fn all_stamps(&self) -> Vec<u64> {
        let mut all: Vec<u64> = self.stamps.iter().flatten().copied().collect();
        all.sort_unstable();
        all
    }

    /// The first and last instant the collection holds a message at.
    fn span(&self) -> (u64, u64) {
        let all = self.all_stamps();
        (
            *all.first().expect("the recording holds data"),
            *all.last().expect("the recording holds data"),
        )
    }

    /// The midpoint of split `n`'s own extent — an anchor that is inside that
    /// split and, for a split a few seconds long, comfortably clear of its ends.
    fn split_midpoint(&self, n: usize) -> u64 {
        let stamps = &self.stamps[n];
        let (first, last) = (
            *stamps.first().expect("the split holds data"),
            *stamps.last().expect("the split holds data"),
        );
        first + (last - first) / 2
    }
}

/// The storage profile a recording `clipper clip` will read has to be written
/// with.
///
/// `clipper clip` plans a window out of the recording's own summary, so it takes
/// the chunked, message-indexed shape and refuses the unchunked one by name —
/// which is why the live tests' `fastwrite` recordings, readable while they
/// grow, are exactly what the cutter cannot take. The two halves of the suite
/// therefore record differently on purpose.
const CUTTABLE_PRESET: &str = "zstd_fast";

/// Record roughly `span` of the source topic and stop cleanly, leaving a
/// finished bag.
///
/// Topic-restricted, so every message in the recording is the test's own and a
/// window over it can be reasoned about message for message: `--all` would fold
/// in `/rosout` and the trigger topic, whose timing no test controls.
/// `split_secs` rolls the bag over at that period, and the split wait is on the
/// count rather than on the clock, since a rollover happens when rosbag2 next
/// writes a message rather than when a timer fires.
///
/// **The wait for data is a plain sleep, where the live tests wait on the
/// recording's own extent.** They have to: a window they trigger has to lie over
/// bytes that are already on disk. Here nothing is triggered until the recorder
/// has stopped, and the window is then chosen from the stamps the finished
/// recording turns out to hold — so how much it recorded is read afterwards
/// rather than promised beforehand. Which is just as well, since a chunked
/// recording holds its messages inside chunks that only the close flushes.
fn finished_recording(env: &TestEnv, span: Duration, split_secs: u64, splits: usize) -> Finished {
    let mut recorder = if split_secs > 0 {
        env.start_recorder_topics_split(&[SRC_TOPIC], CUTTABLE_PRESET, 0, split_secs)
    } else {
        env.start_recorder_topics(&[SRC_TOPIC], CUTTABLE_PRESET, 0)
    };
    let mut source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    std::thread::sleep(span);
    if splits > 1 {
        wait_for_splits(env, splits, Duration::from_secs(90));
    }
    stop_and_read(env, &mut recorder, &mut source, span)
}

/// [`finished_recording`] over a source publishing volume rather than message
/// count — roughly 2 MB/s, so a few seconds of recording is tens of megabytes
/// for a window to copy.
///
/// That is what makes a cut long enough to be caught in the act: every scenario
/// about what a *half-finished* clip looks like — a kill mid-copy, an observer
/// sampling the directory — needs the copy to last longer than the poll that
/// watches it, and a copy is bounded by the bytes it moves rather than by the
/// messages it counts.
fn finished_bulk_recording(env: &TestEnv, span: Duration) -> Finished {
    let mut recorder = env.start_recorder_topics(&[SRC_TOPIC], CUTTABLE_PRESET, 0);
    let mut source = env.start_bulk_source(SRC_TOPIC, BULK_RATE, BULK_PAYLOAD);
    env.wait_for_recording(Duration::from_secs(60));
    std::thread::sleep(span);
    stop_and_read(env, &mut recorder, &mut source, span)
}

/// The rate and payload a bulk source publishes at: together about 2 MB/s, which
/// the ros2 CLI sustains and `ros2 bag record` writes through.
const BULK_RATE: u32 = 50;
const BULK_PAYLOAD: usize = 40_000;

/// The payload an ordinary source publishes: enough to be a message, little
/// enough that a window's copy is over before a poll could see it.
const SRC_PAYLOAD: usize = 200;

/// Stop a recording cleanly and read back what it holds.
///
/// The source is stopped first: a recorder stopped under a live source writes
/// its footer while messages are still arriving, and the last of them may or may
/// not be in the file the test then reads stamps off.
fn stop_and_read(
    env: &TestEnv,
    recorder: &mut Proc,
    source: &mut Proc,
    span: Duration,
) -> Finished {
    source.stop(libc::SIGTERM, Duration::from_secs(10));
    recorder.stop(libc::SIGINT, Duration::from_secs(30));
    let bag = Finished::of(&env.record_dir());
    let (first, last) = bag.span();
    assert!(
        last - first >= span.as_nanos() as u64 / 2,
        "the recorder captured only {} ns of the {span:?} asked for",
        last - first,
    );
    bag
}

/// Block until the bag directory holds at least `count` recordings.
fn wait_for_splits(env: &TestEnv, count: usize, timeout: Duration) {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        let splits = clip::bag::splits(&env.record_dir()).map_or(0, |s| s.len());
        if splits >= count {
            return;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "the recording rolled over into only {splits} of {count} splits within {timeout:?}"
        );
        std::thread::sleep(Duration::from_millis(200));
    }
}

/// One window stated on the command line, cut out of `recording` into
/// `out_dir`.
///
/// The description is the harness's fixed one so that [`clip_id_of`] and the run
/// agree on every field the clip's id hashes — which is what lets a test name
/// the directory the run is about to create.
fn run_param_clip(
    env: &TestEnv,
    tag: &str,
    recording: &Path,
    out_dir: &Path,
    window: ParamWindow,
) -> ClipRun {
    ClipRun::of(&mut spawn_param_clip(env, tag, recording, out_dir, window))
}

/// [`run_param_clip`] left running, for the scenarios whose subject is what a
/// run looks like while it is still going.
fn spawn_param_clip(
    env: &TestEnv,
    tag: &str,
    recording: &Path,
    out_dir: &Path,
    window: ParamWindow,
) -> Proc {
    env.spawn_clip(
        tag,
        recording,
        out_dir,
        &[
            "--trigger-time",
            &window.anchor_ns.to_string(),
            "--preroll",
            &window.preroll_ns.to_string(),
            "--postroll",
            &window.postroll_ns.to_string(),
            "--trigger-name",
            window.name,
            "--trigger-description",
            TRIGGER_DESCRIPTION,
        ],
    )
}

/// The window one `--trigger-source param` run cuts: the four values that,
/// with the run's fixed clock domain and description, decide both what is
/// copied and what the clip is called.
#[derive(Clone, Copy)]
struct ParamWindow {
    anchor_ns: u64,
    preroll_ns: u64,
    postroll_ns: u64,
    name: &'static str,
}

impl ParamWindow {
    /// The directory name this window's clip will have.
    fn clip_id(self) -> String {
        clip_id_of(
            self.name,
            self.anchor_ns,
            self.preroll_ns,
            self.postroll_ns,
            CLIP_MODE_TIME_SOURCE,
            "clip",
        )
    }

    /// The inclusive bounds the cut applies.
    fn bounds(self) -> (u64, u64) {
        (
            self.anchor_ns - self.preroll_ns,
            self.anchor_ns + self.postroll_ns,
        )
    }
}

/// `clipper clip`'s clock domain is fixed: the window lives on `log_time`.
const CLIP_MODE_TIME_SOURCE: clip::TimeSource = clip::TimeSource::Log;

/// T3 — a window inside one recording is one file and a document, and nothing
/// else is in the directory.
///
/// The shape every clip has, asserted entry for entry rather than by predicate:
/// `<id>/` holding `<id>_0.mcap` and `clip_metadata.yaml`, no staging file, no
/// sidecar, no second numbered file. The MCAP's own metadata is asserted the
/// same way — exactly one record, `momentedge.clip`, carrying the clip's id and
/// nothing else — which is what says the recorder's own rosbag2 metadata is not
/// copied through and that a file separated from its directory can still be
/// grouped.
///
/// Expressible against `clipper clip` because the anchor is `--trigger-time`:
/// the id, and so the directory the run must create, is known before it runs.
#[rstest]
fn a_window_inside_one_recording_is_one_file_and_a_document() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_recording(&env, Duration::from_secs(4), 0, 1);
    let window = ParamWindow {
        anchor_ns: bag.split_midpoint(0),
        preroll_ns: SEC,
        postroll_ns: SEC,
        name: "inside",
    };

    let out_dir = env.out_dir();
    let run = run_param_clip(&env, "one-file", bag.file(), &out_dir, window);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);

    let id = window.clip_id();
    assert_eq!(
        env.published_clips(),
        vec![id.clone()],
        "the run leaves one clip directory and nothing else"
    );
    let clip_dir = out_dir.join(&id);
    assert_eq!(
        entry_names(&clip_dir),
        vec![
            format!("{id}_0.mcap"),
            clip::layout::DOCUMENT_FILE.to_string(),
        ],
        "a one-file clip is exactly its file and its document"
    );

    // The document's account of the same clip, and the one metadata record its
    // file is allowed to carry.
    assert_clip_metadata(&clip_dir, "clip", window.preroll_ns, window.postroll_ns);
    let metadata = clip::layout::read_document(&clip_dir).expect("reading the document");
    assert_eq!(metadata.sources.len(), 1);
    assert_eq!(
        metadata.window.files_planned, 1,
        "one recording was planned and one contributed"
    );
    assert!(
        !metadata.clip.short,
        "the recording ran past the window end"
    );

    let file = clip_dir.join(format!("{id}_0.mcap"));
    assert_eq!(
        metadata_record_names(&file),
        vec![clip::manifest::MANIFEST_NAME.to_string()],
        "a clip's file carries clipper's record and no other"
    );
    let record = clip::manifest::read_manifest(&file)
        .expect("reading the clip's metadata record")
        .expect("the clip's file carries one");
    assert_eq!(
        record,
        std::collections::BTreeMap::from([(clip::manifest::CLIP_ID_KEY.to_string(), id.clone())]),
        "the record says which clip the file belongs to, and nothing else"
    );

    let msgs = read_clip_dir(&clip_dir);
    assert!(!msgs.is_empty(), "the window lies over recorded data");
    let (ws, we) = window.bounds();
    assert_clip_within_window(&msgs, ws, we);
    env.assert_out_dir_holds_only_clips();
}

/// T4 — a window straddling a split keeps one file per source recording, each
/// holding its own recording's messages, and the document names both sources.
///
/// The live recorder's side of this is
/// [`window_straddling_an_in_run_split_recovers_both_sides`]; what a finished
/// recording adds is exactness. The splits are on disk and unchanging, so the
/// test can partition the clip's messages against each split's own stamps and
/// assert that `_0` holds the earlier recording's and `_1` the later one's —
/// a claim a live cut, racing the recorder's rollover, cannot pin down.
#[rstest]
fn a_window_straddling_a_split_keeps_one_file_per_source_recording() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // Roll over every 2 s and take three splits, so the middle one is bounded on
    // both sides by data the window can be kept clear of.
    let bag = finished_recording(&env, Duration::from_secs(6), 2, 3);
    // The boundary between splits 0 and 1, with the window reaching half a
    // second into each — well inside two 2 s splits, so no third is touched.
    let window = ParamWindow {
        anchor_ns: bag.stamps[0].last().copied().expect("split 0 holds data"),
        preroll_ns: SEC / 2,
        postroll_ns: SEC / 2,
        name: "straddle",
    };

    let out_dir = env.out_dir();
    let run = run_param_clip(&env, "straddle", &bag.dir, &out_dir, window);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);

    let id = window.clip_id();
    let clip_dir = out_dir.join(&id);
    assert_eq!(env.published_clips(), vec![id.clone()]);
    assert_eq!(
        entry_names(&clip_dir),
        vec![
            format!("{id}_0.mcap"),
            format!("{id}_1.mcap"),
            clip::layout::DOCUMENT_FILE.to_string(),
        ],
        "one file per contributing recording, numbered from 0, plus the document"
    );

    let metadata = clip::layout::read_document(&clip_dir).expect("reading the document");
    assert_eq!(metadata.window.files_planned, 2);
    let sources: Vec<String> = metadata
        .sources
        .iter()
        .map(|s| {
            s.path
                .clone()
                .expect("a cut file names the recording it came from")
        })
        .collect();
    assert_eq!(
        sources,
        vec![
            bag.splits[0].display().to_string(),
            bag.splits[1].display().to_string(),
        ],
        "the per-file list names both source recordings, in source order"
    );

    // Each file holds its own recording's messages and no other's.
    for (n, split_stamps) in bag.stamps.iter().take(2).enumerate() {
        let cut: Vec<u64> = read_clip(&clip_dir.join(format!("{id}_{n}.mcap")))
            .into_iter()
            .map(|(_, log_time)| log_time)
            .collect();
        assert!(!cut.is_empty(), "file _{n} holds its recording's part");
        assert!(
            cut.iter().all(|stamp| split_stamps.contains(stamp)),
            "every message of _{n} comes from split {n}"
        );
    }
    let (ws, we) = window.bounds();
    assert_clip_within_window(&read_clip_dir(&clip_dir), ws, we);
    env.assert_out_dir_holds_only_clips();
}

/// T5 — a window over three recordings whose middle one holds none of its
/// messages: the two that contribute are `_0` and `_1`, renumbered by position,
/// and the document says three were planned.
///
/// **Only a synthetic collection can state this.** A recorder's splits partition
/// time, so a window reaching into the split before and the split after
/// necessarily contains everything in between; a middle recording that is read
/// and contributes nothing needs an extent that spans the window with a gap
/// inside it, which no rollover produces. The three recordings here are written
/// with the mcap crate and read by a real `clipper clip` process over the
/// directory holding them — the same shape
/// [`time_source_selects_the_window_clock_domain`] uses for the same reason.
///
/// What it pins is that a file's number is its position **after** the empty
/// ones are dropped: numbering by plan position would leave `_0` and `_2` and a
/// hole where a consumer expects `_1`.
#[rstest]
fn a_recording_that_contributes_nothing_is_dropped_and_the_rest_renumbered() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let dir = env.path("collection");
    std::fs::create_dir_all(&dir).expect("creating the collection directory");

    // The window is [t - 2 s, t + 2 s]. The recordings are written in the order
    // they are read (no `metadata.yaml`, so modification time is the order), and
    // each one's first stamp ascends with it.
    let t = now_ns();
    let at = |k: i64| (t as i64 + k * SEC as i64) as u64;
    write_plain_recording(&dir.join("collection_0.mcap"), SRC_TOPIC, &[at(-4), at(-1)]);
    // The middle one reaches across the whole window and holds nothing inside
    // it: planned, read, and contributing not one message.
    write_plain_recording(&dir.join("collection_1.mcap"), SRC_TOPIC, &[at(-3), at(3)]);
    write_plain_recording(&dir.join("collection_2.mcap"), SRC_TOPIC, &[at(1), at(4)]);

    let window = ParamWindow {
        anchor_ns: t,
        preroll_ns: 2 * SEC,
        postroll_ns: 2 * SEC,
        name: "renumbered",
    };
    let out_dir = env.out_dir();
    let run = run_param_clip(&env, "renumbered", &dir, &out_dir, window);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);

    let id = window.clip_id();
    let clip_dir = out_dir.join(&id);
    assert_eq!(
        entry_names(&clip_dir),
        vec![
            format!("{id}_0.mcap"),
            format!("{id}_1.mcap"),
            clip::layout::DOCUMENT_FILE.to_string(),
        ],
        "the contributing recordings are numbered 0 and 1, with no hole where \
         the empty one was planned"
    );

    let metadata = clip::layout::read_document(&clip_dir).expect("reading the document");
    assert_eq!(
        metadata.window.files_planned, 3,
        "all three recordings overlapped the window and were read"
    );
    assert_eq!(
        metadata.sources.len(),
        2,
        "only the two that held an in-window message are files of the clip"
    );
    let sources: Vec<String> = metadata
        .sources
        .iter()
        .map(|s| s.path.clone().expect("a cut file names its recording"))
        .collect();
    assert_eq!(
        sources,
        vec![
            dir.join("collection_0.mcap").display().to_string(),
            dir.join("collection_2.mcap").display().to_string(),
        ],
        "the per-file list names the two that contributed, and not the one that \
         did not"
    );
    assert_eq!(
        read_clip_dir(&clip_dir)
            .into_iter()
            .map(|(_, log_time)| log_time)
            .collect::<Vec<_>>(),
        vec![at(-1), at(1)],
        "the clip holds exactly the in-window messages, in time order"
    );
    env.assert_out_dir_holds_only_clips();
}

/// Which window of the four an [`an_empty_window_still_says_which_kind_of_empty_it_is`]
/// case cuts. An enum rather than a label, so the match that places each window
/// has to answer for every one of them.
#[derive(Clone, Copy, Debug)]
enum EmptyWindow {
    /// Before the first recording's first message.
    Before,
    /// Between the two recordings, over the silence separating them.
    Gap,
    /// Inside one recording, between two consecutive messages.
    NothingMatched,
    /// Past the last recording's last message.
    Past,
}

/// T6 — every trigger produces a clip, an empty window included, and the
/// document says which kind of empty it is.
///
/// On disk an empty clip is one `<id>_0.mcap` holding an empty MCAP, whatever
/// emptied it, so the four windows here are told apart only by their documents
/// ([the table](../../../docs/clip-manifest.md)):
///
/// - **before** the collection, and **in the gap between its two recordings**:
///   `files_planned = 0`, `short = false`. No recording held a byte of the
///   window, and the collection ran past its end. These are one kind, not two —
///   which is itself worth pinning, since the epic's catalogue names them
///   separately and the document cannot.
/// - **between two messages** of one recording: `files_planned = 1`,
///   `short = false`. A recording was read and nothing in it matched.
/// - **past** the collection's end: `files_planned = 0`, `short = true`.
///   Nothing covered the window, which is the one thing a clip's contents can
///   never show.
///
/// The gap is real rather than contrived: two recorder runs, seconds apart,
/// copied side by side into one directory — which is what a collection assembled
/// off a device looks like, and the only way a gap falls *between* recordings
/// rather than inside one (a recorder rolls over when it next writes, so a
/// silence never splits a file).
#[rstest]
#[case::before_the_collection(EmptyWindow::Before, 0, false)]
#[case::in_the_gap_between_recordings(EmptyWindow::Gap, 0, false)]
#[case::between_two_messages(EmptyWindow::NothingMatched, 1, false)]
#[case::past_the_collection(EmptyWindow::Past, 0, true)]
fn an_empty_window_still_says_which_kind_of_empty_it_is(
    #[case] which: EmptyWindow,
    #[case] files_planned: usize,
    #[case] short: bool,
) {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let dir = env.path("collection");
    std::fs::create_dir_all(&dir).expect("creating the collection directory");

    // Two real recordings with a real gap between them: record, copy the split
    // aside, wait, record again.
    let first = finished_recording(&env, Duration::from_secs(2), 0, 1);
    std::fs::copy(first.file(), dir.join("collection_0.mcap")).expect("copying the first");
    let (first_start, gap_start) = first.span();
    std::thread::sleep(Duration::from_secs(3));
    let second = finished_recording(&env, Duration::from_secs(2), 0, 1);
    std::fs::copy(second.file(), dir.join("collection_1.mcap")).expect("copying the second");
    let (gap_end, last) = second.span();
    assert!(
        gap_end > gap_start + SEC,
        "precondition: the two recordings must leave a gap to aim a window into"
    );

    // A window a hundredth of a second wide, so `between two messages` really
    // does fall between two of the 20 Hz source's stamps.
    let narrow = SEC / 100;
    let (anchor, preroll, postroll) = match which {
        EmptyWindow::Before => (first_start - 5 * SEC, SEC, SEC),
        EmptyWindow::Gap => (u64::midpoint(gap_start, gap_end), narrow, narrow),
        EmptyWindow::NothingMatched => {
            let stamps = &first.stamps[0];
            let (a, b) = (stamps[stamps.len() / 2], stamps[stamps.len() / 2 + 1]);
            assert!(
                b - a > 2 * narrow,
                "the source's stamps are far enough apart"
            );
            (u64::midpoint(a, b), narrow / 2, narrow / 2)
        }
        EmptyWindow::Past => (last + 5 * SEC, SEC, SEC),
    };
    let window = ParamWindow {
        anchor_ns: anchor,
        preroll_ns: preroll,
        postroll_ns: postroll,
        name: "empty",
    };

    let out_dir = env.out_dir();
    let run = run_param_clip(&env, &format!("{which:?}"), &dir, &out_dir, window);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);

    let id = window.clip_id();
    let clip_dir = out_dir.join(&id);
    assert_eq!(
        entry_names(&clip_dir),
        vec![
            format!("{id}_0.mcap"),
            clip::layout::DOCUMENT_FILE.to_string(),
        ],
        "an empty window is still one file and a document"
    );
    assert!(
        read_clip_dir(&clip_dir).is_empty(),
        "the {which:?} window holds no message"
    );

    let metadata = clip::layout::read_document(&clip_dir).expect("reading the document");
    assert_eq!(metadata.clip.messages, 0);
    assert_eq!(
        metadata.window.files_planned, files_planned,
        "how many recordings the {which:?} window was planned over"
    );
    assert_eq!(
        metadata.clip.short, short,
        "whether the collection ever reached the {which:?} window's end"
    );
    env.assert_out_dir_holds_only_clips();
}

/// T13 — `--out-dir` is created with parents, is never required to be empty, and
/// whatever was already in it is left exactly as it was.
///
/// The three facts an operator pointing a sync tool at a directory relies on,
/// asserted in one run because they are one rule: clipper adds clip directories
/// to that root and does nothing else to it. No staging directory appears
/// either — which a listing of the root proves only because a staging area would
/// be a *directory* too, so the check is that every entry is either something
/// that was already there or a clip named by its id.
#[rstest]
fn an_out_dir_is_created_with_parents_and_what_is_already_in_it_is_left_alone() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_recording(&env, Duration::from_secs(4), 0, 1);

    // A path whose parents do not exist: the run creates the whole tree.
    let out_dir = env.path("deep/er/still/clipped");
    assert!(!out_dir.exists(), "precondition: the tree is missing");
    let first = ParamWindow {
        anchor_ns: bag.split_midpoint(0),
        preroll_ns: SEC,
        postroll_ns: SEC,
        name: "parents",
    };
    let run = run_param_clip(&env, "parents", bag.file(), &out_dir, first);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);
    assert_eq!(entry_names(&out_dir), vec![first.clip_id()]);

    // Foreign entries an operator or another tool left there, of both shapes.
    std::fs::write(out_dir.join("README.txt"), b"not clipper's").expect("writing a foreign file");
    std::fs::create_dir(out_dir.join("incoming")).expect("creating a foreign directory");
    std::fs::write(out_dir.join("incoming/held"), b"nor this").expect("writing under it");
    let foreign = dir_bytes(&out_dir.join("incoming"));

    let second = ParamWindow {
        name: "alongside",
        ..first
    };
    let run = run_param_clip(&env, "alongside", bag.file(), &out_dir, second);
    assert_eq!(run.code(), CLEAN_EXIT_CODE, "log: {}", run.log);

    let mut expected = vec![
        "README.txt".to_string(),
        "incoming".to_string(),
        first.clip_id(),
        second.clip_id(),
    ];
    expected.sort();
    assert_eq!(
        entry_names(&out_dir),
        expected,
        "the root gained a clip directory and nothing else — no staging area, no \
         lock, no sidecar"
    );
    assert_eq!(
        std::fs::read(out_dir.join("README.txt")).expect("the foreign file survives"),
        b"not clipper's",
        "a file that was already there is not touched"
    );
    assert_eq!(
        dir_bytes(&out_dir.join("incoming")),
        foreign,
        "a directory that was already there is not touched"
    );
    assert_eq!(
        complete_clips_in(&out_dir),
        {
            let mut ids = vec![first.clip_id(), second.clip_id()];
            ids.sort();
            ids
        },
        "both windows are complete clips, and the foreign entries are not clips"
    );
}

/// T21 — the same recording as a single file and as the bag directory holding it
/// cuts the same clip.
///
/// Which of the two an operator passes is a property of what they have on hand,
/// never of what they get: the id, the file names, the message bytes and every
/// field of the document but the source path are identical. The exception is the
/// point — `source.path` is the recording each file was read from, and that is
/// the same file either way, so even it agrees here.
#[rstest]
fn a_single_file_and_the_bag_directory_holding_it_cut_the_same_clip() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_recording(&env, Duration::from_secs(4), 0, 1);
    let window = ParamWindow {
        anchor_ns: bag.split_midpoint(0),
        preroll_ns: SEC,
        postroll_ns: SEC,
        name: "same-shape",
    };

    let by_file = env.path("out-file");
    let by_dir = env.path("out-dir");
    let file_run = run_param_clip(&env, "by-file", bag.file(), &by_file, window);
    let dir_run = run_param_clip(&env, "by-dir", &bag.dir, &by_dir, window);
    assert_eq!(file_run.code(), CLEAN_EXIT_CODE, "log: {}", file_run.log);
    assert_eq!(dir_run.code(), CLEAN_EXIT_CODE, "log: {}", dir_run.log);

    let id = window.clip_id();
    assert_eq!(entry_names(&by_file), vec![id.clone()]);
    assert_eq!(entry_names(&by_dir), vec![id.clone()]);
    assert_eq!(
        entry_names(&by_file.join(&id)),
        entry_names(&by_dir.join(&id)),
        "the two inputs yield the same files inside the clip"
    );
    assert_eq!(
        read_clip_dir(&by_file.join(&id)),
        read_clip_dir(&by_dir.join(&id)),
        "the two inputs yield the same messages"
    );
    assert_eq!(
        clip::layout::read_document(&by_file.join(&id)).expect("reading the file run's document"),
        clip::layout::read_document(&by_dir.join(&id)).expect("reading the dir run's document"),
        "the two inputs yield the same document, source path included — it is \
         the same recording that was read"
    );
}

/// A finished `--all` recording carrying one trigger of its own per name, with
/// recorded data on both sides of every window.
///
/// This is the input `clipper clip --trigger-source mcap` takes: a recording
/// that states its own triggers, so a run over it cuts one clip per trigger with
/// nothing on the command line to say which windows those are. Each publish
/// costs the ros2 CLI's startup, which is also what spaces the triggers out.
///
/// **The data on both sides of every window is asserted, and that is the whole
/// value of the fixture.** A window landing past the recorded data cuts a clip
/// that is empty *and complete*, and every downstream assertion about
/// completeness, about byte equality and about which windows were skipped
/// passes over one of those exactly as it passes over a real clip. So the
/// bracketing is checked against the finished recording's own stamps before the
/// fixture hands it over: a source that began too late, or stopped too early,
/// fails here naming the fixture, rather than downstream in four tests that go
/// green having proved nothing about the copy.
///
/// **The waits around the triggers are wall clock, and here they have to be.**
/// The data a window covers must be on disk before that window's trigger is
/// published, and [`TestEnv::wait_for_recording_span`] — the instrument for
/// exactly that — reads a growing recording's top-level message records, which
/// the chunked profile `clipper clip` requires does not have: its messages sit
/// inside chunks the writer flushes by accumulated size, which at this source's
/// rate is far beyond any wait a test would make. A recording that can be read
/// while it grows is unreadable by the cutter, and the one the cutter takes
/// says nothing about itself until it is closed. What is left is to make the
/// sleeps honest: they are counted from a topic that is provably carrying
/// messages ([`TestEnv::wait_for_source_publishing`]) rather than from a CLI
/// that was merely spawned, and the postcondition checks what they were meant
/// to buy.
fn finished_recording_with_triggers(
    env: &TestEnv,
    names: &[&str],
    preroll_ns: u64,
    postroll_ns: u64,
    payload_bytes: usize,
) -> Finished {
    // `--all` so the recorder captures the trigger topic; the triggers have to
    // be in the recording for the run to find them there.
    let mut recorder = env.start_recorder(CUTTABLE_PRESET, 0);
    let mut source = env.start_bulk_source(SRC_TOPIC, SRC_RATE, payload_bytes);
    env.wait_for_recording(Duration::from_secs(60));
    env.wait_for_source_publishing(SRC_TOPIC, Duration::from_secs(60));
    // Data before the first window's preroll reaches back.
    std::thread::sleep(Duration::from_nanos(preroll_ns) + WINDOW_DATA_MARGIN);
    for name in names {
        env.publish_trigger_into_bag(name, preroll_ns, postroll_ns);
    }
    // ... and data past the last window's postroll, so no clip is short.
    std::thread::sleep(Duration::from_nanos(postroll_ns) + WINDOW_DATA_MARGIN);
    source.stop(libc::SIGTERM, Duration::from_secs(10));
    recorder.stop(libc::SIGINT, Duration::from_secs(30));

    let bag = Finished::of(&env.record_dir());
    let anchors = recorded_trigger_anchors(bag.file());
    assert_eq!(
        anchors.len(),
        names.len(),
        "the recording must carry one trigger per name"
    );
    assert_source_data_brackets(&bag, &anchors, preroll_ns, postroll_ns);
    bag
}

/// The slack [`finished_recording_with_triggers`] adds to each of its two
/// waits, on top of the preroll or postroll that wait has to cover.
///
/// A window is anchored on its trigger record's own stamp, which lands
/// somewhere inside the publish that follows the first wait, so the margin is
/// what covers the preroll wherever in that publish the anchor falls. A second
/// is ample against a suite whose live scenarios sleep out whole windows.
const WINDOW_DATA_MARGIN: Duration = Duration::from_secs(1);

/// Every window the recording's own triggers describe lies inside the source
/// data the recording holds.
///
/// The promise [`finished_recording_with_triggers`] makes, read back off the
/// finished recording: the earliest window starts at or after the first source
/// message,
/// and the latest ends at or before the last. Measured on `SRC_TOPIC` alone,
/// because the recording is `--all` and carries its own trigger records — and a
/// trigger record sits at the anchor of the very window it describes, so it is
/// always inside it. Counting a recording's messages, or a clip's, therefore
/// says nothing about whether there was any *data* in the window.
fn assert_source_data_brackets(bag: &Finished, anchors: &[u64], preroll_ns: u64, postroll_ns: u64) {
    let stamps = source_stamps(bag.file());
    let (first, last) = (
        *stamps.first().expect("the recording holds source data"),
        *stamps.last().expect("the recording holds source data"),
    );
    let start = anchors.first().expect("at least one trigger") - preroll_ns;
    let end = anchors.last().expect("at least one trigger") + postroll_ns;
    assert!(
        first <= start,
        "the recording's first {SRC_TOPIC} message is {} ns inside the earliest \
         window — the source began publishing too late for that window's preroll \
         to reach data, and the clips cut from it would be empty and complete",
        first - start
    );
    assert!(
        last >= end,
        "the recording's last {SRC_TOPIC} message is {} ns before the latest \
         window closes — the source stopped publishing too early for that \
         window's postroll to reach data",
        end - last
    );
}

/// Every `SRC_TOPIC` message's `log_time` in a finished recording, ascending —
/// the recording's own account of when the test's data was captured, with the
/// ambient topics an `--all` recorder also takes left out.
fn source_stamps(path: &Path) -> Vec<u64> {
    let mut stamps: Vec<u64> = read_clip(path)
        .into_iter()
        .filter(|(topic, _)| topic == SRC_TOPIC)
        .map(|(_, log_time)| log_time)
        .collect();
    stamps.sort_unstable();
    stamps
}

/// Every clip named in `ids` carries source data on both sides of its anchor.
///
/// The assertion that keeps an embedded-trigger scenario from passing
/// vacuously. Its recording is `--all`, so the trigger record that anchored a
/// window is itself inside that window and is copied into the clip: *every*
/// clip holds a message whatever else happened, and asserting that one does
/// proves nothing about the cut. What a cut has to be shown to have carried is
/// the source topic — on both sides of the anchor, since the preroll and the
/// postroll each select their own half of the window and a clip missing either
/// has lost data the recording was built to hold.
fn assert_clips_hold_source_data(out_dir: &Path, ids: &[String]) {
    assert!(!ids.is_empty(), "no clips to check for their data");
    for id in ids {
        let dir = out_dir.join(id);
        let anchor = anchor_from_clip(&dir);
        let stamps: Vec<u64> = read_clip_dir(&dir)
            .into_iter()
            .filter(|(topic, _)| topic == SRC_TOPIC)
            .map(|(_, log_time)| log_time)
            .collect();
        assert!(
            stamps.iter().any(|&at| at < anchor),
            "clip {id} carries no {SRC_TOPIC} message before its anchor {anchor}: \
             its preroll copied no data, so the clip is complete and says nothing"
        );
        assert!(
            stamps.iter().any(|&at| at > anchor),
            "clip {id} carries no {SRC_TOPIC} message after its anchor {anchor}: \
             its postroll copied no data"
        );
    }
}

/// The names the embedded-trigger scenarios publish into a recording. Distinct
/// names so each trigger is its own clip whatever the stamps come out as.
const EMBEDDED_TRIGGERS: [&str; 5] = [
    "embedded-1",
    "embedded-2",
    "embedded-3",
    "embedded-4",
    "embedded-5",
];

/// T17 — a `clipper clip` run over a recording's own triggers, then the same run
/// again: the second cuts nothing, skips every window with a warning each, exits
/// zero, and changes no byte.
///
/// This is what makes a re-run a resume rather than a conflict to clear by hand.
/// A finished recording and a trigger describe one window over one set of bytes,
/// so a window whose directory exists has already been cut — and an idempotent
/// pipeline that simply runs the job again needs no special case for it.
#[rstest]
fn a_clip_run_repeated_over_one_recording_skips_every_window_and_exits_zero() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_recording_with_triggers(&env, &EMBEDDED_TRIGGERS, SEC, SEC, SRC_PAYLOAD);
    let n = EMBEDDED_TRIGGERS.len();
    let out_dir = env.out_dir();

    let first = env.run_clip(
        "resume-1",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    assert_eq!(first.code(), CLEAN_EXIT_CODE, "log: {}", first.log);
    assert_eq!(
        env.complete_clips().len(),
        n,
        "one complete clip per recorded trigger"
    );
    assert!(
        first
            .log
            .contains(&format!("{n} clip(s) cut, 0 skipped as already there")),
        "the first run cut every window: {}",
        first.log
    );
    // The bytes the re-run must not change are a window's worth of data, not an
    // empty clip that would compare equal to itself just as well.
    assert_clips_hold_source_data(&out_dir, &env.complete_clips());
    let published = env.out_dir_bytes();

    let again = env.run_clip(
        "resume-2",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    assert_eq!(again.code(), CLEAN_EXIT_CODE, "log: {}", again.log);
    assert!(
        again
            .log
            .contains(&format!("0 clip(s) cut, {n} skipped as already there")),
        "the re-run skipped every window: {}",
        again.log
    );
    assert_eq!(
        again
            .log
            .matches("is already there; skipping this window")
            .count(),
        n,
        "each skip warns on its own line, naming its directory: {}",
        again.log
    );
    assert_eq!(
        env.out_dir_bytes(),
        published,
        "a re-run changes not one byte of what is already there"
    );
    env.assert_out_dir_holds_only_clips();
}

/// T18 — a run killed partway through is finished by running it again.
///
/// The kill is a supervisor's, a power cut's or an operator's; what it leaves is
/// whatever the run had published, plus at most one directory the cut in flight
/// had claimed. The re-run skips both — a taken id is taken whatever the
/// directory holds — and cuts the windows that never got one, so the recording's
/// remaining clips arrive without anything being cleared by hand. The residue
/// stays residue: it is the evidence that something died, and repairing it would
/// destroy the only trace.
#[rstest]
fn a_clip_run_killed_partway_is_finished_by_running_it_again() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // A payload heavy enough that each cut takes long enough to be interrupted
    // between windows rather than all five finishing inside one poll.
    let bag =
        finished_recording_with_triggers(&env, &EMBEDDED_TRIGGERS, 2 * SEC, SEC, BULK_PAYLOAD);
    let n = EMBEDDED_TRIGGERS.len();
    let out_dir = env.out_dir();

    let mut run = env.spawn_clip(
        "killed",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    // Kill it once it has published something but before it can have published
    // everything: the first clip's document is the signal that the run is under
    // way, and the remaining windows are what the re-run has to pick up.
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    while env.complete_clips().is_empty() && run.is_running() {
        assert!(
            std::time::Instant::now() < deadline,
            "the run published nothing to interrupt"
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    run.signal_group(libc::SIGKILL);
    run.wait_exit(Duration::from_secs(10))
        .expect("SIGKILL must end the run");

    let interrupted = env.complete_clips();
    let all = env.published_clips();
    assert!(
        !interrupted.is_empty() && interrupted.len() < n,
        "precondition: the kill must land partway, got {} of {n} complete",
        interrupted.len()
    );
    let residue: Vec<String> = all
        .iter()
        .filter(|name| !interrupted.contains(name))
        .cloned()
        .collect();
    let kept = env.out_dir_bytes();

    let resumed = env.run_clip(
        "resumed",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    assert_eq!(resumed.code(), CLEAN_EXIT_CODE, "log: {}", resumed.log);
    assert!(
        resumed.log.contains("skipped as already there"),
        "the re-run skipped what the killed run had claimed: {}",
        resumed.log
    );
    assert_eq!(
        env.published_clips().len(),
        n,
        "every recorded trigger now has its directory"
    );
    assert_eq!(
        env.complete_clips().len(),
        n - residue.len(),
        "every window but the ones the kill left residue for is a complete clip"
    );
    for name in &residue {
        assert!(
            clip::layout::read_document(&out_dir.join(name)).is_err(),
            "the residue of the killed cut is not repaired: {name}"
        );
    }
    // Every window the two runs between them completed is a real cut — the
    // resume finished the job rather than filling the directory with empties.
    assert_clips_hold_source_data(&out_dir, &env.complete_clips());
    // What the killed run had published is untouched by the one that finished it.
    let after = env.out_dir_bytes();
    for (path, bytes) in &kept {
        assert_eq!(
            after.get(path),
            Some(bytes),
            "the interrupted run's output must survive the re-run: {}",
            path.display()
        );
    }
    env.assert_out_dir_holds_only_clips();
}

/// T12 — two `clipper clip` runs over one recording into one output directory at
/// once are harmless to each other: both exit zero, every clip is complete, and
/// together they produce exactly what one run produces.
///
/// No lock file and no coordination: each window claims its directory with one
/// `mkdir`, the kernel picks a winner, and the loser skips. Which of the two
/// processes cut any given clip is therefore not a fact about the output, which
/// is the point — a scheduling accident that starts the job twice costs nothing
/// and loses nothing.
#[rstest]
fn two_clip_runs_into_one_out_dir_together_produce_one_runs_output() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_recording_with_triggers(&env, &EMBEDDED_TRIGGERS, SEC, SEC, BULK_PAYLOAD);
    let n = EMBEDDED_TRIGGERS.len();

    let shared = env.path("out-shared");
    let mut a = env.spawn_clip("race-a", bag.file(), &shared, &["--trigger-source", "mcap"]);
    let mut b = env.spawn_clip("race-b", bag.file(), &shared, &["--trigger-source", "mcap"]);
    for (tag, proc) in [("a", &mut a), ("b", &mut b)] {
        let status = proc.wait_exit(Duration::from_secs(120)).unwrap_or_else(|| {
            proc.dump_log();
            panic!("the concurrent run {tag} did not finish");
        });
        assert_eq!(
            status.code(),
            Some(CLEAN_EXIT_CODE),
            "a run that skipped what the other claimed is a normal run, got {status}"
        );
    }

    // The reference: what one run alone leaves.
    let alone = env.path("out-alone");
    let single = env.run_clip("alone", bag.file(), &alone, &["--trigger-source", "mcap"]);
    assert_eq!(single.code(), CLEAN_EXIT_CODE, "log: {}", single.log);

    assert_eq!(
        complete_clips_in(&shared),
        complete_clips_in(&alone),
        "the union of the two runs is one run's output"
    );
    assert_eq!(complete_clips_in(&shared).len(), n);
    for id in complete_clips_in(&shared) {
        assert_eq!(
            read_clip_dir(&shared.join(&id)),
            read_clip_dir(&alone.join(&id)),
            "the racing runs' clip {id} holds what one run's does"
        );
    }
    assert_eq!(
        entry_names(&shared),
        complete_clips_in(&shared),
        "every directory the race left is a complete clip — a loser writes nothing"
    );
    // What the two runs agreed on is a window's data, not two empty directories
    // that would agree just as readily.
    assert_clips_hold_source_data(&shared, &complete_clips_in(&shared));
}

// ─────────────────────────────────────────────────────────────────────────────
// The same trigger, twice
//
// A repeated trigger is only a *repeated* trigger where the anchor it resolves
// to is reproducible, and under the deployed default — `--trigger-source ros`
// on `--time-source log` — it is not: the anchor is `now_ns()` at the
// subscription instant, so two publishes milliseconds apart resolve to two
// anchors, two ids and two clips. That is correct, and it is why every scenario
// below runs on `--time-source publish`, the one cell that anchors on the
// trigger's own `trigger_time`. Two publishes carrying one `trigger_time` are
// then one window with one id — and the id is a value the test chose, so it can
// name the directory before the recorder creates it.
// ─────────────────────────────────────────────────────────────────────────────

/// The recorder's clock domain for the scenarios that need a reproducible
/// anchor, and the one its clips' ids are computed against.
const REPEATABLE_SOURCE: &str = "publish";
const REPEATABLE_TIME_SOURCE: clip::TimeSource = clip::TimeSource::Publish;

/// The directory name the clip of a trigger published with
/// [`TestEnv::fire_trigger_stamped`] will have, on that cell.
fn stamped_clip_id(name: &str, anchor_ns: u64, preroll_ns: u64, postroll_ns: u64) -> String {
    clip_id_of(
        name,
        anchor_ns,
        preroll_ns,
        postroll_ns,
        REPEATABLE_TIME_SOURCE,
        "tail",
    )
}

/// A live recorder on the cell whose anchor a test can choose, with data
/// already on disk.
///
/// The bring-up every scenario in this section shares: a continuous `ros2 bag
/// record --all`, a source publishing into it, and a `clipper tail` on
/// `--time-source publish`. It returns once the recording holds `data` of the
/// source — measured on the recording's own extent, since the ros2 CLI starts
/// publishing an unbounded moment after it is spawned — so a window anchored
/// that far in the past lies over bytes that are already there.
///
/// `payload_bytes` is the source's message size, which decides how long a
/// window's copy takes ([`TestEnv::start_bulk_source`]) and matters only to the
/// scenarios that watch a cut while it runs.
struct LiveTail {
    /// Kept alive for the test's duration; the tests act on the extractor.
    _recorder: Proc,
    _source: Proc,
    extractor: Proc,
}

fn live_tail(env: &TestEnv, data: Duration, payload_bytes: usize) -> LiveTail {
    let recorder = env.start_recorder("fastwrite", 0);
    let source = env.start_bulk_source(SRC_TOPIC, BULK_RATE, payload_bytes);
    env.wait_for_recording(Duration::from_secs(60));
    let extractor = env.start_extractor_src(30, REPEATABLE_SOURCE);
    env.wait_for_recording_span(data, Duration::from_secs(90));
    LiveTail {
        _recorder: recorder,
        _source: source,
        extractor,
    }
}

/// T9 — a trigger whose clip is already there is skipped: the warning names the
/// directory, not a byte of it changes, and no second `Recorded` goes out.
///
/// A clip is written once. The rule is the directory's existence and nothing
/// else, so a detector that fires the same request twice costs the disk one
/// clip, and a subscriber counting announcements counts one event.
#[rstest]
fn a_trigger_whose_clip_is_already_there_is_skipped_and_announces_nothing() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let (preroll, postroll) = (2 * SEC, SEC);
    let mut live = live_tail(&env, Duration::from_nanos(preroll), SRC_PAYLOAD);
    let extractor = &mut live.extractor;

    // A window entirely in the past, so the recording already covers its end and
    // each handler goes straight to its claim.
    let mut stream = env.start_recorded_stream("repeat");
    let anchor = now_ns() - 3 * SEC;
    env.fire_trigger_stamped("repeat", anchor, preroll, postroll);

    let announced = wait_for_recorded_count(&mut stream, 1, Duration::from_secs(60));
    let id = stamped_clip_id("repeat", anchor, preroll, postroll);
    let clip_dir = env.out_dir().join(&id);
    assert_eq!(
        env.complete_clips(),
        vec![id.clone()],
        "the first trigger's clip is there and complete"
    );
    assert_eq!(
        announced[0].filenames,
        vec![clip_dir.display().to_string()],
        "the announcement names the one directory the clip is"
    );
    let published = env.out_dir_bytes();

    // The same request again: same trigger_time, same window, same id.
    env.fire_trigger_stamped("repeat", anchor, preroll, postroll);
    extractor.expect_log_count("trigger name=\"repeat\"", 2, Duration::from_secs(60));
    extractor.expect_log(
        "is already there; skipping this window",
        Duration::from_secs(60),
    );
    assert!(
        extractor
            .log_text()
            .contains(&clip_dir.display().to_string()),
        "the warning names the directory an operator would have to remove"
    );

    // Give a second announcement every chance to arrive before denying it.
    std::thread::sleep(Duration::from_secs(5));
    assert_eq!(
        env.published_clips(),
        vec![id],
        "the repeat left no second directory"
    );
    assert_eq!(
        env.out_dir_bytes(),
        published,
        "the clip already there is not rewritten, not even identically"
    );
    assert_eq!(
        recorded_so_far(&stream).len(),
        1,
        "a skipped window announces nothing — its warning is its whole trace"
    );
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running(), "a skip is not a fault");
}

/// T10 — an incomplete directory is skipped exactly as a complete one is, and
/// nothing about it is repaired.
///
/// What a killed cut leaves is a directory with no document, and it is evidence:
/// a later trigger with that id neither overwrites it nor finishes it. So the
/// claim's rule is the directory's existence and never what is inside it — and
/// the residue an operator finds is the residue the crash left, not something
/// clipper wrote on top of it.
#[rstest]
fn an_incomplete_clip_directory_is_skipped_and_never_repaired() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let (preroll, postroll) = (2 * SEC, SEC);
    let mut live = live_tail(&env, Duration::from_nanos(preroll), SRC_PAYLOAD);
    let extractor = &mut live.extractor;

    // Crash residue in the shape a killed cut leaves: the directory, an MCAP
    // file under the name a copy writes, and no document.
    let anchor = now_ns() - 3 * SEC;
    let id = stamped_clip_id("residue", anchor, preroll, postroll);
    let clip_dir = env.out_dir().join(&id);
    std::fs::create_dir_all(&clip_dir).expect("planting the residue directory");
    std::fs::write(clip_dir.join(format!("{id}_0.mcap")), b"half a clip")
        .expect("planting the residue file");
    let residue = env.out_dir_bytes();

    let stream = env.start_recorded_stream("residue");
    env.fire_trigger_stamped("residue", anchor, preroll, postroll);
    extractor.expect_log(
        "is already there; skipping this window",
        Duration::from_secs(60),
    );
    assert!(
        extractor
            .log_text()
            .contains(&clip_dir.display().to_string()),
        "the warning names the residue directory"
    );

    std::thread::sleep(Duration::from_secs(5));
    assert_eq!(
        env.out_dir_bytes(),
        residue,
        "the residue is left exactly as it was found — not repaired, not replaced"
    );
    assert!(
        clip::layout::read_document(&clip_dir).is_err(),
        "and it is still incomplete, so no consumer reads it as a clip"
    );
    assert!(
        env.complete_clips().is_empty(),
        "the skipped window produced no clip of its own"
    );
    assert!(recorded_so_far(&stream).is_empty(), "and announced nothing");
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running());
}

/// T11 — the same trigger twice within milliseconds yields exactly one clip and
/// one announcement.
///
/// Two handlers reach the claim at once and the kernel picks the winner: one
/// `mkdir` succeeds, the other sees the directory and skips. There is no lock
/// file and no coordination, so what settles the race is the same operation that
/// settles a repeat seconds apart — which is why the burst needs no separate
/// rule and gets none.
///
/// A single `ros2 topic pub` publishes both copies (`-t 2`), because two
/// processes could not be started milliseconds apart: each costs about a second
/// of python startup, which is the very gap this case is about closing.
#[rstest]
fn the_same_trigger_twice_within_milliseconds_yields_one_clip() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let (preroll, postroll) = (2 * SEC, SEC);
    let mut live = live_tail(&env, Duration::from_nanos(preroll), SRC_PAYLOAD);
    let extractor = &mut live.extractor;

    let stream = env.start_recorded_stream("twice");
    let anchor = now_ns() - 3 * SEC;
    // Two copies 20 ms apart, identical to the byte.
    env.fire_trigger_burst("twice", anchor, preroll, postroll, 2, 50);

    let id = stamped_clip_id("twice", anchor, preroll, postroll);
    env.wait_for_complete_clips(1, Duration::from_secs(60));
    std::thread::sleep(Duration::from_secs(5));

    assert_eq!(
        env.published_clips(),
        vec![id],
        "two copies of one request are one clip"
    );
    assert_eq!(
        env.complete_clips().len(),
        1,
        "and it is complete — the loser wrote nothing into the winner's directory"
    );
    assert_eq!(
        recorded_so_far(&stream).len(),
        1,
        "one clip, one announcement"
    );
    assert!(
        extractor
            .log_text()
            .contains("is already there; skipping this window"),
        "the loser said what happened to it"
    );
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running());
}

/// T25 — sixteen windows open at once, each with its own id, each a complete
/// clip with one `Recorded`.
///
/// Sixteen is the admission cap, so this is the burst the recorder is built to
/// take without turning any trigger away — and the assertion that none was is
/// the log *not* naming a rejection. The windows share one anchor and differ
/// only in name, which is the point twice over: the ids differ because the name
/// is one of the six fields hashed, and the handlers genuinely overlap because
/// one anchor and one postroll park every one of them until the same instant.
#[rstest]
fn sixteen_windows_at_once_each_get_their_own_clip_and_announcement() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let (preroll, postroll) = (2 * SEC, 10 * SEC);
    let mut live = live_tail(&env, Duration::from_nanos(preroll), SRC_PAYLOAD);
    let extractor = &mut live.extractor;

    let names: Vec<String> = (0..16).map(|i| format!("burst-{i:02}")).collect();
    let mut stream = env.start_recorded_stream("sixteen");
    // The window ends ten seconds out, so every handler is still parked on its
    // postroll while the last of the sixteen publishes is still going out.
    let anchor = now_ns();
    env.fire_triggers_at_once(&names, anchor, preroll, postroll);

    let clips = env.wait_for_complete_clips(16, Duration::from_secs(90));
    let mut expected: Vec<String> = names
        .iter()
        .map(|name| stamped_clip_id(name, anchor, preroll, postroll))
        .collect();
    expected.sort();
    assert_eq!(
        clips, expected,
        "sixteen distinct requests at one instant are sixteen distinct clips"
    );

    let announced = wait_for_recorded_count(&mut stream, 16, Duration::from_secs(60));
    let mut announced_names: Vec<String> = announced.iter().map(|r| r.name.clone()).collect();
    announced_names.sort();
    assert_eq!(announced_names, names, "one announcement per clip");
    for recorded in &announced {
        assert_eq!(
            recorded.filenames.len(),
            1,
            "each announcement names one directory"
        );
    }
    assert!(
        !extractor.log_text().contains("trigger rejected"),
        "sixteen at once is the cap, not past it: no trigger may be turned away"
    );
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running());
}

// ─────────────────────────────────────────────────────────────────────────────
// Failure, crash and restart
//
// What these scenarios need is a cut slow enough to be observed while it runs,
// and that is what [`finished_bulk_recording`] and [`TestEnv::start_bulk_source`]
// are for: a window over tens of megabytes takes long enough that a poll at a
// few milliseconds catches the directory in its claimed-but-incomplete state
// with a wide margin. Where a live recorder is involved the postroll does the
// scheduling instead — a handler claims its directory when its window closes,
// so a window that closes seconds from now puts the claim at an instant the
// test is already watching for.
// ─────────────────────────────────────────────────────────────────────────────

/// Block until `dir` exists, or the run that should be creating it has ended.
fn wait_for_claim(dir: &Path, run: &mut Proc, timeout: Duration) {
    let deadline = std::time::Instant::now() + timeout;
    while !dir.exists() {
        assert!(
            run.is_running(),
            "the run ended without leaving {} to catch it at",
            dir.display()
        );
        assert!(
            std::time::Instant::now() < deadline,
            "no clip directory was claimed within {timeout:?}"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
}

/// T7 — a cut killed after it claims its directory and before it writes the
/// document leaves exactly that: a directory, no document.
///
/// Which is the whole of what makes the completion rule safe. There is no
/// teardown step and there could not be one — a SIGKILL runs nothing — so the
/// guarantee has to come from the write order, and what a killed cut leaves has
/// to be something no consumer reads as a clip. It is, because the document is
/// written last and is the only thing "complete" means.
///
/// The window is over tens of megabytes so the copy lasts far longer than the
/// poll that catches it; a kill landing after the document would mean the cut
/// outran the watcher, and the assertion says so rather than reading as a
/// semantics regression.
#[rstest]
fn a_cut_killed_after_its_claim_leaves_a_directory_without_its_document() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let bag = finished_bulk_recording(&env, Duration::from_secs(10));
    let (first, last) = bag.span();
    // A window over the whole recording: every byte of it has to be copied.
    let window = ParamWindow {
        anchor_ns: last,
        preroll_ns: last - first + SEC,
        postroll_ns: SEC,
        name: "killed-mid-cut",
    };
    let out_dir = env.out_dir();
    let clip_dir = out_dir.join(window.clip_id());

    let mut run = spawn_param_clip(&env, "mid-cut", bag.file(), &out_dir, window);
    wait_for_claim(&clip_dir, &mut run, Duration::from_secs(60));
    run.signal_group(libc::SIGKILL);
    run.wait_exit(Duration::from_secs(10))
        .expect("SIGKILL must end the run");

    assert!(
        clip_dir.is_dir(),
        "the killed cut's directory stays: it is the evidence something died"
    );
    assert!(
        clip::layout::read_document(&clip_dir).is_err(),
        "a cut killed before it finished has no document — if this fails the \
         kill landed after the copy, and the window needs more bytes to copy"
    );
    assert!(
        env.complete_clips().is_empty(),
        "so nothing in the output directory reads as a clip"
    );
    assert_eq!(
        env.published_clips(),
        vec![window.clip_id()],
        "and the directory is named by the id the window claimed"
    );
    env.assert_out_dir_holds_only_clips();
}

/// T15 — a recorder killed mid-cut, then restarted: the residue stays, the next
/// trigger works, and the same trigger is skipped.
///
/// The three facts a supervisor loop rests on. A restart cannot re-cut what it
/// finds, because it cannot tell a clip it is about to overwrite from evidence
/// it is about to destroy — so it treats a taken id as taken, whatever the
/// directory holds. And that has to cost nothing else: the restarted recorder
/// is a working recorder, which the new trigger is there to show.
///
/// The kill is scheduled by the postroll rather than raced for: a handler claims
/// its directory when its window closes, so a window closing several seconds out
/// puts the claim well after the publish has been confirmed and the watch has
/// started.
#[rstest]
fn a_recorder_killed_mid_cut_leaves_residue_its_restart_neither_repairs_nor_re_cuts() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    // A preroll reaching back over every recorded byte, so the copy is long, and
    // a postroll putting the claim several seconds after the trigger.
    let (preroll, postroll) = (30 * SEC, 6 * SEC);
    let mut live = live_tail(&env, Duration::from_secs(8), BULK_PAYLOAD);
    let extractor = &mut live.extractor;

    let anchor = now_ns();
    let id = stamped_clip_id("mid-cut", anchor, preroll, postroll);
    let clip_dir = env.out_dir().join(&id);
    env.fire_trigger_stamped("mid-cut", anchor, preroll, postroll);
    wait_for_claim(&clip_dir, extractor, Duration::from_secs(60));
    extractor.signal_group(libc::SIGKILL);
    extractor
        .wait_exit(Duration::from_secs(10))
        .expect("SIGKILL must end the recorder");

    assert!(
        clip::layout::read_document(&clip_dir).is_err(),
        "the killed cut left a directory with no document — if this fails the \
         kill landed after the copy, and the window needs more bytes to copy"
    );
    let residue = env.out_dir_bytes();

    // The restart. Nothing on disk is re-cut or repaired by it.
    let mut restarted = env.start_extractor_src(30, REPEATABLE_SOURCE);
    std::thread::sleep(Duration::from_secs(3));
    assert_eq!(
        env.out_dir_bytes(),
        residue,
        "a restart re-cuts nothing and repairs nothing it finds"
    );

    // A new trigger: the restarted recorder is a working recorder.
    let mut stream = env.start_recorded_stream("after-restart");
    let fresh_anchor = now_ns() - 3 * SEC;
    let (fresh_preroll, fresh_postroll) = (2 * SEC, SEC);
    env.fire_trigger_stamped("after-kill", fresh_anchor, fresh_preroll, fresh_postroll);
    let fresh_id = stamped_clip_id("after-kill", fresh_anchor, fresh_preroll, fresh_postroll);
    wait_for_recorded_count(&mut stream, 1, Duration::from_secs(60));
    assert_eq!(
        env.complete_clips(),
        vec![fresh_id],
        "the restarted recorder cuts the clips it is asked for"
    );

    // And the trigger the kill interrupted, asked again: its id is taken.
    env.fire_trigger_stamped("mid-cut", anchor, preroll, postroll);
    restarted.expect_log(
        "is already there; skipping this window",
        Duration::from_secs(60),
    );
    assert!(
        restarted
            .log_text()
            .contains(&clip_dir.display().to_string()),
        "the warning names the residue directory"
    );
    std::thread::sleep(Duration::from_secs(5));
    for (path, bytes) in &residue {
        assert_eq!(
            env.out_dir_bytes().get(path),
            Some(bytes),
            "the residue survives the repeat untouched: {}",
            path.display()
        );
    }
    assert_eq!(
        recorded_so_far(&stream).len(),
        1,
        "the skipped repeat announced nothing"
    );
    env.assert_out_dir_holds_only_clips();
    assert!(restarted.is_running());
}

/// T14 — an output directory that cannot be written to fails the cut, leaves no
/// directory behind, announces nothing, does not take the recorder down, and
/// costs nothing once it is writable again.
///
/// A device whose disk goes read-only must not lose its recorder: a trigger it
/// cannot answer is one trigger's worth of loss, named in the log, and the next
/// one after the repair succeeds. That the id is free again afterwards is the
/// consequence of a failed cut taking its directory with it — a claim that
/// survived a failure would make the fault permanent for that window.
///
/// **The fault is injected before the claim rather than after it**, because
/// after is not reachable from outside: the claim and the first write are one
/// `mkdir` apart, microseconds no poll can land inside. What the two orderings
/// share is everything observable here — no directory, no announcement, a
/// recorder still up, and a repeat that works once the cause is gone — and the
/// ordering itself is pinned by `clip::layout`'s own tests, where the failure
/// can be injected exactly.
///
/// **It needs an ordinary user.** uid 0 ignores the permission bits this writes,
/// so the scenario would pass having injected no fault at all;
/// [`assert_permissions_bite`] fails the run rather than let that happen. CI's
/// recorder job is a plain GitHub runner, so this holds there; a container run
/// as root (`act`) is where it would not.
#[rstest]
fn an_unwritable_out_dir_costs_one_clip_and_not_the_recorder() {
    if !require_e2e() {
        return;
    }
    assert_permissions_bite();
    let env = TestEnv::new();
    let (preroll, postroll) = (2 * SEC, SEC);
    let mut live = live_tail(&env, Duration::from_nanos(preroll), SRC_PAYLOAD);
    let extractor = &mut live.extractor;

    // The recorder created its output directory at startup; take the write
    // permission away from it.
    let out_dir = env.out_dir();
    set_writable(&out_dir, false);

    let mut stream = env.start_recorded_stream("unwritable");
    let anchor = now_ns() - 3 * SEC;
    let id = stamped_clip_id("unwritable", anchor, preroll, postroll);
    env.fire_trigger_stamped("unwritable", anchor, preroll, postroll);

    extractor.expect_log("trigger handling failed", Duration::from_secs(60));
    assert!(
        extractor.log_text().contains(&format!(
            "claiming clip directory {}",
            out_dir.join(&id).display()
        )),
        "the failure names the directory it could not write"
    );
    std::thread::sleep(Duration::from_secs(3));
    assert!(
        !out_dir.join(&id).exists(),
        "a failed cut leaves no directory, so the id is free for the next try"
    );
    assert!(
        recorded_so_far(&stream).is_empty(),
        "nothing was recorded, so nothing is announced"
    );
    assert!(
        extractor.is_running(),
        "a write fault costs a clip, never the recorder"
    );

    // The cause removed, the same request succeeds.
    set_writable(&out_dir, true);
    env.fire_trigger_stamped("unwritable", anchor, preroll, postroll);
    extractor.expect_log_count("trigger name=\"unwritable\"", 2, Duration::from_secs(60));
    let announced = wait_for_recorded_count(&mut stream, 1, Duration::from_secs(60));
    assert_eq!(announced[0].name, "unwritable");
    assert_eq!(
        env.complete_clips(),
        vec![id.clone()],
        "the window the fault cost is cut on the next identical trigger"
    );
    assert_clip_metadata(&out_dir.join(&id), "tail", preroll, postroll);
    env.assert_out_dir_holds_only_clips();
}

/// T19 — `clipper clip` stops at the first window it cannot cut: exit 1, the
/// clips it had already published stay, the windows after it are never
/// attempted, and a re-run after the repair finishes the job.
///
/// A disk or input problem outlives the window that met it, so going on would
/// raise it once per remaining trigger; stopping puts the reason in the run's
/// last line instead of the arithmetic. What makes stopping safe rather than
/// destructive is the resume: nothing published is unmade, and the re-run skips
/// exactly what is there.
///
/// The fault is the output directory losing its write permission partway through
/// the run — see
/// [`an_unwritable_out_dir_costs_one_clip_and_not_the_recorder`] for why an
/// ordinary user is required, and for what a root runner would do to it.
#[rstest]
fn a_clip_run_stops_at_the_first_failed_window_and_keeps_what_it_published() {
    if !require_e2e() {
        return;
    }
    assert_permissions_bite();
    let env = TestEnv::new();
    let bag =
        finished_recording_with_triggers(&env, &EMBEDDED_TRIGGERS, 2 * SEC, SEC, BULK_PAYLOAD);
    let n = EMBEDDED_TRIGGERS.len();
    let out_dir = env.out_dir();

    let mut run = env.spawn_clip(
        "stopped",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    // Take the write permission away once the run has published something, so
    // the window it fails at is not its first and there is output to keep.
    let deadline = std::time::Instant::now() + Duration::from_secs(60);
    while env.complete_clips().is_empty() {
        assert!(
            run.is_running(),
            "the run finished before the fault could be injected — every window \
             was cut, so there is no stop to observe"
        );
        assert!(
            std::time::Instant::now() < deadline,
            "the run published nothing"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
    set_writable(&out_dir, false);

    let status = run.wait_exit(Duration::from_secs(120)).unwrap_or_else(|| {
        set_writable(&out_dir, true);
        run.dump_log();
        panic!("the stopped run never exited");
    });
    let log = run.log_text();
    set_writable(&out_dir, true);

    assert_eq!(
        status.code(),
        Some(FATAL_EXIT_CODE),
        "a window that fails ends the run with {FATAL_EXIT_CODE} for its caller, got {status}"
    );
    assert!(
        log.contains("stopped the run"),
        "the last line names the window that stopped it: {log}"
    );
    assert!(
        log.contains("no later window was attempted"),
        "and says the windows after it were not tried: {log}"
    );

    let kept = env.complete_clips();
    assert!(
        !kept.is_empty() && kept.len() < n,
        "the run published some but not all of the {n} windows, got {}",
        kept.len()
    );
    assert_eq!(
        env.published_clips(),
        kept,
        "the failed window took its own directory with it, so every directory \
         left is a complete clip"
    );
    let published = env.out_dir_bytes();

    // The cause removed, the re-run finishes the job.
    let resumed = env.run_clip(
        "resumed",
        bag.file(),
        &out_dir,
        &["--trigger-source", "mcap"],
    );
    assert_eq!(resumed.code(), CLEAN_EXIT_CODE, "log: {}", resumed.log);
    assert!(
        resumed
            .log
            .contains(&format!("{} skipped as already there", kept.len())),
        "the re-run skipped exactly what the stopped run had published: {}",
        resumed.log
    );
    assert_eq!(
        env.complete_clips().len(),
        n,
        "and cut the rest, so every recorded trigger now has its clip"
    );
    for (path, bytes) in &published {
        assert_eq!(
            env.out_dir_bytes().get(path),
            Some(bytes),
            "the stopped run's clips survive the re-run: {}",
            path.display()
        );
    }
    // Both halves of the job cut real windows: what the run kept across the
    // fault and what the re-run added carry the data their windows covered.
    assert_clips_hold_source_data(&out_dir, &env.complete_clips());
    env.assert_out_dir_holds_only_clips();
}

/// T23 — a `clipper tail` restart re-cuts nothing: the clips on disk are not
/// touched, and no second clip appears for a trigger the previous run answered.
///
/// This is what makes a supervisor loop safe. A restarted recorder must never
/// destroy evidence, and what guarantees that is the claim: the id of a window
/// already cut is taken, so the window is skipped whatever the recorder makes of
/// the trigger behind it. That the restarted recorder is *live* rather than
/// merely quiet is the second half — a trigger written after it came up is cut
/// normally.
///
/// **What the restart does with the recording's existing trigger is deliberately
/// not asserted here, because it is not what a supervisor depends on and the
/// code and its documentation disagree about it.** `tail::Tailer::with_trigger_tap`
/// says a trigger already on disk before clipper started never fires; that holds
/// for the recordings the discovery iterator is seeded past, but the *newest*
/// one is adopted and indexed from its first byte with the tap on, so its
/// triggers are re-delivered on every restart and each is answered by a skip.
/// Asserting either behaviour would pin a disagreement rather than the
/// guarantee; the guarantee is that the output directory does not change, and
/// that is what is asserted.
#[rstest]
fn a_tail_restart_re_cuts_nothing_and_leaves_the_clips_it_finds() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_source(SRC_TOPIC, SRC_RATE);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor_mcap(15);
    let (preroll, postroll) = (2 * SEC, 2 * SEC);
    env.wait_for_recording_span(Duration::from_nanos(preroll), Duration::from_secs(60));

    env.fire_trigger_into_bag("before-restart", preroll, postroll);
    let first = env.wait_for_clip_named("before-restart", Duration::from_secs(60));
    let published = env.out_dir_bytes();
    let before = env.published_clips();

    // A clean stop, then a restart against the same recording and the same
    // output directory.
    let stopped = extractor.stop(libc::SIGINT, Duration::from_secs(30));
    assert_eq!(
        stopped.code(),
        Some(CLEAN_EXIT_CODE),
        "a requested stop is the orderly one, got {stopped}"
    );
    let mut restarted = env.start_extractor_mcap(15);
    std::thread::sleep(Duration::from_secs(5));

    assert_eq!(
        env.published_clips(),
        before,
        "the restart cut no clip of its own"
    );
    assert_eq!(
        env.out_dir_bytes(),
        published,
        "and changed not a byte of the clip that was already there"
    );
    assert_eq!(
        env.complete_clips().len(),
        1,
        "the window the previous run answered has exactly one clip, still"
    );
    assert!(
        !restarted.log_text().contains(" written: "),
        "and the restart wrote no clip file: whatever it made of the trigger \
         already in the recording, it cut nothing"
    );

    // The second half: a trigger written after the restart is cut normally.
    env.fire_trigger_into_bag("after-restart", preroll, postroll);
    let second = env.wait_for_clip_named("after-restart", Duration::from_secs(60));
    assert_ne!(first, second, "the new trigger is its own clip");
    assert_eq!(env.complete_clips().len(), 2);
    env.assert_out_dir_holds_only_clips();
    assert!(restarted.is_running());
}

/// T24 — the observer on the mcap interface completes on the document and never
/// on a file appearing.
///
/// Under `--trigger-source mcap` nothing is published, so what watches for
/// finished clips is whatever syncs the output directory — and what clipper owes
/// it is an ordering rather than a message. This samples that directory while a
/// cut runs and catches it in the state the two rules disagree about: the clip's
/// directory is there and carries MCAP bytes, and `read_document` still refuses
/// it. A consumer keying on a file would have uploaded a clip that was still
/// being written; the one keying on the document waits, and when it stops
/// waiting every file the document names is a complete MCAP.
#[rstest]
fn the_mcap_observer_completes_on_the_document_and_never_on_a_file() {
    if !require_e2e() {
        return;
    }
    let env = TestEnv::new();
    let _recorder = env.start_recorder("fastwrite", 0);
    let _source = env.start_bulk_source(SRC_TOPIC, BULK_RATE, BULK_PAYLOAD);
    env.wait_for_recording(Duration::from_secs(60));
    let mut extractor = env.start_extractor_mcap(30);
    // A preroll over every recorded byte, so the copy lasts long enough to be
    // sampled mid-flight.
    let (preroll, postroll) = (30 * SEC, 2 * SEC);
    env.wait_for_recording_span(Duration::from_secs(8), Duration::from_secs(90));
    env.fire_trigger_into_bag("observed", preroll, postroll);

    // Sample the directory the way a sync tool would, until a clip is complete.
    let mut caught_half_written = false;
    let deadline = std::time::Instant::now() + Duration::from_secs(90);
    loop {
        for name in env.published_clips() {
            let dir = env.out_dir().join(&name);
            let holds_mcap_bytes = entry_names(&dir).iter().any(|f| {
                Path::new(f)
                    .extension()
                    .is_some_and(|ext| ext == "mcap" || ext == "part")
            });
            if holds_mcap_bytes && clip::layout::read_document(&dir).is_err() {
                caught_half_written = true;
            }
        }
        if !env.complete_clips().is_empty() {
            break;
        }
        assert!(
            std::time::Instant::now() < deadline,
            "no clip was completed to sample"
        );
        std::thread::sleep(Duration::from_millis(2));
    }

    assert!(
        caught_half_written,
        "the cut never showed a directory carrying MCAP bytes without its \
         document — if this fails the copy outran the sampler, and the window \
         needs more bytes to copy"
    );
    let clip = env.wait_for_clip_named("observed", Duration::from_secs(60));
    // The moment the document answers, every file it names is a whole MCAP:
    // `read_clip_dir` parses each through its summary, footer and closing magic.
    let msgs = read_clip_dir(&clip);
    assert!(!msgs.is_empty(), "the completed clip holds the window");
    assert_clip_metadata(&clip, "tail", preroll, postroll);
    env.assert_out_dir_holds_only_clips();
    assert!(extractor.is_running());
}
