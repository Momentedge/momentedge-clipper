//! Live ROS2 end-to-end tests for the continuous extractor: a real
//! `ros2 bag record` (the production `scripts/record-continuous.sh`
//! invocation), real CLI-published triggers, and real `Recorded`
//! announcements, against the deployed storage profiles.
//!
//! Gated on `EDGESTREAM_E2E` (see [`harness::require_e2e`]); cargo-nextest is
//! the required runner. Inside the dev shell:
//!
//! ```text
//! EDGESTREAM_E2E=1 cargo nextest run -p edgestream-rec-cont --profile e2e -E 'binary(e2e)'
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
        msgs.iter().map(|(t, _)| t).collect::<std::collections::HashSet<_>>(),
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
    recorder.stop(libc::SIGINT, Duration::from_secs(30));
    let _recorder2 = env.start_recorder("fastwrite", 0);
    extractor.expect_log("replaced; re-discovering", Duration::from_secs(60));
    env.wait_for_recording(Duration::from_secs(60));
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
    let config = env.write_recorder_topics_config(&[SRC_TOPIC]);
    let _recorder = env.start_recorder_with_config(Some(&config), "fastwrite", 0);
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
    if !require_e2e() {
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
