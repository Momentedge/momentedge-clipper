//! One window over a planned recording becomes durable clips: plan the window,
//! stage one segment per source file through a worker pool, drop the empties,
//! and publish each atomically.
//!
//! [`cut_window`] is the whole of it, over any [`WindowPlanner`]: it snapshots
//! the plans once, hands each to the FIFO staging pool
//! ([`spawn_stage_workers`]) that runs the bulk copies off the caller's thread,
//! and names the results only when the segment count is known — a bare
//! `<base>.mcap` for a window inside one recording, one `<base>_NN.mcap` per
//! source file for one that straddled a rollover.
//!
//! The module is deliberately free of any notion of where the window came from
//! or who is told about it. Nothing here decides a window's bounds, waits for
//! anything before cutting, or announces the clips it produced — a caller
//! resolves the bounds, does whatever waiting its own clock and coverage
//! demand, and reports the returned [`cut::ClipStats`] however it likes.

use std::path::{Path, PathBuf};
use std::thread;

use crossbeam_channel::{Sender, bounded, unbounded};

use crate::index::{WindowPlan, WindowPlanner};
use crate::{cut, panic_text};

/// One queued clip-segment staging: the window-plan snapshot [`cut_window`] took
/// for one source recording, the window bounds and the clock domain they are
/// measured in, the base output path, and the reply channel. Queued by
/// [`cut_window`]; dequeued FIFO by the staging workers, which run the bulk copy
/// into the capturing dir and reply a [`cut::StagedClip`]. Publication is not
/// theirs — [`cut_window`] publishes the staged segments itself, once the
/// window's segment count is known and the names are settled.
///
/// `time_source` rides in the job rather than in the pool because it is a
/// property of the *window*, not of the workers: the same value has to choose
/// the extents the planner returns and the stamp each message's membership is
/// tested on, and a job that carried only the bounds would let those two be
/// answered from different clocks — a clip silently holding the wrong messages
/// rather than an error.
#[derive(Debug)]
pub struct StageJob {
    plan: WindowPlan,
    start_ns: u64,
    end_ns: u64,
    time_source: crate::TimeSource,
    out_path: PathBuf,
    reply: Sender<anyhow::Result<cut::StagedClip>>,
}

/// Spawn the fixed staging worker pool: `parallelism` threads consuming one
/// shared FIFO channel. One worker is the conservative default, and it is the
/// interesting case: the bulk copies then serialize in submission order, which
/// is what you want whenever the staging reads compete for disk bandwidth with
/// whatever is still writing the recording. Widening the pool trades that back.
///
/// Each worker runs only [`cut::stage_clip`] — the bulk copy into the capturing
/// dir — and replies the [`cut::StagedClip`]. The window plan rides in the job,
/// snapshotted once per source recording with that file's `Arc<File>` pinned
/// inside it, so a worker holds everything it needs and never reaches back into
/// whoever planned the window; a retention prune or a rollover between queueing
/// and copying cannot pull the bytes out from under it. The clip compression
/// codec is the one setting fixed for the pool's lifetime and captured here —
/// it is a property of the output, not of any one window, and no window can
/// disagree with it. The window's clock domain travels in the job instead (see
/// [`StageJob`]). A panicking stage is caught and replied as an error — per-job
/// isolation, the pool outlives it.
pub fn spawn_stage_workers(
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
                        cut::stage_clip(
                            &job.plan,
                            &job.out_path,
                            job.start_ns,
                            job.end_ns,
                            compression,
                            job.time_source,
                        )
                    }))
                    .unwrap_or_else(|payload| {
                        Err(anyhow::anyhow!(
                            "staging panicked: {}",
                            panic_text(payload.as_ref())
                        ))
                    });
                    // A send failure means the caller that queued this job is
                    // gone (its thread died); there is no one left to care about
                    // this clip.
                    let _ = job.reply.send(result);
                }
            })
            .expect("spawning staging worker");
    }
    tx
}

/// Cut one window out of the recordings a [`WindowPlanner`] serves: take one
/// multi-file snapshot, stage one segment per source recording, and publish
/// each of them.
///
/// A window inside one recording yields a single segment; one straddling a
/// rollover (a bag split or a restart the planner indexed while running) yields
/// one segment per source file, recovered from the planner's retained
/// collection. Empty segments are dropped when the window produced real data
/// elsewhere, but one segment is always kept so an all-empty window (a rollover
/// gap, all relevant files pruned, or nothing recorded yet) still yields a valid
/// clip. Segments are named only once the count is known: a single segment keeps
/// the bare `<base>.mcap`, several get one `<base>_NN.mcap` per file. Every
/// returned [`cut::ClipStats`] names a durable file, so the caller may announce
/// them all.
///
/// The window lives entirely in `time_source`: it picks the extents the planner
/// returns and the stamp each message's membership is tested on. Whatever has
/// to happen before the cut — a postroll wall floor, a wait for the recording
/// to cover the window end — is the caller's, and so is telling anyone about
/// the clips this returns.
pub fn cut_window(
    planner: &dyn WindowPlanner,
    start_ns: u64,
    end_ns: u64,
    base_out_path: &Path,
    stage_tx: &Sender<StageJob>,
    time_source: crate::TimeSource,
) -> anyhow::Result<Vec<cut::ClipStats>> {
    // 1. One multi-file snapshot on the window's time source — each plan pins its
    //    own recording's Arc<File>, so a retention prune or rollover after this
    //    cannot pull the bytes out.
    let plans = planner.plan_window(start_ns, end_ns, time_source);

    // 2. Stage one segment per plan (FIFO worker pool), or one empty segment
    //    when no recording covers the window — the empty path needs no source
    //    file (a channelless MCAP is just magic + summary + footer).
    let mut staged: Vec<cut::StagedClip> = if plans.is_empty() {
        vec![stage_segment(
            stage_tx,
            WindowPlan::empty(),
            start_ns,
            end_ns,
            time_source,
            base_out_path,
        )?]
    } else {
        let mut v = Vec::with_capacity(plans.len());
        for plan in plans {
            v.push(stage_segment(
                stage_tx,
                plan,
                start_ns,
                end_ns,
                time_source,
                base_out_path,
            )?);
        }
        v
    };

    // 3. Drop empty segments when the window produced real data elsewhere, but
    //    keep one so an all-empty window still announces a valid clip.
    if staged.len() > 1 {
        if staged.iter().any(|c| !c.is_empty()) {
            staged.retain(|c| !c.is_empty());
        } else {
            staged.truncate(1);
        }
    }

    // 4. Publish the staged segments, naming them only now the count is known:
    //    one segment keeps the bare name, several get one `_NN` per source file.
    let n = staged.len();
    let mut stats = Vec::with_capacity(n);
    for (i, mut clip) in staged.into_iter().enumerate() {
        if n > 1 {
            clip.set_final_name(segment_name(base_out_path, i));
        }
        stats.push(cut::publish_clip(clip)?);
    }
    Ok(stats)
}

/// Queue one segment's copy on the staging workers and block on the reply. The
/// plan is [`cut_window`]'s snapshot of one source recording, so a job that
/// waits in the FIFO queue still copies the recording it was taken from.
fn stage_segment(
    stage_tx: &Sender<StageJob>,
    plan: WindowPlan,
    start_ns: u64,
    end_ns: u64,
    time_source: crate::TimeSource,
    out_path: &Path,
) -> anyhow::Result<cut::StagedClip> {
    let (reply_tx, reply_rx) = bounded(1);
    stage_tx
        .send(StageJob {
            plan,
            start_ns,
            end_ns,
            time_source,
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
pub fn sanitize(name: &str) -> String {
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
    use std::collections::HashMap;
    use std::fs::File;
    use std::sync::Arc;

    use super::*;
    use crate::TimeSource;
    use crate::index::{Extent, RecordingIndex, Span, Stamps, op};
    use crate::testing::{
        channel_body, index_file, message_body_pub, raw_record, read_clip, scan_to_end, test_dir,
        write_raw, write_recording,
    };

    /// The clip compression the recorder's default (zstd) maps to; the unit
    /// tests drive the extraction worker pool through the same codec the
    /// recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// The test stand-in for a tail: a fixed collection of per-recording
    /// indexes, planned oldest first exactly as a tail plans its own — one plan
    /// per recording whose extents overlap the window, none for a window no
    /// recording covers.
    struct Indexes(Vec<RecordingIndex>);

    impl WindowPlanner for Indexes {
        fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
            self.0
                .iter()
                .filter_map(|r| r.plan(start_ns, end_ns, source))
                .collect()
        }
    }

    /// Index each recording to its end and collect the indexes into a planner,
    /// oldest first — the tail's scan-then-plan, run synchronously.
    fn indexed(paths: &[&Path]) -> anyhow::Result<Indexes> {
        let mut indexes = Vec::with_capacity(paths.len());
        for path in paths {
            let (mut index, file) = index_file(path)?;
            scan_to_end(&mut index, &file)?;
            indexes.push(index);
        }
        Ok(Indexes(indexes))
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
    fn concurrent_overlapping_windows_serialize_and_take_distinct_paths() -> anyhow::Result<()> {
        let root = test_dir("overlap")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            false,
            &[("/t", 100), ("/t", 200), ("/t", 300), ("/t", 400)],
        )?;

        let planner = Arc::new(indexed(&[&rec])?);

        // Two overlapping windows racing for the same out path and a single
        // staging worker: the copies serialize FIFO, the second writer lands on
        // a `_1` sibling at publish, and both clips come out complete. Neither
        // window straddles a rollover, so each is a single segment.
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let cut = |start_ns: u64, end_ns: u64| {
            let planner = planner.clone();
            let stage_tx = stage_tx.clone();
            let out = out.clone();
            std::thread::spawn(move || {
                cut_window(
                    planner.as_ref(),
                    start_ns,
                    end_ns,
                    &out,
                    &stage_tx,
                    TimeSource::Log,
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
    /// channel; the name collision is settled at publish, on the calling thread.
    #[test]
    fn staged_segments_publish_to_distinct_paths() -> anyhow::Result<()> {
        let root = test_dir("fifo")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let planner = indexed(&[&rec])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let out = root.join("clip.mcap");
        let plan = || {
            planner
                .plan_window(0, 300, TimeSource::Log)
                .into_iter()
                .next()
                .expect("the recording covers the window")
        };
        let first = stage_segment(&stage_tx, plan(), 0, 300, TimeSource::Log, &out)?;
        let second = stage_segment(&stage_tx, plan(), 0, 300, TimeSource::Log, &out)?;

        let a = cut::publish_clip(first)?;
        let b = cut::publish_clip(second)?;
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
    fn cut_window_recovers_across_a_rollover_into_two_segments() -> anyhow::Result<()> {
        // Two finished recordings clipper indexed while running (a split): one
        // `cut_window` over a window straddling the boundary stages one segment
        // per source file, published as `<base>_00.mcap` and `<base>_01.mcap`,
        // each a complete clip.
        let root = test_dir("two-seg")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let planner = indexed(&[&split0, &split1])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = cut_window(&planner, 1_500, 5_500, &base, &stage_tx, TimeSource::Log)?;

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

    /// `cut_window` with two source recordings where BOTH segments are empty
    /// keeps exactly one of them: the drop step truncates to 1 rather than
    /// dropping all, so an all-empty window still yields a valid clip.
    #[test]
    fn cut_window_all_empty_multi_segment_keeps_one() -> anyhow::Result<()> {
        let root = test_dir("all-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // Two recordings whose messages all fall far outside the narrow window
        // [500, 600]: both staged segments will be empty (messages_copied == 0).
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let planner = indexed(&[&split0, &split1])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = cut_window(&planner, 500, 600, &base, &stage_tx, TimeSource::Log)?;

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

    /// `cut_window` with two source recordings where only one segment carries
    /// data drops the empty segment: the `retain(!is_empty)` branch fires, so
    /// the returned clip list contains only the non-empty one.
    #[test]
    fn cut_window_drops_empty_segment_when_other_has_data() -> anyhow::Result<()> {
        let root = test_dir("drop-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // split0 has messages inside the window [1_500, 5_500]; split1 does not.
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 8_000), ("/t", 9_000)])?;

        let planner = indexed(&[&split0, &split1])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let base = root.join("clip.mcap");
        let stats = cut_window(&planner, 1_500, 5_500, &base, &stage_tx, TimeSource::Log)?;

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

    /// A plan pointing at bytes that are not record-framed, so the stage that
    /// copies it always fails. Its `time` span covers everything, so any window
    /// selects it.
    struct Unreadable(Arc<File>);

    impl WindowPlanner for Unreadable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            vec![WindowPlan {
                file: Some(self.0.clone()),
                extents: vec![Extent {
                    offset: 0,
                    len: 64,
                    time: Some(Stamps {
                        log: Span {
                            min: 0,
                            max: u64::MAX,
                        },
                        publish: Span {
                            min: 0,
                            max: u64::MAX,
                        },
                    }),
                }],
                channels: HashMap::new(),
            }]
        }
    }

    /// A stage that fails is reported to the caller that queued it, and the pool
    /// survives to serve the next job.
    ///
    /// This is the property that lets one bad window stay one bad window: the
    /// workers are a long-lived fixed pool shared by every concurrent cut, so a
    /// job that dies taking the pool with it would silently stall every clip
    /// after it — with no error anywhere, because the callers would simply block
    /// on replies that never come. Both halves are asserted here: the failing
    /// cut returns an `Err` rather than hanging, and a good cut queued on the
    /// *same* channel afterwards still completes.
    #[test]
    fn a_failing_stage_is_reported_and_the_pool_serves_the_next_job() -> anyhow::Result<()> {
        let root = test_dir("stage-fails")?;
        let junk = root.join("junk.bin");
        std::fs::write(&junk, [0xFFu8; 64])?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);

        let doomed = Unreadable(Arc::new(File::open(&junk)?));
        let err = cut_window(
            &doomed,
            0,
            u64::MAX,
            &root.join("bad.mcap"),
            &stage_tx,
            TimeSource::Log,
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("framing inconsistent"),
            "the stage's own error reaches the caller: {err:#}"
        );
        assert!(!root.join("bad.mcap").exists(), "no clip is published");

        // The same pool, immediately afterwards: a well-formed window still cuts.
        let planner = indexed(&[&rec])?;
        let stats = cut_window(
            &planner,
            0,
            1_000,
            &root.join("good.mcap"),
            &stage_tx,
            TimeSource::Log,
        )?;
        assert_eq!(stats.len(), 1);
        assert_eq!(
            read_clip(&stats[0].out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 200)],
            "the worker that replied an error is still serving jobs"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A plan whose extent declares more bytes than any allocation can hold, so
    /// the copy panics rather than returning an error. Its `time` span covers
    /// everything, so any window selects it.
    struct Unallocatable(Arc<File>);

    impl WindowPlanner for Unallocatable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            vec![WindowPlan {
                file: Some(self.0.clone()),
                // `copy_window` sizes its read buffer from `len`; a length past
                // `isize::MAX` cannot be a `Vec` capacity, so the allocation
                // panics with "capacity overflow" instead of returning an error.
                extents: vec![Extent {
                    offset: 0,
                    len: u64::MAX,
                    time: Some(Stamps {
                        log: Span {
                            min: 0,
                            max: u64::MAX,
                        },
                        publish: Span {
                            min: 0,
                            max: u64::MAX,
                        },
                    }),
                }],
                channels: HashMap::new(),
            }]
        }
    }

    /// A stage that **panics** is caught, reported to the caller as an error,
    /// and leaves the pool serving.
    ///
    /// This is the arm [`spawn_stage_workers`]' `catch_unwind` exists for, and
    /// it is the one that matters most: a worker thread that unwinds out of its
    /// receive loop is gone for good, and every later job on that channel blocks
    /// forever on a reply nobody will send — a recorder that stops cutting clips
    /// with no error anywhere, because the callers are all parked. The sibling
    /// test above covers the ordinary `Err` return; this one kills the worker
    /// mid-copy and asserts the same two properties hold.
    #[test]
    fn a_panicking_stage_is_reported_and_the_pool_serves_the_next_job() -> anyhow::Result<()> {
        let root = test_dir("stage-panics")?;
        let src = root.join("src.mcap");
        write_recording(&src, false, &[("/t", 100)])?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);

        let doomed = Unallocatable(Arc::new(File::open(&src)?));
        let err = cut_window(
            &doomed,
            0,
            u64::MAX,
            &root.join("boom.mcap"),
            &stage_tx,
            TimeSource::Log,
        )
        .unwrap_err();
        let text = format!("{err:#}");
        assert!(
            text.contains("staging panicked"),
            "the panic is reported as an error, not lost with the thread: {text}"
        );
        assert!(!root.join("boom.mcap").exists(), "no clip is published");

        // The same pool, after a worker caught a panic: a well-formed window
        // still cuts. Without the catch this call never returns.
        let planner = indexed(&[&rec])?;
        let stats = cut_window(
            &planner,
            0,
            1_000,
            &root.join("good.mcap"),
            &stage_tx,
            TimeSource::Log,
        )?;
        assert_eq!(
            read_clip(&stats[0].out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 200)],
            "the worker that caught a panic is still serving jobs"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `time_source` reaches both halves of the cut: the planner chooses extents
    /// on it, and the copy tests each message's membership on it.
    ///
    /// The two are separately capable of being wired to the wrong clock, and a
    /// window whose extents were selected on one domain and whose messages were
    /// filtered on the other would quietly produce a short clip rather than an
    /// error. Pinning both halves needs a window where the extent itself falls
    /// outside one domain — otherwise the planner returns the same extent either
    /// way and only the copy is under test. So the stamps here are far apart:
    /// log times 100/200 against publish times 1_000/2_000, and a window of
    /// [900, 1_500] that the extent's log span misses entirely. A planner stuck
    /// on `log` returns no plan and the cut comes out empty; a copy stuck on
    /// `log` finds no message inside and the cut comes out empty; only both on
    /// `publish` yields the one message. The narrower [180, 320] case below then
    /// exercises the copy's membership test on its own.
    #[test]
    fn cut_window_selects_extents_and_messages_on_the_time_source() -> anyhow::Result<()> {
        let root = test_dir("segment-domain")?;
        let rec = root.join("rec.mcap");
        // log_time ascends 100/200/300; publish_time is independent: 250/150/350.
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 100, 250, b"a")),
                raw_record(op::MESSAGE, &message_body_pub(1, 1, 200, 150, b"b")),
                raw_record(op::MESSAGE, &message_body_pub(1, 2, 300, 350, b"c")),
            ],
        )?;
        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);

        // A second recording whose log span [100, 200] and publish span
        // [1_000, 2_000] do not overlap: the window [900, 1_500] falls inside
        // the publish span and entirely outside the log one, so the extent is
        // planned on `publish` and not on `log`.
        let apart = root.join("apart.mcap");
        write_raw(
            &apart,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 100, 1_000, b"a")),
                raw_record(op::MESSAGE, &message_body_pub(1, 1, 200, 2_000, b"b")),
            ],
        )?;
        let apart_planner = indexed(&[&apart])?;

        let on_publish_apart = cut_window(
            &apart_planner,
            900,
            1_500,
            &root.join("apart-publish.mcap"),
            &stage_tx,
            TimeSource::Publish,
        )?;
        assert_eq!(
            read_clip(&on_publish_apart[0].out_path)?
                .into_iter()
                .map(|(_, t)| t)
                .collect::<Vec<u64>>(),
            vec![100],
            "both halves on publish: the extent is planned and the message at \
             publish 1_000 is inside"
        );

        // The same window on `log`: the extent's log span [100, 200] is nowhere
        // near it, so the planner returns nothing and the cut is a valid empty
        // clip. This is the assertion a planner hard-wired to one domain fails.
        let on_log_apart = cut_window(
            &apart_planner,
            900,
            1_500,
            &root.join("apart-log.mcap"),
            &stage_tx,
            TimeSource::Log,
        )?;
        assert_eq!(
            on_log_apart.len(),
            1,
            "an uncovered window still yields one segment"
        );
        assert_eq!(
            on_log_apart[0].extents_read, 0,
            "the planner selects extents on the window's own domain, so a log \
             window past the log span reads none"
        );
        assert!(read_clip(&on_log_apart[0].out_path)?.is_empty());

        // The window [180, 320] holds log_times 200 and 300 on `log`; on
        // `publish` only the message published at 250 is inside, and its
        // log_time — what a reader of the clip sees — is 100. Both domains plan
        // the same extent here, so this pins the copy's membership test alone.
        let on_log = cut_window(
            &planner,
            180,
            320,
            &root.join("log.mcap"),
            &stage_tx,
            TimeSource::Log,
        )?;
        let mut log_times: Vec<u64> = read_clip(&on_log[0].out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        log_times.sort_unstable();
        assert_eq!(log_times, vec![200, 300], "log windows on log_time");

        let on_publish = cut_window(
            &planner,
            180,
            320,
            &root.join("publish.mcap"),
            &stage_tx,
            TimeSource::Publish,
        )?;
        assert_eq!(
            read_clip(&on_publish[0].out_path)?
                .into_iter()
                .map(|(_, t)| t)
                .collect::<Vec<u64>>(),
            vec![100],
            "publish windows on publish_time; only the message published at 250 is inside"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
