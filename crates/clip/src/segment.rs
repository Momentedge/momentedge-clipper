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
//! What a name the output directory already holds costs is the caller's
//! ([`Publication`]), and it is the one thing decided before anything is staged:
//! a recorder following a live recording publishes beside the earlier clip,
//! while a cut from a finished recording — which would copy the same bytes into
//! the same name a second time — refuses the whole window instead.
//!
//! The module is deliberately free of any notion of where the window came from
//! or who is told about it. Nothing here decides a window's bounds, waits for
//! anything before cutting, or announces the clips it produced — a caller
//! resolves the bounds, does whatever waiting its own clock and coverage
//! demand, and reports the returned [`cut::ClipStats`] however it likes.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread;

use anyhow::Context as _;
use crossbeam_channel::{Sender, bounded, unbounded};

use crate::index::{WindowPlan, WindowPlanner};
use crate::manifest::{CutRequest, Planned, WindowCoverage};
use crate::select::ChannelSelection;
use crate::{cut, panic_text};

/// One queued clip-segment staging: the window-plan snapshot [`cut_window`] took
/// for one source recording, the request that window came from, what the planner
/// found for it, the base output path, and the reply channel. Queued by
/// [`cut_window`]; dequeued FIFO by the staging workers, which run the bulk copy
/// into the capturing dir and reply a [`cut::StagedClip`]. Publication is not
/// theirs — [`cut_window`] publishes the staged segments itself, once the
/// window's segment count is known and the names are settled.
///
/// The [`CutRequest`] rides in the job rather than in the pool because it is a
/// property of the *window*, not of the workers. Its clock domain has to choose
/// both the extents the planner returned and the stamp each message's membership
/// is tested on, and a job that carried only the bounds would let those two be
/// answered from different clocks — a clip silently holding the wrong messages
/// rather than an error. It is also what the segment's manifest is written from,
/// which is why the trigger travels this far down: a clip states what asked for
/// it, and the only way to state it truthfully is to carry it to the writer.
#[derive(Debug)]
pub struct StageJob {
    plan: WindowPlan,
    request: Arc<CutRequest>,
    planned: Planned,
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
/// codec and the [`ChannelSelection`] are the two settings fixed for the pool's
/// lifetime and captured here — both are properties of the output, not of any
/// one window, and no window may disagree with either. The selection travelling
/// with the pool rather than with a window is what makes one configuration cut
/// the same channel set on the device and out of the finished recording
/// afterwards. The window's clock domain travels in the job instead (see
/// [`StageJob`]). A panicking stage is caught and replied as an error — per-job
/// isolation, the pool outlives it.
pub fn spawn_stage_workers(
    parallelism: usize,
    compression: Option<mcap::Compression>,
    selection: ChannelSelection,
) -> Sender<StageJob> {
    let (tx, rx) = unbounded::<StageJob>();
    for i in 0..parallelism.max(1) {
        let rx = rx.clone();
        let selection = selection.clone();
        thread::Builder::new()
            .name(format!("stage-{i}"))
            .spawn(move || {
                for job in rx.iter() {
                    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        cut::stage_clip(
                            &job.plan,
                            &job.out_path,
                            &job.request,
                            job.planned,
                            compression,
                            &selection,
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

/// What a cut does about a clip the output directory already holds under a name
/// this window could publish under.
///
/// The two callers of this path want opposite things from the same collision,
/// because their inputs differ. On a vehicle a taken name means a *second*
/// trigger asked for the same instant and name, and dropping its clip loses data
/// that will never come back — so the recorder publishes beside the earlier
/// clip. A cut from a finished recording is replayable: the same recording and
/// the same trigger describe the same window and copy the same bytes, so a taken
/// name means this cut has already been made, and a second file would be a
/// duplicate rather than data. It refuses instead.
///
/// The policy is per cut rather than fixed for the staging pool because it is a
/// property of the input — whether it can be cut again — not of the output the
/// pool's codec and channel selection describe.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Publication {
    /// Publish beside the existing clip, under the `_<n>`-suffixed sibling
    /// [`cut::publish_clip`] resolves the collision to.
    Suffix,
    /// Refuse the whole window before anything is staged, naming the clip that
    /// is already there ([`ClipExists`]).
    Refuse,
}

/// A clip this window would have written is already in the output directory,
/// under [`Publication::Refuse`].
///
/// The window it names is refused whole: the check runs before the plan is
/// taken, so no segment of it is staged, published, or left half-written, and a
/// multi-segment window whose remaining names are free is refused along with the
/// one that is not.
#[derive(Debug, thiserror::Error)]
#[error(
    "{} already exists: this window has been cut into this output directory \
     before, and cutting it again writes a second copy of the same clip rather \
     than new data. Move or delete it, or cut into a different output \
     directory, to cut this window again",
    existing.display()
)]
pub struct ClipExists {
    /// The clip already on disk: the window's own base name, or a segment or
    /// suffixed sibling beside it.
    pub existing: PathBuf,
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
/// The window lives entirely in the request's time source: it picks the extents
/// the planner returns and the stamp each message's membership is tested on.
/// Whatever has to happen before the cut — a postroll wall floor, a wait for the
/// recording to cover the window end — is the caller's, and so is telling anyone
/// about the clips this returns. `coverage` is that caller's verdict on the wait
/// it did: every segment's manifest repeats it, so a clip that ends early says
/// whether the recording had got there yet.
///
/// `publication` is what a name the output directory already holds costs: a
/// `_<n>` sibling beside the earlier clip, or a refusal of the whole window
/// before anything is staged ([`Publication`]).
pub fn cut_window(
    planner: &dyn WindowPlanner,
    request: &Arc<CutRequest>,
    coverage: WindowCoverage,
    base_out_path: &Path,
    publication: Publication,
    stage_tx: &Sender<StageJob>,
) -> anyhow::Result<Vec<cut::ClipStats>> {
    // 0. The publication policy, applied before anything else happens: under
    //    `Refuse` a clip already published under any name this window could take
    //    ends the cut here — before a plan is taken, before a byte is staged —
    //    so the refusal creates nothing in the output directory or its capturing
    //    area, and a window whose other segment names are still free is refused
    //    whole rather than half-written.
    match publication {
        Publication::Refuse => {
            if let Some(existing) = existing_clip(base_out_path).with_context(|| {
                format!(
                    "reading the output directory for {}",
                    base_out_path.display()
                )
            })? {
                return Err(ClipExists { existing }.into());
            }
        }
        // A taken name is a second trigger, resolved at publish by a `_<n>`
        // sibling; nothing to decide here.
        Publication::Suffix => {}
    }

    // 1. One multi-file snapshot on the window's time source — each plan pins its
    //    own recording's Arc<File>, so a retention prune or rollover after this
    //    cannot pull the bytes out.
    let plans = planner.plan_window(request.start_ns(), request.end_ns(), request.time_source());

    // 2. The window-level facts every segment's manifest repeats. `files` is
    //    counted here, before the plans are consumed: it is what separates a
    //    clip empty because no recording held any byte of the window from one
    //    empty because the bytes held no message inside it.
    let planned = Planned {
        files: plans.len(),
        coverage,
    };

    // 3. Stage one segment per plan (FIFO worker pool), or one empty segment
    //    when no recording covers the window — the empty path needs no source
    //    file (a channelless MCAP is just magic + manifest + summary + footer).
    let mut staged: Vec<cut::StagedClip> = if plans.is_empty() {
        vec![stage_segment(
            stage_tx,
            WindowPlan::empty(),
            request,
            planned,
            base_out_path,
        )?]
    } else {
        let mut v = Vec::with_capacity(plans.len());
        for plan in plans {
            v.push(stage_segment(
                stage_tx,
                plan,
                request,
                planned,
                base_out_path,
            )?);
        }
        v
    };

    // 4. Drop empty segments when the window produced real data elsewhere, but
    //    keep one so an all-empty window still announces a valid clip.
    if staged.len() > 1 {
        if staged.iter().any(|c| !c.is_empty()) {
            staged.retain(|c| !c.is_empty());
        } else {
            staged.truncate(1);
        }
    }

    // 5. Publish the staged segments, naming them only now the count is known:
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
    request: &Arc<CutRequest>,
    planned: Planned,
    out_path: &Path,
) -> anyhow::Result<cut::StagedClip> {
    let (reply_tx, reply_rx) = bounded(1);
    stage_tx
        .send(StageJob {
            plan,
            request: request.clone(),
            planned,
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

/// The clip already beside `base_out_path` that a window published under that
/// base would collide with: the base name itself, or any `<stem>_<digits><ext>`
/// next to it — the one shape both a segment name (`_00`, `_01`, …) and a
/// suffixed sibling (`_1`, `_2`, …) take.
///
/// A window's segment count is settled only once staging has run, so a check
/// that must run *before* anything is staged cannot ask about the names this
/// particular window will use. It asks about every name a window under this base
/// could take, which refuses slightly more than strictly necessary: a
/// single-segment window is refused by a stray `<stem>_00.mcap` it would never
/// have written. That is the trade the right way round — a false refusal costs a
/// rename and a re-run, while a duplicate nobody refused is two files claiming
/// to be the same clip.
///
/// The lowest-sorting collision is the one returned, so a refusal reads the same
/// on every run. An output directory that does not exist yet holds nothing and
/// collides with nothing.
fn existing_clip(base_out_path: &Path) -> std::io::Result<Option<PathBuf>> {
    let dir = match base_out_path.parent() {
        Some(p) if !p.as_os_str().is_empty() => p,
        _ => Path::new("."),
    };
    let base = base_out_path.file_name().unwrap_or_default();
    let stem = base_out_path
        .file_stem()
        .unwrap_or_default()
        .to_string_lossy();
    let ext = base_out_path
        .extension()
        .map(|e| format!(".{}", e.to_string_lossy()))
        .unwrap_or_default();
    let prefix = format!("{stem}_");

    let entries = match std::fs::read_dir(dir) {
        Ok(entries) => entries,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(e),
    };
    let mut collision: Option<std::ffi::OsString> = None;
    for entry in entries {
        let name = entry?.file_name();
        if name != base && !is_numbered_sibling(&name.to_string_lossy(), &prefix, &ext) {
            continue;
        }
        if collision.as_ref().is_none_or(|lowest| name < *lowest) {
            collision = Some(name);
        }
    }
    Ok(collision.map(|name| dir.join(name)))
}

/// Whether `name` is `<prefix><digits><ext>` — a segment or suffixed sibling of
/// the base name `prefix` and `ext` were taken from. The digits are not parsed:
/// what matters is the shape a published clip's name has, not the number in it.
fn is_numbered_sibling(name: &str, prefix: &str, ext: &str) -> bool {
    name.strip_prefix(prefix)
        .and_then(|rest| rest.strip_suffix(ext))
        .is_some_and(|digits| !digits.is_empty() && digits.bytes().all(|b| b.is_ascii_digit()))
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
    use crate::index::{Extent, PlanSource, RecordingIndex, Span, Stamps, op};
    use crate::manifest::read_manifest;
    use crate::testing::{
        channel_body, index_file, message_body_pub, planned_one_file, raw_record, read_clip,
        scan_to_end, test_dir, window_request, write_raw, write_recording,
    };

    /// The window `[start_ns, end_ns]` on `source`, in the `Arc` the cut path
    /// shares between one window's segments.
    fn log_request(start_ns: u64, end_ns: u64, source: TimeSource) -> Arc<CutRequest> {
        Arc::new(window_request(start_ns, end_ns, source))
    }

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

    /// A planner that must never be asked for a window: a refused cut returns
    /// before it plans one, so reaching this is the refusal happening too late.
    struct Unplannable;

    impl WindowPlanner for Unplannable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            panic!("a refused window is never planned")
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

    /// The clip filenames an output directory holds, sorted — the capturing
    /// subdirectory itself excluded, since it is not published output.
    fn published(out_dir: &Path) -> anyhow::Result<Vec<String>> {
        let mut names = Vec::new();
        for entry in std::fs::read_dir(out_dir)? {
            let name = entry?.file_name().to_string_lossy().into_owned();
            if name != ".capturing" {
                names.push(name);
            }
        }
        names.sort();
        Ok(names)
    }

    /// How many files are sitting in an output directory's capturing area.
    fn staged(out_dir: &Path) -> anyhow::Result<usize> {
        Ok(std::fs::read_dir(out_dir.join(".capturing"))?.count())
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
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let out = root.join("clip.mcap");
        let cut = |start_ns: u64, end_ns: u64| {
            let planner = planner.clone();
            let stage_tx = stage_tx.clone();
            let out = out.clone();
            std::thread::spawn(move || {
                cut_window(
                    planner.as_ref(),
                    &log_request(start_ns, end_ns, TimeSource::Log),
                    WindowCoverage::Covered,
                    &out,
                    Publication::Suffix,
                    &stage_tx,
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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let out = root.join("clip.mcap");
        let plan = || {
            planner
                .plan_window(0, 300, TimeSource::Log)
                .into_iter()
                .next()
                .expect("the recording covers the window")
        };
        let first = stage_segment(
            &stage_tx,
            plan(),
            &log_request(0, 300, TimeSource::Log),
            planned_one_file(),
            &out,
        )?;
        let second = stage_segment(
            &stage_tx,
            plan(),
            &log_request(0, 300, TimeSource::Log),
            planned_one_file(),
            &out,
        )?;

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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let base = root.join("clip.mcap");
        let stats = cut_window(
            &planner,
            &log_request(1_500, 5_500, TimeSource::Log),
            WindowCoverage::Covered,
            &base,
            Publication::Suffix,
            &stage_tx,
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

    /// The names in an output directory that make a window under `base` a
    /// duplicate, and the ones that only look like they do.
    ///
    /// Both published shapes count — the `_NN` a multi-segment window writes and
    /// the `_<n>` a suffixing publish resolves a collision to — because a window
    /// whose segment count is not yet known could take either. A name that
    /// merely shares the stem does not: refusing on `clip_x.mcap` would make an
    /// unrelated file in the output directory able to block a cut forever.
    #[test]
    fn existing_clip_matches_the_base_and_every_numbered_sibling() -> anyhow::Result<()> {
        let root = test_dir("existing")?;
        let base = root.join("clip.mcap");

        assert_eq!(
            existing_clip(&root.join("absent").join("clip.mcap"))?,
            None,
            "an output directory that does not exist yet collides with nothing"
        );
        assert_eq!(
            existing_clip(&base)?,
            None,
            "an empty directory collides with nothing"
        );

        for name in [
            "clip_x.mcap",
            "clip_.mcap",
            "clipper.mcap",
            "clip_00.mcap.bak",
            "other_00.mcap",
        ] {
            std::fs::write(root.join(name), b"x")?;
            assert_eq!(
                existing_clip(&base)?,
                None,
                "{name} is not a clip this window could have written"
            );
            std::fs::remove_file(root.join(name))?;
        }

        for name in ["clip.mcap", "clip_00.mcap", "clip_1.mcap", "clip_123.mcap"] {
            std::fs::write(root.join(name), b"x")?;
            assert_eq!(
                existing_clip(&base)?,
                Some(root.join(name)),
                "{name} is a clip a window under this base publishes as"
            );
            std::fs::remove_file(root.join(name))?;
        }

        // Several collisions at once: the refusal names the same one on every
        // run rather than whichever the directory happened to yield first.
        for name in ["clip_07.mcap", "clip_02.mcap", "clip.mcap"] {
            std::fs::write(root.join(name), b"x")?;
        }
        assert_eq!(existing_clip(&base)?, Some(root.join("clip.mcap")));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The cloud policy: a second cut of a window already in the output
    /// directory refuses instead of publishing a second copy of it.
    ///
    /// The first cut writes its clip; the second names that clip, fails, and
    /// leaves the directory exactly as the first left it — no `_1` sibling, and
    /// nothing stranded in the capturing area, because the refusal happens
    /// before a plan is taken or a byte is staged.
    #[test]
    fn cut_window_refuses_a_window_already_in_the_output_directory() -> anyhow::Result<()> {
        let root = test_dir("refuse-rerun")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");
        cut::reset_capturing_dir(&out_dir)?;

        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let base = out_dir.join("clip.mcap");

        let first = cut_window(
            &planner,
            &log_request(0, 300, TimeSource::Log),
            WindowCoverage::Covered,
            &base,
            Publication::Refuse,
            &stage_tx,
        )?;
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].out_path, base);

        // The second cut is handed a planner that panics if it is asked for a
        // window: the refusal is reached before the window is planned, so
        // nothing downstream of it — the plan, the staging copy, the publish —
        // runs at all.
        let err = cut_window(
            &Unplannable,
            &log_request(0, 300, TimeSource::Log),
            WindowCoverage::Covered,
            &base,
            Publication::Refuse,
            &stage_tx,
        )
        .unwrap_err();
        assert_eq!(
            err.downcast_ref::<ClipExists>().map(|e| e.existing.clone()),
            Some(base.clone()),
            "the refusal names the clip that is already there: {err:#}"
        );
        assert_eq!(
            published(&out_dir)?,
            vec!["clip.mcap".to_string()],
            "the refused run publishes nothing, least of all a suffixed sibling"
        );
        assert_eq!(
            staged(&out_dir)?,
            0,
            "the refusal happens before staging, so the capturing area stays empty"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The device policy over the same collision, which the cloud one must not
    /// have changed: a second cut of the same window publishes beside the first.
    ///
    /// The two runs are the same call but for the [`Publication`] argument, so
    /// this is the rival the refusal has to be told apart from.
    #[test]
    fn cut_window_publishes_beside_a_window_already_there_when_suffixing() -> anyhow::Result<()> {
        let root = test_dir("suffix-rerun")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");
        cut::reset_capturing_dir(&out_dir)?;

        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let base = out_dir.join("clip.mcap");
        let cut_it = || {
            cut_window(
                &planner,
                &log_request(0, 300, TimeSource::Log),
                WindowCoverage::Covered,
                &base,
                Publication::Suffix,
                &stage_tx,
            )
        };

        assert_eq!(cut_it()?[0].out_path, base);
        assert_eq!(cut_it()?[0].out_path, out_dir.join("clip_1.mcap"));
        assert_eq!(
            published(&out_dir)?,
            vec!["clip.mcap".to_string(), "clip_1.mcap".to_string()],
            "a second live trigger's clip is data, and lands beside the first"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window that would straddle a rollover is refused whole when only one
    /// of the segments it would write is already there.
    ///
    /// `clip_00.mcap` exists and `clip_01.mcap` does not, so a check made per
    /// segment as each is published would write the second half of a clip whose
    /// first half it refused. The check runs once, before staging, and neither
    /// segment is written.
    #[test]
    fn cut_window_refuses_a_multi_segment_window_whole() -> anyhow::Result<()> {
        let root = test_dir("refuse-segment")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;
        let out_dir = root.join("clipped");
        cut::reset_capturing_dir(&out_dir)?;
        // The window's first segment, left behind by an earlier run; its second
        // segment's name is free.
        std::fs::write(out_dir.join("clip_00.mcap"), b"an earlier segment")?;

        let planner = indexed(&[&split0, &split1])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let err = cut_window(
            &planner,
            &log_request(1_500, 5_500, TimeSource::Log),
            WindowCoverage::Covered,
            &out_dir.join("clip.mcap"),
            Publication::Refuse,
            &stage_tx,
        )
        .unwrap_err();

        assert_eq!(
            err.downcast_ref::<ClipExists>().map(|e| e.existing.clone()),
            Some(out_dir.join("clip_00.mcap")),
            "the refusal names the segment that is already there: {err:#}"
        );
        assert_eq!(
            published(&out_dir)?,
            vec!["clip_00.mcap".to_string()],
            "the free segment name stays free: the window is refused whole"
        );
        assert_eq!(staged(&out_dir)?, 0, "nothing was staged either");

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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let base = root.join("clip.mcap");
        let stats = cut_window(
            &planner,
            &log_request(500, 600, TimeSource::Log),
            WindowCoverage::Covered,
            &base,
            Publication::Suffix,
            &stage_tx,
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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let base = root.join("clip.mcap");
        let stats = cut_window(
            &planner,
            &log_request(1_500, 5_500, TimeSource::Log),
            WindowCoverage::Covered,
            &base,
            Publication::Suffix,
            &stage_tx,
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

    /// A plan pointing at bytes that are not record-framed, so the stage that
    /// copies it always fails. Its `time` span covers everything, so any window
    /// selects it.
    struct Unreadable(PlanSource);

    impl WindowPlanner for Unreadable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            vec![WindowPlan {
                source: Some(self.0.clone()),
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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let doomed = Unreadable(PlanSource {
            path: junk.clone(),
            file: Arc::new(File::open(&junk)?),
        });
        let err = cut_window(
            &doomed,
            &log_request(0, u64::MAX, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("bad.mcap"),
            Publication::Suffix,
            &stage_tx,
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
            &log_request(0, 1_000, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("good.mcap"),
            Publication::Suffix,
            &stage_tx,
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
    struct Unallocatable(PlanSource);

    impl WindowPlanner for Unallocatable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            vec![WindowPlan {
                source: Some(self.0.clone()),
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
    /// This is the arm [`spawn_stage_workers`]' `catch_unwind` exists for. A
    /// worker that unwinds out of its receive loop is gone for good: it drops
    /// the reply channel on the way out, so the caller that queued the job reads
    /// a disconnect and reports "the staging worker dropped the job" — an error
    /// naming the plumbing rather than the fault, and one that says nothing
    /// about the pool now being one worker short. With `parallelism` at its
    /// default of one, that is every later cut. The sibling test above covers
    /// the ordinary `Err` return; this one kills the worker mid-copy and asserts
    /// the same two properties survive it.
    #[test]
    fn a_panicking_stage_is_reported_and_the_pool_serves_the_next_job() -> anyhow::Result<()> {
        let root = test_dir("stage-panics")?;
        let src = root.join("src.mcap");
        write_recording(&src, false, &[("/t", 100)])?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let doomed = Unallocatable(PlanSource {
            path: src.clone(),
            file: Arc::new(File::open(&src)?),
        });
        let err = cut_window(
            &doomed,
            &log_request(0, u64::MAX, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("boom.mcap"),
            Publication::Suffix,
            &stage_tx,
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
            &log_request(0, 1_000, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("good.mcap"),
            Publication::Suffix,
            &stage_tx,
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
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

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
            &log_request(900, 1_500, TimeSource::Publish),
            WindowCoverage::Covered,
            &root.join("apart-publish.mcap"),
            Publication::Suffix,
            &stage_tx,
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
            &log_request(900, 1_500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("apart-log.mcap"),
            Publication::Suffix,
            &stage_tx,
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
            &log_request(180, 320, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("log.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        let mut log_times: Vec<u64> = read_clip(&on_log[0].out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        log_times.sort_unstable();
        assert_eq!(log_times, vec![200, 300], "log windows on log_time");

        let on_publish = cut_window(
            &planner,
            &log_request(180, 320, TimeSource::Publish),
            WindowCoverage::Covered,
            &root.join("publish.mcap"),
            Publication::Suffix,
            &stage_tx,
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

    /// The three ways a clip comes out empty are told apart from their manifests
    /// alone.
    ///
    /// This is what the output-directory contract needs and the file itself
    /// cannot express: all three clips hold zero messages and their message
    /// sections are byte-identical, so without the record a consumer cannot tell
    /// a correct empty clip from a broken recorder. Each case is built for real
    /// here rather than asserted on hand-made keys.
    #[test]
    fn an_empty_clip_says_which_kind_of_empty_it_is() -> anyhow::Result<()> {
        let root = test_dir("empty-kinds")?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        // 1. Nothing covered the window: no recording exists, and the wait for
        //    coverage timed out — the recorder never got there.
        let nothing = cut_window(
            &Indexes(Vec::new()),
            &log_request(500, 600, TimeSource::Log),
            WindowCoverage::Short,
            &root.join("nothing.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        // 2. The window fell in a gap between splits: split0 ends at 2_000 and
        //    split1 starts at 5_000, so no file holds a byte of [2_500, 4_500] —
        //    but the recording ran well past the window, so the coverage wait
        //    was satisfied.
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;
        let gap = cut_window(
            &indexed(&[&split0, &split1])?,
            &log_request(2_500, 4_500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("gap.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        // 3. No message matched: one recording whose extent brackets the window
        //    — so it is planned and read — but whose messages all fall outside
        //    it.
        let quiet = root.join("quiet.mcap");
        write_recording(&quiet, false, &[("/t", 100), ("/t", 900)])?;
        let unmatched = cut_window(
            &indexed(&[&quiet])?,
            &log_request(400, 500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("unmatched.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        // All three are valid, empty, and — without the manifest —
        // indistinguishable.
        for stats in [&nothing[0], &gap[0], &unmatched[0]] {
            assert_eq!(stats.messages_copied, 0);
            assert!(read_clip(&stats.out_path)?.is_empty());
        }

        let kind = |stats: &cut::ClipStats| -> anyhow::Result<(String, String)> {
            let m = read_manifest(&stats.out_path)?.expect("every clip carries a manifest");
            assert_eq!(m["clip.messages"], "0");
            Ok((m["source.files_planned"].clone(), m["clip.short"].clone()))
        };
        assert_eq!(
            kind(&nothing[0])?,
            ("0".to_string(), "true".to_string()),
            "nothing covered the window: no file planned, and the cut ran short"
        );
        assert_eq!(
            kind(&gap[0])?,
            ("0".to_string(), "false".to_string()),
            "a gap between splits: no file planned, but the recording had passed \
             the window end"
        );
        assert_eq!(
            kind(&unmatched[0])?,
            ("1".to_string(), "false".to_string()),
            "no message matched: a file was planned and read, and the recording \
             covered the window"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window straddling a split writes one manifest per segment, each naming
    /// its own source file, and both agreeing on the window-level facts.
    ///
    /// A single record for the whole window would have to name one of the two
    /// files and be wrong about the other; the per-segment record is what lets a
    /// consumer trace each half of a straddling clip back to the recording it
    /// came from.
    #[test]
    fn each_segment_of_a_straddling_window_names_its_own_source() -> anyhow::Result<()> {
        let root = test_dir("two-seg-manifest")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let planner = indexed(&[&split0, &split1])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let stats = cut_window(
            &planner,
            &log_request(1_500, 5_500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        assert_eq!(stats.len(), 2, "a straddling window yields two segments");

        let mut sources = Vec::new();
        for seg in &stats {
            let m = read_manifest(&seg.out_path)?.expect("every segment carries a manifest");
            assert_eq!(
                m["source.files_planned"], "2",
                "both segments report the window's own file count"
            );
            assert_eq!(m["window.start_ns"], "1500");
            assert_eq!(m["window.end_ns"], "5500");
            assert_eq!(m["clip.messages"], "1");
            sources.push(m["source.path"].clone());
        }
        sources.sort();
        assert_eq!(
            sources,
            vec![split0.display().to_string(), split1.display().to_string()],
            "each segment names the split it was cut from"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
