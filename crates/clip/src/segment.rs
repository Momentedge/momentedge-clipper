//! One window over a planned recording becomes a durable clip: claim the
//! directory the clip is, plan the window, copy one file per source recording
//! through a worker pool, drop the empties, number what is left, and write the
//! document that says the clip is complete.
//!
//! [`cut_window`] is the whole of it, over any [`WindowPlanner`], and it is the
//! **one entry point that decides where a clip goes**: a caller hands it the
//! window and the output directory, never a path, so every clip in every output
//! directory is laid out by one rule ([`crate::layout`]).
//!
//! **Three outcomes, and the type says which.** A window is cut, or it is
//! skipped because its id is already taken, or it fails — and a caller cannot
//! confuse the first two, because [`CutOutcome`] makes them different values
//! rather than an empty result. A skip is the answer to a repeated trigger, to a
//! restart meeting its own earlier clips, and to two processes racing for one
//! id; it is not an error and it announces nothing.
//!
//! The module is deliberately free of any notion of where the window came from
//! or who is told about it. Nothing here decides a window's bounds, waits for
//! anything before cutting, or announces the clip it produced — a caller
//! resolves the bounds, does whatever waiting its own clock and coverage
//! demand, and reports the returned [`Clip`] however it likes.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::thread;

use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use log::warn;

use crate::id::ClipId;
use crate::index::{WindowPlan, WindowPlanner};
use crate::layout::{Claim, ClipDir};
use crate::manifest::{ClipMetadata, CutRequest, Planned, WindowCoverage};
use crate::select::ChannelSelection;
use crate::{cut, layout, panic_text};

/// One queued copy: the window-plan snapshot [`cut_window`] took for one source
/// recording, the request that window came from, the path inside the claimed
/// clip directory to write it at, and the reply channel. Queued by
/// [`cut_window`]; dequeued FIFO by the staging workers, which run the bulk copy
/// and reply a `cut::StagedClip`. Naming is not theirs — [`cut_window`] names
/// the finished files itself, once the clip's file count is known.
///
/// The [`CutRequest`] rides in the job rather than in the pool because it is a
/// property of the *window*, not of the workers. Its clock domain has to choose
/// both the extents the planner returned and the stamp each message's membership
/// is tested on, and a job that carried only the bounds would let those two be
/// answered from different clocks — a clip silently holding the wrong messages
/// rather than an error. It is also what the file's id record is written from,
/// which is why the trigger travels this far down.
#[derive(Debug)]
pub struct StageJob {
    plan: WindowPlan,
    request: Arc<CutRequest>,
    out_path: PathBuf,
    reply: Sender<anyhow::Result<cut::StagedClip>>,
}

/// Spawn the fixed staging worker pool: `parallelism` threads consuming one
/// shared FIFO channel. One worker is the conservative default, and it is the
/// interesting case: the bulk copies then serialize in submission order, which
/// is what you want whenever the staging reads compete for disk bandwidth with
/// whatever is still writing the recording. Widening the pool trades that back.
///
/// Each worker runs only `cut::stage_clip` — the bulk copy into the clip
/// directory the caller claimed — and replies the `cut::StagedClip`. The window plan rides in the job,
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
#[must_use]
#[expect(
    clippy::needless_pass_by_value,
    reason = "the pool owns its selection for the process lifetime and hands each \
              worker a clone; borrowing here would only move that clone to the caller"
)]
pub fn spawn_stage_workers(
    parallelism: usize,
    compression: Option<mcap::Compression>,
    selection: ChannelSelection,
) -> Sender<StageJob> {
    let (tx, rx) = unbounded::<StageJob>();
    for i in 0..parallelism.max(1) {
        let rx = rx.clone();
        let selection = selection.clone();
        #[expect(
            clippy::expect_used,
            reason = "the pool is built once at startup; a recorder that cannot \
                      spawn its staging workers can cut no clip at all, so failing \
                      loudly here beats starting a recorder that silently never cuts"
        )]
        thread::Builder::new()
            .name(format!("stage-{i}"))
            .spawn(move || stage_jobs(&rx, compression, &selection))
            .expect("spawning staging worker");
    }
    tx
}

/// One staging worker's whole life: take jobs off the shared FIFO until the
/// queue closes, stage each one, and answer its sender.
///
/// A panic inside [`cut::stage_clip`] is caught and replied as that job's error
/// rather than unwinding the worker, so one malformed window costs one clip
/// instead of a thread out of the pool. A reply that cannot be sent means the
/// caller that queued the job is gone; there is no one left to care about the
/// clip, so the send failure is dropped.
fn stage_jobs(
    rx: &Receiver<StageJob>,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) {
    for job in rx {
        let staged = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            cut::stage_clip(
                &job.plan,
                &job.out_path,
                &job.request,
                compression,
                selection,
            )
        }))
        .unwrap_or_else(|payload| {
            Err(anyhow::anyhow!(
                "staging panicked: {}",
                panic_text(payload.as_ref())
            ))
        });
        let _ = job.reply.send(staged);
    }
}

/// What one window's cut did.
///
/// Three things can happen to a window and a caller has to handle each
/// differently, so the type says which: a clip was written, the id was already
/// taken, or the cut failed (the `Err` of the [`anyhow::Result`] this rides in).
/// A skip is neither of the other two — nothing was written, and nothing went
/// wrong — and making it a variant rather than an empty [`Clip`] is what stops a
/// caller announcing a clip it did not cut.
#[derive(Debug)]
#[must_use = "a window was cut or skipped, and the two are announced differently"]
pub enum CutOutcome {
    /// The window claimed its directory, filled it and completed it.
    Cut(Clip),
    /// `<out_dir>/<id>` was already there — a complete clip, or residue from a
    /// cut that died — so this window wrote nothing and changed nothing.
    /// Carries that directory, already named in a warning by the cut.
    Skipped(PathBuf),
}

/// One complete clip on disk: the directory, and what each file in it holds.
///
/// The directory is the clip — it is what a completion announcement names and
/// what a consumer syncs — and the per-file counters are what a caller logs.
#[derive(Debug)]
pub struct Clip {
    /// `<out_dir>/<id>`, holding every file of this clip and the document that
    /// says it is complete.
    pub dir: PathBuf,
    /// One entry per MCAP file written, in file-number order from `_0`.
    pub files: Vec<cut::ClipStats>,
}

/// Cut one window out of the recordings a [`WindowPlanner`] serves into a clip
/// directory under `out_dir`, or skip it because that directory is already
/// there.
///
/// **This is where a clip's location is decided.** A caller supplies the output
/// directory and the window; everything under it — the directory named by the
/// window's [`ClipId`], one `<id>_N.mcap` per contributing source recording, and
/// the `clip_metadata.yaml` that completes it — is [`crate::layout`]'s, so no
/// caller formats a clip path and the layout moves in one edit.
///
/// **The claim is the first thing that happens.** One atomic `mkdir` either
/// gives this window the directory or tells it the id is taken, which is the
/// whole answer to a repeated trigger, to a restart meeting its own earlier
/// clips, and to two windows or two processes racing for one id. A taken id is
/// warned about here — so both subcommands say the same thing — and returned as
/// [`CutOutcome::Skipped`].
///
/// A window inside one recording yields a single file, `<id>_0.mcap`; one
/// straddling a rollover (a bag split or a restart the planner indexed while
/// running) yields one file per source recording, recovered from the planner's
/// retained collection. Empty files are dropped when the window produced real
/// data elsewhere, but one is always kept so an all-empty window (a rollover
/// gap, all relevant recordings pruned, or nothing recorded yet) still yields a
/// valid clip; the numbering is the position among what is left.
///
/// **A cut that fails after the claim takes its directory with it**, so a
/// directory without `clip_metadata.yaml` is crash residue and nothing else.
///
/// The window lives entirely in the request's time source: it picks the extents
/// the planner returns and the stamp each message's membership is tested on.
/// Whatever has to happen before the cut — a postroll wall floor, a wait for the
/// recording to cover the window end — is the caller's, and so is telling anyone
/// about the clip this returns. `coverage` is that caller's verdict on the wait
/// it did: the clip's document repeats it, so a clip that ends early says whether
/// the recording had got there yet.
pub fn cut_window(
    planner: &dyn WindowPlanner,
    request: &Arc<CutRequest>,
    coverage: WindowCoverage,
    out_dir: &Path,
    stage_tx: &Sender<StageJob>,
) -> anyhow::Result<CutOutcome> {
    layout::prepare_out_dir(out_dir)?;
    let dir = match ClipDir::claim(out_dir, ClipId::of(request))? {
        Claim::Ours(dir) => dir,
        Claim::Taken(path) => {
            warn!(
                "clip {} is already there; skipping this window. A clip is written \
                 once: an id that is taken means this window has been cut, or a cut \
                 of it died leaving the directory behind. Remove it to cut the \
                 window again",
                path.display()
            );
            return Ok(CutOutcome::Skipped(path));
        }
    };

    // From here the directory is this cut's, and the only two ways out are a
    // complete clip and a removed directory.
    match fill(&dir, planner, request, coverage, stage_tx) {
        Ok(files) => Ok(CutOutcome::Cut(Clip {
            dir: dir.path().to_path_buf(),
            files,
        })),
        Err(e) => Err(dir.discard(e)),
    }
}

/// Fill a claimed clip directory and complete it: plan the window, copy one file
/// per source recording, drop the empties, name what is left, and write the
/// document last.
///
/// Every failure is the caller's to answer by removing the directory, which is
/// why this is a separate function: it may return early anywhere without
/// leaving a rule to remember at each `?`.
#[expect(
    clippy::similar_names,
    reason = "`planner`, `plans` and `planned` are the domain's own three words: \
              what serves a window's plans, the per-file plans it served, and the \
              window-level facts the clip's document states. Renaming any of them \
              would cost more than the similarity does"
)]
fn fill(
    dir: &ClipDir,
    planner: &dyn WindowPlanner,
    request: &Arc<CutRequest>,
    coverage: WindowCoverage,
    stage_tx: &Sender<StageJob>,
) -> anyhow::Result<Vec<cut::ClipStats>> {
    // 1. One multi-file snapshot on the window's time source — each plan pins its
    //    own recording's Arc<File>, so a retention prune or rollover after this
    //    cannot pull the bytes out.
    let plans = planner.plan_window(request.start_ns(), request.end_ns(), request.time_source());

    // 2. The window-level facts the clip's document states. `files` is counted
    //    here, before the plans are consumed: it is what separates a clip empty
    //    because no recording held any byte of the window from one empty because
    //    the bytes held no message inside it.
    let planned = Planned {
        files: plans.len(),
        coverage,
    };

    // 3. Copy one file per plan (FIFO worker pool), or one empty file when no
    //    recording covers the window — the empty path needs no source recording
    //    (a channelless MCAP is just magic + record + summary + footer). Each is
    //    written under a staging name derived from its plan's position, since
    //    the number it will be named by is not known yet.
    let mut staged: Vec<cut::StagedClip> = if plans.is_empty() {
        vec![stage_file(stage_tx, WindowPlan::empty(), request, dir, 0)?]
    } else {
        let mut v = Vec::with_capacity(plans.len());
        for (i, plan) in plans.into_iter().enumerate() {
            v.push(stage_file(stage_tx, plan, request, dir, i)?);
        }
        v
    };

    // 4. Drop empty files when the window produced real data elsewhere, but keep
    //    one so an all-empty window still yields a valid clip. A dropped file is
    //    removed rather than forgotten: it is already on disk under its staging
    //    name, and the directory this cut is about to complete must hold its
    //    `<id>_N.mcap` files and its document and nothing else.
    let mut dropped = Vec::new();
    if staged.len() > 1 {
        if staged.iter().any(|c| !c.is_empty()) {
            let (kept, empty) = staged.into_iter().partition::<Vec<_>, _>(|c| !c.is_empty());
            staged = kept;
            dropped = empty;
        } else {
            dropped = staged.split_off(1);
        }
    }
    for clip in dropped {
        clip.discard();
    }

    // 5. Name what is left `<id>_0.mcap`, `<id>_1.mcap`, … — the position among
    //    the files that contributed, not the source recording's place in the
    //    collection.
    let mut files = Vec::with_capacity(staged.len());
    for (n, clip) in staged.into_iter().enumerate() {
        files.push(clip.place(dir, n)?);
    }

    // 6. The document, written once every file above is durable and named: its
    //    presence is what makes the clip complete.
    dir.complete(&ClipMetadata::of(request, planned, &files))?;
    Ok(files)
}

/// Queue one file's copy on the staging workers and block on the reply. The
/// plan is [`cut_window`]'s snapshot of one source recording, so a job that
/// waits in the FIFO queue still copies the recording it was taken from.
fn stage_file(
    stage_tx: &Sender<StageJob>,
    plan: WindowPlan,
    request: &Arc<CutRequest>,
    dir: &ClipDir,
    plan_idx: usize,
) -> anyhow::Result<cut::StagedClip> {
    let (reply_tx, reply_rx) = bounded(1);
    stage_tx
        .send(StageJob {
            plan,
            request: request.clone(),
            out_path: dir.staging(plan_idx),
            reply: reply_tx,
        })
        .map_err(|_| anyhow::anyhow!("the staging workers are gone"))?;
    reply_rx
        .recv()
        .map_err(|_| anyhow::anyhow!("the staging worker dropped the job"))?
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::assert_is_empty,
        clippy::items_after_statements,
        clippy::too_many_lines,
        reason = "a failed unwrap or a panicking index is a failing test, and \
                  `assert!(x.is_empty())` names the claim better than the \
                  empty-array `assert_eq!` the lint asks for, \
                  and a test that builds a fixture, drives it and asserts on the \
                  whole result is long, nested and argument-heavy by \
                  construction — splitting one would scatter the case it states, \
                  as would hoisting a one-test planner out of the test that needs it"
    )]

    use std::collections::HashMap;
    use std::fs::File;
    use std::sync::Arc;

    use super::*;
    use crate::TimeSource;
    use crate::index::{Extent, PlanSource, RecordingIndex, Span, Stamps, op};
    use crate::layout::{METADATA_FILE, read_metadata};
    use crate::manifest::{CLIP_ID_KEY, read_manifest};
    use crate::testing::{
        channel_body, index_file, message_body_pub, raw_record, read_clip, scan_to_end, test_dir,
        window_request, write_raw, write_recording,
    };

    /// The window `[start_ns, end_ns]` on `source`, in the `Arc` the cut path
    /// shares between one window's files.
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

    /// A planner that must never be asked for a window: a skipped window returns
    /// before it plans one, so reaching this is the claim happening too late.
    struct Unplannable;

    impl WindowPlanner for Unplannable {
        fn plan_window(
            &self,
            _start_ns: u64,
            _end_ns: u64,
            _source: TimeSource,
        ) -> Vec<WindowPlan> {
            panic!("a skipped window is never planned")
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

    /// The names a directory holds, sorted. An output directory holds clip
    /// directories and nothing else; a clip directory holds its files and its
    /// metadata document.
    fn entries(dir: &Path) -> anyhow::Result<Vec<String>> {
        let mut names: Vec<String> = std::fs::read_dir(dir)?
            .map(|e| Ok(e?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<_>>()?;
        names.sort();
        Ok(names)
    }

    /// Where `request`'s clip goes under `out_dir`: the directory named by its
    /// id. The scheme is the cut's, so a test states the shape it is about and
    /// reads the name back from the code that decides it.
    fn clip_dir(out_dir: &Path, request: &CutRequest) -> PathBuf {
        out_dir.join(ClipId::of(request).to_string())
    }

    /// The `n`th file of `request`'s clip: `<id>/<id>_N.mcap`.
    fn clip_file(out_dir: &Path, request: &CutRequest, n: usize) -> PathBuf {
        clip_dir(out_dir, request).join(format!("{}_{n}.mcap", ClipId::of(request)))
    }

    /// The clip a cut wrote, or a failure naming what it did instead.
    ///
    /// `whole`'s tests and `tail::handler`'s each spell this out again rather
    /// than share it, and that is the cheaper trade. The match is exhaustive
    /// with no catch-all arm, so a new [`CutOutcome`] variant is a compile error
    /// at all three sites — the copies cannot drift, which is the only thing
    /// sharing would buy. Reaching `tail` would cost the opposite: the helper
    /// would have to be published from `clip::testing` behind `test-support`,
    /// making a five-line test unwrap part of this crate's API for good.
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

    /// A clip is a directory named by its id, and that computation lives here
    /// rather than in either binary.
    ///
    /// The second name is the property the id buys: a trigger whose text would
    /// be hostile in a path produces the same shape of name as any other,
    /// because none of that text reaches it. [`crate::id`] is where the id's own
    /// contract is pinned.
    #[test]
    fn a_clip_is_a_directory_named_by_its_id() -> anyhow::Result<()> {
        let root = test_dir("named")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");

        let named = |name: &str| {
            Arc::new(CutRequest::new(
                crate::testing::TEST_PRODUCER,
                crate::Trigger {
                    name: name.to_string(),
                    description: "hard brake over 0.8 g".to_string(),
                    trigger_time: crate::Stamp { sec: 0, nanosec: 0 },
                    preroll: 5_000_000_000,
                    postroll: 5_000_000_000,
                },
                1_726_300_000_000_000_000,
                TimeSource::Log,
            ))
        };
        assert_eq!(
            ClipId::of(&named("brake-event")).to_string(),
            "1726300000000000000_fc43-6475-ade8-4730"
        );

        // A name that would escape the output directory is one plain directory
        // under it, holding one plainly named file.
        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let hostile = named("../escape");
        let clip = cut(cut_window(
            &planner,
            &hostile,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);

        assert_eq!(clip.dir, clip_dir(&out_dir, &hostile));
        assert_eq!(clip.dir.parent(), Some(out_dir.as_path()));
        assert_eq!(
            entries(&out_dir)?,
            vec![ClipId::of(&hostile).to_string()],
            "the output directory gains one clip directory and nothing else"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window inside one recording: one directory, one numbered file in it,
    /// the document beside it, and nothing else anywhere.
    ///
    /// The file is `_0` even though it is the only one — always numbered, so a
    /// consumer reads one rule rather than two. The file's own metadata record
    /// carries the clip's id and nothing more: everything else a clip says is in
    /// the document, stated once.
    #[test]
    fn a_window_inside_one_recording_is_a_directory_of_one_numbered_file() -> anyhow::Result<()> {
        let root = test_dir("one-file")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(0, 300, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);

        let id = ClipId::of(&request).to_string();
        assert_eq!(clip.dir, out_dir.join(&id));
        assert_eq!(
            entries(&clip.dir)?,
            vec![format!("{id}_0.mcap"), METADATA_FILE.to_string()],
            "one numbered file and the document that completes the clip"
        );
        assert_eq!(clip.files.len(), 1);
        assert_eq!(clip.files[0].out_path, clip_file(&out_dir, &request, 0));
        assert_eq!(
            read_clip(&clip.files[0].out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 200)]
        );

        assert_eq!(
            read_manifest(&clip.files[0].out_path)?,
            Some(std::collections::BTreeMap::from([(
                CLIP_ID_KEY.to_string(),
                id.clone()
            )])),
            "a clip's file carries its id and nothing else"
        );
        let metadata = read_metadata(&clip.dir)?;
        assert_eq!(metadata.clip.id, id);
        assert_eq!(metadata.clip.messages, 2);
        assert_eq!(metadata.sources.len(), 1);
        assert_eq!(metadata.sources[0].file, format!("{id}_0.mcap"));
        assert_eq!(metadata.sources[0].path, Some(rec.display().to_string()));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Two concurrent cuts of one window: exactly one directory, complete, and
    /// the other window is skipped.
    ///
    /// One window is what makes this the race worth pinning: the clip's
    /// directory is named from the window, so two windows that agree on it are
    /// exactly the pair that aim at one directory. The `mkdir` decides it — one
    /// caller creates it, the other is told it exists — so the loser never
    /// writes a byte, and no lock or retry is involved.
    #[test]
    fn two_concurrent_cuts_of_one_window_yield_one_clip() -> anyhow::Result<()> {
        let root = test_dir("overlap")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            false,
            &[("/t", 100), ("/t", 200), ("/t", 300), ("/t", 400)],
        )?;
        let out_dir = root.join("clipped");

        let planner = Arc::new(indexed(&[&rec])?);
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(100, 300, TimeSource::Log);
        let start = || {
            let planner = planner.clone();
            let stage_tx = stage_tx.clone();
            let request = request.clone();
            let out_dir = out_dir.clone();
            std::thread::spawn(move || {
                cut_window(
                    planner.as_ref(),
                    &request,
                    WindowCoverage::Covered,
                    &out_dir,
                    &stage_tx,
                )
            })
        };
        let (ha, hb) = (start(), start());
        let outcomes = [ha.join().unwrap()?, hb.join().unwrap()?];

        let dir = clip_dir(&out_dir, &request);
        let cuts: Vec<&Clip> = outcomes
            .iter()
            .filter_map(|o| match o {
                CutOutcome::Cut(clip) => Some(clip),
                CutOutcome::Skipped(_) => None,
            })
            .collect();
        assert_eq!(cuts.len(), 1, "exactly one of the two windows wrote a clip");
        assert_eq!(cuts[0].dir, dir);
        assert_eq!(
            entries(&out_dir)?,
            vec![ClipId::of(&request).to_string()],
            "and the output directory holds that one clip"
        );
        assert_eq!(read_metadata(&dir)?.clip.messages, 3, "it is complete");
        assert_eq!(
            read_clip(&clip_file(&out_dir, &request, 0))?,
            vec![
                ("/t".to_string(), 100),
                ("/t".to_string(), 200),
                ("/t".to_string(), 300)
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window straddling a rollover: one file per source recording, numbered
    /// `_0` and `_1` in source order, inside one directory.
    #[test]
    fn cut_window_recovers_across_a_rollover_into_two_files() -> anyhow::Result<()> {
        let root = test_dir("two-seg")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&split0, &split1])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(1_500, 5_500, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);

        assert_eq!(clip.files.len(), 2, "a straddling window yields two files");
        let (first, second) = (
            clip_file(&out_dir, &request, 0),
            clip_file(&out_dir, &request, 1),
        );
        assert_eq!(
            clip.files
                .iter()
                .map(|f| f.out_path.clone())
                .collect::<Vec<_>>(),
            vec![first.clone(), second.clone()],
            "numbered in source order"
        );
        let id = ClipId::of(&request).to_string();
        assert_eq!(
            entries(&clip.dir)?,
            vec![
                format!("{id}_0.mcap"),
                format!("{id}_1.mcap"),
                METADATA_FILE.to_string()
            ],
            "the two files and the document, and nothing left staging"
        );
        // The files tile the window: split0's tail, then split1's head.
        assert_eq!(read_clip(&first)?, vec![("/t".to_string(), 2_000)]);
        assert_eq!(read_clip(&second)?, vec![("/t".to_string(), 5_000)]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The id is claimed by the directory's existence, whatever the directory
    /// holds — so a complete clip and a half-written one are both skipped, and
    /// neither is touched.
    ///
    /// The second cut is handed a planner that panics if it is asked for a
    /// window: the skip is reached before the window is planned, so nothing
    /// downstream of it — the plan, the copy, the naming — runs at all. That is
    /// also why a skip costs nothing on a device already busy cutting.
    #[test]
    fn a_window_whose_id_is_taken_is_skipped_whatever_the_directory_holds() -> anyhow::Result<()> {
        let root = test_dir("skip-taken")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&rec])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(0, 300, TimeSource::Log);

        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);
        let dir = clip.dir.clone();
        let before = std::fs::read(&clip.files[0].out_path)?;

        // A complete clip: skipped, and not a byte of it changes.
        let again = skipped(cut_window(
            &Unplannable,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);
        assert_eq!(again, dir, "the skip names the directory that is there");
        assert_eq!(std::fs::read(&clip.files[0].out_path)?, before);
        assert_eq!(entries(&out_dir)?, vec![ClipId::of(&request).to_string()]);

        // Crash residue — a directory with no document in it — is claimed just
        // as hard, and nothing about it is repaired.
        std::fs::remove_file(dir.join(METADATA_FILE))?;
        let residue = entries(&dir)?;
        assert_eq!(
            skipped(cut_window(
                &Unplannable,
                &request,
                WindowCoverage::Covered,
                &out_dir,
                &stage_tx,
            )?),
            dir,
            "an incomplete directory is a taken id too"
        );
        assert_eq!(
            entries(&dir)?,
            residue,
            "residue is left exactly as it was found: a skip repairs nothing"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// `cut_window` with two source recordings where BOTH files are empty keeps
    /// exactly one of them: the drop step truncates to 1 rather than dropping
    /// all, so an all-empty window still yields a valid clip.
    #[test]
    fn cut_window_all_empty_multi_file_keeps_one() -> anyhow::Result<()> {
        let root = test_dir("all-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // Two recordings the window [1_700, 1_900] falls inside the span of —
        // so both are planned and read — and that neither holds a message in:
        // both copies come out empty (messages_copied == 0).
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 1_500), ("/t", 2_500)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&split0, &split1])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(1_700, 1_900, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);

        assert_eq!(
            clip.files.len(),
            1,
            "an all-empty multi-file window keeps exactly one file"
        );
        assert_eq!(clip.files[0].messages_copied, 0, "the kept file is empty");
        assert_eq!(clip.files[0].out_path, clip_file(&out_dir, &request, 0));
        assert!(read_clip(&clip.files[0].out_path)?.is_empty());
        let metadata = read_metadata(&clip.dir)?;
        assert_eq!(metadata.window.files_planned, 2, "both were planned");
        assert_eq!(metadata.sources.len(), 1, "one file came out of them");
        assert_eq!(
            entries(&clip.dir)?,
            vec![
                format!("{}_0.mcap", metadata.clip.id),
                METADATA_FILE.to_string()
            ],
            "the dropped file leaves nothing behind: a complete clip directory \
             holds the files its document names and the document, and no staging \
             name"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window over three recordings whose middle one holds none of its
    /// messages: two files, renumbered `_0` and `_1` by the position they end up
    /// at, and a document that says three were planned.
    ///
    /// The numbering is the position **after** the empty files are dropped, not
    /// the source recording's place in the collection, which is why `_1` here
    /// holds the third recording's data.
    #[test]
    fn cut_window_drops_an_empty_file_and_renumbers_what_is_left() -> anyhow::Result<()> {
        let root = test_dir("drop-empty")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        let split2 = root.join("rec_2.mcap");
        // The window [4_000, 8_500] takes one message from split0 and one from
        // split2; split1's extent spans the window — so it is planned and read
        // — but neither of its messages falls inside it.
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 5_000)])?;
        write_recording(&split1, false, &[("/t", 3_000), ("/t", 9_000)])?;
        write_recording(&split2, false, &[("/t", 8_000), ("/t", 9_500)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&split0, &split1, &split2])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(4_000, 8_500, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);

        assert_eq!(clip.files.len(), 2, "the empty middle file is dropped");
        assert_eq!(
            clip.files
                .iter()
                .map(|f| f.out_path.clone())
                .collect::<Vec<_>>(),
            vec![
                clip_file(&out_dir, &request, 0),
                clip_file(&out_dir, &request, 1),
            ],
            "what is left is renumbered from 0 by position"
        );
        assert_eq!(
            read_clip(&clip.files[1].out_path)?,
            vec![("/t".to_string(), 8_000)]
        );

        let metadata = read_metadata(&clip.dir)?;
        assert_eq!(
            metadata.window.files_planned, 3,
            "three recordings were planned over"
        );
        assert_eq!(
            metadata
                .sources
                .iter()
                .map(|s| (s.file.clone(), s.path.clone()))
                .collect::<Vec<_>>(),
            vec![
                (
                    format!("{}_0.mcap", metadata.clip.id),
                    Some(split0.display().to_string())
                ),
                (
                    format!("{}_1.mcap", metadata.clip.id),
                    Some(split2.display().to_string())
                ),
            ],
            "and two contributed, each entry naming the recording behind its file"
        );
        assert_eq!(
            entries(&clip.dir)?,
            vec![
                format!("{}_0.mcap", metadata.clip.id),
                format!("{}_1.mcap", metadata.clip.id),
                METADATA_FILE.to_string()
            ],
            "the dropped middle file leaves nothing behind — not even the \
             staging name its copy was written under"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A plan pointing at bytes that are not record-framed, so the copy that
    /// reads it always fails. Its `time` span covers everything, so any window
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

    /// A copy that fails is reported to the caller that queued it, the clip's
    /// directory is removed, and the pool survives to serve the next job.
    ///
    /// The removal is what keeps "a directory without the document is crash
    /// residue" true: an ordinary failure must not leave one behind, or an
    /// operator finding one learns nothing from it. The pool half is the
    /// property that lets one bad window stay one bad window: the workers are a
    /// long-lived fixed pool shared by every concurrent cut, so a job that died
    /// taking the pool with it would silently stall every clip after it, with no
    /// error anywhere, because the callers would simply block on replies that
    /// never come.
    #[test]
    fn a_failing_copy_removes_the_clip_and_the_pool_serves_the_next_job() -> anyhow::Result<()> {
        let root = test_dir("stage-fails")?;
        let junk = root.join("junk.bin");
        std::fs::write(&junk, [0xFFu8; 64])?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let doomed = Unreadable(PlanSource {
            path: junk.clone(),
            file: Arc::new(File::open(&junk)?),
        });
        let bad = log_request(0, u64::MAX, TimeSource::Log);
        let err =
            cut_window(&doomed, &bad, WindowCoverage::Covered, &out_dir, &stage_tx).unwrap_err();
        assert!(
            format!("{err:#}").contains("framing inconsistent"),
            "the copy's own error reaches the caller: {err:#}"
        );
        assert!(
            !clip_dir(&out_dir, &bad).exists(),
            "the failed cut took its directory with it"
        );
        assert!(
            entries(&out_dir)?.is_empty(),
            "and left nothing else in the output directory"
        );

        // The same pool, immediately afterwards: a well-formed window still cuts.
        let planner = indexed(&[&rec])?;
        let good = log_request(0, 1_000, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &good,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);
        assert_eq!(
            read_clip(&clip.files[0].out_path)?,
            vec![("/t".to_string(), 100), ("/t".to_string(), 200)],
            "the worker that replied an error is still serving jobs"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window whose second file cannot be copied leaves no completed clip, and
    /// in particular no metadata document.
    ///
    /// The document is written after every file is durable, so there is no
    /// instant at which an incomplete clip carries one — which is exactly what
    /// an upload pipeline filters on. The first recording here copies fine; the
    /// second is junk, so the failure lands between the two.
    #[test]
    fn a_failure_before_the_last_file_leaves_no_metadata() -> anyhow::Result<()> {
        let root = test_dir("partial")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let junk = root.join("junk.bin");
        std::fs::write(&junk, [0xFFu8; 64])?;
        let out_dir = root.join("clipped");

        /// One good plan followed by one whose bytes are not record-framed.
        struct GoodThenBad(Indexes, PlanSource);
        impl WindowPlanner for GoodThenBad {
            fn plan_window(
                &self,
                start_ns: u64,
                end_ns: u64,
                source: TimeSource,
            ) -> Vec<WindowPlan> {
                let mut plans = self.0.plan_window(start_ns, end_ns, source);
                plans.extend(Unreadable(self.1.clone()).plan_window(start_ns, end_ns, source));
                plans
            }
        }

        let planner = GoodThenBad(
            indexed(&[&rec])?,
            PlanSource {
                path: junk.clone(),
                file: Arc::new(File::open(&junk)?),
            },
        );

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(0, 300, TimeSource::Log);
        let err = cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )
        .unwrap_err();

        assert!(
            format!("{err:#}").contains("framing inconsistent"),
            "{err:#}"
        );
        assert!(
            !clip_dir(&out_dir, &request).join(METADATA_FILE).exists(),
            "a clip whose files are not all there never carries the document"
        );
        assert!(
            entries(&out_dir)?.is_empty(),
            "and the whole directory went with the failure"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// When the failed clip's own directory cannot be removed, the error names
    /// it — the one thing an operator has to act on, since every later window
    /// with that id is skipped until the directory is gone.
    ///
    /// The planner here yanks the claimed directory out from under the cut and
    /// leaves a plain file in its place, so both the copy and the removal fail:
    /// a hostile filesystem, simulated deterministically rather than by
    /// permissions, which say nothing to a test running as root.
    #[test]
    fn a_removal_that_fails_names_the_directory_it_left_behind() -> anyhow::Result<()> {
        let root = test_dir("undeletable")?;
        let out_dir = root.join("clipped");
        let request = log_request(0, 300, TimeSource::Log);
        let dir = clip_dir(&out_dir, &request);

        /// Replaces the claimed clip directory with a regular file before
        /// answering, so writing into it and removing it both fail.
        struct Vandal(PathBuf);
        impl WindowPlanner for Vandal {
            fn plan_window(
                &self,
                _start_ns: u64,
                _end_ns: u64,
                _source: TimeSource,
            ) -> Vec<WindowPlan> {
                std::fs::remove_dir(&self.0).expect("the cut claimed the directory");
                std::fs::write(&self.0, b"not a directory").expect("standing in its way");
                vec![WindowPlan::empty()]
            }
        }

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let err = cut_window(
            &Vandal(dir.clone()),
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )
        .unwrap_err();

        let text = format!("{err:#}");
        assert!(
            text.contains(&dir.display().to_string()),
            "the error names the directory an operator has to remove: {text}"
        );
        assert!(
            text.contains("could not be removed"),
            "and says that is what it is: {text}"
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

    /// A copy that **panics** is caught, reported to the caller as an error, and
    /// leaves the pool serving.
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
    fn a_panicking_copy_is_reported_and_the_pool_serves_the_next_job() -> anyhow::Result<()> {
        let root = test_dir("stage-panics")?;
        let src = root.join("src.mcap");
        write_recording(&src, false, &[("/t", 100)])?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let out_dir = root.join("clipped");

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        let doomed = Unallocatable(PlanSource {
            path: src.clone(),
            file: Arc::new(File::open(&src)?),
        });
        let boom = log_request(0, u64::MAX, TimeSource::Log);
        let err =
            cut_window(&doomed, &boom, WindowCoverage::Covered, &out_dir, &stage_tx).unwrap_err();
        let text = format!("{err:#}");
        assert!(
            text.contains("staging panicked"),
            "the panic is reported as an error, not lost with the thread: {text}"
        );
        assert!(
            entries(&out_dir)?.is_empty(),
            "and the clip it was writing is gone"
        );

        // The same pool, after a worker caught a panic: a well-formed window
        // still cuts. Without the catch this call never returns.
        let planner = indexed(&[&rec])?;
        let clip = cut(cut_window(
            &planner,
            &log_request(0, 1_000, TimeSource::Log),
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);
        assert_eq!(
            read_clip(&clip.files[0].out_path)?,
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

        let on_publish_apart = cut(cut_window(
            &apart_planner,
            &log_request(900, 1_500, TimeSource::Publish),
            WindowCoverage::Covered,
            &root.join("apart-publish"),
            &stage_tx,
        )?);
        assert_eq!(
            read_clip(&on_publish_apart.files[0].out_path)?
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
        let on_log_apart = cut(cut_window(
            &apart_planner,
            &log_request(900, 1_500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("apart-log"),
            &stage_tx,
        )?);
        assert_eq!(
            on_log_apart.files.len(),
            1,
            "an uncovered window still yields one file"
        );
        assert_eq!(
            on_log_apart.files[0].extents_read, 0,
            "the planner selects extents on the window's own domain, so a log \
             window past the log span reads none"
        );
        assert!(read_clip(&on_log_apart.files[0].out_path)?.is_empty());

        // The window [180, 320] holds log_times 200 and 300 on `log`; on
        // `publish` only the message published at 250 is inside, and its
        // log_time — what a reader of the clip sees — is 100. Both domains plan
        // the same extent here, so this pins the copy's membership test alone.
        let on_log = cut(cut_window(
            &planner,
            &log_request(180, 320, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("log"),
            &stage_tx,
        )?);
        let mut log_times: Vec<u64> = read_clip(&on_log.files[0].out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        log_times.sort_unstable();
        assert_eq!(log_times, vec![200, 300], "log windows on log_time");

        let on_publish = cut(cut_window(
            &planner,
            &log_request(180, 320, TimeSource::Publish),
            WindowCoverage::Covered,
            &root.join("publish"),
            &stage_tx,
        )?);
        assert_eq!(
            read_clip(&on_publish.files[0].out_path)?
                .into_iter()
                .map(|(_, t)| t)
                .collect::<Vec<u64>>(),
            vec![100],
            "publish windows on publish_time; only the message published at 250 is inside"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The three ways a clip comes out empty are told apart from their documents
    /// alone.
    ///
    /// This is what the output-directory contract needs and the files themselves
    /// cannot express: all three clips hold zero messages and their message
    /// sections are byte-identical, so without the document a consumer cannot
    /// tell a correct empty clip from a broken recorder. Each case is built for
    /// real here rather than asserted on hand-made fields.
    #[test]
    fn an_empty_clip_says_which_kind_of_empty_it_is() -> anyhow::Result<()> {
        let root = test_dir("empty-kinds")?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());

        // 1. Nothing covered the window: no recording exists, and the wait for
        //    coverage timed out — the recorder never got there.
        let nothing = cut(cut_window(
            &Indexes(Vec::new()),
            &log_request(500, 600, TimeSource::Log),
            WindowCoverage::Short,
            &root.join("nothing"),
            &stage_tx,
        )?);

        // 2. The window fell in a gap between splits: split0 ends at 2_000 and
        //    split1 starts at 5_000, so no recording holds a byte of
        //    [2_500, 4_500] — but the recording ran well past the window, so the
        //    coverage wait was satisfied.
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;
        let gap = cut(cut_window(
            &indexed(&[&split0, &split1])?,
            &log_request(2_500, 4_500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("gap"),
            &stage_tx,
        )?);

        // 3. No message matched: one recording whose extent brackets the window
        //    — so it is planned and read — but whose messages all fall outside
        //    it.
        let quiet = root.join("quiet.mcap");
        write_recording(&quiet, false, &[("/t", 100), ("/t", 900)])?;
        let unmatched = cut(cut_window(
            &indexed(&[&quiet])?,
            &log_request(400, 500, TimeSource::Log),
            WindowCoverage::Covered,
            &root.join("unmatched"),
            &stage_tx,
        )?);

        // All three are valid, empty, one-file clips and — without the document
        // — indistinguishable.
        for clip in [&nothing, &gap, &unmatched] {
            assert_eq!(clip.files.len(), 1);
            assert_eq!(clip.files[0].messages_copied, 0);
            assert!(read_clip(&clip.files[0].out_path)?.is_empty());
        }

        let kind = |clip: &Clip| -> anyhow::Result<(usize, bool)> {
            let m = read_metadata(&clip.dir)?;
            assert_eq!(m.clip.messages, 0);
            Ok((m.window.files_planned, m.clip.short))
        };
        assert_eq!(
            kind(&nothing)?,
            (0, true),
            "nothing covered the window: no recording planned, and the cut ran short"
        );
        assert_eq!(
            kind(&gap)?,
            (0, false),
            "a gap between splits: no recording planned, but the recording had \
             passed the window end"
        );
        assert_eq!(
            kind(&unmatched)?,
            (1, false),
            "no message matched: a recording was planned and read, and it covered \
             the window"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A window straddling a split states one `sources` entry per file, each
    /// naming the recording behind it, under one set of window-level facts.
    ///
    /// A document that named one of the two recordings would be wrong about the
    /// other; the per-file list is what lets a consumer trace each half of a
    /// straddling clip back to where it came from. Every file of the clip is
    /// stamped with the same id, so one carried away from the directory can
    /// still be grouped.
    #[test]
    fn each_file_of_a_straddling_window_names_its_own_source() -> anyhow::Result<()> {
        let root = test_dir("two-seg-manifest")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;
        let out_dir = root.join("clipped");

        let planner = indexed(&[&split0, &split1])?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = log_request(1_500, 5_500, TimeSource::Log);
        let clip = cut(cut_window(
            &planner,
            &request,
            WindowCoverage::Covered,
            &out_dir,
            &stage_tx,
        )?);
        assert_eq!(clip.files.len(), 2, "a straddling window yields two files");

        let id = ClipId::of(&request).to_string();
        let metadata = read_metadata(&clip.dir)?;
        assert_eq!(metadata.clip.id, id);
        assert_eq!(metadata.clip.messages, 2, "one message from each recording");
        assert_eq!(metadata.window.files_planned, 2);
        assert_eq!(metadata.window.start_ns, 1_500);
        assert_eq!(metadata.window.end_ns, 5_500);
        assert_eq!(
            metadata
                .sources
                .iter()
                .map(|s| (s.file.clone(), s.path.clone(), s.messages))
                .collect::<Vec<_>>(),
            vec![
                (
                    format!("{id}_0.mcap"),
                    Some(split0.display().to_string()),
                    1
                ),
                (
                    format!("{id}_1.mcap"),
                    Some(split1.display().to_string()),
                    1
                ),
            ],
        );

        for file in &clip.files {
            let record = read_manifest(&file.out_path)?.expect("every file carries its id");
            assert_eq!(record[CLIP_ID_KEY], id, "both files are the same clip");
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
