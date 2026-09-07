//! Tail of the growing MCAP file behind a continuous `ros2 bag record`.
//!
//! Reading a recording while it is still being written is sound because of the
//! MCAP format itself — bytes below the current end of file never change, and
//! every record is length-prefixed — and the incremental scan that exploits
//! that, together with the extent index and schema/channel registry it fills,
//! is [`clip::index`]. This module is the live half around it: discovering
//! recordings, keeping each open, driving that scan pass by pass, and owning
//! everything a scan of an already-complete file has no use for — where each
//! recording sits in the collection's lifecycle, how far the collection
//! provably reaches, and what to do when a pass faults.
//!
//! What the tail adds on top of the index is the **coverage watch**: the
//! collection-wide highest `log_time` and `publish_time` seen ([`Coverage`]). A
//! trigger handler waits on the active time source's mark until the recording
//! reaches its window end — a completeness proof on `log`, a liveness signal on
//! `publish`. The window's clock domain is selectable (`--time-source`):
//! `log_time` is the default base, `publish_time` the alternative, and the gap
//! between the two — the recorder's queue backlog plus the producer's clock
//! skew — is observable either way.
//!
//! Only the 22-byte fixed header of each top-level `Message` record is read
//! during the tail (channel id, sequence, `log_time`, `publish_time`); message
//! bodies are first touched by the extraction ([`clip::cut`]). The one exception
//! is an opt-in trigger tap ([`Tailer::with_trigger_tap`], wired only by the
//! MCAP interface): when set, the scan also lifts the full body of messages on
//! the trigger topic out as [`TriggerRecord`]s for the interface to decode by
//! `message_encoding`. With the tap unset — the default — no message body is
//! read during the scan at all.
//!
//! The tail owns a time-ordered collection of recordings. New `*.mcap` files
//! under the record dir — a rosbag2 split rolling over to `<bag>_<n+1>.mcap`, or
//! a restart recreating the bag directory — are discovered by
//! [`crate::discover`], which yields them in mtime order so several appearing
//! between polls are indexed oldest first, and indexed alongside
//! the ones already known. Each is scanned in turn; a finished recording is
//! retained for a watch window so a clip straddling a rollover recovers across
//! it (beads clipper-gl2), then pruned. Extractions hold their own file handle,
//! so a recording pruned or deleted while a clip reads it stays readable.
//!
//! Damage in the recording is tolerated the way [`clip::cut`] tolerates it at
//! extraction, and the scan itself draws the line: a damaged chunk, an
//! unparseable schema/channel, or a runt message is warned and skipped, the
//! framing intact. A **framing** fault has no resync point, so the scan stops at
//! it, having applied everything before it. The tail then retries from exactly
//! that offset — never re-attaching, never rescanning from scratch — under a
//! bounded, backing-off `MAX_SCAN_FAULTS` budget, treating a recorder restart
//! during the backoff as recovery. Only when the same byte faults through the
//! whole budget does [`Tailer::run`] return an error and the process exit for a
//! supervisor to restart: a tailer wedged on a stuck file would otherwise
//! degrade every clip to a grace-timeout cut with no other signal.

use std::fs::File;
use std::os::unix::fs::{FileExt, MetadataExt};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use clip::TimeSource;
use clip::index::{
    self, MAGIC, RecordingIndex, ScanDelta, ScanProgress, ScanSeed, WindowPlan, WindowPlanner,
};
use clip::trigger::{TriggerRecord, now_ns};
use crossbeam_channel::Sender;
use log::{info, warn};

use crate::watch::Watch;

/// Sleep between scan passes when the file has not grown.
const TAIL_POLL: Duration = Duration::from_millis(50);

/// Sleep between attempts to discover the recording file. Also the first
/// step of the scan-fault backoff (see [`SCAN_BACKOFF_CAP`]).
const DISCOVER_POLL: Duration = Duration::from_millis(200);

/// How many consecutive faulted scan passes [`Tailer::run`] tolerates
/// before giving up on a recording and returning an error. A fault is a
/// framing desync with no resync point (an oversized record length, or an IO
/// error reading a record); skipped localized damage is not a fault and never
/// counts here. The counter resets on any fault-free pass, so transient
/// trouble that clears does not accumulate toward the limit. Reaching it means
/// every retry in a row ended in a fault — usually the same stuck byte — and
/// the recorder is better restarted than tailed forever against a wall.
pub(crate) const MAX_SCAN_FAULTS: u32 = 5;

/// Ceiling on the scan-fault backoff. Between faulted passes the wait doubles
/// from [`DISCOVER_POLL`] (200, 400, 800, 1600 ms) up to this cap, so the
/// `MAX_SCAN_FAULTS` retries span roughly three seconds before exhaustion —
/// long enough to ride out a brief hiccup, short enough that a genuinely stuck
/// file is escalated promptly. The backoff is slept in `DISCOVER_POLL`
/// increments so a recorder restart (the file replaced) is noticed within one
/// increment and treated as recovery.
pub(crate) const SCAN_BACKOFF_CAP: Duration = Duration::from_millis(3200);

/// How far the recordings provably reach on each time source: the highest
/// message stamp the tail has seen on disk, across the whole collection of
/// indexed recordings. A handler waits on the high-water of the source its
/// window lives in.
///
/// **`log`** (`high_water_ns`) is a *completeness* proof. It rests on two
/// properties. First an ordering assumption: messages land in a file in
/// (approximately) non-decreasing `log_time` order. rosbag2 has that shape — one
/// writer, `log_time` stamped at receive — up to millisecond-scale interleaving
/// between concurrent subscription callbacks, which the flush and extraction
/// latency in front of every cut dwarfs. Second, the tail scans strictly in
/// order, one `current` recording at a time, oldest first, finishing each before
/// the next: a later file's coverage can never advance before an earlier file is
/// complete. So a collection-wide high-water at or past a window end implies the
/// window's messages are on disk in whichever recording holds them. The mark is
/// monotonic across retention prunes: a pruned file is below the watch floor and
/// never held the maximum, so dropping it never lowers the high-water.
///
/// **`publish`** (`publish_high_water_ns`) is a *liveness* signal only.
/// `publish_time` carries no ordering guarantee — out-of-order is normal steady
/// state — so a high-water past a window end does not prove every in-window
/// message is on disk: a message can still arrive later with an in-window
/// `publish_time` and be lost from a clip already cut. It advances the same way
/// `log` does and never regresses (the watch only raises it), so a handler's
/// wait is stable; `grace_secs` bounds the wait when the publish stream goes
/// quiet.
#[derive(Clone, Copy, Debug, Default)]
pub struct Coverage {
    pub high_water_ns: u64,
    pub publish_high_water_ns: u64,
}

impl Coverage {
    /// The high-water for the windowing `source`: `log`'s completeness
    /// high-water, or `publish`'s liveness high-water.
    pub fn for_source(&self, source: TimeSource) -> u64 {
        match source {
            TimeSource::Log => self.high_water_ns,
            TimeSource::Publish => self.publish_high_water_ns,
        }
    }
}

/// A monotonic recording sequence number, assigned at insertion. Insertion
/// order is mtime order is time order (rosbag2 opens each split/restart file
/// after closing the previous one), so a larger id is always a later recording.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash, PartialOrd, Ord)]
struct RecordingId(u64);

/// Where a recording is in its lifecycle. Exactly one recording is `Tailing` at
/// a time (the `current` one); successors wait as `New` until it finishes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RecordingState {
    /// Indexed (fd open) but not yet scanned: its 8 magic bytes may not be on
    /// disk yet, and it waits behind the `current` recording.
    New,
    /// The single recording being incrementally scanned (`current`).
    Tailing,
    /// Fully scanned to EOF (footer, inode vanished, or superseded by a
    /// length-stable successor). Nothing more will ever appear; eligible for
    /// retention pruning once its data ages past the watch floor.
    Ended,
}

/// One recording as the tail holds it: what it contains, plus where it sits in
/// the collection.
///
/// The [`RecordingIndex`] knows the recording's *content* — its open file
/// handle, how far it has been scanned, its extents and their time spans, its
/// schema/channel registry, its time bounds — and nothing about any other
/// recording. Tailing adds exactly what a *collection* of recordings needs: an
/// `id` fixing this one's place in time order, and a [`RecordingState`] saying
/// whether it is waiting behind the current file, being scanned, or finished
/// and eligible for retention pruning.
///
/// A consumer cutting a clip out of one already-complete file needs only the
/// former — it opens the file, indexes it once, and plans windows against it;
/// there is no successor, no rollover, no retention horizon. That is why the
/// split falls exactly here: the index is `clip`'s, shared with every such
/// consumer, and the two lifecycle fields stay behind with the tail that is
/// the only thing to have a lifecycle.
#[derive(Debug)]
struct Recording {
    id: RecordingId,
    state: RecordingState,
    index: RecordingIndex,
}

/// The tail-owned collection of [`Recording`]s, in time order
/// (oldest .. newest), plus which one is being incrementally scanned. The tail
/// thread is the sole writer; trigger handlers only read it (under the mutex)
/// via [`WindowPlanner::plan_window`].
#[derive(Debug, Default)]
struct TailState {
    recordings: std::collections::VecDeque<Recording>,
    /// The recording being incrementally tailed (`Tailing`). `None` before the
    /// first file is discovered or after the last one ends with no successor.
    current: Option<RecordingId>,
    /// Source of the next [`RecordingId`]; only ever increases.
    next_id: u64,
}

impl TailState {
    fn recording(&self, id: RecordingId) -> Option<&Recording> {
        self.recordings.iter().find(|r| r.id == id)
    }

    fn recording_mut(&mut self, id: RecordingId) -> Option<&mut Recording> {
        self.recordings.iter_mut().find(|r| r.id == id)
    }

    /// Index a freshly discovered recording at the back of the collection (it is
    /// the newest). Adopts it as `current` when there is none (startup, or after
    /// the last recording ended) so the first file is always tailed; otherwise
    /// it waits as a `New` successor behind the recording in flight.
    fn insert_new_recording(&mut self, path: PathBuf, file: Arc<File>) -> RecordingId {
        let id = RecordingId(self.next_id);
        self.next_id += 1;
        self.recordings.push_back(Recording {
            id,
            state: RecordingState::New,
            index: RecordingIndex::new(path, file),
        });
        if self.current.is_none() {
            self.current = Some(id);
        }
        id
    }

    /// Mark a recording as the one being scanned.
    fn mark_tailing(&mut self, id: RecordingId) {
        if let Some(r) = self.recording_mut(id) {
            r.state = RecordingState::Tailing;
        }
    }

    /// Retire the finished `current` recording and advance to the oldest
    /// remaining non-`Ended` one (the next split/restart), or to no current at
    /// all when none remain.
    fn mark_ended_and_advance(&mut self, id: RecordingId) {
        if let Some(r) = self.recording_mut(id) {
            r.state = RecordingState::Ended;
        }
        self.current = self
            .recordings
            .iter()
            .find(|r| r.state != RecordingState::Ended)
            .map(|r| r.id);
    }

    /// Whether a recording newer than `id` has been indexed — a successor is
    /// present, so rosbag2 has already closed `id`'s file.
    fn has_successor(&self, id: RecordingId) -> bool {
        self.recordings.iter().any(|r| r.id > id)
    }

    /// One single-file [`WindowPlan`] per recording overlapping
    /// `[start_ns, end_ns]` on `source`, oldest first. Empty when no recording
    /// covers the window (a rollover gap, all relevant files pruned, or nothing
    /// indexed yet) — the caller then stages one empty clip.
    fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
        self.recordings
            .iter()
            .filter_map(|r| r.index.plan(start_ns, end_ns, source))
            .collect()
    }

    /// Drop every `Ended` recording whose newest data is older than
    /// `floor_ns` — never the `current` file, never a `New` or `Tailing` one,
    /// never mid-file. Returns the dropped recordings' paths (for optional
    /// on-disk deletion). Dropping a [`Recording`] releases its
    /// `Arc<File>`, closing the descriptor once no in-flight plan still holds a
    /// clone, so the prune bounds both memory and open fds.
    fn prune(&mut self, floor_ns: u64) -> Vec<PathBuf> {
        let mut pruned = Vec::new();
        self.recordings.retain(|r| {
            let expired = r.state == RecordingState::Ended
                && Some(r.id) != self.current
                && r.index.bounds.has_messages
                && r.index.bounds.log.max < floor_ns;
            if expired {
                pruned.push(r.index.path.clone());
            }
            !expired
        });
        pruned
    }

    /// The collection-wide high-water `log_time` — the maximum over all indexed
    /// recordings. The completeness proof for `log`-domain windows.
    fn high_water_ns(&self) -> u64 {
        self.recordings
            .iter()
            .filter(|r| r.index.bounds.has_messages)
            .map(|r| r.index.bounds.log.max)
            .max()
            .unwrap_or(0)
    }

    /// The collection-wide high-water `publish_time` — the maximum over all
    /// indexed recordings. A liveness signal for `publish`-domain windows only:
    /// `publish_time` has no ordering guarantee, so this does not prove every
    /// in-window message is on disk (see [`Coverage`]).
    fn publish_high_water_ns(&self) -> u64 {
        self.recordings
            .iter()
            .filter(|r| r.index.bounds.has_messages)
            .map(|r| r.index.bounds.publish.max)
            .max()
            .unwrap_or(0)
    }
}

/// Shared tail state: the scanning thread feeds it, trigger handlers snapshot
/// it via [`WindowPlanner::plan_window`] and wait on the coverage watch.
#[derive(Debug)]
pub struct Tailer {
    state: Mutex<TailState>,
    coverage: Arc<Watch<Coverage>>,
    /// The trigger tap: the topic whose messages the scan lifts out, and the
    /// channel the MCAP interface drains them from. `None` disables it — the ROS
    /// interface reads triggers from a live subscription instead — and the scan
    /// is then byte-for-byte the timestamp-only tail, reading no message body at
    /// all. Cloned into each pass's [`ScanSeed`], which the scan turns into the
    /// sink it sends triggers down the moment it lifts them; best-effort, so a
    /// full or closed tap never stalls the scan.
    tap: Option<(String, Sender<TriggerRecord>)>,
}

impl Tailer {
    /// A fresh tailer (no trigger tap) plus the coverage watch trigger handlers
    /// wait on. The scan reads only message timestamps; triggers arrive through
    /// the ROS interface, not the file.
    pub fn new() -> (Arc<Self>, Arc<Watch<Coverage>>) {
        Self::build(None)
    }

    /// A tailer whose scan also lifts messages on `trigger_topic` out of the
    /// recording as [`TriggerRecord`]s, sending each on `trigger_tx` — the MCAP
    /// interface's trigger source. Only recordings indexed live emit triggers
    /// (no startup back-indexing); a trigger already on disk before clipper
    /// started never fires.
    pub fn with_trigger_tap(
        trigger_topic: impl Into<String>,
        trigger_tx: Sender<TriggerRecord>,
    ) -> (Arc<Self>, Arc<Watch<Coverage>>) {
        Self::build(Some((trigger_topic.into(), trigger_tx)))
    }

    fn build(tap: Option<(String, Sender<TriggerRecord>)>) -> (Arc<Self>, Arc<Watch<Coverage>>) {
        let coverage = Arc::new(Watch::new(Coverage::default()));
        (
            Arc::new(Tailer {
                state: Mutex::new(TailState::default()),
                coverage: coverage.clone(),
                tap,
            }),
            coverage,
        )
    }

    /// Tail forever: follow the directory's recordings as a time-ordered
    /// collection, scanning each in turn and recovering across rollovers.
    /// Blocking — run on its own thread.
    ///
    /// Discovery is a [`crate::discover::NewFileWatchIterator`]: each poll drains
    /// the `*.mcap` files that appeared since the last and indexes each as a
    /// `New` recording. At startup the newest existing file is adopted directly
    /// and the iterator seeded past every file present then, so a pre-existing
    /// backlog is **not** re-indexed — clipper recovers only rollovers it observes
    /// during its own run, never reconstructing offsets or footers it did not scan
    /// incrementally.
    ///
    /// The `current` recording is scanned incrementally until it finishes — a
    /// footer/DataEnd on disk, its own inode vanishing or being replaced (a
    /// record-script dir wipe + restart), or a length-stable successor appearing
    /// (an abrupt split whose footer never flushed) — the last scan to EOF
    /// having already drained every complete trailing record. Then `current`
    /// advances to the next recording. Every poll prunes `Ended` recordings
    /// whose newest data has aged past the watch floor (`watch`), releasing
    /// their fds and — when `delete_old_files` is set — unlinking them from disk.
    ///
    /// Returns only on an unrecoverable scan fault (the same byte faulting
    /// through the whole `MAX_SCAN_FAULTS` budget) or a magic mismatch; the
    /// supervisor then exits the process for a restart, since limping on would
    /// degrade every clip to a grace-timeout cut silently. A missing or empty
    /// record directory is not a fault — discovery idles until the recorder
    /// creates the bag dir, the documented startup state.
    pub fn run(&self, record_dir: &Path, watch: Duration, delete_old_files: bool) -> Result<()> {
        let watch_ns = watch.as_nanos().min(u64::MAX as u128) as u64;

        // Startup seed: adopt the newest existing recording directly, and seed
        // the iterator past every file present now so the backlog behind it is
        // not indexed — discovery yields only recordings that appear later.
        let mut discover = match newest_mcap(record_dir) {
            Some(newest) => {
                self.index_recording(&newest);
                crate::discover::NewFileWatchIterator::seeded(record_dir)
            }
            None => crate::discover::NewFileWatchIterator::new(record_dir),
        };

        let mut faults = 0u32;
        loop {
            // 1. Discover: index every file that appeared since the last poll.
            //    `by_ref` so the iterator (and its seen-inode set) survives the
            //    poll — it is drained again next iteration as files appear.
            for path in discover.by_ref() {
                self.index_recording(&path);
            }

            // 2. Prune aged-out recordings (every poll, not only at rollover) —
            //    bounds open fds and index memory even when the recorder idles.
            let floor = now_ns().saturating_sub(watch_ns);
            for path in self.state.lock().unwrap().prune(floor) {
                info!("retention: forgetting {}", path.display());
                if delete_old_files {
                    match std::fs::remove_file(&path) {
                        Ok(()) => info!("retention: deleted {}", path.display()),
                        Err(e) => warn!("retention: deleting {}: {e}", path.display()),
                    }
                }
            }

            // 3. Scan the current recording, if one is in flight.
            let Some(id) = self.current_id() else {
                std::thread::sleep(DISCOVER_POLL);
                continue;
            };
            match self.poll_current(id)? {
                PollOutcome::Progressed | PollOutcome::Ended => faults = 0,
                PollOutcome::Idle => {
                    faults = 0;
                    std::thread::sleep(TAIL_POLL);
                }
                PollOutcome::NotReady => std::thread::sleep(TAIL_POLL),
                PollOutcome::Faulted(fault) => {
                    faults += 1;
                    if faults >= MAX_SCAN_FAULTS {
                        return Err(fault).with_context(|| {
                            format!("scan faulted on {faults} consecutive passes; giving up")
                        });
                    }
                    let backoff = backoff_for(faults);
                    warn!(
                        "scan faulted ({fault:#}); retry {faults}/{MAX_SCAN_FAULTS} after {backoff:?}"
                    );
                    std::thread::sleep(backoff);
                }
            }
        }
    }

    /// The id of the recording currently being tailed, if any.
    fn current_id(&self) -> Option<RecordingId> {
        self.state.lock().unwrap().current
    }

    /// Open a freshly discovered recording and index it at the back of the
    /// collection (adopted as `current` when there is none). A file that
    /// vanished between discovery and open (a dir wipe) is skipped; the iterator
    /// has already advanced past it.
    pub(crate) fn index_recording(&self, path: &Path) {
        match File::open(path) {
            Ok(f) => {
                let id = self
                    .state
                    .lock()
                    .unwrap()
                    .insert_new_recording(path.to_path_buf(), Arc::new(f));
                info!("indexing recording {} as {id:?}", path.display());
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => warn!("opening discovered {}: {e}", path.display()),
        }
    }

    /// Retire the finished `current` recording and advance to the next.
    fn end_current(&self, id: RecordingId) {
        self.state.lock().unwrap().mark_ended_and_advance(id);
    }

    /// One scan poll of the `current` recording: verify its magic on first
    /// contact, scan the bytes added since the last pass (applying them to its
    /// index and refreshing coverage), then decide whether it has finished.
    fn poll_current(&self, id: RecordingId) -> Result<PollOutcome> {
        let (path, file, mut offset, magic_ok) = {
            let st = self.state.lock().unwrap();
            let r = st.recording(id).expect("current id is in the collection");
            (
                r.index.path.clone(),
                r.index.file.clone(),
                r.index.offset,
                r.index.magic_ok,
            )
        };

        // First contact: the writer may not have flushed the 8 magic bytes yet.
        if !magic_ok {
            if file_len(&file)? < MAGIC.len() as u64 {
                // Too short to validate. If it vanished before it ever became a
                // valid MCAP, retire it; otherwise wait for the magic.
                if inode_changed(&path, &file)? {
                    self.end_current(id);
                    return Ok(PollOutcome::Ended);
                }
                return Ok(PollOutcome::NotReady);
            }
            let mut magic = [0u8; 8];
            file.read_exact_at(&mut magic, 0)?;
            if magic != MAGIC {
                bail!("{} is not an MCAP file", path.display());
            }
            offset = MAGIC.len() as u64;
            let mut st = self.state.lock().unwrap();
            if let Some(r) = st.recording_mut(id) {
                r.index.offset = offset;
                r.index.magic_ok = true;
            }
            st.mark_tailing(id);
            info!("tailing {}", path.display());
        }

        // Incremental scan to the current EOF; applies the delta to `current`
        // (== id) and refreshes the coverage high-water.
        let progress = self.scan_available(&file, offset, file_len(&file)?);
        let made_progress = progress.offset != offset;

        if let Some(fault) = progress.fault {
            // A restart/replacement during the fault is recovery, not a
            // continued fault: the file we were stuck on is gone.
            if inode_changed(&path, &file)? {
                self.end_current(id);
                return Ok(PollOutcome::Ended);
            }
            let fault = fault.context(format!(
                "scan of {} faulted at offset {}",
                path.display(),
                progress.offset
            ));
            return Ok(PollOutcome::Faulted(fault));
        }

        // Finished on any of three signals; the scan above already drained every
        // complete record to EOF, so nothing trailing is lost.
        let inode_dead = inode_changed(&path, &file)?;
        let has_successor = self.state.lock().unwrap().has_successor(id);
        if progress.ended || inode_dead || (has_successor && !made_progress) {
            let why = if progress.ended {
                "footer on disk"
            } else if inode_dead {
                "inode vanished/replaced"
            } else {
                "successor present, length stable"
            };
            info!("recording {} ended ({why})", path.display());
            self.end_current(id);
            return Ok(PollOutcome::Ended);
        }

        Ok(if made_progress {
            PollOutcome::Progressed
        } else {
            PollOutcome::Idle
        })
    }

    /// Test/setup helper: index `file` as the sole recording and mark it the
    /// `current` one, ready to scan from past the magic. Production discovery
    /// runs through [`Self::run`]'s iterator instead of this.
    #[cfg(test)]
    pub(crate) fn attach(&self, file: Arc<File>) {
        let mut st = self.state.lock().unwrap();
        let id = st.insert_new_recording(PathBuf::new(), file);
        if let Some(r) = st.recording_mut(id) {
            r.index.magic_ok = true;
            r.index.offset = MAGIC.len() as u64;
        }
        st.mark_tailing(id);
    }

    /// Publish one scan pass's delta to the `current` recording — its registry,
    /// extents, and time bounds — record its new scan offset, and refresh the
    /// collection-wide coverage high-waters (both `log` and `publish`, each
    /// monotonic in the watch; never lowered). The brief state lock is the only
    /// one a handler's `plan_window` can contend on; the file IO above ran with
    /// no lock held.
    fn apply_to_current(&self, delta: ScanDelta, progress: &ScanProgress) {
        // Triggers were already sent straight down the tap as the scan lifted
        // them ([`ScanDelta::emit_trigger`]); apply only publishes the index and
        // advances coverage. A handler that received a trigger before this runs
        // still cannot cut its clip until coverage reaches the window end, and
        // coverage advances only here — so the cut never races ahead of the
        // index it reads.
        let (hw, phw) = {
            let mut st = self.state.lock().unwrap();
            if let Some(id) = st.current
                && let Some(r) = st.recording_mut(id)
            {
                r.index.advance(delta, progress);
            }
            (st.high_water_ns(), st.publish_high_water_ns())
        };
        self.coverage.send_if_modified(|c| {
            // Each high-water advances independently and never regresses, so a
            // handler waiting on either source sees a stable, monotonic mark. The
            // publish mark can rise past the log mark (out-of-order publish
            // times), which is exactly the liveness signal a publish window waits
            // on.
            let mut changed = false;
            if hw > c.high_water_ns {
                c.high_water_ns = hw;
                changed = true;
            }
            if phw > c.publish_high_water_ns {
                c.publish_high_water_ns = phw;
                changed = true;
            }
            changed
        });
    }

    /// One incremental pass over the `current` recording: seed the scan from
    /// what this tailer already knows, run it through
    /// [`clip::index::scan_available`], then publish the delta it produced —
    /// registry, extents, scan offset, and the coverage high-waters
    /// ([`Self::apply_to_current`]). Stops without error at the first record
    /// still being appended.
    ///
    /// The seed is read under the state lock and the lock is dropped before the
    /// scan runs, so the file IO — the whole cost of a pass — never holds the
    /// lock a handler's `plan_window` contends on. Publication afterwards is one
    /// short step under the same lock.
    ///
    /// Returns a plain [`ScanProgress`] rather than a `Result`: localized damage
    /// is skipped by the scan itself (a damaged chunk, an unparseable
    /// schema/channel, a runt message — warned and consumed), and only
    /// **framing** faults stop the pass. A framing fault — a record length past
    /// [`clip::index::MAX_RECORD_LEN`], or an IO error reading a record's header
    /// or body — leaves no resync point, so the pass returns the delta it
    /// accumulated up to the faulted record (applied here like any other) and
    /// reports `fault = Some(_)` with `offset` at that record.
    ///
    /// **Resume invariant:** that partial delta is already applied, so a caller
    /// retrying after a fault MUST resume at the returned `offset` (the faulted
    /// record), never earlier. Re-scanning an already-applied region makes the
    /// scan compute `record_end - open.offset` across bytes the open extent
    /// already spans and underflow.
    pub(crate) fn scan_available(&self, file: &File, offset: u64, file_len: u64) -> ScanProgress {
        // Take the seed under the lock, then release it: the scan's file IO must
        // not run with the state lock held.
        let seed = {
            let st = self.state.lock().unwrap();
            let current = st.current.and_then(|id| st.recording(id));
            ScanSeed {
                open: current.and_then(|r| r.index.open),
                // The tap itself is the tailer's; the recording contributes the
                // trigger channels it has already seen, so a trigger message
                // resolves against a channel defined in an earlier pass. Both
                // stay empty/idle when the tap is disabled.
                tap: self.tap.clone(),
                trigger_channels: current
                    .map(|r| r.index.trigger_channels.clone())
                    .unwrap_or_default(),
            }
        };
        let (delta, progress) = index::scan_available(file, offset, file_len, seed);
        self.apply_to_current(delta, &progress);
        progress
    }
}

/// The tail as a [`WindowPlanner`]: the shared cut path asks its planner which
/// bytes of which file cover a window and never learns whether the answer came
/// from a live tail or from a whole-file index over one finished recording.
impl WindowPlanner for Tailer {
    /// Snapshot one single-file plan per recording overlapping
    /// `[start_ns, end_ns]` on `source`, oldest first. A window inside one
    /// recording yields one plan; one straddling a rollover yields one per source
    /// file. Empty when no indexed recording covers the window.
    fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
        self.state
            .lock()
            .unwrap()
            .plan_window(start_ns, end_ns, source)
    }
}

/// The outcome of one [`Tailer::poll_current`] scan pass, telling [`Tailer::run`]
/// how to pace the next iteration and how to count faults.
#[derive(Debug)]
enum PollOutcome {
    /// New bytes were consumed; loop again immediately.
    Progressed,
    /// Caught up to EOF with the recording still live; sleep [`TAIL_POLL`].
    Idle,
    /// `current` is `New` and its 8 magic bytes are not on disk yet.
    NotReady,
    /// The recording finished; `current` has advanced.
    Ended,
    /// A framing fault with no resync point; back off and retry the same byte.
    Faulted(anyhow::Error),
}

/// The scan-fault backoff for the `n`th consecutive fault (1-based): doubling
/// from [`DISCOVER_POLL`], capped at [`SCAN_BACKOFF_CAP`]. `n == 1` yields
/// `DISCOVER_POLL`.
fn backoff_for(n: u32) -> Duration {
    let doublings = n.saturating_sub(1);
    DISCOVER_POLL
        .saturating_mul(1u32.checked_shl(doublings).unwrap_or(u32::MAX))
        .min(SCAN_BACKOFF_CAP)
}

/// The newest `*.mcap` directly under `dir` by modification time (mtime) — the
/// file most recently written to. At a rosbag2 split the just-closed
/// `<bag>_<n>.mcap` stops being written while `<bag>_<n+1>.mcap` keeps growing,
/// so the live recording carries the latest mtime; this resolves to it. Used
/// only to pick the recording adopted at startup. `None` while the directory or
/// file does not exist yet.
fn newest_mcap(dir: &Path) -> Option<PathBuf> {
    std::fs::read_dir(dir)
        .ok()?
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "mcap"))
        .max_by_key(|p| {
            std::fs::metadata(p)
                .map(|m| (m.mtime(), m.mtime_nsec()))
                .unwrap_or((i64::MIN, i64::MIN))
        })
}

/// Whether `path` no longer resolves to `file`'s open inode — the file vanished
/// (the record script wiped the bag dir) or was replaced by a restart's fresh
/// inode. The question is *"is **my** file still live"*, not *"is there a newer
/// file"* — the [`crate::discover::NewFileWatchIterator`] owns the latter. The open
/// `Arc<File>` keeps the old inode readable regardless, so the final scan still
/// drains every complete record before the recording is retired. A `NotFound` on
/// `path` means the file is gone; any other stat error propagates.
fn inode_changed(path: &Path, file: &File) -> Result<bool> {
    let by_fd = file.metadata().context("stat of tailed file")?;
    match std::fs::metadata(path) {
        Ok(m) => Ok((m.dev(), m.ino()) != (by_fd.dev(), by_fd.ino())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(true),
        Err(e) => Err(e).with_context(|| format!("stat {}", path.display())),
    }
}

fn file_len(file: &File) -> Result<u64> {
    Ok(file.metadata().context("stat of tailed file")?.len())
}

#[cfg(test)]
pub(crate) mod tests {
    use std::time::SystemTime;

    use clip::index::{MAX_RECORD_LEN, op};
    // The MCAP fixture writers, shared with `clip`'s own index tests through its
    // `test-support` feature (a dev-dependency of this crate, so a release build
    // compiles none of it).
    use clip::testing::{
        channel_body, message_body, message_body_pub, raw_record, test_dir, write_raw,
        write_recording, write_unfinished_recording,
    };

    use super::*;

    /// Drive scan passes the way `tail_file` does until the recording ends, a
    /// pass faults, or a pass makes no progress (the file stopped growing).
    /// Stays a `Result` only because `file_len` can fail; the scan itself no
    /// longer returns a `Result`. Stops on a fault without retrying — retry and
    /// backoff are `tail_file`'s job, exercised through the `run()`-level tests.
    pub(crate) fn scan_to_end(
        tailer: &Tailer,
        file: &File,
        mut offset: u64,
    ) -> Result<ScanProgress> {
        loop {
            let progress = tailer.scan_available(file, offset, file_len(file)?);
            if progress.ended || progress.fault.is_some() || progress.offset == offset {
                return Ok(progress);
            }
            offset = progress.offset;
        }
    }

    /// Open `path` and attach it as the sole `current` recording, returning the
    /// handle — the setup the single-recording scan tests share. A scan applies
    /// to the `current` recording, so a standalone scan needs one indexed first.
    pub(crate) fn attached(tailer: &Tailer, path: &Path) -> Result<Arc<File>> {
        let file = Arc::new(File::open(path)?);
        tailer.attach(file.clone());
        Ok(file)
    }

    /// The single plan a one-recording test cuts from on the `log` domain: most
    /// tests window on `log_time`, so this defaults there; [`plan_one_src`] takes
    /// an explicit source. [`WindowPlanner::plan_window`] returns one plan per
    /// overlapping recording, and these tests index one, so its `Vec` holds at
    /// most one; no overlap becomes an empty plan.
    pub(crate) fn plan_one(tailer: &Tailer, start_ns: u64, end_ns: u64) -> WindowPlan {
        plan_one_src(tailer, start_ns, end_ns, TimeSource::Log)
    }

    /// [`plan_one`] on an explicit windowing `source`, for the domain-selection
    /// tests.
    pub(crate) fn plan_one_src(
        tailer: &Tailer,
        start_ns: u64,
        end_ns: u64,
        source: TimeSource,
    ) -> WindowPlan {
        tailer
            .plan_window(start_ns, end_ns, source)
            .into_iter()
            .next()
            .unwrap_or_else(WindowPlan::empty)
    }

    /// Drive scan polls until every indexed recording has been scanned to
    /// `Ended` (no `current` remains) — the synchronous equivalent of the run
    /// loop for a fixed set of already-finished recordings. Used by the
    /// collection tests, which index several recordings up front and then drain.
    pub(crate) fn drain(tailer: &Tailer) -> Result<()> {
        let mut guard = 0;
        while let Some(id) = tailer.current_id() {
            tailer.poll_current(id)?;
            guard += 1;
            assert!(guard < 1000, "drain did not converge");
        }
        Ok(())
    }

    /// The tailer seeds the shared scan with its own trigger tap, so a
    /// trigger-topic message in the recording arrives on the tap channel.
    ///
    /// The scan itself is [`clip::index`]'s and is tested there against a fixture
    /// seed. What is only testable here is the *production* seed: that
    /// [`Tailer::with_trigger_tap`]'s topic and sender reach
    /// [`clip::index::ScanSeed`] at all. A tailer that built the seed with
    /// `trigger_tx: None` would tail correctly, index correctly, cover
    /// correctly — and silently never lift a trigger, which under the `mcap`
    /// interface is a recorder that ignores every trigger it is given.
    #[test]
    fn the_tap_seeded_by_the_tailer_lifts_a_trigger() -> Result<()> {
        let root = test_dir("tail-tap")?;
        let rec = root.join("rec.mcap");
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/trig", "json")),
                raw_record(op::MESSAGE, &message_body(1, 0, 100, b"{}")),
            ],
        )?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, _coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = attached(&tailer, &rec)?;
        scan_to_end(&tailer, &file, 8)?;

        let lifted = rx.try_recv().expect("the seeded tap lifts the trigger");
        assert_eq!(lifted.message_encoding, "json");
        assert_eq!(lifted.body, b"{}");
        assert_eq!(lifted.log_time, 100);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The tap's channel registry survives across scan passes, because the
    /// tailer seeds each pass from the recording's accumulated
    /// `trigger_channels`.
    ///
    /// A trigger message references a `Channel` record written earlier, possibly
    /// in a pass that has already completed. A seed that started each pass with
    /// an empty registry would lift triggers only when the channel definition
    /// happened to land in the same pass — the common case in a test that writes
    /// a whole file at once, and the rare case against a live recorder. So the
    /// channel and the message are written in two passes here.
    #[test]
    fn the_seeded_tap_remembers_channels_from_an_earlier_pass() -> Result<()> {
        let root = test_dir("tail-tap-passes")?;
        let rec = root.join("rec.mcap");
        // Pass one sees only the channel definition.
        write_raw(
            &rec,
            &[raw_record(
                op::CHANNEL,
                &channel_body(7, 0, "/trig", "json"),
            )],
        )?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, _coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = attached(&tailer, &rec)?;
        let first = scan_to_end(&tailer, &file, 8)?;
        assert!(rx.try_recv().is_err(), "no trigger has been written yet");

        // Pass two appends the trigger message alone; its channel is known only
        // from the recording's registry.
        {
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new().append(true).open(&rec)?;
            f.write_all(&raw_record(op::MESSAGE, &message_body(7, 0, 250, b"{}")))?;
        }
        scan_to_end(&tailer, &file, first.offset)?;

        let lifted = rx
            .try_recv()
            .expect("a trigger resolves against a channel from an earlier pass");
        assert_eq!(lifted.log_time, 250);
        assert_eq!(lifted.message_encoding, "json");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn non_mcap_file_is_rejected() -> Result<()> {
        let root = test_dir("badmagic")?;
        let path = root.join("rec.mcap");
        std::fs::write(&path, b"definitely not an mcap file")?;

        // Indexed as the current recording, the first scan poll verifies the
        // magic and rejects it: an append-only file whose first eight bytes are
        // wrong can never become a valid MCAP.
        let (tailer, _coverage) = Tailer::new();
        tailer.index_recording(&path);
        let id = tailer.current_id().expect("the file is indexed as current");
        let err = tailer.poll_current(id).unwrap_err();
        assert!(
            err.to_string().contains("not an MCAP file"),
            "unexpected error: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn a_vanished_discovered_file_indexes_nothing() -> Result<()> {
        let root = test_dir("missing")?;
        let path = root.join("gone.mcap");

        // Discovery can race the record script wiping the bag dir: the file
        // vanishes between the iterator yielding it and the open. A NotFound on
        // open is skipped — no recording indexed, no current, no fault.
        let (tailer, _coverage) = Tailer::new();
        tailer.index_recording(&path);
        assert!(
            tailer.current_id().is_none(),
            "a vanished file leaves nothing indexed"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn a_window_straddling_a_rollover_plans_both_files() -> Result<()> {
        // The previous-file preroll case: a recorder split (or restart) clipper
        // indexed while running leaves two finished recordings on disk. A window
        // whose preroll reaches into the earlier file and whose postroll lands in
        // the later one plans BOTH — one single-file plan per source, oldest
        // first — recovering across the boundary (beads clipper-gl2).
        let root = test_dir("straddle")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        write_recording(&split0, false, &[("/t", 1_000), ("/t", 2_000)])?;
        write_recording(&split1, false, &[("/t", 5_000), ("/t", 6_000)])?;

        let (tailer, coverage) = Tailer::new();
        // Index both up front (split0 the older), then scan them to completion in
        // order through the production poll loop.
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        // Coverage is collection-wide: the high-water is the newest file's max.
        assert_eq!(coverage.get().high_water_ns, 6_000);

        // A window inside one file plans exactly one source.
        assert_eq!(tailer.plan_window(900, 2_100, TimeSource::Log).len(), 1);
        assert_eq!(tailer.plan_window(4_900, 6_100, TimeSource::Log).len(), 1);

        // A window straddling the rollover plans both, oldest first.
        let plans = tailer.plan_window(1_500, 5_500, TimeSource::Log);
        assert_eq!(plans.len(), 2, "the straddling window plans both files");
        assert!(plans.iter().all(|p| !p.extents.is_empty()));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn retention_prunes_aged_ended_files_but_keeps_current_and_in_flight() -> Result<()> {
        let root = test_dir("retention")?;
        let split0 = root.join("rec_0.mcap");
        let split1 = root.join("rec_1.mcap");
        // split0's data is "old" (low log_time), split1's is "current".
        write_recording(&split0, false, &[("/t", 1_000)])?;
        write_recording(&split1, false, &[("/t", 9_000)])?;

        let (tailer, _coverage) = Tailer::new();
        tailer.index_recording(&split0);
        tailer.index_recording(&split1);
        drain(&tailer)?;

        // An in-flight extraction holds its own clone of split0's file handle:
        // pruning the index entry must not pull the bytes out from under it.
        let in_flight = tailer.plan_window(900, 1_100, TimeSource::Log);
        assert_eq!(in_flight.len(), 1, "split0 is plannable before the prune");

        // Prune with a floor above split0's data (1_000) but below split1's
        // (9_000): split0 is dropped, split1 retained. (Both are Ended here;
        // the floor, not the state, decides — `current` is None after draining.)
        let dropped = tailer.state.lock().unwrap().prune(5_000);
        assert_eq!(dropped, vec![split0.clone()], "the aged file is pruned");

        assert!(
            tailer.plan_window(900, 1_100, TimeSource::Log).is_empty(),
            "split0's index is gone after the prune"
        );
        assert!(
            !tailer.plan_window(8_900, 9_100, TimeSource::Log).is_empty(),
            "split1 is retained"
        );

        // The pre-prune plan still reads through its own Arc<File> (POSIX
        // unlink-while-open semantics); the index drop did not invalidate it.
        let file = in_flight[0].file.clone().expect("plan pins the file");
        assert!(
            file.metadata().is_ok(),
            "the in-flight handle stays readable"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn the_current_file_is_never_pruned() -> Result<()> {
        let root = test_dir("prune-current")?;
        let path = root.join("rec_0.mcap");
        write_unfinished_recording(&path, "/t", &[1_000])?;

        let (tailer, _coverage) = Tailer::new();
        tailer.index_recording(&path);
        let id = tailer.current_id().expect("indexed as current");
        // One poll indexes the data but, with no footer and no successor, leaves
        // the file `current` (still being tailed).
        tailer.poll_current(id)?;
        assert_eq!(tailer.current_id(), Some(id), "still the current file");

        // Even a floor far above its data does not drop the file being recorded.
        let dropped = tailer.state.lock().unwrap().prune(u64::MAX);
        assert!(dropped.is_empty(), "the current file is never pruned");
        assert!(!tailer.plan_window(900, 1_100, TimeSource::Log).is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn inode_changed_detects_a_vanished_or_replaced_file() -> Result<()> {
        // The narrowed "is *my* file still live" check: whether the tailed path
        // still resolves to the open fd's inode. "Is there a newer file" is the
        // discovery iterator's job, not this one (see the `discover` tests).
        let root = test_dir("inode")?;
        let path = root.join("rec_0.mcap");
        std::fs::write(&path, b"x")?;
        let file = File::open(&path)?;

        assert!(
            !inode_changed(&path, &file)?,
            "the path still resolves to the tailed inode"
        );

        // A recorder restart wiping the dir: the tailed inode vanishes.
        std::fs::remove_file(&path)?;
        assert!(
            inode_changed(&path, &file)?,
            "a deleted recording's path no longer resolves to the fd"
        );

        // Recreated at the same path is a different inode — still changed.
        std::fs::write(&path, b"z")?;
        assert!(
            inode_changed(&path, &file)?,
            "a recreated file is a different inode"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn newest_mcap_picks_the_latest_by_mtime_and_ignores_non_mcap() -> Result<()> {
        let root = test_dir("discover")?;
        assert_eq!(newest_mcap(&root.join("missing")), None);
        assert_eq!(newest_mcap(&root), None, "no mcap yet");

        let old = root.join("old.mcap");
        std::fs::write(&old, b"")?;
        std::thread::sleep(Duration::from_millis(10));
        let newer = root.join("new.mcap");
        std::fs::write(&newer, b"")?;
        std::fs::write(root.join("note.txt"), b"")?; // wrong extension, ignored

        // Discovery is by mtime: the most recently written `*.mcap` wins. The
        // `.txt` is ignored regardless of its time.
        assert_eq!(newest_mcap(&root), Some(newer.clone()));

        // Bumping old.mcap's mtime to the latest makes it the newest — mtime,
        // not creation order, decides.
        std::thread::sleep(Duration::from_millis(10));
        File::options()
            .write(true)
            .open(&old)?
            .set_modified(SystemTime::now())?;
        assert_eq!(newest_mcap(&root), Some(old));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording whose good prefix (channel + two messages at 100 and 200)
    /// indexes cleanly but is followed by a record header with an oversized
    /// length: a framing fault with no resync point. The prefix is the part a
    /// retry must preserve and keep plannable.
    fn write_poisoned_recording(path: &Path) -> Result<()> {
        let mut bytes = MAGIC.to_vec();
        for rec in [
            raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
            raw_record(op::MESSAGE, &message_body(1, 0, 100, b"x")),
            raw_record(op::MESSAGE, &message_body(1, 1, 200, b"y")),
        ] {
            bytes.extend_from_slice(&rec);
        }
        bytes.push(op::MESSAGE);
        bytes.extend_from_slice(&(MAX_RECORD_LEN + 1).to_le_bytes());
        std::fs::write(path, bytes)?;
        Ok(())
    }

    #[test]
    fn run_gives_up_on_a_persistently_faulting_recording() -> Result<()> {
        let root = test_dir("run-fatal")?;
        write_poisoned_recording(&root.join("a.mcap"))?;

        let (tailer, coverage) = Tailer::new();
        let runner = tailer.clone();
        let dir = root.clone();
        let started = std::time::Instant::now();
        let handle = std::thread::spawn(move || runner.run(&dir, Duration::from_secs(600), false));

        // The same byte faults on every pass, so run() must exhaust the retry
        // budget and return Err well within the deadline.
        let deadline = std::time::Instant::now() + Duration::from_secs(25);
        while !handle.is_finished() {
            assert!(
                std::time::Instant::now() < deadline,
                "run() must give up on a stuck recording"
            );
            std::thread::sleep(Duration::from_millis(50));
        }
        let elapsed = started.elapsed();
        let err = handle.join().unwrap().unwrap_err();

        // The escalating backoff (200+400+800+1600 ms) means giving up takes a
        // few seconds, not a fixed cadence. Lower bound only — CI-safe.
        assert!(
            elapsed >= Duration::from_millis(2500),
            "the backoff must escalate before giving up (took {elapsed:?})"
        );
        let msg = format!("{err:#}");
        assert!(
            msg.contains("framing desynchronised")
                && msg.contains("offset")
                && msg.contains("consecutive passes"),
            "the error chain must name the desync, offset, and give-up: {msg}"
        );

        // The good prefix survived every retry — the index was never wiped.
        let cov = coverage.get();
        assert_eq!(cov.high_water_ns, 200, "the prefix's coverage survived");
        assert!(
            !plan_one(&tailer, 50, 250).extents.is_empty(),
            "the prefix's extent stayed plannable through the retries"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn run_recovers_when_the_faulting_recording_is_replaced() -> Result<()> {
        let root = test_dir("run-recover")?;
        let poisoned = root.join("a.mcap");
        write_poisoned_recording(&poisoned)?;

        let (tailer, coverage) = Tailer::new();
        let runner = tailer.clone();
        let dir = root.clone();
        let handle = std::thread::spawn(move || runner.run(&dir, Duration::from_secs(600), false));

        // While run() is backing off over the poisoned file, replace it with a
        // finished good recording: the restart mid-backoff is recovery, and the
        // tail discovers and indexes the replacement.
        std::thread::sleep(Duration::from_millis(500));
        std::fs::remove_file(&poisoned)?;
        let good = root.join("b.mcap");
        write_recording(&good, false, &[("/t", 1_000)])?;

        // Poll the coverage watch until the replacement is fully indexed.
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        loop {
            {
                let cov = coverage.get();
                if cov.high_water_ns == 1_000 {
                    break;
                }
            }
            assert!(
                std::time::Instant::now() < deadline,
                "the replacement recording must be discovered and indexed"
            );
            std::thread::sleep(Duration::from_millis(50));
        }

        // Recovery, not a fatal exit: the run loop is still alive, re-tailing.
        assert!(
            !handle.is_finished(),
            "run() must keep tailing after recovering from the fault"
        );

        // The thread is detached: it loops forever against the good recording
        // and dies with the test process. Do not join it.
        drop(handle);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn run_follows_a_new_split_and_retains_the_previous_one() -> Result<()> {
        // rosbag2 `--max-bag-duration`/`--max-bag-size` keeps every finished
        // split on disk and rolls over to `<bag>_<n+1>.mcap`. The tail advances
        // to the new split AND retains the previous one in its collection (within
        // the watch window), so a window straddling the boundary recovers both
        // (beads clipper-gl2). A long watch duration keeps the older split.
        let root = test_dir("run-split")?;

        // Split 0: a finished recording — rosbag2 closes each split (footer).
        let split0 = root.join("rec_0.mcap");
        write_recording(&split0, false, &[("/t", 1_000)])?;

        let (tailer, coverage) = Tailer::new();
        let runner = tailer.clone();
        let dir = root.clone();
        // A watch larger than the wall clock pins the floor at 0, so the test's
        // synthetic (epoch-relative tiny) log_times never age out — this test
        // checks cross-file retention, not the pruning horizon.
        let handle =
            std::thread::spawn(move || runner.run(&dir, Duration::from_secs(u64::MAX), false));

        // The tail discovers and indexes split 0.
        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while coverage.get().high_water_ns != 1_000 {
            assert!(
                std::time::Instant::now() < deadline,
                "split 0 must be discovered and indexed"
            );
            std::thread::sleep(Duration::from_millis(25));
        }

        // The recorder rolls over: split 1 appears beside split 0 with a later
        // mtime and later message times. The tail must advance to it.
        let split1 = root.join("rec_1.mcap");
        write_recording(&split1, false, &[("/t", 5_000)])?;

        let deadline = std::time::Instant::now() + Duration::from_secs(30);
        while coverage.get().high_water_ns != 5_000 {
            assert!(
                std::time::Instant::now() < deadline,
                "the tail must follow the new split file (high_water stuck at {})",
                coverage.get().high_water_ns
            );
            std::thread::sleep(Duration::from_millis(25));
        }

        // BOTH splits are plannable: split 1 (current) and split 0 (retained in
        // the collection) — the cross-file recovery the redesign provides.
        assert!(
            !plan_one(&tailer, 4_000, 6_000).extents.is_empty(),
            "split 1 is indexed after the tail follows the rollover"
        );
        assert!(
            !plan_one(&tailer, 500, 1_500).extents.is_empty(),
            "split 0 is retained for cross-file recovery, not dropped"
        );
        // A window straddling the boundary plans both source files.
        assert_eq!(
            tailer.plan_window(900, 5_100, TimeSource::Log).len(),
            2,
            "a straddling window recovers both splits"
        );

        // Detached: it loops forever against split 1 and dies with the process.
        drop(handle);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Extent overlap and coverage both track the windowing [`TimeSource`]. A
    /// recording whose log and publish spans are disjoint plans its extent for a
    /// log window over the log span but not for the same window on `publish`, and
    /// vice versa; coverage carries a high-water for each domain.
    #[test]
    fn extent_overlap_and_coverage_track_the_time_source() -> Result<()> {
        let root = test_dir("overlap-domain")?;
        let path = root.join("rec.mcap");
        // log_time 100/200/300; publish_time 900/1000/1100 — disjoint ranges.
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 100, 900, b"a")),
                raw_record(op::MESSAGE, &message_body_pub(1, 1, 200, 1_000, b"b")),
                raw_record(op::MESSAGE, &message_body_pub(1, 2, 300, 1_100, b"c")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        // Coverage carries an independent high-water per domain.
        assert_eq!(coverage.get().high_water_ns, 300);
        assert_eq!(coverage.get().publish_high_water_ns, 1_100);

        // A window over the log span plans the extent on `log` but not on
        // `publish`; a window over the publish span does the reverse.
        assert!(
            !plan_one_src(&tailer, 250, 400, TimeSource::Log)
                .extents
                .is_empty(),
            "log window over the log span plans the extent"
        );
        assert!(
            plan_one_src(&tailer, 250, 400, TimeSource::Publish)
                .extents
                .is_empty(),
            "the same window on publish misses the elsewhere publish span"
        );
        assert!(
            !plan_one_src(&tailer, 950, 1_050, TimeSource::Publish)
                .extents
                .is_empty(),
            "publish window over the publish span plans the extent"
        );
        assert!(
            plan_one_src(&tailer, 950, 1_050, TimeSource::Log)
                .extents
                .is_empty(),
            "the same window on log misses the elsewhere log span"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Retention ages a recording on `log_time`, never `publish_time`: a floor
    /// above the log max prunes the file even when its publish max is far higher.
    /// A producer must not be able to keep a file alive (or force its deletion)
    /// by what it writes into `publish_time`.
    #[test]
    fn retention_prunes_on_log_time_regardless_of_publish_time() -> Result<()> {
        let root = test_dir("retention-domain")?;
        let path = root.join("rec.mcap");
        // log_time 1_000, publish_time 9_000, then a DataEnd so the scan ends and
        // the recording is retirable.
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 1_000, 9_000, b"x")),
                raw_record(op::DATA_END, &[]),
            ],
        )?;

        let (tailer, _coverage) = Tailer::new();
        tailer.index_recording(&path);
        drain(&tailer)?;

        // Floor 5_000 sits above the log max (1_000) but below the publish max
        // (9_000): retention ages on log_time, so the recording is pruned.
        let dropped = tailer.state.lock().unwrap().prune(5_000);
        assert_eq!(
            dropped,
            vec![path.clone()],
            "retention ages on log_time, ignoring the higher publish_time"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
