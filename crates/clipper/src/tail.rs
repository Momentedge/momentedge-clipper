//! Tail of the growing MCAP file behind a continuous `ros2 bag record`.
//!
//! rosbag2's MCAP writer is append-only while recording: bytes below the
//! current end of file never change, and every record is length-prefixed (a
//! 1-byte opcode + u64le length). The tail exploits both properties: it keeps
//! the recording open, repeatedly consumes the complete records that appeared
//! since the previous pass — a record whose declared extent runs past the
//! current file length is still being written and is left for the next pass —
//! and never re-reads a byte it has already consumed.
//!
//! Three artefacts come out of the scan, all served to the per-trigger
//! extraction ([`crate::clip`]):
//!
//! * **Extent index** — contiguous byte ranges of the file (closed at
//!   [`EXTENT_CAP_BYTES`]) with the min/max `log_time` of the messages they
//!   hold. A clip reads only the extents overlapping its window, so cutting a
//!   clip never rescans the file.
//! * **Schema/channel registry** — every `Schema`/`Channel` record seen, keyed
//!   by the file's channel ID (unique within one continuous file). Chunked
//!   recordings carry these *inside* chunks, so chunks are decompressed during
//!   the tail; an unchunked recording (the fastwrite storage profile) pays no
//!   such cost.
//! * **Coverage watch** — the highest `log_time` seen plus an "ended" flag
//!   ([`Coverage`]); a trigger handler waits on it until the recording provably
//!   covers its window end.
//!
//! Only the 14-byte prefix of each top-level `Message` record is read during
//! the tail (channel id, sequence, `log_time`); message bodies are first
//! touched by the extraction. The same "decode only the timestamp" discipline
//! as the rest of the workspace, applied to file tailing. The one exception is
//! an opt-in trigger tap ([`Tailer::with_trigger_tap`], wired only by the MCAP
//! interface): when set, the scan also lifts the full body of messages on the
//! trigger topic out as [`TriggerRecord`]s for the interface to decode by
//! `message_encoding`. With the tap unset — the default — no message body is
//! read during the scan at all.
//!
//! The tail owns a time-ordered collection of recordings. New `*.mcap` files
//! under the record dir — a rosbag2 split rolling over to `<bag>_<n+1>.mcap`, or
//! a restart recreating the bag directory — are discovered (a recorder running
//! its files appear in mtime order, [`crate::discover`]) and indexed alongside
//! the ones already known. Each is scanned in turn; a finished recording is
//! retained for a watch window so a clip straddling a rollover recovers across
//! it (beads clipper-gl2), then pruned. Extractions hold their own file handle,
//! so a recording pruned or deleted while a clip reads it stays readable.
//!
//! Damage in the recording is tolerated the way [`crate::clip`] tolerates it
//! at extraction: a damaged chunk, an unparseable schema/channel, or a runt
//! message is warned and skipped, the framing intact. A **framing** fault has
//! no resync point (a record length past [`MAX_RECORD_LEN`], or an IO error
//! reading a record), so the scan stops at it, having applied everything
//! before it. The tail then retries from exactly that offset — never
//! re-attaching, never rescanning from scratch — under a bounded,
//! backing-off [`MAX_SCAN_FAULTS`] budget, treating a recorder restart during
//! the backoff as recovery. Only when the same byte faults through the whole
//! budget does [`Tailer::run`] return an error and the process exit for a
//! supervisor to restart: a tailer wedged on a stuck file would otherwise
//! degrade every clip to a grace-timeout cut with no other signal.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::os::unix::fs::{FileExt, MetadataExt};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result, bail};
use crossbeam_channel::Sender;
use log::{info, warn};
use mcap::records::Record;

use crate::trigger::TriggerRecord;
use crate::watch::Watch;

/// The 8 magic bytes opening (and, after `finish`, closing) every MCAP file.
const MAGIC: [u8; 8] = *b"\x89MCAP0\r\n";

/// Extents close once they cover this many bytes, bounding both the bytes one
/// index entry stands for and the index's growth (one entry per cap per file).
const EXTENT_CAP_BYTES: u64 = 4 * 1024 * 1024;

/// Upper bound on a plausible single record. A length beyond this means the
/// scan is desynchronised from the record framing (or the file is corrupt).
/// [`crate::clip`] applies the same bound to the records it reads back out
/// of extents, including chunk-interior records after decompression.
pub(crate) const MAX_RECORD_LEN: u64 = 1 << 31;

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

/// MCAP record opcodes the tail dispatches on.
pub(crate) mod op {
    pub const FOOTER: u8 = 0x02;
    pub const SCHEMA: u8 = 0x03;
    pub const CHANNEL: u8 = 0x04;
    pub const MESSAGE: u8 = 0x05;
    pub const CHUNK: u8 = 0x06;
    pub const DATA_END: u8 = 0x0F;
}

/// An owned copy of a `Schema` record.
#[derive(Clone, Debug)]
pub struct SchemaDef {
    pub name: String,
    pub encoding: String,
    pub data: Vec<u8>,
}

/// An owned copy of a `Channel` record, with its schema resolved.
#[derive(Clone, Debug)]
pub struct ChannelDef {
    pub topic: String,
    pub message_encoding: String,
    pub metadata: BTreeMap<String, String>,
    pub schema: Option<SchemaDef>,
}

/// A contiguous byte range of the recording, aligned to top-level record
/// boundaries, with the time bounds of the messages it holds. `time` is `None`
/// while the range carries no timed record (e.g. only schema/channel records).
#[derive(Clone, Copy, Debug)]
pub struct Extent {
    pub offset: u64,
    pub len: u64,
    pub time: Option<(u64, u64)>,
}

impl Extent {
    /// Whether any message in the extent can fall inside `[start_ns, end_ns]`.
    /// Exact, not heuristic: the bounds are the actual min/max of the extent's
    /// messages, so a message in the window implies its extent overlaps it.
    fn overlaps(&self, start_ns: u64, end_ns: u64) -> bool {
        self.time
            .is_some_and(|(min, max)| max >= start_ns && min <= end_ns)
    }
}

/// How far the recordings provably reach: the highest message `log_time` the
/// tail has seen on disk, across the whole collection of indexed recordings.
///
/// "Provably" rests on two properties. First an ordering assumption: messages
/// land in a file in (approximately) non-decreasing `log_time` order. rosbag2
/// has that shape — one writer, `log_time` stamped at receive — up to
/// millisecond-scale interleaving between concurrent subscription callbacks,
/// which the flush and extraction latency in front of every cut dwarfs. Second,
/// the tail scans strictly in order, one `current` recording at a time, oldest
/// first, finishing each before the next: a later file's coverage can never
/// advance before an earlier file is complete. So a collection-wide high-water
/// at or past a window end implies the window's messages are on disk in
/// whichever recording holds them — no per-file coverage is needed. The mark is
/// monotonic across retention prunes: a pruned file is below the watch floor and
/// never held the maximum, so dropping it never lowers the high-water.
#[derive(Clone, Copy, Debug, Default)]
pub struct Coverage {
    pub high_water_ns: u64,
}

/// A snapshot for one clip: the open recording, the extents overlapping the
/// window (in file order), and the channel registry to map IDs with. `file` is
/// `None` while no recording has been discovered yet.
pub struct WindowPlan {
    pub file: Option<Arc<File>>,
    pub extents: Vec<Extent>,
    pub channels: HashMap<u16, ChannelDef>,
}

impl WindowPlan {
    /// A plan with no source file — stages a channelless empty clip (magic +
    /// summary + footer) for a window no recording covers. The empty path needs
    /// no `Arc<File>`, so it serves the "no recording exists yet" case too.
    pub fn empty() -> Self {
        WindowPlan {
            file: None,
            extents: Vec::new(),
            channels: HashMap::new(),
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

/// The `log_time` span of the messages indexed in one recording. `has_messages`
/// is false until the first timed record lands, distinguishing "no data" from
/// "data at time 0". Drives both window overlap and retention (`max_log_time`
/// against the watch floor). `log_time` only — `publish_time` is deferred
/// (beads clipper-75k).
#[derive(Clone, Copy, Debug, Default)]
struct TimeBounds {
    min_log_time: u64,
    max_log_time: u64,
    has_messages: bool,
}

impl TimeBounds {
    fn absorb(&mut self, min: u64, max: u64) {
        if self.has_messages {
            self.min_log_time = self.min_log_time.min(min);
            self.max_log_time = self.max_log_time.max(max);
        } else {
            *self = TimeBounds {
                min_log_time: min,
                max_log_time: max,
                has_messages: true,
            };
        }
    }
}

/// One indexed recording: its open file handle, scan progress, extent index,
/// schema/channel registry, and time bounds. The tail owns a time-ordered
/// collection of these (see [`TailState`]); trigger handlers read them through
/// [`Tailer::plan_window`].
struct RecordingIndex {
    id: RecordingId,
    path: PathBuf,
    file: Arc<File>,
    state: RecordingState,
    /// The scan resume point: bytes below it are consumed, the next pass starts
    /// here. Begins at 0 (magic unverified); set past the magic once verified.
    offset: u64,
    /// Whether the 8 magic bytes have been verified — gates the `New → Tailing`
    /// transition, since a freshly created file may not hold them yet.
    magic_ok: bool,
    extents: Vec<Extent>,
    /// The extent still accumulating records at the end of the scanned region.
    /// Included in window plans — a window may end inside it.
    open: Option<Extent>,
    schemas: HashMap<u16, SchemaDef>,
    channels: HashMap<u16, ChannelDef>,
    /// Channels on the trigger topic (`id -> message_encoding`), the subset of
    /// `channels` the MCAP-interface tap watches. Empty unless the tail was built
    /// with a trigger tap (`Tailer::with_trigger_tap`); seeds each scan pass so a
    /// trigger message references its channel defined in an earlier pass.
    trigger_channels: HashMap<u16, String>,
    bounds: TimeBounds,
}

impl RecordingIndex {
    /// Fold one scan pass's delta into this recording's registry, extents, and
    /// time bounds. Mirrors the single-index `apply`: schemas first (so a
    /// channel resolves its schema against the registry as this pass updates
    /// it), then channels, then extents, then bounds.
    fn apply_delta(&mut self, delta: ScanDelta) {
        for (id, schema) in delta.schemas {
            self.schemas.insert(id, schema);
        }
        for raw in delta.channels {
            let schema = (raw.schema_id != 0)
                .then(|| self.schemas.get(&raw.schema_id).cloned())
                .flatten();
            self.channels.insert(
                raw.id,
                ChannelDef {
                    topic: raw.topic,
                    message_encoding: raw.message_encoding,
                    metadata: raw.metadata,
                    schema,
                },
            );
        }
        for (id, encoding) in delta.trigger_channels {
            self.trigger_channels.insert(id, encoding);
        }
        self.extents.extend(delta.closed);
        self.open = delta.open;
        for extent in self.extents.iter().chain(self.open.iter()) {
            if let Some((min, max)) = extent.time {
                self.bounds.absorb(min, max);
            }
        }
    }

    /// A single-file [`WindowPlan`] over this recording's extents overlapping
    /// `[start_ns, end_ns]`, or `None` if none do.
    fn plan(&self, start_ns: u64, end_ns: u64) -> Option<WindowPlan> {
        let extents: Vec<Extent> = self
            .extents
            .iter()
            .chain(self.open.iter())
            .filter(|e| e.overlaps(start_ns, end_ns))
            .copied()
            .collect();
        (!extents.is_empty()).then(|| WindowPlan {
            file: Some(self.file.clone()),
            extents,
            channels: self.channels.clone(),
        })
    }
}

/// The tail-owned collection of recording indexes, in time order
/// (oldest .. newest), plus which one is being incrementally scanned. The tail
/// thread is the sole writer; trigger handlers only read it (under the mutex)
/// via [`Tailer::plan_window`].
#[derive(Default)]
struct TailState {
    recordings: std::collections::VecDeque<RecordingIndex>,
    /// The recording being incrementally tailed (`Tailing`). `None` before the
    /// first file is discovered or after the last one ends with no successor.
    current: Option<RecordingId>,
    /// Source of the next [`RecordingId`]; only ever increases.
    next_id: u64,
}

impl TailState {
    fn recording(&self, id: RecordingId) -> Option<&RecordingIndex> {
        self.recordings.iter().find(|r| r.id == id)
    }

    fn recording_mut(&mut self, id: RecordingId) -> Option<&mut RecordingIndex> {
        self.recordings.iter_mut().find(|r| r.id == id)
    }

    /// Index a freshly discovered recording at the back of the collection (it is
    /// the newest). Adopts it as `current` when there is none (startup, or after
    /// the last recording ended) so the first file is always tailed; otherwise
    /// it waits as a `New` successor behind the recording in flight.
    fn insert_new_recording(&mut self, path: PathBuf, file: Arc<File>) -> RecordingId {
        let id = RecordingId(self.next_id);
        self.next_id += 1;
        self.recordings.push_back(RecordingIndex {
            id,
            path,
            file,
            state: RecordingState::New,
            offset: 0,
            magic_ok: false,
            extents: Vec::new(),
            open: None,
            schemas: HashMap::new(),
            channels: HashMap::new(),
            trigger_channels: HashMap::new(),
            bounds: TimeBounds::default(),
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
    /// `[start_ns, end_ns]`, oldest first. Empty when no recording covers the
    /// window (a rollover gap, all relevant files pruned, or nothing indexed
    /// yet) — the caller then stages one empty clip.
    fn plan_window(&self, start_ns: u64, end_ns: u64) -> Vec<WindowPlan> {
        self.recordings
            .iter()
            .filter_map(|r| r.plan(start_ns, end_ns))
            .collect()
    }

    /// Drop every `Ended` recording whose newest data is older than
    /// `floor_ns` — never the `current` file, never a `New` or `Tailing` one,
    /// never mid-file. Returns the dropped recordings' paths (for optional
    /// on-disk deletion). Dropping a [`RecordingIndex`] releases its
    /// `Arc<File>`, closing the descriptor once no in-flight plan still holds a
    /// clone, so the prune bounds both memory and open fds.
    fn prune(&mut self, floor_ns: u64) -> Vec<PathBuf> {
        let mut pruned = Vec::new();
        self.recordings.retain(|r| {
            let expired = r.state == RecordingState::Ended
                && Some(r.id) != self.current
                && r.bounds.has_messages
                && r.bounds.max_log_time < floor_ns;
            if expired {
                pruned.push(r.path.clone());
            }
            !expired
        });
        pruned
    }

    /// The collection-wide high-water `log_time` — the maximum over all indexed
    /// recordings.
    fn high_water_ns(&self) -> u64 {
        self.recordings
            .iter()
            .filter(|r| r.bounds.has_messages)
            .map(|r| r.bounds.max_log_time)
            .max()
            .unwrap_or(0)
    }
}

/// Shared tail state: the scanning thread feeds it, trigger handlers snapshot
/// it via [`Tailer::plan_window`] and wait on the coverage watch.
pub struct Tailer {
    state: Mutex<TailState>,
    coverage: Arc<Watch<Coverage>>,
    /// The topic whose messages the scan lifts out as triggers, or `None` to
    /// disable the tap entirely (the ROS interface, which reads triggers from a
    /// live subscription instead). When `None`, the scan is byte-for-byte the
    /// timestamp-only tail; no message body is ever read during the scan.
    trigger_topic: Option<String>,
    /// Where lifted [`TriggerRecord`]s go (the MCAP interface drains the far
    /// end). `Some` exactly when `trigger_topic` is. Best-effort: a full or
    /// closed tap never stalls the scan. Cloned into each scan's [`ScanDelta`],
    /// which sends triggers straight down it the moment it lifts them.
    trigger_tx: Option<Sender<TriggerRecord>>,
}

/// Where one scan pass stopped, whether the recording ended, and whether a
/// fault stopped it short. A fault carries the framing error; `offset` is then
/// the byte offset of the faulted record (where a retry resumes), not the file
/// end. `ended` and `fault` are mutually exclusive — a pass that hits the
/// footer cannot also fault.
#[derive(Debug)]
pub(crate) struct ScanProgress {
    pub(crate) offset: u64,
    pub(crate) ended: bool,
    pub(crate) fault: Option<anyhow::Error>,
}

/// Where a [`ScanDelta`] routes a trigger it lifts. A trigger emits only once it
/// is durable: a top-level record the moment its framing is read, a
/// chunk-interior record only after the chunk's CRC verifies. So the top-level
/// delta carries the live tap and sends straight down it, while a chunk sub-delta
/// stages until [`ScanDelta::absorb_chunk`] re-emits each through the parent.
#[derive(Default)]
enum TriggerSink {
    /// The live tap the MCAP interface drains — the top-level scan delta. A
    /// lifted trigger sends now.
    Live(Sender<TriggerRecord>),
    /// A chunk sub-delta's staging buffer: triggers wait here until the chunk
    /// iterates cleanly, then `absorb_chunk` re-emits them through the parent's
    /// `Live` sink. A damaged chunk is discarded whole, so its staged triggers
    /// never emit.
    Staged(Vec<TriggerRecord>),
    /// The tap is disabled (no `--interface mcap`): no trigger is ever lifted.
    #[default]
    Off,
}

/// Registry and extent updates of one scan pass, collected without the state
/// lock (the pass does file IO) and applied under one short lock at the end.
#[derive(Default)]
struct ScanDelta {
    closed: Vec<Extent>,
    open: Option<Extent>,
    /// min/max log_time of records absorbed since the last extent extension.
    pending_time: Option<(u64, u64)>,
    schemas: Vec<(u16, SchemaDef)>,
    channels: Vec<RawChannel>,
    high_water_ns: u64,
    /// The trigger topic to lift, seeded from the tailer. `None` disables the
    /// tap, so the fields below stay empty/idle and the scan never reads a body.
    trigger_topic: Option<String>,
    /// Channels on the trigger topic seen so far — seeded from the recording's
    /// registry and grown as this pass parses `Channel` records — as
    /// `id -> message_encoding`. Consulted when a message references one of them.
    trigger_channels: HashMap<u16, String>,
    /// Where a lifted trigger goes: the top-level delta sends it straight down the
    /// live tap ([`TriggerSink::Live`]) the instant its framing is read; a chunk
    /// sub-delta stages it ([`TriggerSink::Staged`]) until the chunk's CRC clears.
    trigger_sink: TriggerSink,
}

/// A `Channel` record before its schema is resolved against the registry.
struct RawChannel {
    id: u16,
    schema_id: u16,
    topic: String,
    message_encoding: String,
    metadata: BTreeMap<String, String>,
}

impl ScanDelta {
    fn absorb_time(&mut self, log_time: u64) {
        self.high_water_ns = self.high_water_ns.max(log_time);
        self.pending_time = Some(match self.pending_time {
            Some((min, max)) => (min.min(log_time), max.max(log_time)),
            None => (log_time, log_time),
        });
    }

    /// Lift one trigger-topic message into a [`TriggerRecord`] and emit it. The
    /// two scan sites that find a trigger — a chunk-interior message
    /// ([`Self::absorb_parsed`]) and a top-level message
    /// ([`Tailer::scan_available`]) — funnel through here so the record is built
    /// one way; they differ only in how each obtains `body` (a decoded chunk
    /// `Cow` vs. a direct file read).
    fn capture_trigger(&mut self, message_encoding: String, body: Vec<u8>, log_time: u64) {
        self.emit_trigger(TriggerRecord {
            message_encoding,
            body,
            log_time,
        });
    }

    /// Route one lifted trigger by the delta's [`TriggerSink`]: the top-level
    /// delta's [`TriggerSink::Live`] sends it straight down the tap now; a chunk
    /// sub-delta's [`TriggerSink::Staged`] holds it until [`Self::absorb_chunk`]
    /// re-emits it through the parent once the chunk's CRC verifies — a damaged
    /// chunk emits nothing. The send is best-effort: a full or closed tap never
    /// stalls the scan.
    fn emit_trigger(&mut self, rec: TriggerRecord) {
        match &mut self.trigger_sink {
            // The live tap. `send` fails only when the receiver — the MCAP
            // interface draining the tap — is gone, which happens only once its
            // thread has died and `supervise` is already tearing the process
            // down. There is nothing useful left to do with the trigger then, and
            // the supervisor surfaces the real fault (a panicked interface
            // thread), so the drop is deliberate, not a swallowed error.
            TriggerSink::Live(tx) => {
                let _ = tx.send(rec);
            }
            // A chunk sub-delta: hold the trigger until its chunk's CRC clears.
            TriggerSink::Staged(staged) => staged.push(rec),
            // Unreachable: a trigger is lifted only when `trigger_channels` is
            // non-empty, which the disabled tap never fills.
            TriggerSink::Off => {}
        }
    }

    /// Fold one parsed record into the delta: schema/channel definitions into
    /// the registry, message times into the pending extent bounds.
    fn absorb_parsed(&mut self, rec: Record<'_>) {
        match rec {
            Record::Schema { header, data } => self.schemas.push((
                header.id,
                SchemaDef {
                    name: header.name,
                    encoding: header.encoding,
                    data: data.into_owned(),
                },
            )),
            Record::Channel(ch) => {
                // Register a trigger channel before `ch` is moved, so messages
                // later in this pass (or in later passes, via the recording's
                // registry) resolve their encoding.
                if self.trigger_topic.as_deref() == Some(ch.topic.as_str()) {
                    self.trigger_channels
                        .insert(ch.id, ch.message_encoding.clone());
                }
                self.channels.push(RawChannel {
                    id: ch.id,
                    schema_id: ch.schema_id,
                    topic: ch.topic,
                    message_encoding: ch.message_encoding,
                    metadata: ch.metadata,
                });
            }
            Record::Message { header, data } => {
                self.absorb_time(header.log_time);
                // A message on a trigger channel is lifted whole (its body is the
                // serialized Trigger payload); any other message contributes only
                // its timestamp, its body untouched. Inside a chunk sub-delta this
                // stages the trigger (TriggerSink::Staged) until the CRC clears.
                if let Some(encoding) = self.trigger_channels.get(&header.channel_id).cloned() {
                    self.capture_trigger(encoding, data.into_owned(), header.log_time);
                }
            }
            _ => {}
        }
    }

    /// Decompress one chunk record body and absorb its interior records. The
    /// only reason chunk bodies are read during the tail: chunked writers put
    /// Schema/Channel records inside chunks.
    ///
    /// All-or-nothing: the interior is absorbed into a fresh sub-delta and
    /// merged into `self` only once the chunk iterates cleanly
    /// ([`mcap::read::ChunkReader`] verifies the CRC at the end of iteration).
    /// A chunk that fails to decompress, fails its CRC, or holds an
    /// unparseable interior record therefore contributes nothing — matching
    /// clip.rs, which drops the whole chunk at extraction, so coverage never
    /// claims data the cut would silently leave out. Only the registry and
    /// time bounds move; the extent fields (`closed`/`open`) belong to
    /// [`Self::extend_extent`] and the chunk's own record offset, untouched here.
    fn absorb_chunk(&mut self, body: &[u8]) -> Result<()> {
        let Record::Chunk { header, data } = mcap::parse_record(op::CHUNK, body)? else {
            bail!("chunk opcode did not parse as a chunk record");
        };
        // Seed the sub-delta with the tap context so a Channel and a Message on
        // the trigger topic inside this chunk (or in an earlier pass) resolve.
        let mut sub = ScanDelta {
            trigger_topic: self.trigger_topic.clone(),
            trigger_channels: self.trigger_channels.clone(),
            // Stage triggers lifted from the chunk; they emit only after the
            // chunk iterates cleanly, so a damaged chunk lifts nothing.
            trigger_sink: TriggerSink::Staged(Vec::new()),
            ..ScanDelta::default()
        };
        for rec in mcap::read::ChunkReader::new(header, &data).context("opening chunk")? {
            sub.absorb_parsed(rec.context("reading record inside chunk")?);
        }
        self.schemas.extend(sub.schemas);
        self.channels.extend(sub.channels);
        self.high_water_ns = self.high_water_ns.max(sub.high_water_ns);
        // Trigger channels discovered in the chunk persist for later records;
        // triggers lifted from it emit only here, after the chunk iterated
        // cleanly. The sub-delta staged them (TriggerSink::Staged) rather than
        // sending; re-emitting through the parent's live sink sends each now (a
        // damaged chunk never reaches this point, so it emits nothing).
        self.trigger_channels.extend(sub.trigger_channels);
        if let TriggerSink::Staged(staged) = sub.trigger_sink {
            for rec in staged {
                self.emit_trigger(rec);
            }
        }
        if let Some((min, max)) = sub.pending_time {
            self.pending_time = Some(match self.pending_time {
                Some((omin, omax)) => (omin.min(min), omax.max(max)),
                None => (min, max),
            });
        }
        Ok(())
    }

    /// Append one consumed record (`[record_offset, record_end)`) to the open
    /// extent, folding in the pending time bounds, and close the extent once it
    /// reaches [`EXTENT_CAP_BYTES`]. Records are consumed in offset order.
    fn extend_extent(&mut self, record_offset: u64, record_end: u64) {
        let open = self.open.get_or_insert(Extent {
            offset: record_offset,
            len: 0,
            time: None,
        });
        open.len = record_end - open.offset;
        if let Some((min, max)) = self.pending_time.take() {
            open.time = Some(match open.time {
                Some((omin, omax)) => (omin.min(min), omax.max(max)),
                None => (min, max),
            });
        }
        if open.len >= EXTENT_CAP_BYTES {
            self.closed.push(*open);
            self.open = None;
        }
    }
}

impl Tailer {
    /// A fresh tailer (no trigger tap) plus the coverage watch trigger handlers
    /// wait on. The scan reads only message timestamps; triggers arrive through
    /// the ROS interface, not the file.
    pub fn new() -> (Arc<Self>, Arc<Watch<Coverage>>) {
        Self::build(None, None)
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
        Self::build(Some(trigger_topic.into()), Some(trigger_tx))
    }

    fn build(
        trigger_topic: Option<String>,
        trigger_tx: Option<Sender<TriggerRecord>>,
    ) -> (Arc<Self>, Arc<Watch<Coverage>>) {
        let coverage = Arc::new(Watch::new(Coverage::default()));
        (
            Arc::new(Tailer {
                state: Mutex::new(TailState::default()),
                coverage: coverage.clone(),
                trigger_topic,
                trigger_tx,
            }),
            coverage,
        )
    }

    /// Snapshot one single-file plan per recording overlapping
    /// `[start_ns, end_ns]`, oldest first. A window inside one recording yields
    /// one plan; one straddling a rollover yields one per source file. Empty
    /// when no indexed recording covers the window.
    pub fn plan_window(&self, start_ns: u64, end_ns: u64) -> Vec<WindowPlan> {
        self.state.lock().unwrap().plan_window(start_ns, end_ns)
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
    /// through the whole [`MAX_SCAN_FAULTS`] budget) or a magic mismatch; the
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
            (r.path.clone(), r.file.clone(), r.offset, r.magic_ok)
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
                r.offset = offset;
                r.magic_ok = true;
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
            r.magic_ok = true;
            r.offset = MAGIC.len() as u64;
        }
        st.mark_tailing(id);
    }

    /// Publish one scan pass's delta to the `current` recording — its registry,
    /// extents, and time bounds — record its new scan offset, and refresh the
    /// collection-wide coverage high-water (monotonic; never lowered). The brief
    /// state lock is the only one a handler's `plan_window` can contend on; the
    /// file IO above ran with no lock held.
    fn apply_to_current(&self, delta: ScanDelta, offset: u64) {
        // Triggers were already sent straight down the tap as the scan lifted
        // them ([`ScanDelta::emit_trigger`]); apply only publishes the index and
        // advances coverage. A handler that received a trigger before this runs
        // still cannot cut its clip until coverage reaches the window end, and
        // coverage advances only here — so the cut never races ahead of the
        // index it reads.
        let hw = {
            let mut st = self.state.lock().unwrap();
            if let Some(id) = st.current
                && let Some(r) = st.recording_mut(id)
            {
                r.apply_delta(delta);
                r.offset = offset;
            }
            st.high_water_ns()
        };
        self.coverage.send_if_modified(|c| {
            if hw > c.high_water_ns {
                c.high_water_ns = hw;
                true
            } else {
                false
            }
        });
    }

    /// One incremental pass: consume every record completely on disk in
    /// `[offset, file_len)`, then publish the index/registry/coverage updates.
    /// Stops without error at the first record still being appended.
    ///
    /// Returns a plain [`ScanProgress`] rather than a `Result`: localized
    /// damage is skipped (a damaged chunk, an unparseable schema/channel, a
    /// runt message — warned and consumed), and only **framing** faults stop
    /// the pass. A framing fault — a record length past [`MAX_RECORD_LEN`], or
    /// an IO error reading a record's header or body — leaves no resync point,
    /// so the pass applies the delta it accumulated up to the faulted record
    /// and reports `fault = Some(_)` with `offset` at that record.
    ///
    /// **Resume invariant:** the partial delta is already applied, so a caller
    /// retrying after a fault MUST resume at the returned `offset` (the faulted
    /// record), never earlier. Re-scanning an already-applied region makes
    /// [`ScanDelta::extend_extent`] compute `record_end - open.offset` across
    /// bytes the open extent already spans and underflow.
    pub(crate) fn scan_available(
        &self,
        file: &File,
        mut offset: u64,
        file_len: u64,
    ) -> ScanProgress {
        let mut delta = {
            let st = self.state.lock().unwrap();
            let current = st.current.and_then(|id| st.recording(id));
            ScanDelta {
                open: current.and_then(|r| r.open),
                // Seed the tap from the tailer and the current recording's known
                // trigger channels; both stay empty/idle when the tap is disabled.
                // The top-level delta carries the live sink, so triggers it lifts
                // send straight down the tap as the scan finds them.
                trigger_topic: self.trigger_topic.clone(),
                trigger_sink: self
                    .trigger_tx
                    .clone()
                    .map_or(TriggerSink::Off, TriggerSink::Live),
                trigger_channels: current
                    .map(|r| r.trigger_channels.clone())
                    .unwrap_or_default(),
                ..ScanDelta::default()
            }
        };
        let mut ended = false;
        // The offset of the faulted record is `offset` (left unadvanced) when
        // a fault breaks the loop; the partial delta is applied regardless.
        let mut fault: Option<anyhow::Error> = None;

        while offset + 9 <= file_len {
            let mut hdr = [0u8; 9];
            if let Err(e) = file.read_exact_at(&mut hdr, offset) {
                fault = Some(anyhow::Error::new(e).context(format!(
                    "reading record header at {offset}; framing desynchronised?"
                )));
                break;
            }
            let opcode = hdr[0];
            let len = u64::from_le_bytes(hdr[1..9].try_into().unwrap());
            if len > MAX_RECORD_LEN {
                fault = Some(anyhow::anyhow!(
                    "record at offset {offset} declares {len} bytes; framing desynchronised?"
                ));
                break;
            }
            let end = offset + 9 + len;
            if end > file_len {
                break; // still being appended; complete on a later pass
            }
            match opcode {
                op::SCHEMA | op::CHANNEL => {
                    let body = match read_body(file, offset + 9, len) {
                        Ok(body) => body,
                        Err(e) => {
                            fault = Some(e.context(format!(
                                "reading record body at {offset}; framing desynchronised?"
                            )));
                            break;
                        }
                    };
                    // An unparseable Schema/Channel (e.g. an invalid-UTF-8
                    // name or topic — spec-legal bytes the parser rejects) is
                    // warned and consumed, not propagated: the framing is
                    // intact, so the scan skips the record and keeps indexing
                    // the rest, as the CHUNK arm does for a damaged chunk.
                    match mcap::parse_record(opcode, &body) {
                        Ok(rec) => delta.absorb_parsed(rec),
                        Err(e) => warn!("parsing record at {offset}: {e:#}; skipping it"),
                    }
                }
                op::MESSAGE => {
                    // Decode the 14-byte prefix: channel_id u16, sequence u32,
                    // log_time u64 (all LE). A message on a trigger channel also
                    // has its payload (past the 22-byte fixed fields) lifted out;
                    // every other body stays untouched until extraction.
                    if len >= 14 {
                        let mut prefix = [0u8; 14];
                        if let Err(e) = file.read_exact_at(&mut prefix, offset + 9) {
                            fault = Some(anyhow::Error::new(e).context(format!(
                                "reading message prefix at {offset}; framing desynchronised?"
                            )));
                            break;
                        }
                        let channel_id = u16::from_le_bytes(prefix[0..2].try_into().unwrap());
                        let log_time = u64::from_le_bytes(prefix[6..14].try_into().unwrap());
                        delta.absorb_time(log_time);

                        if let Some(encoding) = delta.trigger_channels.get(&channel_id).cloned() {
                            // The payload follows the 22-byte fixed fields. A
                            // conformant Message is >= 22 B; a shorter trigger
                            // record carries no payload to decode and is skipped
                            // (a warning), the framing intact.
                            if len >= 22 {
                                let mut payload = vec![0u8; (len - 22) as usize];
                                if let Err(e) = file.read_exact_at(&mut payload, offset + 9 + 22) {
                                    fault = Some(anyhow::Error::new(e).context(format!(
                                        "reading trigger payload at {offset}; framing desynchronised?"
                                    )));
                                    break;
                                }
                                // A top-level record is durable the instant its
                                // framing is read, so capture_trigger sends it
                                // straight down the tap (no chunk CRC to clear).
                                delta.capture_trigger(encoding, payload, log_time);
                            } else {
                                warn!(
                                    "trigger message at {offset} is only {len} B; no payload to decode"
                                );
                            }
                        }
                    } else {
                        // A conformant Message body is >= 22 bytes (its fixed
                        // fields alone); below 14 not even log_time exists.
                        // No writer produces this — corrupt or mis-framed
                        // data. The record is still consumed (the framing is
                        // self-consistent), but its time cannot count toward
                        // extent bounds or coverage.
                        warn!("message record at {offset} is only {len} B; no timestamp to index");
                    }
                }
                op::CHUNK => {
                    let body = match read_body(file, offset + 9, len) {
                        Ok(body) => body,
                        Err(e) => {
                            fault = Some(e.context(format!(
                                "reading record body at {offset}; framing desynchronised?"
                            )));
                            break;
                        }
                    };
                    if let Err(e) = delta.absorb_chunk(&body) {
                        // A chunk that fails to decompress, fails its CRC, or
                        // holds an unparseable interior record cannot say which
                        // of its bytes are lying, so its whole contribution is
                        // dropped — clip.rs drops the same chunk at extraction.
                        // The record's framing is intact (its length prefix is
                        // self-consistent), so it is still consumed: the scan
                        // skips it and keeps indexing the records behind it.
                        warn!("absorbing chunk at {offset}: {e:#}; skipping it");
                    }
                }
                op::DATA_END | op::FOOTER => {
                    ended = true;
                }
                _ => {} // Header, message/chunk indexes, attachments, …
            }
            if ended {
                break;
            }
            delta.extend_extent(offset, end);
            offset = end;
        }

        self.apply_to_current(delta, offset);
        ScanProgress {
            offset,
            ended,
            fault,
        }
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
/// inode. This is the narrowed survivor of the old whole-directory `superseded`
/// check: *"is **my** file still live"*, not *"is there a newer file"* — the
/// [`crate::discover::NewFileWatchIterator`] owns the latter. The open
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

/// Nanoseconds since the Unix epoch on the system clock — the same base as
/// message `log_time`, for computing the retention watch floor.
fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos().min(u64::MAX as u128) as u64)
        .unwrap_or(0)
}

fn file_len(file: &File) -> Result<u64> {
    Ok(file.metadata().context("stat of tailed file")?.len())
}

fn read_body(file: &File, offset: u64, len: u64) -> Result<Vec<u8>> {
    let mut body = vec![0u8; len as usize];
    file.read_exact_at(&mut body, offset)
        .with_context(|| format!("reading {len} B record body at {offset}"))?;
    Ok(body)
}

#[cfg(test)]
pub(crate) mod tests {
    use std::io::BufWriter;
    use std::time::{SystemTime, UNIX_EPOCH};

    use super::*;

    /// Write a finished recording with one message per `(topic, log_time)`.
    pub(crate) fn write_recording(
        path: &Path,
        chunked: bool,
        stamps: &[(&str, u64)],
    ) -> Result<()> {
        let opts = if chunked {
            // A tiny chunk size forces a chunk per message or two, so a test
            // window spans several chunks.
            mcap::WriteOptions::new()
                .use_chunks(true)
                .compression(Some(mcap::Compression::Zstd))
                .chunk_size(Some(128))
        } else {
            mcap::WriteOptions::new()
                .use_chunks(false)
                .compression(None)
        };
        write_recording_opts(path, opts, b"payload", stamps)
    }

    /// [`write_recording`] with explicit writer options and payload, for tests
    /// that need a specific chunk layout or extent-cap-sized messages.
    pub(crate) fn write_recording_opts(
        path: &Path,
        opts: mcap::WriteOptions,
        payload: &[u8],
        stamps: &[(&str, u64)],
    ) -> Result<()> {
        let mut writer = opts.create(BufWriter::new(File::create(path)?))?;
        let mut ids: HashMap<&str, u16> = HashMap::new();
        for (seq, (topic, log_time)) in stamps.iter().enumerate() {
            let id = match ids.get(topic) {
                Some(id) => *id,
                None => {
                    let schema =
                        writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data")?;
                    let id = writer.add_channel(schema, topic, "cdr", &BTreeMap::new())?;
                    ids.insert(topic, id);
                    id
                }
            };
            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: id,
                    sequence: seq as u32,
                    log_time: *log_time,
                    publish_time: *log_time,
                },
                payload,
            )?;
        }
        writer.finish()?;
        Ok(())
    }

    pub(crate) fn test_dir(name: &str) -> Result<PathBuf> {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
        let path =
            std::env::temp_dir().join(format!("clipper-{name}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&path)?;
        Ok(path)
    }

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

    /// The single plan a one-recording test cuts from: [`Tailer::plan_window`]
    /// returns one plan per overlapping recording, and these tests index one, so
    /// its `Vec` holds at most one; no overlap becomes an empty plan.
    pub(crate) fn plan_one(tailer: &Tailer, start_ns: u64, end_ns: u64) -> WindowPlan {
        tailer
            .plan_window(start_ns, end_ns)
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

    #[test]
    fn scan_consumes_only_complete_records_and_resumes() -> Result<()> {
        let root = test_dir("grow")?;
        let finished = root.join("finished.mcap");
        write_recording(
            &finished,
            false,
            &[("/a", 100), ("/a", 200), ("/b", 300), ("/a", 400)],
        )?;
        let full = std::fs::read(&finished)?;

        // Expose only a prefix that ends inside some record, as a writer
        // mid-append would.
        let growing = root.join("growing.mcap");
        let cut = full.len() / 2;
        std::fs::write(&growing, &full[..cut])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(File::open(&growing)?);
        tailer.attach(file.clone());

        let p1 = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert!(!p1.ended, "prefix must not look finished");
        assert!(
            p1.offset <= cut as u64,
            "scan must stop at or before the cut ({} > {cut})",
            p1.offset
        );

        // The file "grows" to its full content; the scan resumes where it
        // stopped and runs into DataEnd.
        std::fs::write(&growing, &full)?;
        let p2 = scan_to_end(&tailer, &file, p1.offset)?;
        assert!(p2.ended, "full file ends with DataEnd/Footer");

        let cov = coverage.get();
        assert_eq!(cov.high_water_ns, 400);

        let plan = plan_one(&tailer, 150, 350);
        assert!(!plan.extents.is_empty());
        let topics: Vec<_> = plan.channels.values().map(|c| c.topic.clone()).collect();
        assert!(topics.contains(&"/a".to_string()) && topics.contains(&"/b".to_string()));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn scan_harvests_registry_and_times_from_inside_chunks() -> Result<()> {
        let root = test_dir("chunked")?;
        let path = root.join("rec.mcap");
        write_recording(&path, true, &[("/a", 10), ("/b", 20), ("/a", 30)])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        assert!(progress.ended);
        assert_eq!(coverage.get().high_water_ns, 30);
        let plan = plan_one(&tailer, 0, 100);
        assert_eq!(plan.channels.len(), 2, "channels live inside the chunks");
        assert!(
            plan.channels.values().all(|c| c.schema.is_some()),
            "schemas must be resolved"
        );
        assert!(!plan.extents.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn extents_outside_the_window_are_not_planned() -> Result<()> {
        let root = test_dir("window")?;
        let path = root.join("rec.mcap");
        write_recording(&path, false, &[("/a", 100), ("/a", 200)])?;

        let (tailer, _coverage) = Tailer::new();
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        assert!(plan_one(&tailer, 300, 500).extents.is_empty());
        assert!(!plan_one(&tailer, 150, 500).extents.is_empty());

        // Inclusive boundaries, exactly at the extent's min/max (100, 200):
        // a window touching a bound by one nanosecond still plans the extent.
        assert!(!plan_one(&tailer, 200, 500).extents.is_empty());
        assert!(plan_one(&tailer, 201, 500).extents.is_empty());
        assert!(!plan_one(&tailer, 0, 100).extents.is_empty());
        assert!(plan_one(&tailer, 0, 99).extents.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A length-prefixed top-level record as the writer lays it down.
    pub(crate) fn raw_record(opcode: u8, body: &[u8]) -> Vec<u8> {
        let mut rec = vec![opcode];
        rec.extend_from_slice(&(body.len() as u64).to_le_bytes());
        rec.extend_from_slice(body);
        rec
    }

    /// A conformant `Message` record body (22 fixed bytes + payload).
    pub(crate) fn message_body(
        channel_id: u16,
        sequence: u32,
        log_time: u64,
        payload: &[u8],
    ) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&channel_id.to_le_bytes());
        body.extend_from_slice(&sequence.to_le_bytes());
        body.extend_from_slice(&log_time.to_le_bytes());
        body.extend_from_slice(&log_time.to_le_bytes()); // publish_time
        body.extend_from_slice(payload);
        body
    }

    /// A `Channel` record body (id, schema_id, topic, encoding, empty metadata).
    pub(crate) fn channel_body(id: u16, schema_id: u16, topic: &str, encoding: &str) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&id.to_le_bytes());
        body.extend_from_slice(&schema_id.to_le_bytes());
        body.extend_from_slice(&(topic.len() as u32).to_le_bytes());
        body.extend_from_slice(topic.as_bytes());
        body.extend_from_slice(&(encoding.len() as u32).to_le_bytes());
        body.extend_from_slice(encoding.as_bytes());
        body.extend_from_slice(&0u32.to_le_bytes());
        body
    }

    /// An uncompressed `Chunk` record body wrapping `records` (each a raw
    /// length-prefixed interior record), with a caller-supplied
    /// `uncompressed_crc`. `mcap::read::ChunkReader` yields the interior
    /// records as it walks and verifies the CRC only at the end of iteration,
    /// so a deliberately wrong CRC lets a test absorb the messages and then
    /// fail. `compression` is the chunk's algorithm string (empty for none);
    /// an unknown string fails `ChunkReader` construction outright.
    pub(crate) fn chunk_body(
        compression: &str,
        uncompressed_crc: u32,
        records: &[Vec<u8>],
    ) -> Vec<u8> {
        let interior: Vec<u8> = records.concat();
        let mut body = Vec::new();
        body.extend_from_slice(&0u64.to_le_bytes()); // message_start_time
        body.extend_from_slice(&0u64.to_le_bytes()); // message_end_time
        body.extend_from_slice(&(interior.len() as u64).to_le_bytes()); // uncompressed_size
        body.extend_from_slice(&uncompressed_crc.to_le_bytes());
        body.extend_from_slice(&(compression.len() as u32).to_le_bytes());
        body.extend_from_slice(compression.as_bytes());
        body.extend_from_slice(&(interior.len() as u64).to_le_bytes()); // records length
        body.extend_from_slice(&interior);
        body
    }

    /// The magic followed by the given raw records, as one file.
    pub(crate) fn write_raw(path: &Path, records: &[Vec<u8>]) -> Result<()> {
        let mut bytes = MAGIC.to_vec();
        for rec in records {
            bytes.extend_from_slice(rec);
        }
        std::fs::write(path, bytes)?;
        Ok(())
    }

    /// A recording that is still being written: one schemaless channel and its
    /// messages, with no DataEnd/Footer — exactly the shape a live tail sees.
    pub(crate) fn write_unfinished_recording(
        path: &Path,
        topic: &str,
        stamps: &[u64],
    ) -> Result<()> {
        let mut records = vec![raw_record(op::CHANNEL, &channel_body(1, 0, topic, "cdr"))];
        for (seq, t) in stamps.iter().enumerate() {
            records.push(raw_record(
                op::MESSAGE,
                &message_body(1, seq as u32, *t, b"payload"),
            ));
        }
        write_raw(path, &records)
    }

    /// A `Schema` record body (id, name, encoding, length-prefixed data).
    fn schema_body(id: u16, name: &str, encoding: &str, data: &[u8]) -> Vec<u8> {
        let mut body = Vec::new();
        body.extend_from_slice(&id.to_le_bytes());
        body.extend_from_slice(&(name.len() as u32).to_le_bytes());
        body.extend_from_slice(name.as_bytes());
        body.extend_from_slice(&(encoding.len() as u32).to_le_bytes());
        body.extend_from_slice(encoding.as_bytes());
        body.extend_from_slice(&(data.len() as u32).to_le_bytes());
        body.extend_from_slice(data);
        body
    }

    #[test]
    fn schema_following_its_channel_in_one_pass_still_resolves() -> Result<()> {
        let root = test_dir("schema-after")?;
        let path = root.join("rec.mcap");
        // The spec orders Schema before any Channel referencing it; this file
        // violates that. `apply` inserts a pass's schemas before resolving its
        // channels, so the inversion still resolves — leniency, not a promise:
        // a schema arriving only in a *later* pass stays unresolved (see
        // `dangling_schema_id_yields_a_channel_without_schema`).
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 5, "/x", "cdr")),
                raw_record(
                    op::SCHEMA,
                    &schema_body(5, "std_msgs/msg/String", "ros2msg", b"string data"),
                ),
                // A message so the channel's extent carries a time and is
                // plannable; the registry resolution is what this test checks.
                raw_record(op::MESSAGE, &message_body(1, 0, 10, b"x")),
            ],
        )?;

        let (tailer, _coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let plan = plan_one(&tailer, 0, u64::MAX);
        let ch = plan.channels.get(&1).expect("channel registered");
        let schema = ch.schema.as_ref().expect("same-pass schema resolves");
        assert_eq!(schema.name, "std_msgs/msg/String");

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
    fn oversized_record_length_faults_after_applying_the_good_prefix() -> Result<()> {
        let root = test_dir("desync")?;
        let path = root.join("rec.mcap");
        // A clean prefix — channel + two messages — then a record header whose
        // declared length is past MAX_RECORD_LEN. The scan applies the prefix,
        // then faults at the oversized record: there is no resync point, but
        // the index the prefix built must survive (this is what makes the
        // bounded retry idempotent — it resumes exactly at the fault offset).
        let good = [
            raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
            raw_record(op::MESSAGE, &message_body(1, 0, 100, b"x")),
            raw_record(op::MESSAGE, &message_body(1, 1, 200, b"y")),
        ];
        let bad_offset = MAGIC.len() as u64 + good.iter().map(|r| r.len() as u64).sum::<u64>();
        let mut bytes = MAGIC.to_vec();
        for rec in &good {
            bytes.extend_from_slice(rec);
        }
        bytes.push(op::MESSAGE);
        bytes.extend_from_slice(&(MAX_RECORD_LEN + 1).to_le_bytes());
        std::fs::write(&path, bytes)?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let fault = progress.fault.expect("the oversized record must fault");
        let msg = format!("{fault:#}");
        assert!(
            msg.contains("framing desynchronised") && msg.contains(&bad_offset.to_string()),
            "fault must name the framing desync and the offset: {msg}"
        );
        assert_eq!(
            progress.offset, bad_offset,
            "the fault offset is the oversized record, so a retry resumes there"
        );

        // The good prefix was applied before the fault.
        assert_eq!(coverage.get().high_water_ns, 200);
        assert!(
            !plan_one(&tailer, 50, 250).extents.is_empty(),
            "the prefix's extent stays plannable across the fault"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn corrupt_chunk_is_skipped_without_poisoning_the_scan() -> Result<()> {
        let root = test_dir("badchunk")?;
        let path = root.join("rec.mcap");
        // A chunk record whose body is garbage cannot absorb; the scan must
        // warn, consume it (the framing is intact — the length prefix is
        // self-consistent), and keep indexing the records behind it, exactly
        // as clip.rs drops a damaged chunk during extraction.
        write_raw(
            &path,
            &[
                raw_record(op::CHUNK, &[0xFF; 16]),
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 42, b"x")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "the bad chunk and the good records after it are all consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 42);

        let plan = plan_one(&tailer, 0, 100);
        assert!(!plan.extents.is_empty(), "the good message is indexed");
        let ch = plan
            .channels
            .get(&1)
            .expect("channel after the chunk registered");
        assert_eq!(ch.topic, "/t");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn chunk_failing_its_crc_contributes_nothing() -> Result<()> {
        let root = test_dir("chunk-rollback")?;
        let path = root.join("rec.mcap");
        // A chunk whose interior is a valid channel + message but whose
        // uncompressed_crc is wrong: ChunkReader yields both records and only
        // fails the CRC at the end of iteration. Because extraction would drop
        // the whole chunk, the scan must claim none of it — no channel
        // registered, no time folded into coverage or extent bounds — even
        // though the records absorbed cleanly before the CRC check failed.
        let chunk = chunk_body(
            "",
            0xDEAD_BEEF, // not the real CRC of the interior
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/inside", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 500, b"x")),
            ],
        );
        // A good message after the chunk proves the scan keeps going.
        write_raw(
            &path,
            &[
                raw_record(op::CHUNK, &chunk),
                raw_record(op::CHANNEL, &channel_body(2, 0, "/after", "cdr")),
                raw_record(op::MESSAGE, &message_body(2, 0, 700, b"y")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "the chunk and the records after it are all consumed"
        );
        assert_eq!(
            coverage.get().high_water_ns,
            700,
            "the dropped chunk's message (500) never reaches coverage"
        );

        let plan = plan_one(&tailer, 0, u64::MAX);
        assert!(
            plan.channels.contains_key(&2),
            "the post-chunk channel registers"
        );
        assert!(
            !plan.channels.contains_key(&1),
            "the failed chunk's channel must not register"
        );
        // No extent may claim the dropped message's time (500); only the good
        // post-chunk message (700) is in the bounds.
        for e in &plan.extents {
            if let Some((min, max)) = e.time {
                assert!(
                    !(min <= 500 && 500 <= max),
                    "extent {e:?} must not cover the dropped message's time"
                );
            }
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn unsupported_chunk_compression_is_skipped() -> Result<()> {
        let root = test_dir("chunk-compression")?;
        let path = root.join("rec.mcap");
        // A spec-legal chunk whose compression algorithm this build does not
        // support: ChunkReader construction fails, so the chunk is skipped
        // whole rather than poisoning the scan — the records behind it index.
        let chunk = chunk_body(
            "custom-xyz",
            0,
            &[raw_record(op::MESSAGE, &message_body(1, 0, 100, b"x"))],
        );
        write_raw(
            &path,
            &[
                raw_record(op::CHUNK, &chunk),
                raw_record(op::CHANNEL, &channel_body(9, 0, "/after", "cdr")),
                raw_record(op::MESSAGE, &message_body(9, 0, 300, b"y")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 300);
        assert!(
            plan_one(&tailer, 0, u64::MAX).channels.contains_key(&9),
            "data after the unsupported chunk still indexes"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn unparseable_top_level_channel_is_skipped() -> Result<()> {
        let root = test_dir("bad-channel")?;
        let path = root.join("rec.mcap");
        // A Channel record whose topic field carries invalid UTF-8 bytes:
        // mcap::parse_record fails on it. The scan must warn, consume the
        // record (its framing is intact), and keep indexing — the same
        // leniency the CHUNK arm already gives a damaged chunk.
        let mut bad_channel = Vec::new();
        bad_channel.extend_from_slice(&1u16.to_le_bytes()); // id
        bad_channel.extend_from_slice(&0u16.to_le_bytes()); // schema_id
        bad_channel.extend_from_slice(&2u32.to_le_bytes()); // topic length
        bad_channel.extend_from_slice(&[0xFF, 0xFE]); // invalid UTF-8 topic
        bad_channel.extend_from_slice(&(3u32).to_le_bytes()); // encoding length
        bad_channel.extend_from_slice(b"cdr");
        bad_channel.extend_from_slice(&0u32.to_le_bytes()); // empty metadata

        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &bad_channel),
                raw_record(op::CHANNEL, &channel_body(2, 0, "/good", "cdr")),
                raw_record(op::MESSAGE, &message_body(2, 0, 55, b"x")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 55);

        let plan = plan_one(&tailer, 0, 100);
        assert!(
            !plan.channels.contains_key(&1),
            "the unparseable channel must not register"
        );
        let ch = plan.channels.get(&2).expect("the good channel registers");
        assert_eq!(ch.topic, "/good");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn runt_message_is_consumed_without_poisoning_the_index() -> Result<()> {
        let root = test_dir("runt")?;
        let path = root.join("rec.mcap");
        // First record: a Message too short to even hold a log_time. The scan
        // must warn, consume it (the framing is self-consistent) and keep
        // indexing the records behind it.
        write_raw(
            &path,
            &[
                raw_record(op::MESSAGE, &[0xAA; 4]),
                raw_record(op::MESSAGE, &message_body(1, 0, 42, b"x")),
            ],
        )?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "both records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 42);

        let plan = plan_one(&tailer, 0, 100);
        assert_eq!(plan.extents.len(), 1);
        assert_eq!(
            plan.extents[0].time,
            Some((42, 42)),
            "the runt contributes no time bound"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn bare_magic_or_partial_header_makes_no_progress() -> Result<()> {
        let root = test_dir("stub")?;
        let path = root.join("rec.mcap");
        let mut bytes = MAGIC.to_vec();
        bytes.extend_from_slice(&[0x05, 0x01, 0x02, 0x03, 0x04]); // 5 of 9 header bytes
        std::fs::write(&path, bytes)?;

        let (tailer, coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        let start = MAGIC.len() as u64;

        // Only the magic on disk: nothing to scan, nothing to fault on.
        let p = tailer.scan_available(&file, start, start);
        assert_eq!(p.offset, start);
        assert!(!p.ended && p.fault.is_none());

        // A record header still being appended: same outcome.
        let p = tailer.scan_available(&file, start, file.metadata()?.len());
        assert_eq!(p.offset, start);
        assert!(!p.ended && p.fault.is_none());
        assert_eq!(coverage.get().high_water_ns, 0);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn out_of_order_stamps_keep_high_water_and_widen_extent_bounds() -> Result<()> {
        let root = test_dir("ooo")?;
        let path = root.join("rec.mcap");
        write_recording(&path, false, &[("/a", 100), ("/a", 50)])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        assert_eq!(
            coverage.get().high_water_ns,
            100,
            "high water never moves backwards"
        );
        let plan = plan_one(&tailer, 40, 60);
        assert_eq!(plan.extents.len(), 1, "the late stamp widens the bounds");
        assert_eq!(plan.extents[0].time, Some((50, 100)));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn extents_close_at_the_cap_and_tile_contiguously() -> Result<()> {
        let root = test_dir("cap")?;
        let path = root.join("rec.mcap");
        let payload = vec![0u8; 1 << 20]; // 1 MiB per message, ~9 MiB total
        let stamps: Vec<(&str, u64)> = (1..=9).map(|i| ("/big", i)).collect();
        write_recording_opts(
            &path,
            mcap::WriteOptions::new()
                .use_chunks(false)
                .compression(None),
            &payload,
            &stamps,
        )?;

        let (tailer, _coverage) = Tailer::new();
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let plan = plan_one(&tailer, 0, u64::MAX);
        assert!(plan.extents.len() >= 2, "the cap must have closed extents");
        assert_eq!(plan.extents[0].offset, MAGIC.len() as u64);
        for pair in plan.extents.windows(2) {
            assert_eq!(
                pair[1].offset,
                pair[0].offset + pair[0].len,
                "extents tile the data section with no gap or overlap"
            );
        }
        for e in &plan.extents[..plan.extents.len() - 1] {
            assert!(e.len >= EXTENT_CAP_BYTES, "closed extents reached the cap");
        }

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
        assert_eq!(tailer.plan_window(900, 2_100).len(), 1);
        assert_eq!(tailer.plan_window(4_900, 6_100).len(), 1);

        // A window straddling the rollover plans both, oldest first.
        let plans = tailer.plan_window(1_500, 5_500);
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
        let in_flight = tailer.plan_window(900, 1_100);
        assert_eq!(in_flight.len(), 1, "split0 is plannable before the prune");

        // Prune with a floor above split0's data (1_000) but below split1's
        // (9_000): split0 is dropped, split1 retained. (Both are Ended here;
        // the floor, not the state, decides — `current` is None after draining.)
        let dropped = tailer.state.lock().unwrap().prune(5_000);
        assert_eq!(dropped, vec![split0.clone()], "the aged file is pruned");

        assert!(
            tailer.plan_window(900, 1_100).is_empty(),
            "split0's index is gone after the prune"
        );
        assert!(
            !tailer.plan_window(8_900, 9_100).is_empty(),
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
        assert!(!tailer.plan_window(900, 1_100).is_empty());

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

    #[test]
    fn dangling_schema_id_yields_a_channel_without_schema() -> Result<()> {
        let root = test_dir("dangling")?;
        let path = root.join("rec.mcap");
        // A Channel referencing schema 7, which never appears on disk —
        // either corruption or a schema record still in flight. The channel
        // must still register (messages on it are clippable, schemaless).
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 7, "/raw", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 10, b"x")),
            ],
        )?;

        let (tailer, _coverage) = Tailer::new();
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let plan = plan_one(&tailer, 0, 100);
        let ch = plan.channels.get(&1).expect("channel registered");
        assert_eq!(ch.topic, "/raw");
        assert!(ch.schema.is_none());

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
            tailer.plan_window(900, 5_100).len(),
            2,
            "a straddling window recovers both splits"
        );

        // Detached: it loops forever against split 1 and dies with the process.
        drop(handle);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── trigger tap ─────────────────────────────────────────────────────────

    /// A top-level message on the tapped topic is lifted whole as a
    /// `TriggerRecord` and sent down the tap the instant its framing is read;
    /// messages on other topics contribute only their timestamp.
    #[test]
    fn tap_lifts_a_top_level_trigger_message() -> Result<()> {
        let root = test_dir("tap-toplevel")?;
        let path = root.join("rec.mcap");
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/trig", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 500, b"PAYLOAD")),
                raw_record(op::CHANNEL, &channel_body(2, 0, "/data", "cdr")),
                raw_record(op::MESSAGE, &message_body(2, 0, 600, b"ignored")),
            ],
        )?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, _coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let lifted: Vec<_> = rx.try_iter().collect();
        assert_eq!(lifted.len(), 1, "only the trigger-topic message is lifted");
        assert_eq!(lifted[0].message_encoding, "cdr");
        assert_eq!(lifted[0].body, b"PAYLOAD");
        assert_eq!(lifted[0].log_time, 500);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A message on the tapped topic that lives inside a chunk is lifted too, but
    /// only after the chunk iterates cleanly (real chunked writer, valid CRC).
    #[test]
    fn tap_lifts_a_chunk_interior_trigger_message() -> Result<()> {
        let root = test_dir("tap-chunk")?;
        let path = root.join("rec.mcap");
        write_recording(&path, true, &[("/trig", 700), ("/data", 800)])?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, _coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let lifted: Vec<_> = rx.try_iter().collect();
        assert_eq!(lifted.len(), 1, "the chunk-interior trigger is lifted");
        assert_eq!(lifted[0].message_encoding, "cdr");
        assert_eq!(lifted[0].body, b"payload");
        assert_eq!(lifted[0].log_time, 700);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The failure path for the chunk's all-or-nothing staging: a trigger lifted
    /// from inside a chunk that then fails its CRC must never reach the tap. The
    /// sub-delta is discarded whole, so the staged trigger is dropped with it; a
    /// good trigger after the chunk still lifts.
    #[test]
    fn a_damaged_chunk_lifts_no_trigger() -> Result<()> {
        let root = test_dir("tap-bad-chunk")?;
        let path = root.join("rec.mcap");
        let chunk = chunk_body(
            "",
            0xDEAD_BEEF, // not the real interior CRC — ChunkReader fails at the end
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/trig", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 500, b"DROPPED")),
            ],
        );
        write_raw(
            &path,
            &[
                raw_record(op::CHUNK, &chunk),
                raw_record(op::CHANNEL, &channel_body(2, 0, "/trig", "cdr")),
                raw_record(op::MESSAGE, &message_body(2, 0, 900, b"GOOD")),
            ],
        )?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, _coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let lifted: Vec<_> = rx.try_iter().collect();
        assert_eq!(
            lifted.len(),
            1,
            "the damaged chunk's trigger is dropped; only the post-chunk one lifts"
        );
        assert_eq!(lifted[0].body, b"GOOD");
        assert_eq!(lifted[0].log_time, 900);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A trigger-topic message shorter than the 22-byte fixed fields carries no
    /// payload to lift: its timestamp still advances coverage, but no
    /// `TriggerRecord` is sent (a warning, the framing intact).
    #[test]
    fn a_runt_trigger_message_lifts_nothing() -> Result<()> {
        let root = test_dir("tap-runt")?;
        let path = root.join("rec.mcap");
        let mut runt = Vec::new();
        runt.extend_from_slice(&1u16.to_le_bytes()); // channel_id
        runt.extend_from_slice(&0u32.to_le_bytes()); // sequence
        runt.extend_from_slice(&500u64.to_le_bytes()); // log_time — 14 bytes, < 22
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/trig", "cdr")),
                raw_record(op::MESSAGE, &runt),
            ],
        )?;

        let (tx, rx) = crossbeam_channel::unbounded();
        let (tailer, coverage) = Tailer::with_trigger_tap("/trig", tx);
        let file = attached(&tailer, &path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        assert!(
            rx.try_iter().next().is_none(),
            "a runt trigger message lifts no TriggerRecord"
        );
        assert_eq!(
            coverage.get().high_water_ns,
            500,
            "but its timestamp still advances coverage"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
