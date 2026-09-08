//! The format layer of one MCAP recording: its schema/channel registry, its
//! extent index carrying both time spans, and the incremental scan that fills
//! them from a file that may still be growing.
//!
//! Reading a recording while it is still being written is sound because of two
//! properties of the format. The writer is **append-only** while recording:
//! bytes below the current end of file never change, and the summary/footer is
//! appended only at close, so everything behind the last complete record is
//! immutable. And **every record is length-prefixed** (a 1-byte opcode + u64le
//! length), so a record whose declared extent runs past the current file length
//! is still being appended — the scan stops there and resumes on a later pass,
//! never re-reading a byte it has already consumed. An in-progress file is
//! therefore indistinguishable from a crash-truncated one, which MCAP readers
//! are built to tolerate. (Both properties are a producer requirement in
//! disguise — append complete records, never seek back to rewrite one; the full
//! treatment, including the seek-back writer that violates it, is in
//! `crates/clipper/CLAUDE.md`.)
//!
//! Two artefacts come out of a pass ([`scan_available`]), folded into a
//! [`RecordingIndex`] by the caller ([`RecordingIndex::apply_delta`]) and served
//! to the cut path as a [`WindowPlan`]:
//!
//! * **Extent index** — contiguous byte ranges of the file (closed at
//!   `EXTENT_CAP_BYTES`) carrying the min/max `log_time` and `publish_time` of
//!   the messages they hold. A clip reads only the extents whose span on the
//!   active time source overlaps its window, so cutting a clip never rescans the
//!   file.
//! * **Schema/channel registry** — every `Schema`/`Channel` record seen, keyed
//!   by the file's channel ID (unique within one continuous file). Chunked
//!   recordings carry these *inside* chunks, so chunks are decompressed during
//!   the scan; an unchunked recording (the fastwrite storage profile) pays no
//!   such cost.
//!
//! Coverage — how far a caller can prove the recording reaches — is read off
//! [`RecordingIndex::bounds`] once a pass has been applied, never off the pass
//! itself. The two disagree exactly when a pass faults partway: a message whose
//! stamps were folded but whose extent was never closed counts toward neither
//! the index nor anything a window can plan from, so a caller that trusted the
//! pass would claim coverage over data the cut would leave out.
//!
//! Only the 22-byte fixed header of each top-level `Message` record is read
//! during a scan (channel id, sequence, `log_time`, `publish_time`); message
//! bodies are first touched by the cut ([`crate::cut`]). The one exception is an
//! opt-in trigger tap ([`ScanSeed::tap`]): when set, the scan also lifts
//! the full body of messages on the trigger topic out as
//! [`TriggerRecord`]s for the caller to decode by `message_encoding`. With the
//! tap unset — the default — no message body is read during the scan at all.
//!
//! Damage in the recording is tolerated the way [`crate::cut`] tolerates it at
//! extraction: a damaged chunk, an unparseable schema/channel, or a runt message
//! is warned and skipped, the framing intact. A **framing** fault has no resync
//! point (a record length past [`MAX_RECORD_LEN`], or an IO error reading a
//! record), so the scan stops at it, having accumulated everything before it,
//! and the caller retries from exactly that offset — never rescanning from
//! scratch.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::os::unix::fs::FileExt;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use crossbeam_channel::Sender;
use log::{debug, warn};
use mcap::records::Record;

use crate::TimeSource;
use crate::trigger::TriggerRecord;

/// The 8 magic bytes opening (and, after `finish`, closing) every MCAP file.
pub const MAGIC: [u8; 8] = *b"\x89MCAP0\r\n";

/// Extents close once they cover this many bytes, bounding both the bytes one
/// index entry stands for and the index's growth (one entry per cap per file).
const EXTENT_CAP_BYTES: u64 = 4 * 1024 * 1024;

/// Upper bound on a plausible single record. A length beyond this means the
/// scan is desynchronised from the record framing (or the file is corrupt).
/// [`crate::cut`] applies the same bound to the records it reads back out
/// of extents, including chunk-interior records after decompression.
pub const MAX_RECORD_LEN: u64 = 1 << 31;

/// MCAP record opcodes the tail dispatches on.
pub mod op {
    pub const FOOTER: u8 = 0x02;
    pub const SCHEMA: u8 = 0x03;
    pub const CHANNEL: u8 = 0x04;
    pub const MESSAGE: u8 = 0x05;
    pub const CHUNK: u8 = 0x06;
    /// The clip manifest's record type — never produced by the scan, read back
    /// by [`crate::manifest::read_manifest`].
    pub const METADATA: u8 = 0x0C;
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

/// The inclusive minimum and maximum of one timestamp source over a set of
/// messages.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Span {
    pub min: u64,
    pub max: u64,
}

impl Span {
    fn point(t: u64) -> Self {
        Span { min: t, max: t }
    }

    fn extend(&mut self, t: u64) {
        self.min = self.min.min(t);
        self.max = self.max.max(t);
    }

    fn merge(&mut self, other: Span) {
        self.min = self.min.min(other.min);
        self.max = self.max.max(other.max);
    }
}

/// The `log_time` and `publish_time` spans of a set of messages, carried
/// together because both come from the same message header. Both spans are
/// exact: either may drive extent overlap and coverage, selected by the active
/// [`TimeSource`]. Retention reads only `log` — a producer must not be able to
/// drive file deletion through `publish_time`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Stamps {
    pub log: Span,
    pub publish: Span,
}

impl Stamps {
    fn point(log: u64, publish: u64) -> Self {
        Stamps {
            log: Span::point(log),
            publish: Span::point(publish),
        }
    }

    fn extend(&mut self, log: u64, publish: u64) {
        self.log.extend(log);
        self.publish.extend(publish);
    }

    fn merge(&mut self, other: Stamps) {
        self.log.merge(other.log);
        self.publish.merge(other.publish);
    }
}

/// The span of `log_time − publish_time` over a set of messages — the
/// recorder's queue backlog plus the producer's clock skew, in nanoseconds
/// (signed: a `publish_time` past its `log_time` reads negative). Accumulated
/// per scan pass and logged at debug; no windowing reads it.
#[derive(Clone, Copy, Debug)]
struct Skew {
    min: i128,
    max: i128,
}

impl Skew {
    fn observe(&mut self, gap: i128) {
        self.min = self.min.min(gap);
        self.max = self.max.max(gap);
    }
}

/// A contiguous byte range of the recording, aligned to top-level record
/// boundaries, with the time bounds of the messages it holds. `time` is `None`
/// while the range carries no timed record (e.g. only schema/channel records).
#[derive(Clone, Copy, Debug)]
pub struct Extent {
    pub offset: u64,
    pub len: u64,
    pub time: Option<Stamps>,
}

impl Extent {
    /// Whether any message in the extent can fall inside `[start_ns, end_ns]` on
    /// the windowing `source`. Exact, not heuristic: the bounds are the actual
    /// min/max of the extent's messages on that source, so a message in the
    /// window implies its extent overlaps it. `source` picks which of the two
    /// carried spans to test — `log` or `publish`.
    fn overlaps(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> bool {
        self.time.is_some_and(|s| {
            let span = match source {
                TimeSource::Log => s.log,
                TimeSource::Publish => s.publish,
            };
            span.max >= start_ns && span.min <= end_ns
        })
    }
}

/// The recording one [`WindowPlan`] reads: the descriptor the copy pulls its
/// extents out of, and the path that descriptor was opened at.
///
/// The two travel together because a clip's manifest names the file its bytes
/// came from ([`crate::manifest`]), and a plan holding the descriptor alone
/// could not say which of a rollover's split files a segment was cut from — the
/// descriptor outlives the name, since a pruned or rotated recording stays
/// readable through the pinned `Arc<File>` after its path is gone.
#[derive(Clone, Debug)]
pub struct PlanSource {
    pub path: PathBuf,
    pub file: Arc<File>,
}

/// A snapshot for one clip: the open recording, the extents overlapping the
/// window (in file order), and the channel registry to map IDs with. `source` is
/// `None` while no recording has been discovered yet.
#[derive(Debug)]
pub struct WindowPlan {
    pub source: Option<PlanSource>,
    pub extents: Vec<Extent>,
    pub channels: HashMap<u16, ChannelDef>,
}

impl WindowPlan {
    /// A plan with no source recording — stages a channelless empty clip
    /// (magic + manifest + summary + footer) for a window no recording covers.
    /// The empty path needs no [`PlanSource`], so it serves the "no recording
    /// exists yet" case too.
    pub fn empty() -> Self {
        WindowPlan {
            source: None,
            extents: Vec::new(),
            channels: HashMap::new(),
        }
    }
}

/// The seam the shared cut path takes its plans through: whatever holds indexed
/// recordings answers "which bytes of which file cover this window", and the cut
/// reads only what comes back.
///
/// The live tailer implements it over its time-ordered collection of
/// recordings, so a window straddling a rollover yields one single-file plan per
/// source recording. An index built in one pass over an already-complete
/// recording fits the same shape, yielding at most one plan, which is what this
/// trait exists to leave room for. The cut path never learns which of
/// the two it is talking to.
pub trait WindowPlanner {
    /// One single-file [`WindowPlan`] per recording overlapping
    /// `[start_ns, end_ns]` on `source`, oldest first. Empty when nothing the
    /// planner knows covers the window (a rollover gap, all relevant files
    /// pruned, or nothing indexed yet) — the caller then stages one empty clip.
    fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan>;
}

/// The `log_time` and `publish_time` spans of the messages indexed in one
/// recording. `has_messages` is false until the first timed record lands,
/// distinguishing "no data" from "data at time 0". Either span drives window
/// overlap and coverage, selected by the active time source; retention ages on
/// `log.max` alone (against the watch floor), whatever the window's source.
#[derive(Clone, Copy, Debug, Default)]
pub struct TimeBounds {
    pub log: Span,
    pub publish: Span,
    pub has_messages: bool,
}

impl TimeBounds {
    fn absorb(&mut self, stamps: Stamps) {
        if self.has_messages {
            self.log.merge(stamps.log);
            self.publish.merge(stamps.publish);
        } else {
            self.log = stamps.log;
            self.publish = stamps.publish;
            self.has_messages = true;
        }
    }
}

/// One indexed recording: its open file handle, scan progress, extent index,
/// schema/channel registry, and time bounds. The caller owns it and drives it —
/// a live tailer holds a time-ordered collection of them, one per recording it
/// follows — feeding each scan pass's [`ScanDelta`] back in through
/// [`Self::apply_delta`] and serving windows out of it through [`Self::plan`].
#[derive(Debug)]
pub struct RecordingIndex {
    pub path: PathBuf,
    pub file: Arc<File>,
    /// The scan resume point: bytes below it are consumed, the next pass starts
    /// here. Begins at 0 (magic unverified); set past the magic once verified.
    ///
    /// **Resume invariant:** a pass that faulted returns the offset of the
    /// faulted record with its partial delta already applied here, so a caller
    /// retrying MUST resume at that offset, never earlier. Re-scanning an
    /// already-applied region makes the open extent's extension compute
    /// `record_end - open.offset` across bytes the open extent already spans and
    /// underflow.
    pub offset: u64,
    /// Whether the 8 magic bytes have been verified — a freshly created file may
    /// not hold them yet, so a caller checks before it starts scanning and
    /// leaves `offset` at 0 until it has.
    pub magic_ok: bool,
    pub extents: Vec<Extent>,
    /// The extent still accumulating records at the end of the scanned region.
    /// Included in window plans — a window may end inside it.
    pub open: Option<Extent>,
    pub schemas: HashMap<u16, SchemaDef>,
    pub channels: HashMap<u16, ChannelDef>,
    /// Channels on the trigger topic (`id -> message_encoding`), the subset of
    /// `channels` a tapping caller watches. Empty unless the scan is seeded with
    /// a trigger tap ([`ScanSeed::tap`]); seeds each scan pass so a
    /// trigger message references its channel defined in an earlier pass.
    pub trigger_channels: HashMap<u16, String>,
    pub bounds: TimeBounds,
}

impl RecordingIndex {
    /// A freshly discovered recording, indexed but not yet scanned: nothing
    /// consumed (`offset` 0, magic unverified), no extents, an empty registry.
    pub fn new(path: PathBuf, file: Arc<File>) -> Self {
        RecordingIndex {
            path,
            file,
            offset: 0,
            magic_ok: false,
            extents: Vec::new(),
            open: None,
            schemas: HashMap::new(),
            channels: HashMap::new(),
            trigger_channels: HashMap::new(),
            bounds: TimeBounds::default(),
        }
    }

    /// Fold one scan pass's result into this recording and move the cursor to
    /// where the pass stopped — the whole of what a caller does between passes.
    ///
    /// The two steps belong together. Applying the delta without advancing
    /// `offset` leaves the next pass re-reading records this one already folded,
    /// and the open extent it re-extends then computes `record_end -
    /// open.offset` across bytes it already spans, underflowing. Advancing
    /// without applying loses a pass's registry and extents outright. Take
    /// [`Self::apply_delta`] alone only when there is no [`ScanProgress`] to
    /// advance to.
    ///
    /// The pass may have stopped on a fault; `progress.offset` is then the
    /// faulted record rather than the file end, and resuming there — never
    /// earlier — is what makes a retry safe.
    pub fn advance(&mut self, delta: ScanDelta, progress: &ScanProgress) {
        self.apply_delta(delta);
        self.offset = progress.offset;
    }

    /// Fold one scan pass's delta into this recording's registry, extents, and
    /// time bounds: schemas first (so a channel resolves its schema against the
    /// registry as this pass updates it), then channels, then extents, then
    /// bounds.
    ///
    /// Leaves the scan cursor alone, which is almost never what a caller wants:
    /// [`Self::advance`] pairs it with the cursor move and is what a scan loop
    /// should call. This half stays reachable for a caller that folds a delta it
    /// did not get from [`scan_available`] — replaying one, or merging an index
    /// built elsewhere — where there is no [`ScanProgress`] to advance to.
    pub fn apply_delta(&mut self, delta: ScanDelta) {
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
            if let Some(stamps) = extent.time {
                self.bounds.absorb(stamps);
            }
        }
    }

    /// A single-file [`WindowPlan`] over this recording's extents overlapping
    /// `[start_ns, end_ns]` on `source`, or `None` if none do.
    pub fn plan(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Option<WindowPlan> {
        let extents: Vec<Extent> = self
            .extents
            .iter()
            .chain(self.open.iter())
            .filter(|e| e.overlaps(start_ns, end_ns, source))
            .copied()
            .collect();
        (!extents.is_empty()).then(|| WindowPlan {
            source: Some(PlanSource {
                path: self.path.clone(),
                file: self.file.clone(),
            }),
            extents,
            channels: self.channels.clone(),
        })
    }
}

/// Where one scan pass stopped, whether the recording ended, and whether a
/// fault stopped it short. A fault carries the framing error; `offset` is then
/// the byte offset of the faulted record (where a retry resumes), not the file
/// end. `ended` and `fault` are mutually exclusive — a pass that hits the
/// footer cannot also fault.
#[derive(Debug)]
pub struct ScanProgress {
    pub offset: u64,
    pub ended: bool,
    pub fault: Option<anyhow::Error>,
}

/// The caller's snapshot of the recording a pass resumes: the extent still open
/// at the resume offset, and the trigger tap to lift through. Everything else a
/// pass needs it reads out of the file.
///
/// A caller building one from a [`RecordingIndex`] copies its `open` extent and
/// its `trigger_channels`; the two tap fields are the caller's own choice and
/// are `None` for a scan that only indexes timestamps — the default, under which
/// no message body is read at all.
#[derive(Debug)]
pub struct ScanSeed {
    /// The extent still accumulating at the resume offset, from
    /// [`RecordingIndex::open`]. The pass keeps extending it, so records either
    /// side of a pass boundary land in one extent.
    pub open: Option<Extent>,
    /// The trigger tap: the topic whose messages the scan lifts out, and where
    /// the lifted [`TriggerRecord`]s go (the caller drains the far end).
    ///
    /// The two travel together because neither means anything alone — a topic
    /// with nowhere to send lifts bodies and drops them, a sender with no topic
    /// never fires — and a seed able to carry one without the other would fail
    /// silently either way. `None` disables the tap, and the scan is then
    /// byte-for-byte the timestamp-only pass: no message body is read at all.
    /// Sending is best-effort; a full or closed tap never stalls the scan.
    pub tap: Option<(String, Sender<TriggerRecord>)>,
    /// Channels on the trigger topic already known (`id -> message_encoding`),
    /// from [`RecordingIndex::trigger_channels`], so a trigger message resolves
    /// against its channel defined in an earlier pass.
    pub trigger_channels: HashMap<u16, String>,
}

/// Where a [`ScanDelta`] routes a trigger it lifts. A trigger emits only once it
/// is durable: a top-level record the moment its framing is read, a
/// chunk-interior record only after the chunk's CRC verifies. So the top-level
/// delta carries the live tap and sends straight down it, while a chunk sub-delta
/// stages until [`ScanDelta::absorb_chunk`] re-emits each through the parent.
#[derive(Debug, Default)]
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

/// Registry and extent updates of one scan pass, collected while the pass does
/// its file IO and folded into the caller's [`RecordingIndex`] afterwards
/// ([`RecordingIndex::apply_delta`]) — so the IO runs with no lock held and the
/// publication is one short step at the end.
#[derive(Debug, Default)]
pub struct ScanDelta {
    closed: Vec<Extent>,
    open: Option<Extent>,
    /// min/max of both stamps for the records absorbed since the last extent
    /// extension, folded into the open extent by [`Self::extend_extent`].
    pending_time: Option<Stamps>,
    schemas: Vec<(u16, SchemaDef)>,
    channels: Vec<RawChannel>,
    /// min/max of `log_time − publish_time` over the messages this pass folded.
    /// `None` until the first message; logged once per pass at debug.
    skew: Option<Skew>,
    /// The trigger topic to lift, seeded from [`ScanSeed::trigger_topic`].
    /// `None` disables the tap, so the fields below stay empty/idle and the scan
    /// never reads a body.
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
#[derive(Debug)]
struct RawChannel {
    id: u16,
    schema_id: u16,
    topic: String,
    message_encoding: String,
    metadata: BTreeMap<String, String>,
}

impl ScanDelta {
    fn absorb_time(&mut self, log_time: u64, publish_time: u64) {
        self.pending_time = Some(match self.pending_time {
            Some(mut s) => {
                s.extend(log_time, publish_time);
                s
            }
            None => Stamps::point(log_time, publish_time),
        });
        let gap = log_time as i128 - publish_time as i128;
        self.skew = Some(match self.skew {
            Some(mut sk) => {
                sk.observe(gap);
                sk
            }
            None => Skew { min: gap, max: gap },
        });
    }

    /// Route one lifted trigger by the delta's [`TriggerSink`]: the top-level
    /// delta's [`TriggerSink::Live`] sends it straight down the tap now; a chunk
    /// sub-delta's [`TriggerSink::Staged`] holds it until [`Self::absorb_chunk`]
    /// re-emits it through the parent once the chunk's CRC verifies — a damaged
    /// chunk emits nothing. The send is best-effort: a full or closed tap never
    /// stalls the scan.
    ///
    /// The two scan sites that find a trigger — a chunk-interior message
    /// ([`Self::absorb_parsed`]) and a top-level message ([`scan_available`]) —
    /// both arrive here; they differ only in how each obtains the body (a
    /// decoded chunk `Cow` vs. a direct file read).
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
        // `Record` is the mcap crate's enum, not ours: a dozen record kinds this
        // scan has no opinion about, and upstream may add more. A catch-all is
        // the right shape for a foreign enum — the lint exists to guard the
        // enums this workspace owns, where every arm is a decision.
        #[expect(
            clippy::wildcard_enum_match_arm,
            reason = "foreign enum whose variant set this crate does not control"
        )]
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
                self.absorb_time(header.log_time, header.publish_time);
                // A message on a trigger channel is lifted whole (its body is the
                // serialized Trigger payload); any other message contributes only
                // its timestamp, its body untouched. Inside a chunk sub-delta this
                // stages the trigger (TriggerSink::Staged) until the CRC clears.
                if let Some(encoding) = self.trigger_channels.get(&header.channel_id).cloned() {
                    self.emit_trigger(TriggerRecord {
                        message_encoding: encoding,
                        body: data.into_owned(),
                        log_time: header.log_time,
                        publish_time: header.publish_time,
                    });
                }
            }
            // Header, the message/chunk indexes, attachments, statistics and
            // metadata: nothing a scan for stamps and channels reads.
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
    /// cut.rs, which drops the whole chunk at extraction, so coverage never
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
        if let Some(sub_stamps) = sub.pending_time {
            self.pending_time = Some(match self.pending_time {
                Some(mut s) => {
                    s.merge(sub_stamps);
                    s
                }
                None => sub_stamps,
            });
        }
        if let Some(sub_skew) = sub.skew {
            self.skew = Some(match self.skew {
                Some(mut sk) => {
                    sk.observe(sub_skew.min);
                    sk.observe(sub_skew.max);
                    sk
                }
                None => sub_skew,
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
        if let Some(stamps) = self.pending_time.take() {
            open.time = Some(match open.time {
                Some(mut existing) => {
                    existing.merge(stamps);
                    existing
                }
                None => stamps,
            });
        }
        if open.len >= EXTENT_CAP_BYTES {
            self.closed.push(*open);
            self.open = None;
        }
    }
}

/// One incremental pass over `file`: consume every record completely on disk in
/// `[offset, file_len)` and return the registry/extent updates it collected.
/// Stops without error at the first record still being appended.
///
/// The pass does file IO and owns no state: `seed` carries the caller's snapshot
/// of the recording being resumed, and the caller folds the returned
/// [`ScanDelta`] back into its [`RecordingIndex`]
/// ([`RecordingIndex::apply_delta`]) and records the returned offset. Coverage,
/// if the caller keeps any, comes from the index's [`TimeBounds`] afterwards —
/// not from the pass, which counts messages whose extent a fault left unclosed.
///
/// The delta comes with a plain [`ScanProgress`] rather than a `Result`:
/// localized damage is skipped (a damaged chunk, an unparseable schema/channel,
/// a runt message — warned and consumed), and only **framing** faults stop the
/// pass. A framing fault — a record length past [`MAX_RECORD_LEN`], or an IO
/// error reading a record's header or body — leaves no resync point, so the pass
/// returns the delta it accumulated up to the faulted record and reports
/// `fault = Some(_)` with `offset` at that record.
///
/// **Resume invariant:** that partial delta is applied like any other, so a
/// caller retrying after a fault MUST resume at the returned `offset` (the
/// faulted record), never earlier. Re-scanning an already-applied region makes
/// the open extent's extension compute `record_end - open.offset` across bytes
/// the open extent already spans and underflow.
pub fn scan_available(
    file: &File,
    mut offset: u64,
    file_len: u64,
    seed: ScanSeed,
) -> (ScanDelta, ScanProgress) {
    // Seed the pass from the caller's snapshot: the extent left open at this
    // offset, and the tap context (both stay empty/idle when the tap is
    // disabled). The top-level delta carries the live sink, so triggers it lifts
    // send straight down the tap as the scan finds them.
    let (trigger_topic, trigger_sink) = match seed.tap {
        Some((topic, tx)) => (Some(topic), TriggerSink::Live(tx)),
        None => (None, TriggerSink::Off),
    };
    let mut delta = ScanDelta {
        open: seed.open,
        trigger_topic,
        trigger_sink,
        trigger_channels: seed.trigger_channels,
        ..ScanDelta::default()
    };
    let mut ended = false;
    // The offset of the faulted record is `offset` (left unadvanced) when
    // a fault breaks the loop; the partial delta is returned regardless.
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
            // u64::MAX is the placeholder a seek-back (chunked) writer leaves
            // in a Chunk header until it back-patches the real length at chunk
            // close — a recording written that way is unreadable mid-write, so
            // name the cause rather than implying corruption.
            fault = Some(if len == u64::MAX {
                anyhow::anyhow!(
                    "record at offset {offset} declares u64::MAX bytes — an \
                     unpatched length from a seek-back (chunked) writer? such \
                     a recording cannot be tailed until it is finalised"
                )
            } else {
                anyhow::anyhow!(
                    "record at offset {offset} declares {len} bytes; framing desynchronised?"
                )
            });
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
                // Decode the 22-byte fixed header in one read: channel_id
                // u16, sequence u32, log_time u64, publish_time u64 (all
                // LE). A message on a trigger channel also has its payload
                // (past those 22 fixed fields) lifted out; every other body
                // stays untouched until extraction.
                if len >= 22 {
                    let mut header = [0u8; 22];
                    if let Err(e) = file.read_exact_at(&mut header, offset + 9) {
                        fault = Some(anyhow::Error::new(e).context(format!(
                            "reading message header at {offset}; framing desynchronised?"
                        )));
                        break;
                    }
                    let channel_id = u16::from_le_bytes(header[0..2].try_into().unwrap());
                    let log_time = u64::from_le_bytes(header[6..14].try_into().unwrap());
                    let publish_time = u64::from_le_bytes(header[14..22].try_into().unwrap());
                    delta.absorb_time(log_time, publish_time);

                    if let Some(encoding) = delta.trigger_channels.get(&channel_id).cloned() {
                        // The payload follows the 22-byte fixed fields; an
                        // exactly-22-byte trigger message lifts an empty
                        // body.
                        let mut payload = vec![0u8; (len - 22) as usize];
                        if let Err(e) = file.read_exact_at(&mut payload, offset + 9 + 22) {
                            fault = Some(anyhow::Error::new(e).context(format!(
                                "reading trigger payload at {offset}; framing desynchronised?"
                            )));
                            break;
                        }
                        // A top-level record is durable the instant its
                        // framing is read, so this goes straight down the
                        // tap (no chunk CRC to clear).
                        delta.emit_trigger(TriggerRecord {
                            message_encoding: encoding,
                            body: payload,
                            log_time,
                            publish_time,
                        });
                    }
                } else {
                    // A conformant Message body is >= 22 bytes — its fixed
                    // header alone. A shorter one cannot yield both stamps,
                    // so it is skipped like other localized damage: the
                    // record is still consumed (the framing is
                    // self-consistent), but its time counts toward neither
                    // extent bounds nor coverage.
                    warn!(
                        "message record at {offset} is only {len} B; \
                         the 22-byte fixed header is incomplete, skipping it"
                    );
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
                    // dropped — cut.rs drops the same chunk at extraction.
                    // The record's framing is intact (its length prefix is
                    // self-consistent), so it is still consumed: the scan
                    // skips it and keeps indexing the records behind it.
                    warn!("absorbing chunk at {offset}: {e:#}; skipping it");
                }
            }
            op::DATA_END | op::FOOTER => {
                ended = true;
            }
            // Every other opcode: the header, the message and chunk indexes,
            // attachments, statistics, metadata and the summary offsets. None
            // carries a stamp or a channel, and all are framed like the rest, so
            // the walk consumes each by its length and moves on.
            _ => {}
        }
        if ended {
            break;
        }
        delta.extend_extent(offset, end);
        offset = end;
    }

    if let Some(skew) = delta.skew {
        debug!(
            "scan pass to offset {offset}: log-vs-publish skew spans [{} ns, {} ns]",
            skew.min, skew.max
        );
    }
    (
        delta,
        ScanProgress {
            offset,
            ended,
            fault,
        },
    )
}

fn read_body(file: &File, offset: u64, len: u64) -> Result<Vec<u8>> {
    let mut body = vec![0u8; len as usize];
    file.read_exact_at(&mut body, offset)
        .with_context(|| format!("reading {len} B record body at {offset}"))?;
    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::{
        channel_body, chunk_body, index_file, message_body, message_body_pub, plan_one, raw_record,
        scan_passes, scan_to_end, schema_body, seed_with, test_dir, write_raw, write_recording,
        write_recording_opts,
    };

    /// [`scan_to_end`] with the trigger tap armed on `topic`, so messages there
    /// lift as [`TriggerRecord`]s down `tx`.
    fn scan_to_end_tapped(
        index: &mut RecordingIndex,
        file: &File,
        topic: &str,
        tx: &Sender<TriggerRecord>,
    ) -> Result<ScanProgress> {
        scan_passes(index, file, Some((topic, tx)))
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

        let (mut index, file) = index_file(&growing)?;

        let p1 = scan_to_end(&mut index, &file)?;
        assert!(!p1.ended, "prefix must not look finished");
        assert!(
            p1.offset <= cut as u64,
            "scan must stop at or before the cut ({} > {cut})",
            p1.offset
        );

        // The file "grows" to its full content; the scan resumes where it
        // stopped and runs into DataEnd.
        std::fs::write(&growing, &full)?;
        let p2 = scan_to_end(&mut index, &file)?;
        assert!(p2.ended, "full file ends with DataEnd/Footer");

        assert_eq!(index.bounds.log.max, 400);

        let plan = plan_one(&index, 150, 350);
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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;

        assert!(progress.ended);
        assert_eq!(index.bounds.log.max, 30);
        let plan = plan_one(&index, 0, 100);
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

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        assert!(plan_one(&index, 300, 500).extents.is_empty());
        assert!(!plan_one(&index, 150, 500).extents.is_empty());

        // Inclusive boundaries, exactly at the extent's min/max (100, 200):
        // a window touching a bound by one nanosecond still plans the extent.
        assert!(!plan_one(&index, 200, 500).extents.is_empty());
        assert!(plan_one(&index, 201, 500).extents.is_empty());
        assert!(!plan_one(&index, 0, 100).extents.is_empty());
        assert!(plan_one(&index, 0, 99).extents.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A [`Stamps`] with equal `log` and `publish` spans — the shape every
    /// helper that stamps `publish_time = log_time` produces.
    fn same_stamps(min: u64, max: u64) -> Stamps {
        Stamps {
            log: Span { min, max },
            publish: Span { min, max },
        }
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

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        let plan = plan_one(&index, 0, u64::MAX);
        let ch = plan.channels.get(&1).expect("channel registered");
        let schema = ch.schema.as_ref().expect("same-pass schema resolves");
        assert_eq!(schema.name, "std_msgs/msg/String");

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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;

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
        assert_eq!(index.bounds.log.max, 200);
        assert!(
            !plan_one(&index, 50, 250).extents.is_empty(),
            "the prefix's extent stays plannable across the fault"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A declared length of exactly u64::MAX is the placeholder a seek-back
    /// (chunked) writer leaves in a Chunk header until it back-patches the real
    /// value, so the fault names that cause instead of implying corruption.
    #[test]
    fn unpatched_placeholder_length_fault_names_the_seek_back_writer() -> Result<()> {
        let root = test_dir("placeholder")?;
        let path = root.join("rec.mcap");
        let mut bytes = MAGIC.to_vec();
        bytes.push(op::CHUNK);
        bytes.extend_from_slice(&u64::MAX.to_le_bytes());
        std::fs::write(&path, bytes)?;

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;

        let fault = progress.fault.expect("the placeholder length must fault");
        let msg = format!("{fault:#}");
        assert!(
            msg.contains("u64::MAX") && msg.contains("seek-back"),
            "fault must name the unpatched seek-back placeholder: {msg}"
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
        // as cut.rs drops a damaged chunk during extraction.
        write_raw(
            &path,
            &[
                raw_record(op::CHUNK, &[0xFF; 16]),
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 42, b"x")),
            ],
        )?;

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "the bad chunk and the good records after it are all consumed"
        );
        assert_eq!(index.bounds.log.max, 42);

        let plan = plan_one(&index, 0, 100);
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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "the chunk and the records after it are all consumed"
        );
        assert_eq!(
            index.bounds.log.max, 700,
            "the dropped chunk's message (500) never reaches coverage"
        );

        let plan = plan_one(&index, 0, u64::MAX);
        assert!(
            plan.channels.contains_key(&2),
            "the post-chunk channel registers"
        );
        assert!(
            !plan.channels.contains_key(&1),
            "the failed chunk's channel must not register"
        );
        // No extent may claim the dropped message's log_time (500); only the
        // good post-chunk message (700) is in the bounds.
        for e in &plan.extents {
            if let Some(s) = e.time {
                assert!(
                    !(s.log.min <= 500 && 500 <= s.log.max),
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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(index.bounds.log.max, 300);
        assert!(
            plan_one(&index, 0, u64::MAX).channels.contains_key(&9),
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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(index.bounds.log.max, 55);

        let plan = plan_one(&index, 0, 100);
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

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "both records consumed"
        );
        assert_eq!(index.bounds.log.max, 42);

        let plan = plan_one(&index, 0, 100);
        assert_eq!(plan.extents.len(), 1);
        assert_eq!(
            plan.extents[0].time,
            Some(same_stamps(42, 42)),
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

        let (mut index, file) = index_file(&path)?;
        let start = MAGIC.len() as u64;

        // Only the magic on disk: nothing to scan, nothing to fault on.
        let (delta, p) = scan_available(&file, start, start, seed_with(&index, None));
        assert_eq!(p.offset, start);
        assert!(!p.ended && p.fault.is_none());
        index.apply_delta(delta);
        assert!(
            !index.bounds.has_messages,
            "a pass over bare magic folds no message, so the index still has no \
             time bounds and a caller reading coverage off it claims nothing"
        );

        // A record header still being appended: same outcome.
        let (delta, p) = scan_available(
            &file,
            start,
            file.metadata()?.len(),
            seed_with(&index, None),
        );
        assert_eq!(p.offset, start);
        assert!(!p.ended && p.fault.is_none());
        index.apply_delta(delta);
        assert!(!index.bounds.has_messages);
        assert!(index.extents.is_empty() && index.open.is_none());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn out_of_order_stamps_keep_high_water_and_widen_extent_bounds() -> Result<()> {
        let root = test_dir("ooo")?;
        let path = root.join("rec.mcap");
        write_recording(&path, false, &[("/a", 100), ("/a", 50)])?;

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        assert_eq!(
            index.bounds.log.max, 100,
            "high water never moves backwards"
        );
        let plan = plan_one(&index, 40, 60);
        assert_eq!(plan.extents.len(), 1, "the late stamp widens the bounds");
        assert_eq!(plan.extents[0].time, Some(same_stamps(50, 100)));

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

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        let plan = plan_one(&index, 0, u64::MAX);
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

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        let plan = plan_one(&index, 0, 100);
        let ch = plan.channels.get(&1).expect("channel registered");
        assert_eq!(ch.topic, "/raw");
        assert!(ch.schema.is_none());

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
        let (mut index, file) = index_file(&path)?;
        scan_to_end_tapped(&mut index, &file, "/trig", &tx)?;

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
        let (mut index, file) = index_file(&path)?;
        scan_to_end_tapped(&mut index, &file, "/trig", &tx)?;

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
        let (mut index, file) = index_file(&path)?;
        scan_to_end_tapped(&mut index, &file, "/trig", &tx)?;

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

    /// A trigger-topic message shorter than the 22-byte fixed header cannot
    /// yield both stamps, so it is skipped like other localized damage: no
    /// `TriggerRecord` is lifted and its time reaches neither coverage nor the
    /// extent bounds (a warning, the framing intact).
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
        let (mut index, file) = index_file(&path)?;
        scan_to_end_tapped(&mut index, &file, "/trig", &tx)?;

        assert!(
            rx.try_iter().next().is_none(),
            "a runt trigger message lifts no TriggerRecord"
        );
        assert!(
            !index.bounds.has_messages,
            "a header too short for both stamps is skipped, advancing nothing"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The extent index and the recording's [`TimeBounds`] carry the min/max of
    /// both `log_time` and `publish_time`, independently. `log_time` ascends
    /// while `publish_time` is out of order, so the two spans differ and neither
    /// contaminates the other. Coverage stays `log_time` only.
    #[test]
    fn extent_and_bounds_carry_both_stamps() -> Result<()> {
        let root = test_dir("both-stamps")?;
        let path = root.join("rec.mcap");
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 0, 100, 1_000, b"a")),
                raw_record(op::MESSAGE, &message_body_pub(1, 1, 200, 900, b"b")),
                raw_record(op::MESSAGE, &message_body_pub(1, 2, 300, 1_100, b"c")),
            ],
        )?;

        let (mut index, file) = index_file(&path)?;
        scan_to_end(&mut index, &file)?;

        // Coverage is the max log_time, never a publish_time.
        assert_eq!(index.bounds.log.max, 300);

        let plan = plan_one(&index, 0, u64::MAX);
        assert_eq!(plan.extents.len(), 1, "the three messages fit one extent");
        assert_eq!(
            plan.extents[0].time,
            Some(Stamps {
                log: Span { min: 100, max: 300 },
                publish: Span {
                    min: 900,
                    max: 1_100,
                },
            }),
            "the extent carries min/max of both stamps independently"
        );

        // The recording's TimeBounds carry both spans too.
        assert_eq!(index.bounds.log, Span { min: 100, max: 300 });
        assert_eq!(
            index.bounds.publish,
            Span {
                min: 900,
                max: 1_100
            }
        );
        assert!(index.bounds.has_messages);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A `Message` whose body holds `log_time` but stops short of the full
    /// 22-byte fixed header (`len` in `[14, 21]`) cannot yield both stamps, so
    /// it is skipped like other localized damage: consumed via its intact
    /// framing, contributing to neither coverage nor the extent bounds. The
    /// good message behind the two runts still indexes.
    #[test]
    fn a_message_missing_publish_time_is_skipped_cleanly() -> Result<()> {
        let root = test_dir("short-header")?;
        let path = root.join("rec.mcap");
        // 21 bytes: the full log fields plus a publish_time one byte short.
        let mut short21 = Vec::new();
        short21.extend_from_slice(&1u16.to_le_bytes()); // channel_id
        short21.extend_from_slice(&0u32.to_le_bytes()); // sequence
        short21.extend_from_slice(&400u64.to_le_bytes()); // log_time
        short21.extend_from_slice(&[0u8; 7]); // publish_time, one byte short
        // 14 bytes: log_time present, publish_time entirely absent.
        let mut short14 = Vec::new();
        short14.extend_from_slice(&1u16.to_le_bytes()); // channel_id
        short14.extend_from_slice(&0u32.to_le_bytes()); // sequence
        short14.extend_from_slice(&500u64.to_le_bytes()); // log_time only
        write_raw(
            &path,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &short21),
                raw_record(op::MESSAGE, &short14),
                raw_record(op::MESSAGE, &message_body_pub(1, 3, 700, 650, b"good")),
            ],
        )?;

        let (mut index, file) = index_file(&path)?;
        let progress = scan_to_end(&mut index, &file)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "both runts and the good message are all consumed"
        );
        assert_eq!(
            index.bounds.log.max, 700,
            "neither short-header record advances coverage"
        );

        let plan = plan_one(&index, 0, u64::MAX);
        assert_eq!(plan.extents.len(), 1);
        assert_eq!(
            plan.extents[0].time,
            Some(Stamps {
                log: Span { min: 700, max: 700 },
                publish: Span { min: 650, max: 650 },
            }),
            "the short-header records fold no time into the bounds"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
