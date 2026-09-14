//! Window extraction from a recording, whether or not anyone is still writing it.
//!
//! Given a [`WindowPlan`] — the open recording, the byte extents overlapping the
//! window, and the channel registry — this assembles one output MCAP holding
//! every message whose stamp on the window's [`TimeSource`] falls in
//! `[start_ns, end_ns]` **and whose topic the [`ChannelSelection`] keeps**. It
//! is a **direct copy** of message payload bytes: registry schemas/channels are
//! registered in the output writer by content, then each message is emitted with
//! its raw serialized body. Message bodies are never decoded — the only thing
//! inspected is the one stamp the window lives on.
//!
//! The two conditions are asked at the two places they can be: the window on
//! every message (`ClipWriter::copy_message`), and the selection once per
//! channel (`ClipWriter::route`) — the same step that registers a channel in
//! the output, so an excluded topic contributes to a clip neither a channel, nor
//! a schema, nor a message, nor a per-channel tally.
//!
//! Each extent is read with `read_at`, so a copy shares no seek state with
//! whatever else holds the file open, and its records are walked with our own
//! opcode + length framing — the same walk the scan performed to build the
//! extent, so the boundaries are known
//! to tile. That ownership of the framing is what makes extraction
//! **damage-tolerant**, the way the MCAP format is designed to be (length
//! prefixes delimit every record; chunk CRCs exist to detect and discard a
//! damaged chunk): a record whose *body* fails to parse is skipped with an
//! error log, and a chunk that fails decompression, CRC, or interior parsing
//! is dropped whole — its messages are buffered and written only when the
//! chunk completes cleanly, because a bad CRC cannot say which bytes are
//! lying. Localized corruption costs the affected record or chunk, counted in
//! [`ClipStats`], never the clip. Only framing inconsistencies (the extent no
//! longer matches the tail's scan), recording IO errors, and output errors
//! abort the file. `Writer::finish` writes the summary section, footer and
//! closing magic, so every file of a clip is a complete, standalone MCAP.
//!
//! **This module never decides where a file goes.** It writes the path it is
//! given, inside a clip directory the caller has already claimed
//! ([`crate::layout`]), and cannot name the file it wrote — a file's number is
//! its position among the recordings that *contributed*, settled only once every
//! copy has run. [`crate::segment::cut_window`] names them afterwards.

use std::collections::{BTreeMap, HashMap};
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use log::{error, info, warn};
use mcap::records::Record;

use crate::TimeSource;
use crate::index::{ChannelDef, MAX_RECORD_LEN, WindowPlan, op};
use crate::manifest::{ChannelTally, CutRequest};
use crate::select::ChannelSelection;

/// A recording whose bytes no longer frame the way the tail's scan read them.
///
/// The copy walks each planned extent's framing afresh, so it is where bytes
/// that changed *after* they were indexed surface. A framing walk has no resync
/// point, so there is no boundary to pick the records behind the fault up at,
/// and the clip is refused rather than assembled out of bytes whose meaning is
/// unknown.
///
/// It is a type rather than a message because a caller cutting repeatedly from
/// one recording has to tell **this** fault from a full disk, an IO error or an
/// output failure: they are different things with different remedies, and only
/// this one is permanent for the recording it names. That is the distinction the
/// recorder's per-recording tally is built on (`tail::CutFaults`); cutting once
/// from a finished recording (`clipper clip`) only ever prints it.
///
/// Both arms carry the `recording` the bytes belong to and the `extent_offset`
/// the walk entered at — the fault's identity and its blast radius, since every
/// window whose plan includes that extent is refused with it.
#[derive(Debug, thiserror::Error)]
pub enum FramingDesync {
    /// A length prefix no valid record can carry (past
    /// [`MAX_RECORD_LEN`]), or one whose record
    /// would run past the extent the scan closed around it.
    #[error(
        "record at extent offset {offset} declares {declared} B; \
         extent framing inconsistent with the tail's scan"
    )]
    RecordLength {
        recording: PathBuf,
        extent_offset: u64,
        /// Where in the extent the broken length prefix sits; the absolute file
        /// offset is `extent_offset + offset`.
        offset: usize,
        declared: u64,
    },
    /// The extent's records do not tile it: the walk ran out of bytes mid-record
    /// where the scan had found a boundary.
    #[error("extent ends mid-record at offset {offset}; framing inconsistent with the tail's scan")]
    ShortExtent {
        recording: PathBuf,
        extent_offset: u64,
        offset: usize,
    },
}

impl FramingDesync {
    /// The recording whose bytes changed — the identity a per-recording tally of
    /// these refusals is kept under, and the file an operator repairs or rolls
    /// over.
    #[must_use]
    pub fn recording(&self) -> &Path {
        match self {
            Self::RecordLength { recording, .. } | Self::ShortExtent { recording, .. } => recording,
        }
    }

    /// The offset of the extent the walk entered at. The refusal covers that
    /// whole extent — up to `EXTENT_CAP_BYTES` (4 MiB) of recording — not just
    /// the broken record, because the walk has no resync point behind it.
    #[must_use]
    pub fn extent_offset(&self) -> u64 {
        match self {
            Self::RecordLength { extent_offset, .. } | Self::ShortExtent { extent_offset, .. } => {
                *extent_offset
            }
        }
    }
}

/// The stamp a message's window membership is tested on, per the window's
/// [`TimeSource`]: its `log_time` or its `publish_time`.
fn message_stamp(header: &mcap::records::MessageHeader, source: TimeSource) -> u64 {
    match source {
        TimeSource::Log => header.log_time,
        TimeSource::Publish => header.publish_time,
    }
}

/// What one copy produced: the file it wrote, the recording it read, and the
/// counters.
///
/// It is also the whole of what a clip's [`ClipMetadata`] says per file
/// ([`crate::manifest::SourceMeta`]), which is why the per-channel tallies end
/// up here rather than being written into the file as they are accumulated: a
/// clip states its account once, in the document beside its files, so the copy
/// hands its findings back rather than emitting them.
///
/// [`ClipMetadata`]: crate::manifest::ClipMetadata
#[derive(Debug, Default)]
pub struct ClipStats {
    /// The file this copy wrote, under its final `<id>_N.mcap` name.
    pub out_path: PathBuf,
    /// The recording it was copied from; `None` for the one file a window no
    /// recording covered still produces.
    pub source: Option<PathBuf>,
    pub extents_read: usize,
    /// The bytes those extents spanned — what the copy read off the recording,
    /// as against `bytes_copied`, what it wrote into the clip.
    pub bytes_read: u64,
    pub messages_copied: u64,
    pub bytes_copied: u64,
    /// Records skipped over localized damage: an unparseable body, or a
    /// message on a channel with no Channel record.
    pub records_skipped: u64,
    /// Chunks dropped whole: decompression, CRC, or interior parse failure.
    pub chunks_dropped: u64,
    /// Per **output** channel id of this file, what that channel contributed.
    /// Filled by the same step that writes a message through, so a channel
    /// nothing was copied from has no entry rather than a row of zeroes.
    pub channels: BTreeMap<u16, ChannelTally>,
}

/// One finished MCAP file of a clip, written and fsynced under a staging name
/// inside the clip's own directory, waiting for the number it will be named by.
///
/// A file's number is its position among the files that *contributed*, and the
/// empty ones are dropped only once every copy has run — so the name cannot be
/// settled where the copy is. [`stage_clip`] produces one of these and
/// [`crate::segment::cut_window`] renames it into place.
///
/// There is no cleanup on it. A file staged into a clip directory that is never
/// completed dies with that directory: a cut that fails removes the whole thing
/// ([`crate::layout::ClipDir::discard`]), which is the one rule that also covers
/// a partially written clip, a panicking copy and a crash.
#[must_use = "a staged file must be named into its clip or the clip is incomplete"]
#[derive(Debug)]
pub(crate) struct StagedClip {
    /// The complete, fsynced file, under its staging name.
    path: PathBuf,
    /// The copy counters, carried through to the placed [`ClipStats`].
    stats: ClipStats,
}

impl StagedClip {
    /// Whether this file copied no in-window messages — a rollover whose new
    /// recording held nothing inside the window stages such an empty trailing
    /// file, which the caller drops when other files carry data.
    #[must_use]
    pub(crate) fn is_empty(&self) -> bool {
        self.stats.messages_copied == 0
    }

    /// Rename the file to the `n`th of its clip and report what it holds.
    ///
    /// Consuming the value is what keeps the two halves together: there is no
    /// way to learn a file's final path without having given it one.
    pub(crate) fn place(mut self, dir: &crate::layout::ClipDir, n: usize) -> Result<ClipStats> {
        self.stats.out_path = dir.place(&self.path, n)?;
        Ok(self.stats)
    }
}

/// Copy every message in `[start_ns, end_ns]` (inclusive bounds) from the
/// planned extents into an MCAP at `out_path`, and report what came out.
/// Localized damage in the recording — an unparseable record body, a message on
/// an unregistered channel, a chunk failing CRC or decompression — is skipped
/// with an error log and counted in [`ClipStats`]; the file keeps everything
/// else.
///
/// `compression` is the codec the clip's `mcap::Writer` is built with (`None`
/// for uncompressed); it is set explicitly rather than inherited from the mcap
/// crate default. `selection` is which of the recording's topics the clip is
/// cut from; both are properties of the output rather than of the window.
///
/// A real cut stages under a name it is renamed out of once the clip's file
/// count is known ([`stage_clip`] and [`crate::segment::cut_window`]); this
/// one-call form names the file outright and serves the copy's own tests.
#[cfg(test)]
pub fn extract_clip(
    plan: &WindowPlan,
    out_path: &Path,
    request: &CutRequest,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<ClipStats> {
    let mut staged = stage_clip(plan, out_path, request, compression, selection)?;
    staged.stats.out_path = staged.path.clone();
    Ok(staged.stats)
}

/// Log what each file of one finished clip holds.
///
/// A window straddling a rollover holds one file per contributing recording, so
/// this runs over however many the cut produced. Damage the cut worked around is
/// warned about per file rather than summed: a file is missing data or it is
/// not, and an operator reading one line should see what *that* file is missing.
#[expect(
    clippy::cast_precision_loss,
    reason = "a log line's MiB figure; the loss starts past 8 PiB in one clip"
)]
pub fn report_clips(clips: &[ClipStats]) {
    for stats in clips {
        info!(
            "clip {} written: {} msgs from {} extents, {:.1} MiB",
            stats.out_path.display(),
            stats.messages_copied,
            stats.extents_read,
            stats.bytes_copied as f64 / 1_048_576.0,
        );
        if stats.records_skipped > 0 || stats.chunks_dropped > 0 {
            warn!(
                "clip {} is missing data over damage in the recording: \
                 {} records skipped, {} chunks dropped",
                stats.out_path.display(),
                stats.records_skipped,
                stats.chunks_dropped,
            );
        }
    }
}

/// Copy one source recording's share of the window into a fresh file at `path`,
/// fsync it, and hand it back for naming.
///
/// `path` is inside the clip's own directory, which the caller claimed with one
/// atomic `mkdir` and nothing else may write to — so the file is created with
/// `create_new` and a name that is already taken is a bug rather than a race to
/// resolve. A copy that fails removes its own partial file, so a `StagedClip`
/// that exists names a complete MCAP; the clip directory is removed whole on any
/// failure regardless ([`crate::layout::ClipDir::discard`]).
pub(crate) fn stage_clip(
    plan: &WindowPlan,
    path: &Path,
    request: &CutRequest,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<StagedClip> {
    let file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .with_context(|| format!("creating {}", path.display()))?;
    let stats = copy_window(plan, file, request, compression, selection).inspect_err(|_| {
        // A failed copy must not leave a half-written, footer-less file behind
        // the name of a finished one; the error itself is what the caller
        // reports.
        if let Err(e) = std::fs::remove_file(path) {
            warn!("removing partial clip file {}: {e}", path.display());
        }
    })?;
    Ok(StagedClip {
        path: path.to_path_buf(),
        stats,
    })
}

/// Assemble the clip file into the freshly created `out_file`: register window
/// channels from the registry on first use, stream the planned extents, write
/// the clip's id record, finish and fsync the file. The caller removes the
/// partial file if this fails.
///
/// The writer is built from [`mcap::WriteOptions`] carrying one deliberate
/// setting: the `compression` codec (`None` = uncompressed) the caller chose.
/// Chunk layout is the mcap crate's, inherited — this crate holds no opinion
/// about the size a clip's chunks are cut at, so it moves with the crate.
/// `selection` decides which of the registry's topics are registered at all.
///
/// The record goes in between the last copied message and `finish`, which is
/// what puts it in the summary's metadata index and the statistics' metadata
/// count, so a reader addresses it rather than walking the file.
fn copy_window(
    plan: &WindowPlan,
    out_file: File,
    request: &CutRequest,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<ClipStats> {
    let mut clip = ClipWriter {
        writer: mcap::WriteOptions::new()
            .compression(compression)
            .create(BufWriter::new(out_file))
            .context("opening mcap writer")?,
        channels: &plan.channels,
        selection,
        routes: HashMap::new(),
        request,
        stats: ClipStats {
            source: plan.source.as_ref().map(|s| s.path.clone()),
            ..ClipStats::default()
        },
    };

    if let Some(source) = &plan.source {
        for extent in &plan.extents {
            clip.stats.extents_read += 1;
            clip.stats.bytes_read += extent.len;
            #[expect(
                clippy::cast_possible_truncation,
                reason = "an extent closes at EXTENT_CAP_BYTES (4 MiB), so its \
                          length is inside `usize` on every target this builds for"
            )]
            let mut buf = vec![0u8; extent.len as usize];
            source
                .file
                .read_exact_at(&mut buf, extent.offset)
                .with_context(|| {
                    format!("reading extent at {} (+{} B)", extent.offset, extent.len)
                })?;
            clip.copy_extent(&buf, &source.path, extent.offset)?;
        }
    }
    clip.write_id_record()?;

    let ClipWriter {
        mut writer, stats, ..
    } = clip;
    writer.finish().context("finalising output mcap")?;
    // `finish` can leave bytes in the BufWriter; flush them and fsync the file,
    // so that by the time the clip's document is written every file it names is
    // already durable.
    writer
        .into_inner()
        .into_inner()
        .map_err(|e| anyhow::anyhow!("flushing clip: {e}"))?
        .sync_all()
        .context("syncing clip to disk")?;
    Ok(stats)
}

/// One clip being assembled: the output writer, the recording's channel
/// registry, the window being cut, and the running counters — the state every
/// copied message touches.
///
/// The window arrives as the whole [`CutRequest`], not as loose bounds, so the
/// membership test and the manifest's `window.*` keys read one value: a clip
/// cannot claim a window its messages were not tested against.
struct ClipWriter<'a> {
    writer: mcap::Writer<BufWriter<File>>,
    channels: &'a HashMap<u16, ChannelDef>,
    /// Which topics this clip is cut from. Read once per recording channel id,
    /// in [`ClipWriter::route`] — which is also where a kept channel is
    /// registered, so an excluded topic reaches neither the output's channels
    /// nor its schemas.
    selection: &'a ChannelSelection,
    /// Recording channel ID → what this clip does with its messages, decided on
    /// first use so each verdict is reached, and logged, once rather than per
    /// message.
    routes: HashMap<u16, Route>,
    /// The window this clip is being cut for: its bounds, its clock domain, and
    /// the trigger and producer the clip's document names.
    request: &'a CutRequest,
    /// What this copy read and wrote, per-channel tallies included — the file's
    /// whole entry in the clip's document.
    stats: ClipStats,
}

impl ClipWriter<'_> {
    /// Walk one extent's records and write the in-window messages through.
    /// Only messages are copied out of the extent bytes; the clip's
    /// Schema/Channel records come from the registry ([`Self::route`]),
    /// not from here — the recording writes them where a topic first appears,
    /// which is usually far before the window and outside every planned extent.
    ///
    /// The framing walk is our own (opcode + u64le length, the walk the tail
    /// already performed to build this extent) rather than an mcap reader's:
    /// owning the boundaries lets a record whose *body* fails to parse be
    /// skipped — resyncing at the next boundary exactly, not heuristically —
    /// where the library readers halt on the first error. Framing that no
    /// longer matches the tail's scan (an oversized length, a record running
    /// past or short of the extent) means the bytes changed since the scan,
    /// and that aborts the clip as a typed [`FramingDesync`] naming `recording`
    /// and `extent_offset` — the identity a caller cutting repeatedly from the
    /// same recording counts these refusals under.
    #[expect(
        clippy::indexing_slicing,
        clippy::unwrap_used,
        clippy::cast_possible_truncation,
        reason = "the framing walk is the design here: the loop guard proves the \
                  9-byte header is present, the refusal below proves the body is, \
                  and `MAX_RECORD_LEN` (2^31) keeps every length inside `usize`. \
                  Replacing the indexing with `get()` would add error plumbing no \
                  input can reach"
    )]
    fn copy_extent(&mut self, buf: &[u8], recording: &Path, extent_offset: u64) -> Result<()> {
        let mut offset = 0usize;
        while offset + 9 <= buf.len() {
            let opcode = buf[offset];
            let len = u64::from_le_bytes(buf[offset + 1..offset + 9].try_into().unwrap());
            if len > MAX_RECORD_LEN || offset + 9 + len as usize > buf.len() {
                return Err(FramingDesync::RecordLength {
                    recording: recording.to_path_buf(),
                    extent_offset,
                    offset,
                    declared: len,
                }
                .into());
            }
            let end = offset + 9 + len as usize;
            let body = &buf[offset + 9..end];
            match opcode {
                op::MESSAGE => match mcap::parse_record(opcode, body) {
                    Ok(Record::Message { header, data }) => self.copy_message(&header, &data)?,
                    Ok(_) => unreachable!("a MESSAGE opcode parses to Record::Message"),
                    Err(e) => {
                        error!("skipping unparseable message at extent offset {offset}: {e}");
                        self.stats.records_skipped += 1;
                    }
                },
                op::CHUNK => self.copy_chunk(body, offset)?,
                _ => {} // schemas/channels (registry covers them), indexes, …
            }
            offset = end;
        }
        if offset != buf.len() {
            return Err(FramingDesync::ShortExtent {
                recording: recording.to_path_buf(),
                extent_offset,
                offset,
            }
            .into());
        }
        Ok(())
    }

    /// Copy a chunk's in-window messages — all of them or none. The messages
    /// are buffered while the chunk iterates and written only once it
    /// completes cleanly: the chunk CRC is verified at the end of iteration,
    /// and a failure anywhere (decompression, CRC, an interior record) cannot
    /// say which of the chunk's bytes are damaged, so the whole chunk is
    /// dropped with an error log and counted. Output-side errors stay fatal.
    fn copy_chunk(&mut self, body: &[u8], at: usize) -> Result<()> {
        let mut pending: Vec<(mcap::records::MessageHeader, Vec<u8>)> = Vec::new();
        match Self::salvage_chunk(body, self.request, &mut pending) {
            Ok(()) => {
                for (header, data) in pending {
                    self.copy_message(&header, &data)?;
                }
            }
            Err(e) => {
                error!("dropping chunk at extent offset {at}: {e}");
                self.stats.chunks_dropped += 1;
            }
        }
        Ok(())
    }

    /// Buffer every message in `body`'s chunk that falls inside `request`'s
    /// window, on `request`'s own time source.
    ///
    /// An `Err` means the chunk could not be read whole — decompression, the
    /// chunk CRC verified at the end of iteration, or an interior record — and
    /// `pending` then holds whatever was collected before the failure. Making
    /// that all-or-nothing is [`Self::copy_chunk`]'s: it drops the buffer
    /// instead of copying it.
    fn salvage_chunk(
        body: &[u8],
        request: &CutRequest,
        pending: &mut Vec<(mcap::records::MessageHeader, Vec<u8>)>,
    ) -> mcap::McapResult<()> {
        let (start_ns, end_ns, time_source) =
            (request.start_ns(), request.end_ns(), request.time_source());
        let Record::Chunk { header, data } = mcap::parse_record(op::CHUNK, body)? else {
            unreachable!("a CHUNK opcode parses to Record::Chunk");
        };
        for rec in mcap::read::ChunkReader::new(header, &data)? {
            // Only messages are copied out of a chunk; the schema and channel
            // records inside it were registered from the recording's own index
            // before the copy began.
            let Record::Message { header, data } = rec? else {
                continue;
            };
            let stamp = message_stamp(&header, time_source);
            if stamp >= start_ns && stamp <= end_ns {
                pending.push((header, data.into_owned()));
            }
        }
        Ok(())
    }

    /// Write one message through if its windowing stamp is in the window and
    /// the clip is cut from its topic. The stamp is the message's `log_time` or
    /// `publish_time`, per the window's [`TimeSource`].
    ///
    /// The two ways a message does not reach the clip are not the same thing. A
    /// message the [`ChannelSelection`] excludes is *not in this clip's scope*:
    /// nothing is counted, because nothing went wrong. A message on a channel
    /// the recording never declared is damage — there is no Schema/Channel to
    /// emit for it — so it is skipped and counted with the rest of the damage.
    fn copy_message(&mut self, header: &mcap::records::MessageHeader, data: &[u8]) -> Result<()> {
        let stamp = message_stamp(header, self.request.time_source());
        if stamp < self.request.start_ns() || stamp > self.request.end_ns() {
            return Ok(());
        }
        let channel_id = match self.route(header.channel_id)? {
            Route::Copy(id) => id,
            Route::Excluded => return Ok(()),
            Route::Unregistered => {
                self.stats.records_skipped += 1;
                return Ok(());
            }
        };
        self.writer
            .write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    ..*header
                },
                data,
            )
            .context("writing message")?;
        self.count(channel_id, stamp, data.len() as u64);
        Ok(())
    }

    /// Fold one written message into the clip's counters — the whole-clip totals
    /// and the manifest's per-channel tally, keyed by the id the clip's own
    /// `Channel` record carries.
    ///
    /// One call site, right after the write, so a message is counted exactly
    /// when it lands in the clip: whatever decides *which* messages are copied
    /// (the window test above, a channel selection beside it) cannot leave the
    /// accounting behind, because a message that is never written is never
    /// counted here.
    fn count(&mut self, channel_id: u16, stamp: u64, bytes: u64) {
        self.stats.messages_copied += 1;
        self.stats.bytes_copied += bytes;
        self.stats
            .channels
            .entry(channel_id)
            .and_modify(|tally| tally.absorb(stamp))
            .or_insert_with(|| ChannelTally::opened(stamp));
    }

    /// Write the one record an MCAP file of a clip carries: the clip's id
    /// ([`crate::manifest::id_record`]), so a file separated from its directory
    /// can still be grouped. What the clip holds, where it came from and what
    /// asked for it are stated once in the document beside the files.
    fn write_id_record(&mut self) -> Result<()> {
        self.writer
            .write_metadata(&crate::manifest::id_record(self.request))
            .context("writing the clip id record")
    }

    /// Decide what this clip does with a recording channel ID, and cache the
    /// verdict.
    ///
    /// This is the one place a channel enters the output, so it is also the one
    /// place the [`ChannelSelection`] can keep one out: a topic the selection
    /// refuses is never registered, so the clip carries no `Channel` record for
    /// it, no `Schema` record that only it referenced, and — since registration
    /// is what a copy needs — no message and no manifest tally either. The
    /// writer deduplicates schemas/channels by content, so a registered mapping
    /// stays stable however often the definition is offered.
    ///
    /// Errors are output-side failures; a recording with no `Channel` record for
    /// the ID (a spec-violating file or a registry gap) is [`Route::Unregistered`],
    /// logged once per ID.
    fn route(&mut self, src_id: u16) -> Result<Route> {
        if let Some(cached) = self.routes.get(&src_id) {
            return Ok(*cached);
        }
        let Some(def) = self.channels.get(&src_id) else {
            error!(
                "messages on channel {src_id} have no Channel record in the recording; skipping them"
            );
            self.routes.insert(src_id, Route::Unregistered);
            return Ok(Route::Unregistered);
        };
        if !self.selection.selects(&def.topic) {
            // Every clip drops the announcement topic, so saying so on every
            // clip is noise; a selection that was configured to narrow is worth
            // a line naming what it dropped.
            if self.selection.is_narrowing() {
                info!(
                    "{}: excluded from the clip by the topic selection",
                    def.topic
                );
            }
            self.routes.insert(src_id, Route::Excluded);
            return Ok(Route::Excluded);
        }
        let schema_id = match &def.schema {
            Some(schema) => self
                .writer
                .add_schema(&schema.name, &schema.encoding, &schema.data)
                .with_context(|| format!("adding schema {}", schema.name))?,
            None => 0,
        };
        let channel_id = self
            .writer
            .add_channel(schema_id, &def.topic, &def.message_encoding, &def.metadata)
            .with_context(|| format!("adding channel {}", def.topic))?;
        self.routes.insert(src_id, Route::Copy(channel_id));
        Ok(Route::Copy(channel_id))
    }
}

/// What a clip does with the messages on one of the recording's channels,
/// decided once per channel ID by [`ClipWriter::route`].
///
/// The two ways out are deliberately distinct variants rather than one absence:
/// [`Self::Excluded`] is this clip's scope and costs nothing, while
/// [`Self::Unregistered`] is damage in the recording and is counted as such.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Route {
    /// Copy its messages, under this output channel ID.
    Copy(u16),
    /// The topic selection does not include its topic: no channel, no schema,
    /// no message, no manifest key.
    Excluded,
    /// The recording declares no `Channel` record for the ID, so there is
    /// nothing to register and nothing to write its messages under.
    Unregistered,
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::assert_is_empty,
        clippy::items_after_statements,
        clippy::cast_possible_truncation,
        clippy::cognitive_complexity,
        reason = "a failed unwrap or a panicking index is a failing test, and \
                  `assert!(x.is_empty())` names the claim better than the \
                  empty-array `assert_eq!` the lint asks for, \
                  and a test that builds a fixture, drives it and asserts on the \
                  whole result is long, nested and argument-heavy by \
                  construction — splitting one would scatter the case it states"
    )]

    use std::collections::BTreeMap;
    use std::sync::Arc;

    use super::*;
    use crate::index::{Extent, PlanSource, RecordingIndex, Span, Stamps, WindowPlan, op};
    use crate::manifest::{CLIP_ID_KEY, MANIFEST_NAME, read_manifest};
    use crate::select::Spec;
    use crate::testing::{
        channel_body, index_file, message_body, message_body_pub, metadata_body, raw_record,
        read_clip, scan_to_end, schema_body, test_dir, window_request, write_raw, write_recording,
        write_recording_opts,
    };
    use crate::trigger::{ANNOUNCE_TOPIC, TRIGGER_TOPIC};

    /// [`window_request`] on the `log` domain — the domain almost every clip
    /// test windows on, as [`plan_one`] is for the plan that feeds it.
    fn log_window(start_ns: u64, end_ns: u64) -> CutRequest {
        window_request(start_ns, end_ns, TimeSource::Log)
    }

    /// Plan the single source recording a clip test cuts from on the `log`
    /// domain — the domain almost every clip test windows on; [`plan_one_src`]
    /// takes an explicit source. These tests each index one recording, so
    /// [`RecordingIndex::plan`] offers at most one plan; no overlapping extent
    /// (no recording data in the window) becomes an empty plan.
    fn plan_one(index: &RecordingIndex, start_ns: u64, end_ns: u64) -> WindowPlan {
        plan_one_src(index, start_ns, end_ns, TimeSource::Log)
    }

    /// [`plan_one`] on an explicit windowing `source`.
    fn plan_one_src(
        index: &RecordingIndex,
        start_ns: u64,
        end_ns: u64,
        source: TimeSource,
    ) -> WindowPlan {
        index
            .plan(start_ns, end_ns, source)
            .unwrap_or_else(WindowPlan::empty)
    }

    /// The clip compression the recorder's default (zstd) maps to; most tests
    /// cut clips through the same codec the recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// The selection a clip test cuts with unless it is about selection: every
    /// topic, which is what a run with no configuration file cuts.
    fn every_topic() -> ChannelSelection {
        ChannelSelection::default()
    }

    /// Index one finished recording whole: open it, scan every record, and hand
    /// back the index the window plans are cut from.
    fn index_whole(path: &Path) -> Result<RecordingIndex> {
        let (mut index, file) = index_file(path)?;
        scan_to_end(&mut index, &file)?;
        Ok(index)
    }

    /// A copy always creates its file, and never opens one that is there.
    ///
    /// It writes into a directory one atomic `mkdir` just claimed for this clip
    /// and nothing else may write to, so a name already taken is a bug in the
    /// caller rather than a race to resolve — and failing loudly is what keeps
    /// two writers from interleaving bytes into one file. The clip a successful
    /// copy leaves behind is complete and valid the moment it returns.
    #[test]
    fn a_copy_creates_its_file_and_refuses_one_already_there() -> Result<()> {
        let root = test_dir("clip-create-new")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20), ("/t", 30)])?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);

        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.out_path, out);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/t".to_string(), 10),
                ("/t".to_string(), 20),
                ("/t".to_string(), 30),
            ]
        );
        assert_eq!(
            stats.source.as_deref(),
            Some(rec.as_path()),
            "the copy reports the recording it read, for the clip's document"
        );

        let err = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("creating"),
            "a taken name is refused rather than opened: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn clip_keeps_only_the_window_and_terminates_properly() -> Result<()> {
        let root = test_dir("clip-window")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            false,
            &[
                ("/t", 50),
                ("/t", 100),
                ("/t", 150),
                ("/t", 200),
                ("/t", 250),
            ],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 100, 200);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(100, 200),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.messages_copied, 3);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/t".to_string(), 100),
                ("/t".to_string(), 150),
                ("/t".to_string(), 200),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn clip_extracts_across_chunked_input() -> Result<()> {
        let root = test_dir("clip-chunked")?;
        let rec = root.join("rec.mcap");
        write_recording(
            &rec,
            true,
            &[("/a", 10), ("/b", 20), ("/a", 30), ("/b", 40), ("/a", 50)],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 20, 40);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(20, 40),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.messages_copied, 3);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/b".to_string(), 20),
                ("/a".to_string(), 30),
                ("/b".to_string(), 40),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn clip_before_any_recording_is_a_valid_empty_mcap() -> Result<()> {
        let root = test_dir("clip-empty")?;

        let out = root.join("clip.mcap");
        // Nothing is indexed, so no recording offers a plan: the empty plan a
        // trigger arriving before any data still cuts a valid clip from.
        let plan = WindowPlan::empty();
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.messages_copied, 0);
        assert!(read_clip(&out)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn topics_spread_over_the_file_collapse_to_one_channel_each() -> Result<()> {
        let root = test_dir("clip-remap")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/a", 10), ("/b", 20), ("/a", 30)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        let buf = std::fs::read(&out)?;
        let summary = mcap::Summary::read(&buf)?.expect("clip has a summary");
        assert_eq!(summary.channels.len(), 2);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn deleted_recording_still_extracts_through_the_open_handle() -> Result<()> {
        let root = test_dir("clip-deleted")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);

        // The recorder-restart scenario: the bag directory is wiped while a
        // window is still being cut. The plan's `Arc<File>` keeps the inode
        // alive, so the extraction must succeed against the deleted path.
        std::fs::remove_file(&rec)?;
        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_clip(&out)?,
            vec![("/t".to_string(), 10), ("/t".to_string(), 20)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn truncated_recording_fails_extraction_and_removes_the_partial_clip() -> Result<()> {
        let root = test_dir("clip-truncated")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20), ("/t", 30)])?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);

        // Shrink the recording under the plan (append-only violated — e.g. a
        // damaged filesystem). The extent read must fail, and the failure must
        // not leave a half-written clip behind.
        let last = plan.extents.last().expect("plan covers the recording");
        std::fs::OpenOptions::new()
            .write(true)
            .open(&rec)?
            .set_len(last.offset + last.len / 2)?;

        let out = root.join("clip.mcap");
        let err = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("reading extent"),
            "unexpected error: {err:#}"
        );
        assert!(
            !out.exists(),
            "the half-written file is removed rather than left under the name of \
             a finished one"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn message_on_unknown_channel_is_skipped_and_counted() -> Result<()> {
        let root = test_dir("clip-nochannel")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10)])?;
        let index = index_whole(&rec)?;
        let mut plan = plan_one(&index, 0, 100);
        // No Channel record for the message's ID: nothing to emit a
        // Schema/Channel from, so the message is skipped — the clip stays
        // valid rather than failing.
        plan.channels.clear();

        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.messages_copied, 0);
        assert_eq!(stats.records_skipped, 1);
        assert!(read_clip(&out)?.is_empty(), "a valid, empty clip");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn garbage_extent_bytes_fail_parsing_and_remove_the_partial_clip() -> Result<()> {
        let root = test_dir("clip-garbage")?;
        let junk = root.join("junk.bin");
        std::fs::write(&junk, [0xFFu8; 64])?;

        // A plan whose extent points at bytes that are not record-framed at
        // all — the index is corrupt or reading the wrong file. Unlike a bad
        // record *body* (skippable), bad framing leaves no boundary to
        // resync at, so this must stay fatal.
        let plan = WindowPlan {
            source: Some(PlanSource {
                path: junk.clone(),
                file: Arc::new(File::open(&junk)?),
            }),
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
        };

        let out = root.join("clip.mcap");
        let err = extract_clip(
            &plan,
            &out,
            &log_window(0, u64::MAX),
            TEST_COMPRESSION,
            &every_topic(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("framing inconsistent"),
            "unexpected error: {err:#}"
        );
        // The refusal is a *type*, not a message. A caller cutting repeatedly
        // from one recording branches on this to tell file damage from a full
        // disk, and reads the recording off it to count under and the extent off
        // it to state the blast radius.
        let desync = err
            .downcast_ref::<FramingDesync>()
            .unwrap_or_else(|| panic!("a framing refusal is a FramingDesync: {err:#}"));
        assert_eq!(desync.recording(), junk, "the desync names its recording");
        assert_eq!(desync.extent_offset(), 0, "and the extent the walk entered");
        assert!(
            !out.exists(),
            "the half-written file is removed rather than left under the name of \
             a finished one"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn window_between_messages_is_a_valid_empty_clip() -> Result<()> {
        let root = test_dir("clip-gap")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 100), ("/t", 200)])?;
        let index = index_whole(&rec)?;

        // The extent (time bounds 100..200) overlaps the window, so it is
        // planned and read — but no individual message falls inside it.
        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 120, 180);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(120, 180),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert!(stats.extents_read > 0, "the covering extent is read");
        assert_eq!(stats.messages_copied, 0);
        assert!(read_clip(&out)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn clip_cuts_inside_a_single_chunk() -> Result<()> {
        let root = test_dir("clip-onechunk")?;
        let rec = root.join("rec.mcap");
        // A chunk size far above the data volume puts every message into one
        // chunk; the window must still select individual messages inside it.
        let opts = mcap::WriteOptions::new()
            .use_chunks(true)
            .compression(Some(mcap::Compression::Zstd))
            .chunk_size(Some(1 << 20));
        write_recording_opts(
            &rec,
            opts,
            b"payload",
            &[("/t", 10), ("/t", 20), ("/t", 30), ("/t", 40), ("/t", 50)],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 20, 40);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(20, 40),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.messages_copied, 3);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/t".to_string(), 20),
                ("/t".to_string(), 30),
                ("/t".to_string(), 40),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn schemaless_channel_clips_through() -> Result<()> {
        let root = test_dir("clip-schemaless")?;
        let rec = root.join("rec.mcap");
        // MCAP allows a channel with schema_id 0 (no schema). The registry
        // must carry it as `schema: None` and the clip must reproduce it.
        let mut writer = mcap::WriteOptions::new()
            .use_chunks(false)
            .compression(None)
            .create(BufWriter::new(File::create(&rec)?))?;
        let ch = writer.add_channel(0, "/raw", "cdr", &BTreeMap::new())?;
        writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: ch,
                sequence: 0,
                log_time: 10,
                publish_time: 10,
            },
            b"x",
        )?;
        writer.finish()?;

        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);
        assert!(
            plan.channels.values().all(|c| c.schema.is_none()),
            "schema_id 0 must resolve to no schema"
        );

        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.messages_copied, 1);
        assert_eq!(read_clip(&out)?, vec![("/raw".to_string(), 10)]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn clip_spanning_multiple_extents_copies_each_message_once() -> Result<()> {
        let root = test_dir("clip-multiextent")?;
        let rec = root.join("rec.mcap");
        // 1 MiB messages close an extent every ~4 messages; the window must
        // straddle at least one extent boundary and lose nothing at the seam.
        let payload = vec![0u8; 1 << 20];
        let stamps: Vec<(&str, u64)> = (1..=9).map(|i| ("/big", i * 10)).collect();
        write_recording_opts(
            &rec,
            mcap::WriteOptions::new()
                .use_chunks(false)
                .compression(None),
            &payload,
            &stamps,
        )?;
        let index = index_whole(&rec)?;
        assert!(
            plan_one(&index, 0, u64::MAX).extents.len() >= 2,
            "precondition: the recording spans several extents"
        );

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 30, 70);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(30, 70),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert!(
            stats.extents_read >= 2,
            "the window must cross an extent boundary, read {}",
            stats.extents_read
        );
        assert_eq!(stats.messages_copied, 5);
        assert_eq!(
            read_clip(&out)?,
            (3..=7)
                .map(|i| ("/big".to_string(), i * 10))
                .collect::<Vec<_>>()
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn record_larger_than_the_extent_cap_stays_whole_and_extracts() -> Result<()> {
        let root = test_dir("clip-oversized")?;
        let rec = root.join("rec.mcap");
        // Every record exceeds EXTENT_CAP_BYTES on its own: extents close at
        // record boundaries, so each must hold exactly one (oversized) record
        // rather than splitting it.
        let payload = vec![0u8; 5 << 20];
        write_recording_opts(
            &rec,
            mcap::WriteOptions::new()
                .use_chunks(false)
                .compression(None),
            &payload,
            &[("/big", 10), ("/big", 20), ("/big", 30)],
        )?;
        let index = index_whole(&rec)?;

        let all = plan_one(&index, 0, u64::MAX);
        assert_eq!(all.extents.len(), 3, "one oversized extent per message");
        for pair in all.extents.windows(2) {
            assert_eq!(pair[1].offset, pair[0].offset + pair[0].len);
        }

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 15, 25);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(15, 25),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(stats.extents_read, 1);
        assert_eq!(stats.messages_copied, 1);
        assert_eq!(
            stats.bytes_copied,
            5 << 20,
            "the oversized body must come through intact"
        );
        assert_eq!(read_clip(&out)?, vec![("/big".to_string(), 20)]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn every_chunk_compression_extracts_identically() -> Result<()> {
        let root = test_dir("clip-compressions")?;
        let stamps = [("/a", 10), ("/b", 20), ("/a", 30), ("/b", 40)];
        let expected = vec![("/b".to_string(), 20), ("/a".to_string(), 30)];

        for (name, compression) in [
            ("uncompressed", None),
            ("lz4", Some(mcap::Compression::Lz4)),
            ("zstd", Some(mcap::Compression::Zstd)),
        ] {
            let rec = root.join(format!("rec-{name}.mcap"));
            write_recording_opts(
                &rec,
                mcap::WriteOptions::new()
                    .use_chunks(true)
                    .compression(compression)
                    .chunk_size(Some(128)),
                b"payload",
                &stamps,
            )?;
            let index = index_whole(&rec)?;
            let plan = plan_one(&index, 20, 30);
            assert_eq!(plan.channels.len(), 2, "{name}: registry from chunks");

            let out = root.join(format!("clip-{name}.mcap"));
            let stats = extract_clip(
                &plan,
                &out,
                &log_window(20, 30),
                TEST_COMPRESSION,
                &every_topic(),
            )?;
            assert_eq!(stats.messages_copied, 2, "{name}");
            assert_eq!(read_clip(&out)?, expected, "{name}");
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The configurable *output* codec reaches the clip writer: each setting
    /// produces a valid clip that reads back identically, and the clip's chunk
    /// records carry the matching compression string — so the codec is genuinely
    /// applied, not silently ignored or left at the mcap crate default. The
    /// acceptance test for beads clipper-hcd.
    #[test]
    fn output_compression_setting_is_applied_to_the_clip() -> Result<()> {
        let root = test_dir("clip-outcomp")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20), ("/t", 30)])?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);
        let expected = vec![
            ("/t".to_string(), 10),
            ("/t".to_string(), 20),
            ("/t".to_string(), 30),
        ];

        // mcap's default WriteOptions chunks, so each clip holds at least one
        // Chunk record whose `compression` names the codec ("" = uncompressed).
        for (compression, want) in [
            (None, ""),
            (Some(mcap::Compression::Zstd), "zstd"),
            (Some(mcap::Compression::Lz4), "lz4"),
        ] {
            let out = root.join(format!("clip-{want}.mcap"));
            let stats = extract_clip(
                &plan,
                &out,
                &log_window(0, 100),
                compression,
                &every_topic(),
            )?;
            assert_eq!(stats.messages_copied, 3, "{want}: every message copied");
            assert_eq!(read_clip(&out)?, expected, "{want}: clip reads back intact");

            let buf = std::fs::read(&out)?;
            let codecs: Vec<String> = mcap::read::LinearReader::new(&buf)?
                .filter_map(|rec| match rec {
                    Ok(Record::Chunk { header, .. }) => Some(header.compression),
                    _ => None,
                })
                .collect();
            assert!(!codecs.is_empty(), "{want}: the clip is chunked");
            assert!(
                codecs.iter().all(|c| c == want),
                "{want}: chunks must use the set codec, got {codecs:?}"
            );
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Everything a clip carries per message is copied verbatim: the topic, both
    /// stamps, the sequence number and the payload bytes. The copy decodes only
    /// the one stamp the window lives on, so no other field is ever rebuilt —
    /// this is what pins that, and what an mcap version change has to keep true.
    #[test]
    fn clip_copies_topic_both_stamps_sequence_and_payload_verbatim() -> Result<()> {
        let root = test_dir("clip-verbatim")?;
        let rec = root.join("rec.mcap");
        // Two topics; sequence numbers that are neither zero nor the message's
        // position; publish stamps that disagree with the log stamps; a
        // different payload per message.
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/a", "cdr")),
                raw_record(op::CHANNEL, &channel_body(2, 0, "/b", "cdr")),
                raw_record(op::MESSAGE, &message_body_pub(1, 7, 100, 250, b"alpha")),
                raw_record(op::MESSAGE, &message_body_pub(2, 42, 200, 150, b"bravo")),
                raw_record(op::MESSAGE, &message_body_pub(1, 9, 300, 350, b"charlie")),
            ],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 1000);
        extract_clip(
            &plan,
            &out,
            &log_window(0, 1000),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        let buf = std::fs::read(&out)?;
        let copied: Vec<(String, u64, u64, u32, Vec<u8>)> = mcap::MessageStream::new(&buf)?
            .map(|msg| {
                let msg = msg?;
                Ok((
                    msg.channel.topic.clone(),
                    msg.log_time,
                    msg.publish_time,
                    msg.sequence,
                    msg.data.to_vec(),
                ))
            })
            .collect::<Result<_>>()?;
        assert_eq!(
            copied,
            vec![
                ("/a".to_string(), 100, 250, 7, b"alpha".to_vec()),
                ("/b".to_string(), 200, 150, 42, b"bravo".to_vec()),
                ("/a".to_string(), 300, 350, 9, b"charlie".to_vec()),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn highly_compressed_chunk_extracts_despite_small_extent() -> Result<()> {
        let root = test_dir("clip-ratio")?;
        let rec = root.join("rec.mcap");
        // One 8 MiB zero-filled message in one zstd chunk: the chunk record on
        // disk is a few KiB, so the extent holding it is far smaller than the
        // decompressed interior record. The record length cap must be the
        // framing bound, not the extent size, or this conformant recording
        // fails extraction.
        let payload = vec![0u8; 8 << 20];
        write_recording_opts(
            &rec,
            mcap::WriteOptions::new()
                .use_chunks(true)
                .compression(Some(mcap::Compression::Zstd))
                .chunk_size(Some(16 << 20)),
            &payload,
            &[("/big", 10)],
        )?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);
        assert!(
            plan.extents.iter().map(|e| e.len).sum::<u64>() < (1 << 20),
            "precondition: the chunk compressed far below the payload size"
        );

        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.messages_copied, 1);
        assert_eq!(stats.bytes_copied, 8 << 20);
        assert_eq!(read_clip(&out)?, vec![("/big".to_string(), 10)]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn corrupt_chunk_is_dropped_and_the_other_chunks_survive() -> Result<()> {
        let root = test_dir("clip-chunkcrc")?;
        let rec = root.join("rec.mcap");
        // An uncompressed chunk has no codec to notice corruption — only its
        // CRC. Messages larger than the chunk size land one per chunk, so
        // corrupting the last message's payload damages exactly one chunk.
        let payload = vec![b'Z'; 200];
        write_recording_opts(
            &rec,
            mcap::WriteOptions::new()
                .use_chunks(true)
                .compression(None)
                .chunk_size(Some(128)),
            &payload,
            &[("/t", 10), ("/t", 20), ("/t", 30), ("/t", 40)],
        )?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);

        // Corrupt a payload byte *after* the tail scanned (and CRC-checked)
        // the chunk: post-scan disk damage. Payload bytes exist only inside
        // chunks, and the framing is untouched. Rewriting the path truncates
        // the same inode, so the plan's handle sees the new bytes. The
        // damaged chunk must be dropped whole — the CRC cannot say which of
        // its bytes are lying — and every other chunk must come through.
        let mut bytes = std::fs::read(&rec)?;
        let pos = bytes
            .iter()
            .rposition(|&b| b == b'Z')
            .expect("payload bytes present");
        bytes[pos] ^= 0xFF;
        std::fs::write(&rec, &bytes)?;

        let out = root.join("clip.mcap");
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.chunks_dropped, 1);
        assert_eq!(stats.messages_copied, 3);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/t".to_string(), 10),
                ("/t".to_string(), 20),
                ("/t".to_string(), 30),
            ]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn malformed_record_inside_an_extent_is_skipped_and_the_rest_extracts() -> Result<()> {
        let root = test_dir("clip-poisoned")?;
        let rec = root.join("rec.mcap");
        // A runt Message (4-byte body) with intact framing, wedged between
        // valid messages — disk corruption that still frames. The framing
        // boundary is exact, so extraction skips just the damaged record and
        // the clip keeps the messages around it.
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 10, b"x")),
                raw_record(op::MESSAGE, &[0xAA; 4]),
                raw_record(op::MESSAGE, &message_body(1, 2, 30, b"x")),
            ],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        // The scan indexes past the runt, so the message behind it is planned
        // and copied through below.
        let plan = plan_one(&index, 0, 100);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.records_skipped, 1);
        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_clip(&out)?,
            vec![("/t".to_string(), 10), ("/t".to_string(), 30)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Window membership is tested on the configured [`TimeSource`]: the same
    /// window over the same recording copies a different message set under `log`
    /// than under `publish`, because each message's two stamps disagree. The
    /// extent overlap that plans the bytes is on the same domain.
    #[test]
    fn window_membership_selects_on_the_configured_time_source() -> Result<()> {
        let root = test_dir("clip-domain")?;
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
        let index = index_whole(&rec)?;
        let out = root.join("clip.mcap");

        // The window [180, 320] selects different messages per domain: on `log`
        // it holds log_times 200 and 300; on `publish` only the message
        // published at 250 lands inside, and that message's log_time is 100.
        let log_clip = extract_clip(
            &plan_one_src(&index, 180, 320, TimeSource::Log),
            &out,
            &window_request(180, 320, TimeSource::Log),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        let mut log_times: Vec<u64> = read_clip(&log_clip.out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        log_times.sort_unstable();
        assert_eq!(log_times, vec![200, 300], "log windows on log_time");

        let pub_clip = extract_clip(
            &plan_one_src(&index, 180, 320, TimeSource::Publish),
            &root.join("clip-publish.mcap"),
            &window_request(180, 320, TimeSource::Publish),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        let pub_times: Vec<u64> = read_clip(&pub_clip.out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        assert_eq!(
            pub_times,
            vec![100],
            "publish windows on publish_time; only the message published at 250 is inside"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Every clip carries its manifest, and the writer puts it where an MCAP
    /// reader finds it without a walk: indexed in the summary and counted in the
    /// statistics.
    ///
    /// The three properties are separable and all three matter. A metadata
    /// record written *after* `finish` would not be in the file at all; one
    /// written with the summary suppressed would be in the data section but
    /// unfindable without scanning; and a reader that trusts `metadata_count`
    /// (`mcap info`) reports the wrong thing if the count and the record
    /// disagree. Asserting the index, the count, and a round-trip read pins all
    /// three to the one write.
    #[test]
    fn every_clip_carries_a_manifest_the_summary_indexes_and_counts() -> Result<()> {
        let root = test_dir("clip-manifest")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20), ("/t", 30)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        let buf = std::fs::read(&stats.out_path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        assert_eq!(
            summary
                .metadata_indexes
                .iter()
                .filter(|i| i.name == MANIFEST_NAME)
                .count(),
            1,
            "the summary indexes exactly one record"
        );
        assert_eq!(
            summary
                .stats
                .expect("a finished clip has statistics")
                .metadata_count,
            1,
            "the statistics count it"
        );

        let record = read_manifest(&stats.out_path)?.expect("the clip file carries its id");
        assert_eq!(
            record.keys().collect::<Vec<_>>(),
            vec![CLIP_ID_KEY],
            "one key: what a clip holds is stated once, in the document beside it"
        );
        assert_eq!(
            record[CLIP_ID_KEY],
            crate::ClipId::of(&log_window(0, 100)).to_string()
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// What the copy reports is what the copy did, checked against the clip's
    /// own contents rather than against the numbers that were handed in.
    ///
    /// These counters become one `sources` entry in the clip's document
    /// ([`crate::manifest::SourceMeta`]), so every claim is cross-checked here:
    /// the source path against the recording the plan read, the extent and byte
    /// counts against the plan, the per-channel tally against the channel id the
    /// clip's own `Channel` record carries, and the counts and stamps against
    /// the messages on that channel.
    #[test]
    fn what_a_copy_reports_matches_the_clip_it_wrote() -> Result<()> {
        let root = test_dir("clip-stats-truth")?;
        let rec = root.join("rec.mcap");
        // 10 and 90 are outside the window [20, 80]; 20/50/80 are inside, so the
        // per-channel first/last stamps are the window's own edges.
        write_recording(
            &rec,
            false,
            &[("/t", 10), ("/t", 20), ("/t", 50), ("/t", 80), ("/t", 90)],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 20, 80);
        let extents = plan.extents.len();
        let bytes: u64 = plan.extents.iter().map(|e| e.len).sum();
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(20, 80),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        // Every stamp in the clip lies inside the window and both edges are
        // present, so a wider or narrower claim would be visibly wrong.
        assert_eq!(
            read_clip(&stats.out_path)?
                .into_iter()
                .map(|(_, t)| t)
                .collect::<Vec<u64>>(),
            vec![20, 50, 80],
            "the clip holds the in-window messages"
        );

        assert_eq!(stats.source.as_deref(), Some(rec.as_path()));
        assert_eq!(stats.extents_read, extents);
        assert_eq!(stats.bytes_read, bytes);
        assert_eq!(stats.messages_copied, 3);

        // The tally is keyed by the id the *clip's* own Channel record carries,
        // not the recording's, so a reader can join the two.
        let buf = std::fs::read(&stats.out_path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let (id, channel) = summary
            .channels
            .iter()
            .next()
            .expect("the clip declares its one channel");
        assert_eq!(channel.topic, "/t");
        assert_eq!(
            stats.channels,
            BTreeMap::from([(
                *id,
                ChannelTally {
                    messages: 3,
                    first_ns: 20,
                    last_ns: 80,
                },
            )])
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The per-channel keys name the channels the clip actually holds, keyed by
    /// the id the *clip's own* `Channel` record carries.
    ///
    /// Two properties in one fixture, because one fixture is what makes both
    /// checkable. The recording declares channel 9 and channel 4; only 9 has a
    /// message inside the window, and the clip renumbers what it copies from
    /// scratch. So a tally keyed by the recording's ids would say `9` — an id no
    /// reader of the clip can join to anything — and one that walked the
    /// registry rather than the copy would describe channel 4, which is not in
    /// the clip at all.
    #[test]
    fn per_channel_tallies_name_the_clips_own_channels_by_its_own_ids() -> Result<()> {
        let root = test_dir("clip-manifest-channels")?;
        let rec = root.join("rec.mcap");
        write_raw(
            &rec,
            &[
                raw_record(op::CHANNEL, &channel_body(9, 0, "/kept", "cdr")),
                raw_record(op::CHANNEL, &channel_body(4, 0, "/left-out", "cdr")),
                raw_record(op::MESSAGE, &message_body(9, 0, 30, b"a")),
                raw_record(op::MESSAGE, &message_body(4, 1, 90, b"b")),
                raw_record(op::MESSAGE, &message_body(9, 2, 35, b"c")),
            ],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 20, 40);
        assert_eq!(
            plan.channels.len(),
            2,
            "the plan carries both of the recording's channels"
        );
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(20, 40),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(
            stats.channels.len(),
            1,
            "exactly one channel is tallied: {:?}",
            stats.channels
        );

        let buf = std::fs::read(&stats.out_path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let kept: Vec<u16> = summary
            .channels
            .iter()
            .filter(|(_, c)| c.topic == "/kept")
            .map(|(id, _)| *id)
            .collect();
        assert_eq!(kept.len(), 1, "only the copied channel is in the clip");
        let clip_id = kept[0];
        assert_ne!(
            clip_id, 9,
            "the clip renumbers its channels, so the two id spaces differ here"
        );
        assert_eq!(
            stats.channels,
            BTreeMap::from([(
                clip_id,
                ChannelTally {
                    messages: 2,
                    first_ns: 30,
                    last_ns: 35,
                },
            )])
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The name a `ros2 bag record` MCAP carries its own metadata record under.
    /// Nothing in the crate reads it; it is spelled here so the test below
    /// checks a real name rather than a tautology.
    const ROSBAG2_METADATA_NAME: &str = "rosbag2";

    /// A clip cut from a recording that carries the recorder's own metadata
    /// record still has exactly one metadata record — its own manifest, under a
    /// name that is not the recorder's.
    ///
    /// The two records would otherwise be indistinguishable to a reader looking
    /// one up by name: MCAP allows repeated metadata names, so a manifest called
    /// `rosbag2` would be found by whichever record a tool happened to read
    /// first. The vendor prefix is what keeps the lookup unambiguous.
    #[test]
    fn the_manifest_does_not_collide_with_the_recorders_own_metadata() -> Result<()> {
        let root = test_dir("clip-manifest-name")?;
        let rec = root.join("rec.mcap");
        // A recording shaped like one `ros2 bag record` writes: its own metadata
        // record under the bare name `rosbag2`, then the data.
        write_raw(
            &rec,
            &[
                raw_record(
                    op::METADATA,
                    &metadata_body(ROSBAG2_METADATA_NAME, &[("ROS_DISTRO", "jazzy")]),
                ),
                raw_record(op::CHANNEL, &channel_body(1, 0, "/t", "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 50, b"a")),
            ],
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        let buf = std::fs::read(&stats.out_path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let names: Vec<&str> = summary
            .metadata_indexes
            .iter()
            .map(|i| i.name.as_str())
            .collect();
        assert_eq!(
            names,
            vec![MANIFEST_NAME],
            "the clip's only metadata record is its manifest"
        );
        assert_ne!(MANIFEST_NAME, ROSBAG2_METADATA_NAME);
        assert!(
            MANIFEST_NAME.starts_with("momentedge."),
            "the manifest name is vendor-namespaced: {MANIFEST_NAME}"
        );
        assert!(read_manifest(&stats.out_path)?.is_some());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording carrying one topic per schema, so a clip's schema registry
    /// says something: a schema reaches a clip only through a channel that was
    /// registered, and each of these is reachable through exactly one.
    fn write_four_topic_recording(path: &Path) -> Result<()> {
        write_raw(
            path,
            &[
                raw_record(
                    op::SCHEMA,
                    &schema_body(1, "pkg/Image", "ros2msg", b"image"),
                ),
                raw_record(op::CHANNEL, &channel_body(1, 1, "/camera/image_raw", "cdr")),
                raw_record(op::SCHEMA, &schema_body(2, "pkg/Imu", "ros2msg", b"imu")),
                raw_record(op::CHANNEL, &channel_body(2, 2, "/imu/data", "cdr")),
                raw_record(op::SCHEMA, &schema_body(3, "pkg/Diag", "ros2msg", b"diag")),
                raw_record(op::CHANNEL, &channel_body(3, 3, "/diagnostics", "cdr")),
                raw_record(
                    op::SCHEMA,
                    &schema_body(4, "pkg/Trigger", "ros2msg", b"trig"),
                ),
                raw_record(op::CHANNEL, &channel_body(4, 4, TRIGGER_TOPIC, "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 10, b"a")),
                raw_record(op::MESSAGE, &message_body(2, 1, 20, b"b")),
                raw_record(op::MESSAGE, &message_body(3, 2, 30, b"c")),
                raw_record(op::MESSAGE, &message_body(4, 3, 40, b"d")),
            ],
        )
    }

    /// The topics a finished clip declares, and the schemas its registry holds —
    /// both sorted, so a test states a set rather than a registration order.
    fn clip_channels_and_schemas(path: &Path) -> Result<(Vec<String>, Vec<String>)> {
        let buf = std::fs::read(path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let mut topics: Vec<String> = summary.channels.values().map(|c| c.topic.clone()).collect();
        let mut schemas: Vec<String> = summary.schemas.values().map(|s| s.name.clone()).collect();
        topics.sort();
        schemas.sort();
        Ok((topics, schemas))
    }

    /// One clip a selection test reads back three ways: what it holds, what it
    /// declares, and which schemas came with those declarations.
    struct SelectedClip {
        root: PathBuf,
        messages: Vec<(String, u64)>,
        topics: Vec<String>,
        schemas: Vec<String>,
    }

    /// Cut `selection` out of a four-topic recording and read the clip back.
    fn cut_selected(name: &str, selection: &ChannelSelection) -> Result<SelectedClip> {
        let root = test_dir(name)?;
        let rec = root.join("rec.mcap");
        write_four_topic_recording(&rec)?;
        let index = index_whole(&rec)?;
        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            selection,
        )?;
        let messages = read_clip(&stats.out_path)?;
        let (topics, schemas) = clip_channels_and_schemas(&stats.out_path)?;
        Ok(SelectedClip {
            root,
            messages,
            topics,
            schemas,
        })
    }

    /// A clip cut with an include list holds those topics and no others, and its
    /// schema registry holds only their schemas — a schema only ever reaches a
    /// clip through a channel the copy registered, and an excluded topic never
    /// registers one.
    #[test]
    fn an_include_list_holds_those_topics_and_only_their_schemas() -> Result<()> {
        let selection = ChannelSelection::try_from(Spec {
            include: vec!["/imu/data".to_string(), "/camera/image_raw".to_string()],
            ..Spec::default()
        })?;
        let SelectedClip {
            root,
            messages,
            topics,
            schemas,
        } = cut_selected("clip-include", &selection)?;

        assert_eq!(
            messages,
            vec![
                ("/camera/image_raw".to_string(), 10),
                ("/imu/data".to_string(), 20)
            ]
        );
        assert_eq!(topics, vec!["/camera/image_raw", "/imu/data"]);
        assert_eq!(
            schemas,
            vec!["pkg/Image", "pkg/Imu"],
            "the schemas of the excluded topics never reach the clip"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A clip cut with an exclude regular expression holds no matching topic and
    /// no matching schema; everything else is untouched.
    #[test]
    fn an_exclude_regex_drops_the_matching_topics_and_their_schemas() -> Result<()> {
        let selection = ChannelSelection::try_from(Spec {
            exclude_regex: Some("^/diagnostics|^/camera/".to_string()),
            ..Spec::default()
        })?;
        let SelectedClip {
            root,
            messages,
            topics,
            schemas,
        } = cut_selected("clip-exclude-regex", &selection)?;

        assert!(
            !messages
                .iter()
                .any(|(topic, _)| topic == "/diagnostics" || topic == "/camera/image_raw"),
            "no matching topic is copied: {messages:?}"
        );
        assert_eq!(topics, vec![TRIGGER_TOPIC, "/imu/data"]);
        assert_eq!(schemas, vec!["pkg/Imu", "pkg/Trigger"]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// An excluded topic leaves nothing behind in the clip's document either:
    /// the per-channel tally is filled by the step that writes a message
    /// through, and an excluded channel never reaches it.
    #[test]
    fn an_excluded_channel_has_no_per_channel_tally() -> Result<()> {
        let root = test_dir("clip-excluded-manifest")?;
        let rec = root.join("rec.mcap");
        write_four_topic_recording(&rec)?;
        let index = index_whole(&rec)?;
        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        let selection = ChannelSelection::try_from(Spec {
            include: vec!["/imu/data".to_string()],
            ..Spec::default()
        })?;
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &selection,
        )?;

        assert_eq!(stats.messages_copied, 1);
        // One channel is tallied and it is the kept topic's: its tally reports
        // the one message that was copied, at the stamp only that topic has.
        let tallies: Vec<&ChannelTally> = stats.channels.values().collect();
        assert_eq!(
            tallies.len(),
            1,
            "one kept channel is tallied and nothing else is: {:?}",
            stats.channels
        );
        assert_eq!(tallies[0].messages, 1);
        assert_eq!(tallies[0].first_ns, 20);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// clipper's own announcement topic is absent from every clip, whatever the
    /// configuration asks for — including a configuration that names it
    /// outright.
    #[test]
    fn the_announcement_topic_is_absent_from_every_clip() -> Result<()> {
        let root = test_dir("clip-announcement")?;
        let rec = root.join("rec.mcap");
        write_raw(
            &rec,
            &[
                raw_record(op::SCHEMA, &schema_body(1, "pkg/Imu", "ros2msg", b"imu")),
                raw_record(op::CHANNEL, &channel_body(1, 1, "/imu/data", "cdr")),
                raw_record(
                    op::SCHEMA,
                    &schema_body(2, "pkg/Recorded", "ros2msg", b"recorded"),
                ),
                raw_record(op::CHANNEL, &channel_body(2, 2, ANNOUNCE_TOPIC, "cdr")),
                raw_record(op::MESSAGE, &message_body(1, 0, 10, b"a")),
                raw_record(op::MESSAGE, &message_body(2, 1, 20, b"b")),
            ],
        )?;
        let index = index_whole(&rec)?;

        for (n, selection) in [
            ChannelSelection::default(),
            ChannelSelection::try_from(Spec {
                all: Some(true),
                ..Spec::default()
            })?,
            ChannelSelection::try_from(Spec {
                include: vec![ANNOUNCE_TOPIC.to_string(), "/imu/data".to_string()],
                ..Spec::default()
            })?,
            ChannelSelection::try_from(Spec {
                include_regex: Some("^/".to_string()),
                ..Spec::default()
            })?,
        ]
        .into_iter()
        .enumerate()
        {
            let out = root.join(format!("clip{n}.mcap"));
            let plan = plan_one(&index, 0, 100);
            let stats = extract_clip(
                &plan,
                &out,
                &log_window(0, 100),
                TEST_COMPRESSION,
                &selection,
            )?;
            let (topics, schemas) = clip_channels_and_schemas(&stats.out_path)?;
            assert_eq!(topics, vec!["/imu/data"], "selection {n}");
            assert_eq!(schemas, vec!["pkg/Imu"], "selection {n}");
            assert_eq!(stats.channels.len(), 1, "selection {n}");
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// With the trigger key off a clip carries its triggers; with it on it does
    /// not. It is the only key that governs the trigger topic.
    #[test]
    fn the_trigger_topic_goes_only_when_its_key_says_so() -> Result<()> {
        let kept = ChannelSelection::try_from(Spec::default())?;
        let SelectedClip {
            root,
            messages,
            topics,
            ..
        } = cut_selected("clip-trigger-kept", &kept)?;
        assert!(
            messages.iter().any(|(topic, _)| topic == TRIGGER_TOPIC),
            "a clip keeps its triggers by default: {messages:?}"
        );
        assert!(topics.contains(&TRIGGER_TOPIC.to_string()));
        std::fs::remove_dir_all(root)?;

        let dropped = ChannelSelection::try_from(Spec {
            exclude_trigger_topic: true,
            ..Spec::default()
        })?;
        let SelectedClip {
            root,
            messages,
            topics,
            schemas,
        } = cut_selected("clip-trigger-dropped", &dropped)?;
        assert!(
            !messages.iter().any(|(topic, _)| topic == TRIGGER_TOPIC),
            "the key drops them: {messages:?}"
        );
        assert!(!topics.contains(&TRIGGER_TOPIC.to_string()));
        assert!(!schemas.contains(&"pkg/Trigger".to_string()));
        assert_eq!(
            topics,
            vec!["/camera/image_raw", "/diagnostics", "/imu/data"],
            "and governs that topic alone"
        );
        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
