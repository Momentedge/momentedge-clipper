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
//! every message ([`ClipWriter::copy_message`]), and the selection once per
//! channel ([`ClipWriter::route`]) — the same step that registers a channel in
//! the output, so an excluded topic contributes to a clip neither a channel, nor
//! a schema, nor a message, nor a manifest key.
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
//! abort the clip. `Writer::finish` writes the summary section, footer and
//! closing magic, so a clip is always a complete, standalone MCAP.

use std::collections::{BTreeMap, HashMap};
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use log::{error, info, warn};
use mcap::records::Record;

use crate::TimeSource;
use crate::index::{ChannelDef, MAX_RECORD_LEN, WindowPlan, op};
#[cfg(test)]
use crate::manifest::WindowCoverage;
use crate::manifest::{ChannelTally, ClipManifest, CutRequest, Planned};
use crate::select::ChannelSelection;

/// The stamp a message's window membership is tested on, per the window's
/// [`TimeSource`]: its `log_time` or its `publish_time`.
fn message_stamp(header: &mcap::records::MessageHeader, source: TimeSource) -> u64 {
    match source {
        TimeSource::Log => header.log_time,
        TimeSource::Publish => header.publish_time,
    }
}

/// Outcome of an extraction: where the clip actually landed (`out_path`
/// carries a `_<n>` suffix when the desired name already existed) and the
/// copy counters, for logging.
#[derive(Debug, Default)]
pub struct ClipStats {
    pub out_path: PathBuf,
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
}

/// The uncompressed size a clip's chunks are cut at: 768 KiB.
///
/// The mcap writer closes a chunk on the first message that carries it past
/// this target, so a chunk holds a little over 768 KiB of pre-compression bytes
/// and the clip's seek granularity — and the memory a reader spends
/// decompressing one chunk — follow from it. It is set on every clip's
/// [`mcap::WriteOptions`] rather than inherited, so the layout a clip is
/// written in is this crate's decision and moves only when this line does.
///
/// It differs from [`mcap::WriteOptions::DEFAULT_CHUNK_SIZE`] on purpose, and
/// `clip_chunk_size_is_the_size_the_cut_path_names` holds the two apart. A
/// value equal to the crate's own default would write the same bytes whether
/// the cut path set it or not, so nothing would notice the setting being
/// dropped — and the next bump of that default would move every clip's layout,
/// which is the whole thing naming the size exists to prevent.
pub const CLIP_CHUNK_SIZE: u64 = 1024 * 768;

/// The name of the capturing subdirectory under the final output directory.
/// A clip is assembled here and moved out only once complete; observers of the
/// final directory therefore never see an in-progress or footer-less file. A
/// subdirectory (not a sibling) guarantees the same filesystem, so the
/// stage-two move is a true atomic link rather than a cross-device copy.
const CAPTURING_DIR: &str = ".capturing";

/// Prepare a fresh capturing directory under `out_dir`, to be called once at
/// startup before any clip is cut. Removing and recreating it discards any
/// leftover from a previous run — a crash between [`publish_clip`]'s hard link
/// and the staged-file unlink strands a stale link in the capturing directory,
/// harmless to published clips but otherwise accumulating across restarts. The
/// recreate (`create_dir_all`) also ensures `out_dir` itself exists, so a first
/// run with no output tree is ready to publish into. A missing capturing
/// directory is not an error; any other IO failure is, since a process that
/// cannot prepare its output directory must not start.
pub fn reset_capturing_dir(out_dir: &Path) -> Result<()> {
    let capturing = out_dir.join(CAPTURING_DIR);
    match std::fs::remove_dir_all(&capturing) {
        Ok(()) => {}
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
        Err(e) => {
            return Err(e)
                .with_context(|| format!("clearing capturing dir {}", capturing.display()));
        }
    }
    std::fs::create_dir_all(&capturing)
        .with_context(|| format!("creating capturing dir {}", capturing.display()))
}

/// A clip that finished assembling in the capturing directory, awaiting its
/// move into the final directory. [`stage_clip`] produces one and
/// [`publish_clip`] consumes it. Dropping one without publishing — an early
/// return or a panic between the stages — removes the staged file, so a clip
/// that never reached the final directory never lingers in the capturing area
/// either.
#[must_use = "a staged clip must be published or it is cleaned up unpublished"]
#[derive(Debug)]
pub struct StagedClip {
    /// Where the completed, fsynced file currently lives in the capturing dir.
    staged_path: PathBuf,
    /// The final directory the clip belongs in once published.
    out_dir: PathBuf,
    /// The caller's desired final filename (no directory). Publication resolves
    /// collisions against the final directory starting from this name, so the
    /// suffixed name used while staging never leaks into the final path.
    desired_name: std::ffi::OsString,
    /// The copy counters, carried through to the published [`ClipStats`].
    stats: ClipStats,
    /// Cleared once the file is linked into the final directory, so the `Drop`
    /// cleanup unlinks the staged file only while it is still the live copy.
    staged: bool,
}

impl StagedClip {
    /// Override the filename this clip will be published under. Used to assign a
    /// `_NN` segment suffix once a window's segment count is known: a window that
    /// stayed in one file keeps its bare desired name, a window that straddled a
    /// rollover gets one numbered segment per source file.
    pub fn set_final_name(&mut self, name: std::ffi::OsString) {
        self.desired_name = name;
    }

    /// Whether this staged segment copied no in-window messages — a rollover
    /// whose new file held nothing inside the window stages such an empty
    /// trailing segment, which the caller drops when other segments carry data.
    pub fn is_empty(&self) -> bool {
        self.stats.messages_copied == 0
    }
}

impl Drop for StagedClip {
    fn drop(&mut self) {
        // Only the unpublished staged file is ours to remove; once it is linked
        // into the final directory the staged name has already been unlinked.
        if self.staged
            && let Err(e) = std::fs::remove_file(&self.staged_path)
        {
            warn!("removing staged clip {}: {e}", self.staged_path.display());
        }
    }
}

/// Copy every message in `[start_ns, end_ns]` (inclusive bounds) from the
/// planned extents into a clip published at
/// `out_path` (or a `_<n>`-suffixed sibling if that name is taken — see
/// [`link_into`]). This composes the two stages: [`stage_clip`] assembles and
/// fsyncs the clip in the capturing directory, then [`publish_clip`] moves it
/// atomically into the final directory. Localized damage in the recording — an
/// unparseable record body, a message on an unregistered channel, a chunk
/// failing CRC or decompression — is skipped with an error log and counted in
/// [`ClipStats`]; the clip keeps everything else. Errors that do surface are
/// all-or-nothing: on success the clip is complete and durably in the final
/// directory before this returns, so a caller may announce it as on disk; on
/// error nothing partial reaches the final directory — cleanup is confined to
/// the capturing directory.
///
/// `compression` is the codec the clip's `mcap::Writer` is built with (`None`
/// for uncompressed); it is set explicitly rather than inherited from the mcap
/// crate default. `selection` is which of the recording's topics the clip is
/// cut from; both are properties of the output rather than of the window.
///
/// The recorder stages and publishes in two explicit steps (so a window
/// straddling a rollover can publish all its segments together once their count
/// is known); this one-call composition serves the clip-assembly tests.
#[cfg(test)]
pub fn extract_clip(
    plan: &WindowPlan,
    out_path: &Path,
    request: &CutRequest,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<ClipStats> {
    let planned = Planned {
        files: usize::from(plan.source.is_some()),
        coverage: WindowCoverage::Covered,
    };
    publish_clip(stage_clip(
        plan,
        out_path,
        request,
        planned,
        compression,
        selection,
    )?)
}

/// Stage one: assemble the clip in the capturing directory under
/// `out_path`'s parent, fsync the file, and return it for publication. The
/// final directory is never touched here, so an observer of it never sees the
/// in-progress file. On copy failure the partial file is removed from the
/// capturing directory only. `out_path`'s file name is carried as the desired
/// final name; the capturing file may take a `_<n>` suffix to avoid an
/// in-flight collision with a concurrent stage, independent of the final name.
pub fn stage_clip(
    plan: &WindowPlan,
    out_path: &Path,
    request: &CutRequest,
    planned: Planned,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<StagedClip> {
    let out_dir = out_path
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));
    let desired_name = out_path
        .file_name()
        .context("clip path has no file name")?
        .to_os_string();
    let capturing = out_dir.join(CAPTURING_DIR);
    std::fs::create_dir_all(&capturing)
        .with_context(|| format!("creating capturing dir {}", capturing.display()))?;

    let (file, staged_path) = create_new_file(&capturing.join(&desired_name))?;
    let stats =
        copy_window(plan, file, request, planned, compression, selection).inspect_err(|_| {
            // A failed copy must not leave a half-written, footer-less file even in
            // the capturing dir; the error itself is what the caller reports. No
            // `StagedClip` is constructed on this path, so its `Drop` cannot do it.
            if let Err(e) = std::fs::remove_file(&staged_path) {
                warn!("removing partial clip {}: {e}", staged_path.display());
            }
        })?;
    Ok(StagedClip {
        staged_path,
        out_dir,
        desired_name,
        stats,
        staged: true,
    })
}

/// Stage two: atomically move the staged clip into the final directory and
/// fsync that directory so the new entry survives a crash. The move never
/// replaces an existing clip — `std::fs::rename` would silently clobber one,
/// so a hard link (atomic, failing with `AlreadyExists`) resolves collisions
/// with the same `_<n>` suffix retry against the *desired* final name. The
/// link is the commit point: once it succeeds the final directory holds a
/// complete clip (the staged file was fsynced before this), so the staged name
/// is unlinked and the directory fsynced, and the [`ClipStats`] carries the
/// published path. A failed link (e.g. the suffix cap is exhausted) leaves the
/// final directory untouched and the dropped [`StagedClip`] removes the staged
/// file, so a failed publish leaves nothing behind in either directory.
///
/// The link and the staged-file unlink are two steps, not one: a crash between
/// them leaves the published clip intact (the link is the durable copy) but
/// strands the staged file in the capturing directory. That leftover is
/// harmless — observers read only the final directory — and bounded to one run
/// by [`reset_capturing_dir`] clearing the capturing directory at startup.
pub fn publish_clip(mut staged: StagedClip) -> Result<ClipStats> {
    let final_path = link_into(
        &staged.staged_path,
        &staged.out_dir.join(&staged.desired_name),
    )?;
    // The link committed a complete clip to the final directory; the staged
    // file is no longer the live copy, so suppress the `Drop` cleanup and drop
    // the capturing-dir name ourselves.
    staged.staged = false;
    if let Err(e) = std::fs::remove_file(&staged.staged_path) {
        warn!("removing staged clip {}: {e}", staged.staged_path.display());
    }
    // fsync the directory so the new entry — not just the file's data —
    // survives a crash. Opening a directory and `sync_all`ing it is the POSIX
    // way to flush directory metadata; it works on Linux.
    File::open(&staged.out_dir)
        .and_then(|d| d.sync_all())
        .with_context(|| format!("syncing output dir {}", staged.out_dir.display()))?;
    let mut stats = std::mem::take(&mut staged.stats);
    stats.out_path = final_path;
    Ok(stats)
}

/// Assemble the clip into the freshly created `out_file`: register window
/// channels from the registry on first use, stream the planned extents, write
/// the manifest, finish and fsync the file. The caller removes the staged file
/// if this fails.
///
/// The writer is built from explicit [`mcap::WriteOptions`] with both knobs that
/// decide what a clip looks like set outright rather than inherited: the
/// `compression` codec (`None` = uncompressed) the caller chose, and
/// [`CLIP_CHUNK_SIZE`]. Chunking itself stays on, at the `WriteOptions` default.
/// `selection` decides which of the registry's topics are registered at all.
///
/// The manifest goes in between the last copied message and `finish`, which is
/// what puts it in the summary's metadata index and the statistics' metadata
/// count: writing it any earlier would have to guess the counters it reports.
fn copy_window(
    plan: &WindowPlan,
    out_file: File,
    request: &CutRequest,
    planned: Planned,
    compression: Option<mcap::Compression>,
    selection: &ChannelSelection,
) -> Result<ClipStats> {
    let mut clip = ClipWriter {
        writer: mcap::WriteOptions::new()
            .compression(compression)
            .chunk_size(Some(CLIP_CHUNK_SIZE))
            .create(BufWriter::new(out_file))
            .context("opening mcap writer")?,
        channels: &plan.channels,
        selection,
        routes: HashMap::new(),
        request,
        tallies: BTreeMap::new(),
        stats: ClipStats::default(),
    };

    if let Some(source) = &plan.source {
        for extent in &plan.extents {
            clip.stats.extents_read += 1;
            clip.stats.bytes_read += extent.len;
            let mut buf = vec![0u8; extent.len as usize];
            source
                .file
                .read_exact_at(&mut buf, extent.offset)
                .with_context(|| {
                    format!("reading extent at {} (+{} B)", extent.offset, extent.len)
                })?;
            clip.copy_extent(&buf)?;
        }
    }
    clip.write_manifest(planned, plan.source.as_ref().map(|s| s.path.as_path()))?;

    let ClipWriter {
        mut writer, stats, ..
    } = clip;
    writer.finish().context("finalising output mcap")?;
    // `finish` can leave bytes in the BufWriter; flush them and fsync the file
    // so its contents are durable in the capturing dir before publication
    // moves it into the final directory.
    writer
        .into_inner()
        .into_inner()
        .map_err(|e| anyhow::anyhow!("flushing clip: {e}"))?
        .sync_all()
        .context("syncing clip to disk")?;
    Ok(stats)
}

/// Create a fresh file at `desired`, never opening an existing one — two
/// concurrent stages aiming at the same capturing name get distinct files
/// (`_<n>`-suffixed) instead of interleaving bytes into one. Returns the open
/// file and the path it landed at.
fn create_new_file(desired: &Path) -> Result<(File, PathBuf)> {
    let mut file = None;
    let path = with_suffix_retry(desired, "creating", |candidate| {
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(candidate)
            .map(|f| file = Some(f))
    })?;
    Ok((file.expect("a successful create yields the file"), path))
}

/// Hard-link `src` to `desired`, never replacing an existing file — a duplicate
/// trigger (same stamp and name) publishes to a `_<n>`-suffixed sibling instead
/// of clobbering the earlier clip. `rename` would replace silently;
/// `hard_link` is equally atomic but fails with `AlreadyExists`, which the
/// suffix retry resolves. Returns the path the link landed at.
fn link_into(src: &Path, desired: &Path) -> Result<PathBuf> {
    with_suffix_retry(desired, "publishing", |candidate| {
        std::fs::hard_link(src, candidate)
    })
}

/// Run `attempt` against `desired`, then `desired` with `_1`, `_2`, … inserted
/// before the extension, until it succeeds — resolving a name collision the
/// same way for both staging (`create_new`) and publishing (`hard_link`), the
/// two operations that fail with `AlreadyExists` on a taken name. Gives up
/// after 1000 suffixes so a directory wedged full of collisions cannot loop
/// forever. `verb` names the operation for error context.
fn with_suffix_retry(
    desired: &Path,
    verb: &str,
    mut attempt: impl FnMut(&Path) -> std::io::Result<()>,
) -> Result<PathBuf> {
    let stem = desired.file_stem().unwrap_or_default().to_string_lossy();
    let ext = desired
        .extension()
        .map(|e| format!(".{}", e.to_string_lossy()))
        .unwrap_or_default();
    let mut path = desired.to_path_buf();
    for n in 1.. {
        match attempt(&path) {
            Ok(()) => {
                if path != desired {
                    warn!(
                        "clip {} already exists; using {}",
                        desired.display(),
                        path.display()
                    );
                }
                return Ok(path);
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists && n <= 1000 => {
                path = desired.with_file_name(format!("{stem}_{n}{ext}"));
            }
            Err(e) => return Err(e).with_context(|| format!("{verb} {}", path.display())),
        }
    }
    unreachable!("loop returns or errors within 1000 attempts");
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
    /// the trigger and producer the manifest names.
    request: &'a CutRequest,
    /// Per **output** channel id, what that channel has contributed so far —
    /// the manifest's per-channel keys. Filled by the same step that writes a
    /// message through, so a channel no message was copied from has no entry.
    tallies: BTreeMap<u16, ChannelTally>,
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
    /// and that aborts the clip.
    fn copy_extent(&mut self, buf: &[u8]) -> Result<()> {
        let mut offset = 0usize;
        while offset + 9 <= buf.len() {
            let opcode = buf[offset];
            let len = u64::from_le_bytes(buf[offset + 1..offset + 9].try_into().unwrap());
            if len > MAX_RECORD_LEN || offset + 9 + len as usize > buf.len() {
                bail!(
                    "record at extent offset {offset} declares {len} B; \
                     extent framing inconsistent with the tail's scan"
                );
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
            bail!(
                "extent ends mid-record at offset {offset}; framing inconsistent with the tail's scan"
            );
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
        let (start_ns, end_ns, time_source) = (
            self.request.start_ns(),
            self.request.end_ns(),
            self.request.time_source(),
        );
        let mut pending: Vec<(mcap::records::MessageHeader, Vec<u8>)> = Vec::new();
        let salvage = (|| -> mcap::McapResult<()> {
            let Record::Chunk { header, data } = mcap::parse_record(op::CHUNK, body)? else {
                unreachable!("a CHUNK opcode parses to Record::Chunk");
            };
            for rec in mcap::read::ChunkReader::new(header, &data)? {
                if let Record::Message { header, data } = rec? {
                    let stamp = message_stamp(&header, time_source);
                    if stamp >= start_ns && stamp <= end_ns {
                        pending.push((header, data.into_owned()));
                    }
                }
            }
            Ok(())
        })();
        match salvage {
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
        self.tallies
            .entry(channel_id)
            .and_modify(|tally| tally.absorb(stamp))
            .or_insert_with(|| ChannelTally::opened(stamp));
    }

    /// Write the clip's manifest ([`crate::manifest`]) from what the copy did:
    /// the caller's window and producer, what the planner offered, this
    /// segment's own source recording, and the counters above.
    fn write_manifest(&mut self, planned: Planned, source: Option<&Path>) -> Result<()> {
        let record = ClipManifest {
            request: self.request,
            planned,
            source,
            extents_read: self.stats.extents_read,
            bytes_read: self.stats.bytes_read,
            messages: self.stats.messages_copied,
            channels: &self.tallies,
        }
        .record();
        self.writer
            .write_metadata(&record)
            .context("writing the clip manifest")
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
    use std::collections::BTreeMap;
    use std::sync::Arc;

    use super::*;
    use crate::index::{Extent, PlanSource, RecordingIndex, Span, Stamps, WindowPlan, op};
    use crate::manifest::{MANIFEST_NAME, MANIFEST_VERSION, read_manifest};
    use crate::select::Spec;
    use crate::testing::{
        channel_body, index_file, message_body, message_body_pub, metadata_body, planned_one_file,
        raw_record, read_clip, scan_to_end, schema_body, test_dir, window_request, write_raw,
        write_recording, write_recording_opts,
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

    #[test]
    fn two_staged_publication_lands_a_valid_clip_and_drains_the_capturing_dir() -> Result<()> {
        let root = test_dir("clip-staged")?;
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

        // The final path is the published location, holding a complete clip.
        assert_eq!(stats.out_path, out);
        assert_eq!(
            read_clip(&out)?,
            vec![
                ("/t".to_string(), 10),
                ("/t".to_string(), 20),
                ("/t".to_string(), 30),
            ]
        );
        // The capturing area exists but holds nothing once publication moved
        // the file out of it: no staged leftover survives a success.
        let capturing = root.join(".capturing");
        assert!(capturing.is_dir(), "the capturing dir is created");
        assert_eq!(
            std::fs::read_dir(&capturing)?.count(),
            0,
            "the staged file is moved out, not left behind"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn staged_clip_is_invisible_in_the_final_dir_until_published() -> Result<()> {
        let root = test_dir("clip-invisible")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);

        // After stage 1 only: the final dir holds no clip, but the staged file
        // in the capturing dir is already complete and read_clip-valid.
        let staged = stage_clip(
            &plan,
            &out,
            &window_request(0, 100, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert!(
            !out.exists(),
            "the clip is invisible in the final dir before publication"
        );
        assert_eq!(
            read_clip(&staged.staged_path)?,
            vec![("/t".to_string(), 10), ("/t".to_string(), 20)],
            "the staged file is already a complete, valid clip"
        );

        // Publication makes it appear in the final dir.
        let stats = publish_clip(staged)?;
        assert_eq!(stats.out_path, out);
        assert_eq!(
            read_clip(&out)?,
            vec![("/t".to_string(), 10), ("/t".to_string(), 20)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn dropping_a_staged_clip_unpublished_cleans_the_capturing_dir() -> Result<()> {
        let root = test_dir("clip-dropped")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);

        // A staged clip abandoned without publishing — an early return or a
        // panic between the stages — must not strand the file in the capturing
        // dir; its `Drop` removes it, and nothing ever reaches the final dir.
        let staged = stage_clip(
            &plan,
            &out,
            &window_request(0, 100, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert!(staged.staged_path.exists(), "the staged file exists");
        drop(staged);

        assert!(!out.exists(), "nothing reached the final dir");
        assert_eq!(
            std::fs::read_dir(root.join(".capturing"))?.count(),
            0,
            "the abandoned staged clip is cleaned up on drop"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn reset_clears_a_stale_capturing_dir_and_leaves_it_empty() -> Result<()> {
        let root = test_dir("clip-reset-stale")?;
        let out = root.join("clips");
        let capturing = out.join(".capturing");
        std::fs::create_dir_all(&capturing)?;
        // A leftover from a previous run — the crash-window stale link the
        // reset exists to clear.
        std::fs::write(capturing.join("stale.mcap"), b"leftover")?;

        reset_capturing_dir(&out)?;

        assert!(capturing.is_dir(), "the capturing dir exists after reset");
        assert_eq!(
            std::fs::read_dir(&capturing)?.count(),
            0,
            "the stale leftover is gone"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn reset_creates_the_dirs_when_none_exist() -> Result<()> {
        let root = test_dir("clip-reset-fresh")?;
        // Neither the final dir nor its capturing subdir exists yet: a fresh
        // run must end up with both, the capturing dir empty.
        let out = root.join("nested").join("clips");
        assert!(!out.exists(), "precondition: nothing exists");

        reset_capturing_dir(&out)?;

        assert!(out.is_dir(), "the final dir is created");
        let capturing = out.join(".capturing");
        assert!(capturing.is_dir(), "the capturing dir is created");
        assert_eq!(std::fs::read_dir(&capturing)?.count(), 0);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn staging_and_publishing_work_after_a_reset() -> Result<()> {
        let root = test_dir("clip-reset-then-cut")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let index = index_whole(&rec)?;

        let out_dir = root.join("clips");
        reset_capturing_dir(&out_dir)?;

        let out = out_dir.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
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
            vec![("/t".to_string(), 10), ("/t".to_string(), 20)]
        );
        assert_eq!(
            std::fs::read_dir(out_dir.join(".capturing"))?.count(),
            0,
            "the capturing dir is drained after publication"
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
    fn duplicate_desired_name_publishes_to_a_suffixed_sibling() -> Result<()> {
        let root = test_dir("clip-dup")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let index = index_whole(&rec)?;
        let plan = plan_one(&index, 0, 100);

        // Two publications of the same desired name: the collision is resolved
        // at the publish stage against the final dir, so the second lands as a
        // `_1` sibling and both clips are complete.
        let out = root.join("clip.mcap");
        let first = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        let second = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_eq!(first.out_path, out);
        assert_eq!(second.out_path, root.join("clip_1.mcap"));
        assert_eq!(read_clip(&first.out_path)?, read_clip(&second.out_path)?);
        assert_eq!(
            std::fs::read_dir(root.join(".capturing"))?.count(),
            0,
            "both publications drain the capturing dir"
        );

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
        assert!(!out.exists(), "nothing partial reaches the final dir");
        assert_eq!(
            std::fs::read_dir(root.join(".capturing"))?.count(),
            0,
            "the partial clip is cleaned out of the capturing dir"
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
        assert!(!out.exists(), "nothing partial reaches the final dir");
        assert_eq!(
            std::fs::read_dir(root.join(".capturing"))?.count(),
            0,
            "the partial clip is cleaned out of the capturing dir"
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
    fn publish_suffix_search_gives_up_after_1000_and_cleans_the_staged_file() -> Result<()> {
        let root = test_dir("clip-suffix-cap")?;
        let out = root.join("clip.mcap");
        std::fs::write(&out, b"existing")?;
        for n in 1..=1000 {
            std::fs::write(root.join(format!("clip_{n}.mcap")), b"existing")?;
        }

        // Nothing is indexed, so the window plans empty; the clip's content is
        // beside the point here — the naming collision is what is under test.
        let plan = WindowPlan::empty();
        // Staging succeeds — the capturing dir is empty, so the clip assembles
        // there — and the collision only surfaces at publish, where 1000
        // suffixes against the pre-filled final dir are exhausted.
        let err = extract_clip(
            &plan,
            &out,
            &log_window(0, 100),
            TEST_COMPRESSION,
            &every_topic(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("publishing"),
            "unexpected error: {err:#}"
        );
        // The pre-existing final files are not ours to disturb, and the staged
        // file is cleaned out of the capturing dir on the failed publish.
        assert_eq!(std::fs::read(&out)?, b"existing");
        assert_eq!(
            std::fs::read_dir(root.join(".capturing"))?.count(),
            0,
            "the staged clip is removed when publish fails"
        );

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

    /// The clip's chunk layout is the size the cut path names, not whatever the
    /// mcap crate defaults to. Messages a sixteenth of [`CLIP_CHUNK_SIZE`] fill
    /// several chunks, and the writer closes a chunk on the first message that
    /// carries it past the target, so every chunk but the last holds more than
    /// `CLIP_CHUNK_SIZE` uncompressed bytes and less than one message more. Both
    /// bounds are derived from the constant, so they move with it: a cut path
    /// that inherited the crate default instead would fail here as soon as the
    /// named size and the default disagree. The acceptance test for beads
    /// clipper-z8r.
    #[test]
    fn clip_chunk_size_is_the_size_the_cut_path_names() -> Result<()> {
        // The named size and the crate default must differ, or this test cannot
        // tell a cut path that sets the size from one that inherits it: both
        // would write identical bytes and deleting the setting would stay green.
        assert_ne!(
            CLIP_CHUNK_SIZE,
            mcap::WriteOptions::DEFAULT_CHUNK_SIZE,
            "the named chunk size must not be the crate's own default, or \
             nothing here can observe the cut path naming it"
        );

        let root = test_dir("clip-chunksize")?;
        let rec = root.join("rec.mcap");

        // Four targets' worth of payload, in messages small enough that the
        // overshoot past the target is a small fraction of a chunk.
        const MSG_LEN: u64 = CLIP_CHUNK_SIZE / 16;
        let payload = vec![b'p'; MSG_LEN as usize];
        let stamps: Vec<(&str, u64)> = (0..64u64).map(|i| ("/t", 10 + i)).collect();
        write_recording_opts(
            &rec,
            mcap::WriteOptions::new()
                .use_chunks(false)
                .compression(None),
            &payload,
            &stamps,
        )?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 1000);
        let stats = extract_clip(
            &plan,
            &out,
            &log_window(0, 1000),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        assert_eq!(stats.messages_copied, stamps.len() as u64);

        let buf = std::fs::read(&out)?;
        let sizes: Vec<u64> = mcap::read::LinearReader::new(&buf)?
            .filter_map(|rec| match rec {
                Ok(Record::Chunk { header, .. }) => Some(header.uncompressed_size),
                _ => None,
            })
            .collect();
        let (last, closed) = sizes.split_last().expect("the clip is chunked");
        assert!(
            closed.len() >= 3,
            "the payload must fill several chunks, got {sizes:?}"
        );
        for size in closed {
            assert!(
                *size > CLIP_CHUNK_SIZE,
                "a chunk is closed only past the named size, got {size} of {CLIP_CHUNK_SIZE}"
            );
            // One message plus a kilobyte of record framing (and, in the first
            // chunk, the schema and channel records) is all a chunk may carry
            // past the target.
            assert!(
                *size <= CLIP_CHUNK_SIZE + MSG_LEN + 1024,
                "a chunk overshoots the named size by at most one message, got {size}"
            );
        }
        assert!(*last > 0, "the final chunk holds the remainder");

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

    /// `reset_capturing_dir` treats `NotFound` as success (the directory simply
    /// did not exist yet) but propagates any other IO error — for example when
    /// the `.capturing` path already exists as a regular file rather than a
    /// directory, causing `remove_dir_all` to fail with `ENOTDIR` on Linux.
    #[test]
    fn reset_fails_when_capturing_path_is_a_file() -> Result<()> {
        let root = test_dir("clip-reset-file")?;
        let out = root.join("clips");
        std::fs::create_dir_all(&out)?;
        // Place a plain file where the capturing directory should be.
        let capturing = out.join(".capturing");
        std::fs::write(&capturing, b"I am a file, not a directory")?;

        // `remove_dir_all` on a regular file path fails with ENOTDIR (not
        // NotFound), so `reset_capturing_dir` must surface that as an error.
        let err = reset_capturing_dir(&out).unwrap_err();
        assert!(
            format!("{err:#}").contains("clearing capturing dir"),
            "unexpected error message: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Dropping a `StagedClip` whose staged file was externally removed before
    /// the drop logs a warning rather than panicking. The `Drop` impl's
    /// `remove_file` will fail with `NotFound`; that failure must be swallowed
    /// as a warning, not propagated (drops must not panic/unwind).
    #[test]
    fn dropping_staged_clip_after_staged_file_removed_does_not_panic() -> Result<()> {
        let root = test_dir("clip-drop-missing")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);
        let staged = stage_clip(
            &plan,
            &out,
            &window_request(0, 100, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        // Simulate the staged file disappearing (e.g. an admin removed it or
        // the capturing dir was wiped) before `Drop` runs its cleanup.
        std::fs::remove_file(&staged.staged_path)?;

        // Drop must not panic even though the file is gone; the warn! arm on
        // line 133 fires instead.
        drop(staged);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// When two concurrent stages aim at the same desired filename inside the
    /// capturing directory, `create_new_file` (`with_suffix_retry` for staging)
    /// resolves the collision with a `_1` suffix — the second stage lands with
    /// a different capturing name. This covers the suffix-retry warn! path in
    /// `with_suffix_retry` (lines 358-361) for the staging side.
    #[test]
    fn concurrent_stages_to_same_name_get_distinct_capturing_files() -> Result<()> {
        let root = test_dir("clip-stage-collision")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let index = index_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&index, 0, 100);

        // Stage the same desired name twice without publishing between them;
        // both clips land in the capturing dir, each under a distinct path.
        let first = stage_clip(
            &plan,
            &out,
            &window_request(0, 100, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?;
        let second = stage_clip(
            &plan,
            &out,
            &window_request(0, 100, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?;

        assert_ne!(
            first.staged_path, second.staged_path,
            "concurrent stages must get distinct staging paths"
        );
        assert!(first.staged_path.exists());
        assert!(second.staged_path.exists());

        // Both staged files are valid, complete clips.
        assert_eq!(read_clip(&first.staged_path)?.len(), 2);
        assert_eq!(read_clip(&second.staged_path)?.len(), 2);

        drop(first);
        drop(second);

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
        let log_clip = publish_clip(stage_clip(
            &plan_one_src(&index, 180, 320, TimeSource::Log),
            &out,
            &window_request(180, 320, TimeSource::Log),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?)?;
        let mut log_times: Vec<u64> = read_clip(&log_clip.out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        log_times.sort_unstable();
        assert_eq!(log_times, vec![200, 300], "log windows on log_time");

        let pub_clip = publish_clip(stage_clip(
            &plan_one_src(&index, 180, 320, TimeSource::Publish),
            &out,
            &window_request(180, 320, TimeSource::Publish),
            planned_one_file(),
            TEST_COMPRESSION,
            &every_topic(),
        )?)?;
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
            "the summary indexes exactly one manifest"
        );
        assert_eq!(
            summary
                .stats
                .expect("a finished clip has statistics")
                .metadata_count,
            1,
            "the statistics count the manifest"
        );

        let manifest = read_manifest(&stats.out_path)?.expect("the clip carries a manifest");
        assert_eq!(manifest["manifest.version"], MANIFEST_VERSION);
        assert_eq!(manifest["producer.name"], "clipper");
        assert_eq!(manifest["producer.mode"], "test");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The trigger, window, source and per-channel groups report what the cut
    /// actually did, checked against the clip's own contents rather than against
    /// the numbers that were handed in.
    ///
    /// Every claim is cross-checked: the window against the messages that landed
    /// inside it, the source path against the file the plan read, the extent and
    /// byte counts against the plan, the per-channel key against the channel id
    /// the clip's own `Channel` record carries, and the counts and stamps against
    /// the messages on that channel.
    #[test]
    fn a_manifests_groups_match_the_clip_they_describe() -> Result<()> {
        let root = test_dir("clip-manifest-truth")?;
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
        let manifest = read_manifest(&stats.out_path)?.expect("the clip carries a manifest");

        // The trigger group is what asked for the window: `log_window` anchors at
        // the window end with the whole width as preroll.
        assert_eq!(manifest["trigger.name"], "test");
        assert_eq!(manifest["trigger.anchor_ns"], "80");
        assert_eq!(manifest["trigger.preroll_ns"], "60");
        assert_eq!(manifest["trigger.postroll_ns"], "0");

        // The window group is the window the messages were actually tested
        // against: every stamp in the clip lies inside it, and both edges are
        // present, so a wider or narrower claim would be visibly wrong.
        assert_eq!(manifest["window.time_source"], "log");
        assert_eq!(manifest["window.start_ns"], "20");
        assert_eq!(manifest["window.end_ns"], "80");
        let stamps: Vec<u64> = read_clip(&stats.out_path)?
            .into_iter()
            .map(|(_, t)| t)
            .collect();
        assert_eq!(
            stamps,
            vec![20, 50, 80],
            "the clip holds the in-window messages"
        );

        // The source group names the recording the bytes came from and how much
        // of it was read.
        assert_eq!(manifest["source.path"], rec.display().to_string());
        assert_eq!(manifest["source.files_planned"], "1");
        assert_eq!(manifest["source.extents_read"], extents.to_string());
        assert_eq!(manifest["source.bytes_read"], bytes.to_string());
        assert_eq!(manifest["clip.messages"], "3");
        assert_eq!(manifest["clip.short"], "false");

        // The per-channel group is keyed by the id the *clip's* own Channel
        // record carries, not the recording's, so a reader can join the two.
        let buf = std::fs::read(&stats.out_path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let (id, channel) = summary
            .channels
            .iter()
            .next()
            .expect("the clip declares its one channel");
        assert_eq!(channel.topic, "/t");
        assert_eq!(manifest[&format!("channel.{id}.messages")], "3");
        assert_eq!(manifest[&format!("channel.{id}.first_ns")], "20");
        assert_eq!(manifest[&format!("channel.{id}.last_ns")], "80");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The per-channel keys name the channels the clip actually holds, keyed by
    /// the id the *clip's own* `Channel` record carries.
    ///
    /// Two properties in one fixture, because one fixture is what makes both
    /// checkable. The recording declares channel 9 and channel 4; only 9 has a
    /// message inside the window, and the clip renumbers what it copies from
    /// scratch. So a manifest keyed by the recording's ids would say
    /// `channel.9.*` — a key no reader of the clip can join to anything — and
    /// one that walked the registry rather than the copy would describe channel
    /// 4, which is not in the clip at all.
    #[test]
    fn per_channel_keys_name_the_clips_own_channels_by_its_own_ids() -> Result<()> {
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
        let manifest = read_manifest(&stats.out_path)?.expect("the clip carries a manifest");

        let channel_keys: Vec<&String> = manifest
            .keys()
            .filter(|k| k.starts_with("channel."))
            .collect();
        assert_eq!(
            channel_keys.len(),
            3,
            "exactly one channel is described, in three keys: {channel_keys:?}"
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
        assert_eq!(manifest[&format!("channel.{clip_id}.messages")], "2");
        assert_eq!(manifest[&format!("channel.{clip_id}.first_ns")], "30");
        assert_eq!(manifest[&format!("channel.{clip_id}.last_ns")], "35");

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

    /// The `channel.<id>.*` keys a clip's manifest carries, sorted.
    fn manifest_channel_keys(path: &Path) -> Result<Vec<String>> {
        let manifest = read_manifest(path)?.expect("the clip carries a manifest");
        let mut keys: Vec<String> = manifest
            .keys()
            .filter(|k| k.starts_with("channel."))
            .cloned()
            .collect();
        keys.sort();
        Ok(keys)
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

    /// An excluded topic leaves nothing behind in the manifest either: the
    /// per-channel keys are written by the step that writes a message through,
    /// and an excluded channel never reaches it.
    #[test]
    fn an_excluded_channel_has_no_per_channel_manifest_keys() -> Result<()> {
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

        let keys = manifest_channel_keys(&stats.out_path)?;
        assert_eq!(
            keys.len(),
            3,
            "one kept channel carries three keys and nothing else does: {keys:?}"
        );
        let manifest = read_manifest(&stats.out_path)?.expect("the clip carries a manifest");
        assert_eq!(manifest["clip.messages"], "1");
        // The one channel's keys report the one message that was copied, so the
        // surviving keys are the kept topic's rather than an excluded one's.
        let id = keys[0]
            .split('.')
            .nth(1)
            .expect("a channel key is channel.<id>.<field>")
            .to_string();
        assert_eq!(manifest[&format!("channel.{id}.messages")], "1");
        assert_eq!(manifest[&format!("channel.{id}.first_ns")], "20");

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
            assert_eq!(
                manifest_channel_keys(&stats.out_path)?.len(),
                3,
                "selection {n}"
            );
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
