//! Window extraction from the tailed continuous recording.
//!
//! Given a [`WindowPlan`] snapshot from the tail — the open recording, the
//! byte extents overlapping the window, and the channel registry — this
//! assembles one output MCAP holding every message whose `log_time` falls in
//! `[start_ns, end_ns]`. It is a **direct copy** of message payload bytes:
//! registry schemas/channels are registered in the output writer by content,
//! then each message is emitted with its raw serialized body. The CDR message
//! bodies are never decoded — the only thing inspected is each record's
//! `log_time`.
//!
//! Each extent is read with `read_at` (no shared seek state with the tail)
//! and its records are walked with our own opcode + length framing — the same
//! walk the tail performed to build the extent, so the boundaries are known
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

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use log::{error, warn};
use mcap::records::Record;

use crate::tail::{ChannelDef, MAX_RECORD_LEN, WindowPlan, op};

/// Outcome of an extraction: where the clip actually landed (`out_path`
/// carries a `_<n>` suffix when the desired name already existed) and the
/// copy counters, for logging.
#[derive(Debug, Default)]
pub struct ClipStats {
    pub out_path: PathBuf,
    pub extents_read: usize,
    pub messages_copied: u64,
    pub bytes_copied: u64,
    /// Records skipped over localized damage: an unparseable body, or a
    /// message on a channel with no Channel record.
    pub records_skipped: u64,
    /// Chunks dropped whole: decompression, CRC, or interior parse failure.
    pub chunks_dropped: u64,
}

/// The name of the capturing subdirectory under the final output directory.
/// A clip is assembled here and moved out only once complete; observers of the
/// final directory therefore never see an in-progress or footer-less file. A
/// subdirectory (not a sibling) guarantees the same filesystem, so the
/// stage-two move is a true atomic rename rather than a copy.
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
/// crate default.
///
/// The recorder stages and publishes in two explicit steps (so a window
/// straddling a rollover can publish all its segments together once their count
/// is known); this one-call composition serves the clip-assembly tests.
#[cfg(test)]
pub fn extract_clip(
    plan: &WindowPlan,
    out_path: &Path,
    start_ns: u64,
    end_ns: u64,
    compression: Option<mcap::Compression>,
) -> Result<ClipStats> {
    let staged = stage_clip(plan, out_path, start_ns, end_ns, compression)?;
    publish_clip(staged)
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
    start_ns: u64,
    end_ns: u64,
    compression: Option<mcap::Compression>,
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
    let stats = copy_window(plan, file, start_ns, end_ns, compression).inspect_err(|_| {
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
/// channels from the registry on first use, stream the planned extents,
/// finish and fsync the file. The caller removes the staged file if this fails.
///
/// The writer is built from explicit [`mcap::WriteOptions`] with `compression`
/// set (`None` = uncompressed), so the codec is a deliberate choice rather than
/// the mcap crate default. Chunk size and chunking stay at the `WriteOptions`
/// default.
fn copy_window(
    plan: &WindowPlan,
    out_file: File,
    start_ns: u64,
    end_ns: u64,
    compression: Option<mcap::Compression>,
) -> Result<ClipStats> {
    let mut clip = ClipWriter {
        writer: mcap::WriteOptions::new()
            .compression(compression)
            .create(BufWriter::new(out_file))
            .context("opening mcap writer")?,
        channels: &plan.channels,
        out_ids: HashMap::new(),
        start_ns,
        end_ns,
        stats: ClipStats::default(),
    };

    if let Some(file) = &plan.file {
        for extent in &plan.extents {
            clip.stats.extents_read += 1;
            let mut buf = vec![0u8; extent.len as usize];
            file.read_exact_at(&mut buf, extent.offset)
                .with_context(|| {
                    format!("reading extent at {} (+{} B)", extent.offset, extent.len)
                })?;
            clip.copy_extent(&buf)?;
        }
    }

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
/// registry, the window bounds, and the running stats — the state every
/// copied message touches.
struct ClipWriter<'a> {
    writer: mcap::Writer<BufWriter<File>>,
    channels: &'a HashMap<u16, ChannelDef>,
    /// Recording channel ID → output channel ID, filled on first use.
    /// `None` caches a known-missing Channel record, so the miss is logged
    /// once rather than per message.
    out_ids: HashMap<u16, Option<u16>>,
    start_ns: u64,
    end_ns: u64,
    stats: ClipStats,
}

impl ClipWriter<'_> {
    /// Walk one extent's records and write the in-window messages through.
    /// Only messages are copied out of the extent bytes; the clip's
    /// Schema/Channel records come from the registry ([`Self::output_channel_id`]),
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
        let (start_ns, end_ns) = (self.start_ns, self.end_ns);
        let mut pending: Vec<(mcap::records::MessageHeader, Vec<u8>)> = Vec::new();
        let salvage = (|| -> mcap::McapResult<()> {
            let Record::Chunk { header, data } = mcap::parse_record(op::CHUNK, body)? else {
                unreachable!("a CHUNK opcode parses to Record::Chunk");
            };
            for rec in mcap::read::ChunkReader::new(header, &data)? {
                if let Record::Message { header, data } = rec?
                    && header.log_time >= start_ns
                    && header.log_time <= end_ns
                {
                    pending.push((header, data.into_owned()));
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

    /// Write one message through if its `log_time` is in the window. A
    /// message on a channel the recording never declared is skipped and
    /// counted — there is no Schema/Channel to emit for it.
    fn copy_message(&mut self, header: &mcap::records::MessageHeader, data: &[u8]) -> Result<()> {
        if header.log_time < self.start_ns || header.log_time > self.end_ns {
            return Ok(());
        }
        let Some(channel_id) = self.output_channel_id(header.channel_id)? else {
            self.stats.records_skipped += 1;
            return Ok(());
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
        self.stats.messages_copied += 1;
        self.stats.bytes_copied += data.len() as u64;
        Ok(())
    }

    /// Map a recording channel ID into the output file and cache the result.
    /// The writer deduplicates schemas/channels by content, so the mapping
    /// stays stable however often a definition is registered. `Ok(None)`
    /// means the recording holds no Channel record for the ID (a
    /// spec-violating file or a registry gap), logged once per ID; errors
    /// are output-side failures.
    fn output_channel_id(&mut self, src_id: u16) -> Result<Option<u16>> {
        if let Some(cached) = self.out_ids.get(&src_id) {
            return Ok(*cached);
        }
        let Some(def) = self.channels.get(&src_id) else {
            error!(
                "messages on channel {src_id} have no Channel record in the recording; skipping them"
            );
            self.out_ids.insert(src_id, None);
            return Ok(None);
        };
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
        self.out_ids.insert(src_id, Some(channel_id));
        Ok(Some(channel_id))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use std::collections::BTreeMap;
    use std::sync::Arc;

    use super::*;
    use crate::tail::tests::{
        channel_body, message_body, raw_record, scan_to_end, test_dir, write_raw, write_recording,
        write_recording_opts,
    };
    use crate::tail::{Extent, Tailer, WindowPlan, op};

    /// Plan the single source recording a clip test cuts from. These tests each
    /// index one recording, so [`Tailer::plan_window`]'s `Vec` holds at most one
    /// plan; an empty `Vec` (no recording yet) becomes an empty plan.
    fn plan_one(tailer: &Tailer, start_ns: u64, end_ns: u64) -> WindowPlan {
        tailer
            .plan_window(start_ns, end_ns)
            .into_iter()
            .next()
            .unwrap_or_else(WindowPlan::empty)
    }

    /// The clip compression the recorder's default (zstd) maps to; most tests
    /// cut clips through the same codec the recorder uses by default.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// Read a finished clip back; `MessageStream` insists on a complete
    /// summary/footer/magic, so this doubles as a validity check.
    pub(crate) fn read_clip(path: &Path) -> Result<Vec<(String, u64)>> {
        let buf = std::fs::read(path)?;
        mcap::MessageStream::new(&buf)?
            .map(|msg| {
                let msg = msg?;
                Ok((msg.channel.topic.clone(), msg.log_time))
            })
            .collect()
    }

    fn tail_whole(path: &Path) -> Result<Arc<Tailer>> {
        let (tailer, _coverage) = Tailer::new();
        let file = Arc::new(File::open(path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        Ok(tailer)
    }

    #[test]
    fn two_staged_publication_lands_a_valid_clip_and_drains_the_capturing_dir() -> Result<()> {
        let root = test_dir("clip-staged")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20), ("/t", 30)])?;
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);

        // After stage 1 only: the final dir holds no clip, but the staged file
        // in the capturing dir is already complete and read_clip-valid.
        let staged = stage_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);

        // A staged clip abandoned without publishing — an early return or a
        // panic between the stages — must not strand the file in the capturing
        // dir; its `Drop` removes it, and nothing ever reaches the final dir.
        let staged = stage_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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
        let tailer = tail_whole(&rec)?;

        let out_dir = root.join("clips");
        reset_capturing_dir(&out_dir)?;

        let out = out_dir.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 100, 200);
        let stats = extract_clip(&plan, &out, 100, 200, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 20, 40);
        let stats = extract_clip(&plan, &out, 20, 40, TEST_COMPRESSION)?;

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
        let (tailer, _coverage) = Tailer::new();

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);

        // Two publications of the same desired name: the collision is resolved
        // at the publish stage against the final dir, so the second lands as a
        // `_1` sibling and both clips are complete.
        let out = root.join("clip.mcap");
        let first = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
        let second = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);
        extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);

        // The recorder-restart scenario: the bag directory is wiped while a
        // window is still being cut. The plan's `Arc<File>` keeps the inode
        // alive, so the extraction must succeed against the deleted path.
        std::fs::remove_file(&rec)?;
        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);

        // Shrink the recording under the plan (append-only violated — e.g. a
        // damaged filesystem). The extent read must fail, and the failure must
        // not leave a half-written clip behind.
        let last = plan.extents.last().expect("plan covers the recording");
        std::fs::OpenOptions::new()
            .write(true)
            .open(&rec)?
            .set_len(last.offset + last.len / 2)?;

        let out = root.join("clip.mcap");
        let err = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION).unwrap_err();
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
        let tailer = tail_whole(&rec)?;
        let mut plan = plan_one(&tailer, 0, 100);
        // No Channel record for the message's ID: nothing to emit a
        // Schema/Channel from, so the message is skipped — the clip stays
        // valid rather than failing.
        plan.channels.clear();

        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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
            file: Some(Arc::new(File::open(&junk)?)),
            extents: vec![Extent {
                offset: 0,
                len: 64,
                time: Some((0, u64::MAX)),
            }],
            channels: HashMap::new(),
        };

        let out = root.join("clip.mcap");
        let err = extract_clip(&plan, &out, 0, u64::MAX, TEST_COMPRESSION).unwrap_err();
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
        let tailer = tail_whole(&rec)?;

        // The extent (time bounds 100..200) overlaps the window, so it is
        // planned and read — but no individual message falls inside it.
        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 120, 180);
        let stats = extract_clip(&plan, &out, 120, 180, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 20, 40);
        let stats = extract_clip(&plan, &out, 20, 40, TEST_COMPRESSION)?;

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

        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);
        assert!(
            plan.channels.values().all(|c| c.schema.is_none()),
            "schema_id 0 must resolve to no schema"
        );

        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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

        let (tailer, _coverage) = Tailer::new();
        let plan = plan_one(&tailer, 0, 100);
        // Staging succeeds — the capturing dir is empty, so the clip assembles
        // there — and the collision only surfaces at publish, where 1000
        // suffixes against the pre-filled final dir are exhausted.
        let err = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION).unwrap_err();
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
        let tailer = tail_whole(&rec)?;
        assert!(
            plan_one(&tailer, 0, u64::MAX).extents.len() >= 2,
            "precondition: the recording spans several extents"
        );

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 30, 70);
        let stats = extract_clip(&plan, &out, 30, 70, TEST_COMPRESSION)?;

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
        let tailer = tail_whole(&rec)?;

        let all = plan_one(&tailer, 0, u64::MAX);
        assert_eq!(all.extents.len(), 3, "one oversized extent per message");
        for pair in all.extents.windows(2) {
            assert_eq!(pair[1].offset, pair[0].offset + pair[0].len);
        }

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 15, 25);
        let stats = extract_clip(&plan, &out, 15, 25, TEST_COMPRESSION)?;

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
            let tailer = tail_whole(&rec)?;
            let plan = plan_one(&tailer, 20, 30);
            assert_eq!(plan.channels.len(), 2, "{name}: registry from chunks");

            let out = root.join(format!("clip-{name}.mcap"));
            let stats = extract_clip(&plan, &out, 20, 30, TEST_COMPRESSION)?;
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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);
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
            let stats = extract_clip(&plan, &out, 0, 100, compression)?;
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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);
        assert!(
            plan.extents.iter().map(|e| e.len).sum::<u64>() < (1 << 20),
            "precondition: the chunk compressed far below the payload size"
        );

        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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
        let tailer = tail_whole(&rec)?;
        let plan = plan_one(&tailer, 0, 100);

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
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
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
        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(File::open(&rec)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, 8)?;
        assert_eq!(
            coverage.get().high_water_ns,
            30,
            "the tail scans past the runt"
        );

        let out = root.join("clip.mcap");
        let plan = plan_one(&tailer, 0, 100);
        let stats = extract_clip(&plan, &out, 0, 100, TEST_COMPRESSION)?;
        assert_eq!(stats.records_skipped, 1);
        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_clip(&out)?,
            vec![("/t".to_string(), 10), ("/t".to_string(), 30)]
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
