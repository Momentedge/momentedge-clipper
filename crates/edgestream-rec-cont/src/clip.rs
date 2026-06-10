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
//! Each extent is read with `read_at` (no shared seek state with the tail),
//! and its records are iterated with `mcap::read::LinearReader::sans_magic`,
//! which accepts a mid-file slice. Chunk records are descended into either by
//! the reader or by the explicit [`mcap::read::ChunkReader`] arm, so chunked
//! and unchunked recordings extract through the same path. `Writer::finish`
//! writes the summary section, footer and closing magic, so a clip is always a
//! complete, standalone MCAP.

use std::collections::HashMap;
use std::fs::{File, OpenOptions};
use std::io::BufWriter;
use std::os::unix::fs::FileExt;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use log::warn;
use mcap::records::Record;

use crate::tail::{ChannelDef, WindowPlan};

/// Outcome of an extraction: where the clip actually landed (`out_path`
/// carries a `_<n>` suffix when the desired name already existed) and the
/// copy counters, for logging.
#[derive(Debug, Default)]
pub struct ClipStats {
    pub out_path: PathBuf,
    pub extents_read: usize,
    pub messages_copied: u64,
    pub bytes_copied: u64,
}

/// Copy every message in `[start_ns, end_ns]` (inclusive bounds, matching the
/// split-based recorder) from the planned extents into a freshly created MCAP
/// at `out_path` (or a `_<n>`-suffixed sibling if that name is taken — see
/// [`create_clip_file`]). All-or-nothing: on success the clip is complete and
/// fsynced before this returns, so a caller may announce it as durably on
/// disk; on error the partly written file is removed, so a failed extraction
/// leaves nothing that could be mistaken for a clip.
pub fn extract_clip(
    plan: &WindowPlan,
    out_path: &Path,
    start_ns: u64,
    end_ns: u64,
) -> Result<ClipStats> {
    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating output dir {}", parent.display()))?;
    }
    let (out_file, out_path) = create_clip_file(out_path)?;
    copy_window(plan, out_file, out_path.clone(), start_ns, end_ns).inspect_err(|_| {
        // A failed copy must not leave a half-written, footer-less file that
        // looks like a clip; the error itself is what the caller reports.
        if let Err(e) = std::fs::remove_file(&out_path) {
            warn!("removing partial clip {}: {e}", out_path.display());
        }
    })
}

/// Assemble the clip into the freshly created `out_file`: register window
/// channels from the registry on first use, stream the planned extents,
/// finish and fsync. The caller removes `out_path` if this fails.
fn copy_window(
    plan: &WindowPlan,
    out_file: File,
    out_path: PathBuf,
    start_ns: u64,
    end_ns: u64,
) -> Result<ClipStats> {
    let mut clip = ClipWriter {
        writer: mcap::Writer::new(BufWriter::new(out_file)).context("opening mcap writer")?,
        channels: &plan.channels,
        out_ids: HashMap::new(),
        start_ns,
        end_ns,
        stats: ClipStats {
            out_path,
            ..ClipStats::default()
        },
    };

    if let Some(file) = &plan.file {
        for extent in &plan.extents {
            clip.stats.extents_read += 1;
            let mut buf = vec![0u8; extent.len as usize];
            file.read_exact_at(&mut buf, extent.offset)
                .with_context(|| {
                    format!("reading extent at {} (+{} B)", extent.offset, extent.len)
                })?;
            for rec in mcap::read::LinearReader::sans_magic(&buf) {
                match rec.context("parsing extent record")? {
                    // Only messages are copied out of the extent bytes. The
                    // clip's Schema/Channel records come from the registry
                    // (`output_channel_id`), not from here: the recording
                    // writes them where a topic first appears, which is
                    // usually far before the window and outside every
                    // planned extent.
                    Record::Message { header, data } => clip.copy_message(&header, &data)?,
                    Record::Chunk { header, data } => {
                        for rec in
                            mcap::read::ChunkReader::new(header, &data).context("opening chunk")?
                        {
                            if let Record::Message { header, data } =
                                rec.context("reading record inside chunk")?
                            {
                                clip.copy_message(&header, &data)?;
                            }
                        }
                    }
                    _ => {}
                }
            }
        }
    }

    let ClipWriter {
        mut writer, stats, ..
    } = clip;
    writer.finish().context("finalising output mcap")?;
    // `finish` can leave bytes in the BufWriter; flush them and fsync so the
    // clip is durably on disk before the caller publishes `Recorded`.
    writer
        .into_inner()
        .into_inner()
        .map_err(|e| anyhow::anyhow!("flushing clip: {e}"))?
        .sync_all()
        .context("syncing clip to disk")?;
    Ok(stats)
}

/// Create the clip file, never opening an existing one: a duplicate trigger
/// (same stamp and name) gets a `_<n>`-suffixed sibling instead of two
/// writers interleaving bytes into one file.
fn create_clip_file(desired: &Path) -> Result<(File, PathBuf)> {
    let stem = desired.file_stem().unwrap_or_default().to_string_lossy();
    let ext = desired
        .extension()
        .map(|e| format!(".{}", e.to_string_lossy()))
        .unwrap_or_default();
    let mut path = desired.to_path_buf();
    for n in 1.. {
        match OpenOptions::new().write(true).create_new(true).open(&path) {
            Ok(file) => {
                if path != desired {
                    warn!(
                        "clip {} already exists; writing {}",
                        desired.display(),
                        path.display()
                    );
                }
                return Ok((file, path));
            }
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists && n <= 1000 => {
                path = desired.with_file_name(format!("{stem}_{n}{ext}"));
            }
            Err(e) => return Err(e).with_context(|| format!("creating {}", path.display())),
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
    out_ids: HashMap<u16, u16>,
    start_ns: u64,
    end_ns: u64,
    stats: ClipStats,
}

impl ClipWriter<'_> {
    /// Write one message through if its `log_time` is in the window.
    fn copy_message(&mut self, header: &mcap::records::MessageHeader, data: &[u8]) -> Result<()> {
        if header.log_time < self.start_ns || header.log_time > self.end_ns {
            return Ok(());
        }
        let channel_id = self.output_channel_id(header.channel_id)?;
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
    /// stays stable however often a definition is registered.
    fn output_channel_id(&mut self, src_id: u16) -> Result<u16> {
        if let Some(id) = self.out_ids.get(&src_id) {
            return Ok(*id);
        }
        let def = self.channels.get(&src_id).with_context(|| {
            format!("message on channel {src_id} with no Channel record in the recording")
        })?;
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
        self.out_ids.insert(src_id, channel_id);
        Ok(channel_id)
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    use std::collections::BTreeMap;
    use std::sync::Arc;

    use crate::tail::tests::{scan_to_end, test_dir, write_recording, write_recording_opts};
    use crate::tail::{Extent, Tailer};

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
        let plan = tailer.plan_window(100, 200);
        let stats = extract_clip(&plan, &out, 100, 200)?;

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
        let plan = tailer.plan_window(20, 40);
        let stats = extract_clip(&plan, &out, 20, 40)?;

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
        let plan = tailer.plan_window(0, 100);
        let stats = extract_clip(&plan, &out, 0, 100)?;

        assert_eq!(stats.messages_copied, 0);
        assert!(read_clip(&out)?.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn duplicate_out_path_gets_a_suffixed_sibling() -> Result<()> {
        let root = test_dir("clip-dup")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10), ("/t", 20)])?;
        let tailer = tail_whole(&rec)?;
        let plan = tailer.plan_window(0, 100);

        let out = root.join("clip.mcap");
        let first = extract_clip(&plan, &out, 0, 100)?;
        let second = extract_clip(&plan, &out, 0, 100)?;

        assert_eq!(first.out_path, out);
        assert_eq!(second.out_path, root.join("clip_1.mcap"));
        assert_eq!(read_clip(&second.out_path)?, read_clip(&out)?);

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
        let plan = tailer.plan_window(0, 100);
        extract_clip(&plan, &out, 0, 100)?;

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
        let plan = tailer.plan_window(0, 100);

        // The recorder-restart scenario: the bag directory is wiped while a
        // window is still being cut. The plan's `Arc<File>` keeps the inode
        // alive, so the extraction must succeed against the deleted path.
        std::fs::remove_file(&rec)?;
        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100)?;

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
        let plan = tailer.plan_window(0, 100);

        // Shrink the recording under the plan (append-only violated — e.g. a
        // damaged filesystem). The extent read must fail, and the failure must
        // not leave a half-written clip behind.
        let last = plan.extents.last().expect("plan covers the recording");
        std::fs::OpenOptions::new()
            .write(true)
            .open(&rec)?
            .set_len(last.offset + last.len / 2)?;

        let out = root.join("clip.mcap");
        let err = extract_clip(&plan, &out, 0, 100).unwrap_err();
        assert!(
            format!("{err:#}").contains("reading extent"),
            "unexpected error: {err:#}"
        );
        assert!(!out.exists(), "partial clip must be removed on failure");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn message_on_unknown_channel_fails_and_removes_the_partial_clip() -> Result<()> {
        let root = test_dir("clip-nochannel")?;
        let rec = root.join("rec.mcap");
        write_recording(&rec, false, &[("/t", 10)])?;
        let tailer = tail_whole(&rec)?;
        let mut plan = tailer.plan_window(0, 100);
        plan.channels.clear();

        let out = root.join("clip.mcap");
        let err = extract_clip(&plan, &out, 0, 100).unwrap_err();
        assert!(
            format!("{err:#}").contains("no Channel record"),
            "unexpected error: {err:#}"
        );
        assert!(!out.exists(), "partial clip must be removed on failure");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn garbage_extent_bytes_fail_parsing_and_remove_the_partial_clip() -> Result<()> {
        let root = test_dir("clip-garbage")?;
        let junk = root.join("junk.bin");
        std::fs::write(&junk, [0xFFu8; 64])?;

        // A plan whose extent points at bytes that are not record-framed at
        // all — the index is corrupt or reading the wrong file.
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
        let err = extract_clip(&plan, &out, 0, u64::MAX).unwrap_err();
        assert!(
            format!("{err:#}").contains("parsing extent record"),
            "unexpected error: {err:#}"
        );
        assert!(!out.exists(), "partial clip must be removed on failure");

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
        let plan = tailer.plan_window(120, 180);
        let stats = extract_clip(&plan, &out, 120, 180)?;

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
        let plan = tailer.plan_window(20, 40);
        let stats = extract_clip(&plan, &out, 20, 40)?;

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
        let plan = tailer.plan_window(0, 100);
        assert!(
            plan.channels.values().all(|c| c.schema.is_none()),
            "schema_id 0 must resolve to no schema"
        );

        let out = root.join("clip.mcap");
        let stats = extract_clip(&plan, &out, 0, 100)?;
        assert_eq!(stats.messages_copied, 1);
        assert_eq!(read_clip(&out)?, vec![("/raw".to_string(), 10)]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn suffix_search_gives_up_after_1000_and_touches_nothing() -> Result<()> {
        let root = test_dir("clip-suffix-cap")?;
        let out = root.join("clip.mcap");
        std::fs::write(&out, b"existing")?;
        for n in 1..=1000 {
            std::fs::write(root.join(format!("clip_{n}.mcap")), b"existing")?;
        }

        let (tailer, _coverage) = Tailer::new();
        let plan = tailer.plan_window(0, 100);
        let err = extract_clip(&plan, &out, 0, 100).unwrap_err();
        assert!(
            format!("{err:#}").contains("creating"),
            "unexpected error: {err:#}"
        );
        // The failure happened before any file was created; the pre-existing
        // files are not ours to delete.
        assert_eq!(std::fs::read(&out)?, b"existing");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
