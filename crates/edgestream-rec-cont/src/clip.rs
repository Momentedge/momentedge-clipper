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
use std::fs::File;
use std::io::BufWriter;
use std::os::unix::fs::FileExt;
use std::path::Path;

use anyhow::{Context, Result};
use mcap::records::Record;

use crate::tail::{ChannelDef, WindowPlan};

/// Outcome of an extraction, for logging.
#[derive(Debug, Default)]
pub struct ClipStats {
    pub extents_read: usize,
    pub messages_copied: u64,
    pub bytes_copied: u64,
}

/// Copy every message in `[start_ns, end_ns]` (inclusive bounds, matching the
/// split-based recorder) from the planned extents into a freshly created MCAP
/// at `out_path`.
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
    let out_file =
        File::create(out_path).with_context(|| format!("creating {}", out_path.display()))?;
    let mut clip = ClipWriter {
        writer: mcap::Writer::new(BufWriter::new(out_file)).context("opening mcap writer")?,
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
            for rec in mcap::read::LinearReader::sans_magic(&buf) {
                match rec.context("parsing extent record")? {
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

    clip.writer.finish().context("finalising output mcap")?;
    Ok(clip.stats)
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
mod tests {
    use super::*;

    use std::sync::Arc;

    use crate::tail::Tailer;
    use crate::tail::tests::{scan_to_end, test_dir, write_recording};

    /// Read a finished clip back; `MessageStream` insists on a complete
    /// summary/footer/magic, so this doubles as a validity check.
    fn read_clip(path: &Path) -> Result<Vec<(String, u64)>> {
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
}
