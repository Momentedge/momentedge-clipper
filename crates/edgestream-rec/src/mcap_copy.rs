//! Bulk clip extraction from the continuous rosbag2 recording.
//!
//! Given the `./record` directory of 5 s rosbag2 splits and a time window, this
//! assembles one output MCAP holding every message whose `log_time` falls in
//! `[start_ns, end_ns]`. It is a **direct copy** of message payload bytes: source
//! schemas/channels are remapped into the output writer by content, then each
//! message is emitted with its raw serialized body. The CDR message bodies are
//! never decoded — the only thing inspected is each record's `log_time`.
//!
//! Splits whose summary time range does not overlap the window are skipped
//! without being read. Extraction is normally bounded by the `write_split`
//! event's closed file, so the still-open split is not scanned. Read/write
//! errors from closed input files are returned to the trigger handler rather
//! than being reported as successful clips.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use anyhow::{Context, Result};
use log::debug;
use memmap2::Mmap;

/// Outcome of an extraction, for logging.
#[derive(Debug, Default)]
pub struct ClipStats {
    pub files_scanned: usize,
    pub files_used: usize,
    pub messages_copied: u64,
    pub bytes_copied: u64,
}

/// Copy every message in `[start_ns, end_ns]` from the `*.mcap` splits in
/// `record_dir` into a freshly created MCAP at `out_path`.
pub fn extract_clip(
    record_dir: &Path,
    out_path: &Path,
    start_ns: u64,
    end_ns: u64,
    closed_through: Option<&Path>,
) -> Result<ClipStats> {
    let mut inputs = list_mcap_files(record_dir)
        .with_context(|| format!("listing splits in {}", record_dir.display()))?;
    sort_by_modified_time(&mut inputs)?;
    if let Some(closed_file) = closed_through {
        truncate_after_closed_file(&mut inputs, closed_file)?;
    }

    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating output dir {}", parent.display()))?;
    }
    let out_file =
        File::create(out_path).with_context(|| format!("creating {}", out_path.display()))?;
    let mut writer = mcap::Writer::new(BufWriter::new(out_file)).context("opening mcap writer")?;

    let mut stats = ClipStats::default();
    let mut channels = HashMap::new();
    for input in &inputs {
        stats.files_scanned += 1;
        match copy_overlapping(input, &mut writer, &mut channels, start_ns, end_ns) {
            Ok(Some(file_stats)) => {
                stats.files_used += 1;
                stats.messages_copied += file_stats.0;
                stats.bytes_copied += file_stats.1;
            }
            Ok(None) => debug!("skip {} (outside window)", input.display()),
            Err(e) => return Err(e).with_context(|| format!("copying {}", input.display())),
        }
    }

    writer.finish().context("finalising output mcap")?;
    Ok(stats)
}

/// Copy the in-window messages of one split. Returns `Ok(None)` when the split's
/// summarised time range proves it cannot overlap the window, or
/// `Ok(Some((messages, bytes)))` with what was copied.
fn copy_overlapping(
    path: &Path,
    writer: &mut mcap::Writer<BufWriter<File>>,
    channels: &mut HashMap<mcap::Channel<'static>, u16>,
    start_ns: u64,
    end_ns: u64,
) -> Result<Option<(u64, u64)>> {
    let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    // SAFETY: the split is a regular file we only read; if rosbag2 truncates or
    // grows it under us the worst case is a read error caught below.
    let mmap = unsafe { Mmap::map(&file) }.with_context(|| format!("mmap {}", path.display()))?;

    // Cheap rejection: a complete split carries a summary with its time span; if
    // that span is provably disjoint from the window, skip without reading on.
    let provably_outside = match mcap::Summary::read(&mmap) {
        Ok(Some(summary)) => summary
            .stats
            .map(|s| s.message_end_time < start_ns || s.message_start_time > end_ns)
            .unwrap_or(false),
        _ => false,
    };
    if provably_outside {
        return Ok(None);
    }

    let mut messages = 0u64;
    let mut bytes = 0u64;
    for msg in mcap::MessageStream::new(&mmap).context("reading message stream")? {
        let msg = msg.with_context(|| format!("reading message from {}", path.display()))?;
        if msg.log_time < start_ns || msg.log_time > end_ns {
            continue;
        }
        let channel_id = output_channel_id(writer, channels, msg.channel.as_ref())
            .with_context(|| format!("mapping channel {}", msg.channel.topic))?;
        bytes += msg.data.len() as u64;
        writer
            .write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence: msg.sequence,
                    log_time: msg.log_time,
                    publish_time: msg.publish_time,
                },
                msg.data.as_ref(),
            )
            .with_context(|| format!("writing message from {}", path.display()))?;
        messages += 1;
    }

    Ok(Some((messages, bytes)))
}

/// Map one input channel into the output file and cache the resulting output ID.
/// MCAP split files can independently reuse numeric schema/channel IDs, so the
/// output writer must assign its own IDs by content before messages are written.
fn output_channel_id(
    writer: &mut mcap::Writer<BufWriter<File>>,
    channels: &mut HashMap<mcap::Channel<'static>, u16>,
    channel: &mcap::Channel<'static>,
) -> Result<u16> {
    if let Some(channel_id) = channels.get(channel) {
        return Ok(*channel_id);
    }

    let schema_id = match channel.schema.as_ref() {
        Some(schema) => writer
            .add_schema(&schema.name, &schema.encoding, schema.data.as_ref())
            .with_context(|| format!("adding schema {}", schema.name))?,
        None => 0,
    };
    let channel_id = writer
        .add_channel(
            schema_id,
            &channel.topic,
            &channel.message_encoding,
            &channel.metadata,
        )
        .with_context(|| format!("adding channel {}", channel.topic))?;
    channels.insert(channel.clone(), channel_id);
    Ok(channel_id)
}

/// All `*.mcap` files directly under `dir` (non-recursive). A missing directory
/// yields an empty list rather than an error — the recorder may start before
/// the first split lands.
fn list_mcap_files(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut out = Vec::new();
    let rd = match std::fs::read_dir(dir) {
        Ok(rd) => rd,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(out),
        Err(e) => return Err(e).with_context(|| format!("read_dir {}", dir.display())),
    };
    for entry in rd {
        let path = entry?.path();
        if path.extension().is_some_and(|e| e == "mcap") {
            out.push(path);
        }
    }
    Ok(out)
}

/// Order splits by file modification time so the still-open split — the one
/// rosbag2 is currently writing — sorts last. rosbag2 writes splits
/// sequentially, so modification time reflects write order without depending on
/// the `<bag>_<n>.mcap` naming convention (a lexicographic sort of which places
/// `_10` before `_2`). [`truncate_after_closed_file`] relies on this ordering to
/// find the boundary at the closed split.
fn sort_by_modified_time(inputs: &mut [PathBuf]) -> Result<()> {
    let mut modified: HashMap<PathBuf, SystemTime> = HashMap::new();
    for path in inputs.iter() {
        let mtime = std::fs::metadata(path)
            .and_then(|m| m.modified())
            .with_context(|| format!("reading modification time of {}", path.display()))?;
        modified.insert(path.clone(), mtime);
    }
    inputs.sort_by(|a, b| modified[a].cmp(&modified[b]).then_with(|| a.cmp(b)));
    Ok(())
}

/// Keep only files up to and including the split reported closed by rosbag2.
/// Newer files include the active open split, which has no complete footer yet.
/// Relies on [`sort_by_modified_time`] having ordered `inputs` by write time.
fn truncate_after_closed_file(inputs: &mut Vec<PathBuf>, closed_file: &Path) -> Result<()> {
    let closed_name = closed_file
        .file_name()
        .with_context(|| format!("closed split has no filename: {}", closed_file.display()))?;
    let closed_idx = inputs
        .iter()
        .position(|path| path == closed_file || path.file_name() == Some(closed_name))
        .with_context(|| {
            format!(
                "closed split {} was not found in the record directory",
                closed_file.display()
            )
        })?;
    inputs.truncate(closed_idx + 1);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::BTreeMap;
    use std::time::{Duration, SystemTime, UNIX_EPOCH};

    #[test]
    fn extract_clip_remaps_conflicting_source_channel_ids() -> Result<()> {
        let root = test_dir("remap")?;
        let record_dir = root.join("record");
        std::fs::create_dir_all(&record_dir)?;
        let first = record_dir.join("split_0.mcap");
        let second = record_dir.join("split_1.mcap");
        write_input(&first, "/topic_a", 10, b"a")?;
        write_input(&second, "/topic_b", 20, b"b")?;

        let out = root.join("out/clip.mcap");
        let stats = extract_clip(&record_dir, &out, 0, 30, Some(&second))?;

        assert_eq!(stats.files_scanned, 2);
        assert_eq!(stats.files_used, 2);
        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_topics_and_times(&out)?,
            vec![("/topic_a".to_string(), 10), ("/topic_b".to_string(), 20)]
        );
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn extract_clip_orders_splits_by_modified_time() -> Result<()> {
        // Ordering follows modification time, not the filename. Lexicographically
        // "rec_10" sorts before "rec_2", but rec_2 is written (and stamped)
        // earlier, so it is the older split. With rec_10 as the closing split the
        // window spans both, and rec_2 — older, in the preroll — must be kept
        // rather than truncated away.
        let root = test_dir("order")?;
        let record_dir = root.join("record");
        std::fs::create_dir_all(&record_dir)?;
        let older = record_dir.join("rec_2.mcap");
        let newer = record_dir.join("rec_10.mcap");
        write_input(&older, "/topic", 2, b"x")?;
        write_input(&newer, "/topic", 10, b"y")?;
        set_mtime(&older, UNIX_EPOCH + Duration::from_secs(1_000))?;
        set_mtime(&newer, UNIX_EPOCH + Duration::from_secs(2_000))?;

        let out = root.join("out/clip.mcap");
        let stats = extract_clip(&record_dir, &out, 0, 20, Some(&newer))?;

        assert_eq!(stats.files_used, 2);
        assert_eq!(stats.messages_copied, 2);
        assert_eq!(
            read_topics_and_times(&out)?,
            vec![("/topic".to_string(), 2), ("/topic".to_string(), 10)]
        );
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn extract_clip_returns_closed_split_errors() -> Result<()> {
        let root = test_dir("error")?;
        let record_dir = root.join("record");
        std::fs::create_dir_all(&record_dir)?;
        let corrupt = record_dir.join("split_0.mcap");
        std::fs::write(&corrupt, b"not an mcap")?;

        let out = root.join("out/clip.mcap");
        let err = extract_clip(&record_dir, &out, 0, 30, Some(&corrupt)).unwrap_err();

        assert!(format!("{err:#}").contains("copying"));
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    fn write_input(path: &Path, topic: &str, log_time: u64, data: &[u8]) -> Result<()> {
        let mut writer = mcap::Writer::new(BufWriter::new(File::create(path)?))?;
        let schema_id = writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data")?;
        let channel_id = writer.add_channel(schema_id, topic, "cdr", &BTreeMap::new())?;
        writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id,
                sequence: 0,
                log_time,
                publish_time: log_time,
            },
            data,
        )?;
        writer.finish()?;
        Ok(())
    }

    fn set_mtime(path: &Path, when: SystemTime) -> Result<()> {
        File::options().write(true).open(path)?.set_modified(when)?;
        Ok(())
    }

    fn read_topics_and_times(path: &Path) -> Result<Vec<(String, u64)>> {
        let file = File::open(path)?;
        let mmap = unsafe { Mmap::map(&file)? };
        mcap::MessageStream::new(&mmap)?
            .map(|msg| {
                let msg = msg?;
                Ok((msg.channel.topic.clone(), msg.log_time))
            })
            .collect()
    }

    fn test_dir(name: &str) -> Result<PathBuf> {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
        let path = std::env::temp_dir().join(format!(
            "edgestream-rec-{name}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&path)?;
        Ok(path)
    }
}
