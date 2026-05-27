//! Bulk clip extraction from the continuous rosbag2 recording.
//!
//! Given the `./record` directory of 5 s rosbag2 splits and a time window, this
//! assembles one output MCAP holding every message whose `log_time` falls in
//! `[start_ns, end_ns]`. It is a **direct copy**: `mcap::Writer::write` re-emits
//! each message's raw serialized bytes verbatim and deduplicates channels and
//! schemas by content, so the same topic spread across several splits collapses
//! to one channel in the output. The CDR message bodies are never decoded — the
//! only thing inspected is each record's `log_time`.
//!
//! Splits whose summary time range does not overlap the window are skipped
//! without being read. A split that cannot be summarised (e.g. the one rosbag2
//! still has open, with no footer yet) is scanned linearly instead, and a read
//! error part-way through is treated as end-of-file for that split — the
//! messages already copied are kept.

use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use log::{debug, warn};
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
) -> Result<ClipStats> {
    let mut inputs = list_mcap_files(record_dir)
        .with_context(|| format!("listing splits in {}", record_dir.display()))?;
    inputs.sort();

    if let Some(parent) = out_path.parent() {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("creating output dir {}", parent.display()))?;
    }
    let out_file =
        File::create(out_path).with_context(|| format!("creating {}", out_path.display()))?;
    let mut writer = mcap::Writer::new(BufWriter::new(out_file)).context("opening mcap writer")?;

    let mut stats = ClipStats::default();
    for input in &inputs {
        stats.files_scanned += 1;
        match copy_overlapping(input, &mut writer, start_ns, end_ns) {
            Ok(Some(file_stats)) => {
                stats.files_used += 1;
                stats.messages_copied += file_stats.0;
                stats.bytes_copied += file_stats.1;
            }
            Ok(None) => debug!("skip {} (outside window)", input.display()),
            Err(e) => warn!("skip {} ({e:#})", input.display()),
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
        let msg = match msg {
            Ok(m) => m,
            // A truncated tail (e.g. the still-open split) ends the scan; keep
            // everything read so far.
            Err(e) => {
                debug!("{}: stopping scan at {e}", path.display());
                break;
            }
        };
        if msg.log_time < start_ns || msg.log_time > end_ns {
            continue;
        }
        bytes += msg.data.len() as u64;
        writer
            .write(&msg)
            .with_context(|| format!("writing message from {}", path.display()))?;
        messages += 1;
    }

    Ok(Some((messages, bytes)))
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
