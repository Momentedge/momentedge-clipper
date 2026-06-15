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
//!   the tail; the default `record-continuous.sh` profile (fastwrite) is
//!   unchunked and pays no such cost.
//! * **Coverage watch** — the highest `log_time` seen plus an "ended" flag
//!   ([`Coverage`]); a trigger handler waits on it until the recording provably
//!   covers its window end.
//!
//! Only the 14-byte prefix of each top-level `Message` record is read during
//! the tail (channel id, sequence, `log_time`); message bodies are first
//! touched by the extraction. The same "decode only the timestamp" discipline
//! as the rest of the workspace, applied to file tailing.
//!
//! The recording is discovered as the newest `*.mcap` under the record dir and
//! re-discovered when the path stops pointing at the tailed inode (the record
//! script wipes and recreates the bag directory on restart). Re-discovery
//! resets the whole index — the old file's data is gone. Extractions already
//! holding the old file handle finish safely against the deleted inode.
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
use log::{info, warn};
use mcap::records::Record;

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

/// How many consecutive faulted scan passes [`Tailer::tail_file`] tolerates
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

/// How far the recording provably reaches: the highest message `log_time` the
/// tail has seen on disk, and whether the recording has ended (DataEnd/Footer
/// scanned — nothing more will ever appear).
///
/// "Provably" rests on an ordering assumption: messages land in the file in
/// (approximately) non-decreasing `log_time` order. rosbag2 has that shape —
/// one writer, `log_time` stamped at receive — up to millisecond-scale
/// interleaving between concurrent subscription callbacks, which the flush
/// and extraction latency in front of every cut dwarfs. A high-water mark at
/// or past a window end therefore implies the window's messages are on disk.
#[derive(Clone, Copy, Debug, Default)]
pub struct Coverage {
    pub high_water_ns: u64,
    pub ended: bool,
}

/// A snapshot for one clip: the open recording, the extents overlapping the
/// window (in file order), and the channel registry to map IDs with. `file` is
/// `None` while no recording has been discovered yet.
pub struct WindowPlan {
    pub file: Option<Arc<File>>,
    pub extents: Vec<Extent>,
    pub channels: HashMap<u16, ChannelDef>,
}

#[derive(Default)]
struct IndexState {
    file: Option<Arc<File>>,
    extents: Vec<Extent>,
    /// The extent still accumulating records at the end of the scanned region.
    /// Included in window plans — a window may end inside it.
    open: Option<Extent>,
    schemas: HashMap<u16, SchemaDef>,
    channels: HashMap<u16, ChannelDef>,
}

/// Shared tail state: the scanning thread feeds it, trigger handlers snapshot
/// it via [`Tailer::plan_window`] and wait on the coverage watch.
pub struct Tailer {
    state: Mutex<IndexState>,
    coverage: Arc<Watch<Coverage>>,
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
            Record::Channel(ch) => self.channels.push(RawChannel {
                id: ch.id,
                schema_id: ch.schema_id,
                topic: ch.topic,
                message_encoding: ch.message_encoding,
                metadata: ch.metadata,
            }),
            Record::Message { header, .. } => self.absorb_time(header.log_time),
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
        let mut sub = ScanDelta::default();
        for rec in mcap::read::ChunkReader::new(header, &data).context("opening chunk")? {
            sub.absorb_parsed(rec.context("reading record inside chunk")?);
        }
        self.schemas.extend(sub.schemas);
        self.channels.extend(sub.channels);
        self.high_water_ns = self.high_water_ns.max(sub.high_water_ns);
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
    /// A fresh tailer plus the coverage watch trigger handlers wait on.
    pub fn new() -> (Arc<Self>, Arc<Watch<Coverage>>) {
        let coverage = Arc::new(Watch::new(Coverage::default()));
        (
            Arc::new(Tailer {
                state: Mutex::new(IndexState::default()),
                coverage: coverage.clone(),
            }),
            coverage,
        )
    }

    /// Snapshot everything one clip needs for `[start_ns, end_ns]`.
    pub fn plan_window(&self, start_ns: u64, end_ns: u64) -> WindowPlan {
        let st = self.state.lock().unwrap();
        let extents = st
            .extents
            .iter()
            .chain(st.open.iter())
            .filter(|e| e.overlaps(start_ns, end_ns))
            .copied()
            .collect();
        WindowPlan {
            file: st.file.clone(),
            extents,
            channels: st.channels.clone(),
        }
    }

    /// Tail forever: discover the newest `*.mcap` under `record_dir`, scan it
    /// until it is replaced, start over. Blocking — run on its own thread.
    ///
    /// Returns only when [`Self::tail_file`] fails: a scan fault it could not
    /// retry past, a recording that is not an MCAP, or an IO error around the
    /// tailed file. The error names the file, and the supervisor exits the
    /// process for a restart — limping on would degrade every clip to a
    /// grace-timeout cut silently. A missing or empty record directory is not a fault: discovery
    /// idles until the continuous recording creates the bag dir, the
    /// documented startup state. `tail_file` returning `Ok` (a recorder
    /// restart) just loops back to re-discover the new file.
    pub fn run(&self, record_dir: &Path) -> Result<()> {
        loop {
            let path = loop {
                if let Some(p) = newest_mcap(record_dir) {
                    break p;
                }
                std::thread::sleep(DISCOVER_POLL);
            };
            info!("tailing {}", path.display());
            self.tail_file(&path)
                .with_context(|| format!("tailing {}", path.display()))?;
            info!("recording {} replaced; re-discovering", path.display());
        }
    }

    /// Point the shared state at a new recording, dropping everything known
    /// about a previous one.
    pub(crate) fn attach(&self, file: Arc<File>) {
        let mut st = self.state.lock().unwrap();
        *st = IndexState {
            file: Some(file),
            ..IndexState::default()
        };
        drop(st);
        self.coverage.send_replace(Coverage::default());
    }

    /// Scan one recording until the path stops referring to it (recorder
    /// restart → `Ok`), alternating incremental passes with short sleeps.
    ///
    /// The index is attached once here and never wiped while this recording is
    /// tailed: a scan fault does not re-attach or rescan from scratch. A fault
    /// leaves the partial delta applied and reports the faulted offset, so a
    /// retry resumes exactly there (see [`Self::scan_available`]). Consecutive
    /// faults are bounded by [`MAX_SCAN_FAULTS`] with backoff growing from
    /// [`DISCOVER_POLL`] to [`SCAN_BACKOFF_CAP`]; a fault-free pass resets the
    /// count. The backoff is slept in `DISCOVER_POLL` increments while polling
    /// [`replaced`], so a recorder restart during backoff is recovery (`Ok`),
    /// not a continued fault. Exhausting the retries returns the last fault,
    /// named with the path, offset, and attempt count: the same byte faulted
    /// every retry, so the file is genuinely stuck and the process exits for a
    /// supervisor to restart rather than degrading every clip silently.
    ///
    /// A `NotFound` on open means the file vanished between discovery and open
    /// (the record script wiping the bag dir) — treated as a replacement, not
    /// an error. A magic mismatch stays fatal: an append-only file whose first
    /// eight bytes are wrong can never become a valid MCAP.
    fn tail_file(&self, path: &Path) -> Result<()> {
        let file = match File::open(path) {
            Ok(f) => Arc::new(f),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(()),
            Err(e) => return Err(e).with_context(|| format!("opening {}", path.display())),
        };
        self.attach(file.clone());

        // The writer may not have put the 8 magic bytes on disk yet.
        while file_len(&file)? < MAGIC.len() as u64 {
            if replaced(path, &file)? {
                return Ok(());
            }
            std::thread::sleep(TAIL_POLL);
        }
        let mut magic = [0u8; 8];
        file.read_exact_at(&mut magic, 0)?;
        if magic != MAGIC {
            bail!("{} is not an MCAP file", path.display());
        }

        let mut offset = MAGIC.len() as u64;
        let mut consecutive_faults = 0u32;
        loop {
            let progress = self.scan_available(&file, offset, file_len(&file)?);
            let made_progress = progress.offset != offset;
            offset = progress.offset;

            if let Some(fault) = progress.fault {
                consecutive_faults += 1;
                if consecutive_faults >= MAX_SCAN_FAULTS {
                    return Err(fault).with_context(|| {
                        format!(
                            "scan of {} faulted at offset {offset} on {consecutive_faults} \
                             consecutive passes; giving up",
                            path.display()
                        )
                    });
                }
                // Back off (doubling from DISCOVER_POLL, capped) before
                // retrying at the same offset, but watch for a replacement
                // throughout: a restart mid-backoff is recovery, not a fault.
                let backoff = backoff_for(consecutive_faults);
                warn!(
                    "scan of {} faulted at offset {offset} ({fault:#}); \
                     retry {consecutive_faults}/{MAX_SCAN_FAULTS} after {backoff:?}",
                    path.display()
                );
                if self.replaced_during(path, &file, backoff)? {
                    return Ok(());
                }
                continue;
            }

            consecutive_faults = 0;

            if progress.ended {
                // DataEnd/Footer scanned: the recording is complete. Nothing
                // more will appear; wait for the next recording to replace it.
                info!("recording {} ended (footer on disk)", path.display());
                while !replaced(path, &file)? {
                    std::thread::sleep(DISCOVER_POLL);
                }
                return Ok(());
            }
            if !made_progress {
                if replaced(path, &file)? {
                    return Ok(());
                }
                std::thread::sleep(TAIL_POLL);
            }
        }
    }

    /// Sleep up to `total`, in [`DISCOVER_POLL`] increments, returning early
    /// `true` as soon as `path` no longer refers to `file` (recorder restart).
    /// Lets a fault backoff treat a replacement during the wait as recovery.
    fn replaced_during(&self, path: &Path, file: &File, total: Duration) -> Result<bool> {
        let mut waited = Duration::ZERO;
        while waited < total {
            if replaced(path, file)? {
                return Ok(true);
            }
            let step = DISCOVER_POLL.min(total - waited);
            std::thread::sleep(step);
            waited += step;
        }
        replaced(path, file)
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
        let mut delta = ScanDelta {
            open: self.state.lock().unwrap().open,
            ..ScanDelta::default()
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
                    // Decode only the 14-byte prefix: channel_id u16,
                    // sequence u32, log_time u64 (all LE). The body stays
                    // untouched until extraction.
                    if len >= 14 {
                        let mut prefix = [0u8; 14];
                        if let Err(e) = file.read_exact_at(&mut prefix, offset + 9) {
                            fault = Some(anyhow::Error::new(e).context(format!(
                                "reading message prefix at {offset}; framing desynchronised?"
                            )));
                            break;
                        }
                        delta.absorb_time(u64::from_le_bytes(prefix[6..14].try_into().unwrap()));
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

        self.apply(delta, ended);
        ScanProgress {
            offset,
            ended,
            fault,
        }
    }

    /// Publish one pass's delta: registry inserts (channels resolve their
    /// schema against the registry as updated by this pass), extent appends,
    /// then the coverage watch.
    ///
    /// The MCAP spec requires a Schema record to appear before any Channel
    /// referencing it, so a channel's schema is always complete on disk — and
    /// thus in the registry — by the pass that consumes the channel record.
    /// A file violating that order still resolves when both records land in
    /// one pass (schemas apply first); otherwise the channel degrades to
    /// schemaless instead of failing, where the reference reader hard-errors
    /// (`McapError::UnknownSchema`).
    fn apply(&self, delta: ScanDelta, ended: bool) {
        let mut st = self.state.lock().unwrap();
        for (id, schema) in delta.schemas {
            st.schemas.insert(id, schema);
        }
        for raw in delta.channels {
            let schema = (raw.schema_id != 0)
                .then(|| st.schemas.get(&raw.schema_id).cloned())
                .flatten();
            st.channels.insert(
                raw.id,
                ChannelDef {
                    topic: raw.topic,
                    message_encoding: raw.message_encoding,
                    metadata: raw.metadata,
                    schema,
                },
            );
        }
        st.extents.extend(delta.closed);
        st.open = delta.open;
        drop(st);

        self.coverage.send_if_modified(|c| {
            let mut changed = false;
            if delta.high_water_ns > c.high_water_ns {
                c.high_water_ns = delta.high_water_ns;
                changed = true;
            }
            if ended && !c.ended {
                c.ended = true;
                changed = true;
            }
            changed
        });
    }
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

/// The newest `*.mcap` directly under `dir` by modification time — the file
/// rosbag2 is writing. `None` while the directory or file does not exist yet.
fn newest_mcap(dir: &Path) -> Option<PathBuf> {
    std::fs::read_dir(dir)
        .ok()?
        .filter_map(|e| Some(e.ok()?.path()))
        .filter(|p| p.extension().is_some_and(|e| e == "mcap"))
        .max_by_key(|p| std::fs::metadata(p).and_then(|m| m.modified()).ok())
}

/// Whether `path` no longer refers to the open `file` (deleted or recreated —
/// the recorder was restarted into a fresh bag directory).
fn replaced(path: &Path, file: &File) -> Result<bool> {
    let by_path = match std::fs::metadata(path) {
        Ok(m) => m,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(true),
        Err(e) => return Err(e).with_context(|| format!("stat {}", path.display())),
    };
    let by_fd = file.metadata().context("stat of tailed file")?;
    Ok((by_path.dev(), by_path.ino()) != (by_fd.dev(), by_fd.ino()))
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
        assert!(cov.ended);

        let plan = tailer.plan_window(150, 350);
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
        let plan = tailer.plan_window(0, 100);
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

        assert!(tailer.plan_window(300, 500).extents.is_empty());
        assert!(!tailer.plan_window(150, 500).extents.is_empty());

        // Inclusive boundaries, exactly at the extent's min/max (100, 200):
        // a window touching a bound by one nanosecond still plans the extent.
        assert!(!tailer.plan_window(200, 500).extents.is_empty());
        assert!(tailer.plan_window(201, 500).extents.is_empty());
        assert!(!tailer.plan_window(0, 100).extents.is_empty());
        assert!(tailer.plan_window(0, 99).extents.is_empty());

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
            ],
        )?;

        let (tailer, _coverage) = Tailer::new();
        let file = File::open(&path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let plan = tailer.plan_window(0, u64::MAX);
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

        let (tailer, _coverage) = Tailer::new();
        let err = tailer.tail_file(&path).unwrap_err();
        assert!(
            err.to_string().contains("not an MCAP file"),
            "unexpected error: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn tail_file_on_a_missing_path_is_not_an_error() -> Result<()> {
        let root = test_dir("missing")?;
        let path = root.join("gone.mcap");

        // The newest-mcap discovery can race the record script wiping the bag
        // dir: the file vanishes between discovery and open. A NotFound on
        // open is the same as a replacement — Ok(()), re-discover — not a fault.
        let (tailer, _coverage) = Tailer::new();
        tailer.tail_file(&path)?;

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
        let file = File::open(&path)?;
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
            !tailer.plan_window(50, 250).extents.is_empty(),
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
        let file = File::open(&path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "the bad chunk and the good records after it are all consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 42);

        let plan = tailer.plan_window(0, 100);
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
        let file = File::open(&path)?;
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

        let plan = tailer.plan_window(0, u64::MAX);
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
        let file = File::open(&path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 300);
        assert!(
            tailer.plan_window(0, u64::MAX).channels.contains_key(&9),
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
        let file = File::open(&path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "all records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 55);

        let plan = tailer.plan_window(0, 100);
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
        let file = File::open(&path)?;
        let progress = scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(
            progress.offset,
            file.metadata()?.len(),
            "both records consumed"
        );
        assert_eq!(coverage.get().high_water_ns, 42);

        let plan = tailer.plan_window(0, 100);
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
        let file = File::open(&path)?;
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
        let plan = tailer.plan_window(40, 60);
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

        let plan = tailer.plan_window(0, u64::MAX);
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
    fn attach_resets_index_registry_and_coverage() -> Result<()> {
        let root = test_dir("reattach")?;
        let path = root.join("rec.mcap");
        write_recording(&path, false, &[("/a", 400)])?;

        let (tailer, coverage) = Tailer::new();
        let file = Arc::new(File::open(&path)?);
        tailer.attach(file.clone());
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;
        assert_eq!(coverage.get().high_water_ns, 400);
        assert!(!tailer.plan_window(0, u64::MAX).extents.is_empty());

        // The recorder restarted into a fresh file: everything known about
        // the old one is gone, including its coverage.
        let empty = root.join("empty.mcap");
        std::fs::write(&empty, b"")?;
        tailer.attach(Arc::new(File::open(&empty)?));

        let cov = coverage.get();
        assert_eq!(cov.high_water_ns, 0);
        assert!(!cov.ended);
        let plan = tailer.plan_window(0, u64::MAX);
        assert!(plan.extents.is_empty());
        assert!(plan.channels.is_empty());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn replaced_detects_deletion_and_recreation() -> Result<()> {
        let root = test_dir("replaced")?;
        let path = root.join("rec.mcap");
        std::fs::write(&path, b"x")?;
        let file = File::open(&path)?;

        assert!(!replaced(&path, &file)?);
        std::fs::remove_file(&path)?;
        assert!(replaced(&path, &file)?, "deleted path means replaced");
        std::fs::write(&path, b"y")?;
        assert!(
            replaced(&path, &file)?,
            "a recreated file is a different inode"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn newest_mcap_picks_the_latest_and_ignores_non_mcap() -> Result<()> {
        let root = test_dir("discover")?;
        assert_eq!(newest_mcap(&root.join("missing")), None);
        assert_eq!(newest_mcap(&root), None, "no mcap yet");

        let old = root.join("old.mcap");
        std::fs::write(&old, b"")?;
        File::options()
            .write(true)
            .open(&old)?
            .set_modified(SystemTime::now() - Duration::from_secs(60))?;
        let newer = root.join("new.mcap");
        std::fs::write(&newer, b"")?;
        std::fs::write(root.join("note.txt"), b"")?; // newest mtime, wrong extension

        assert_eq!(newest_mcap(&root), Some(newer));

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
        let file = File::open(&path)?;
        scan_to_end(&tailer, &file, MAGIC.len() as u64)?;

        let plan = tailer.plan_window(0, 100);
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
        let handle = std::thread::spawn(move || runner.run(&dir));

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
            msg.contains("framing desynchronised") && msg.contains("faulted at offset"),
            "the error chain must name the desync and offset: {msg}"
        );

        // The good prefix survived every retry — the index was never wiped.
        let cov = coverage.get();
        assert_eq!(cov.high_water_ns, 200, "the prefix's coverage survived");
        assert!(
            !tailer.plan_window(50, 250).extents.is_empty(),
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
        let handle = std::thread::spawn(move || runner.run(&dir));

        // While run() is backing off over the poisoned file, replace it with a
        // finished good recording: the restart mid-backoff is recovery, and the
        // tail discovers and indexes the replacement.
        std::thread::sleep(Duration::from_millis(500));
        std::fs::remove_file(&poisoned)?;
        let good = root.join("b.mcap");
        write_recording(&good, false, &[("/t", 1_000)])?;

        // Poll the coverage watch until the replacement is fully indexed.
        let deadline = std::time::Instant::now() + Duration::from_secs(10);
        loop {
            {
                let cov = coverage.get();
                if cov.high_water_ns == 1_000 && cov.ended {
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
}
