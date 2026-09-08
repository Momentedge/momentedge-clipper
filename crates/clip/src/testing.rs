//! The MCAP fixtures this crate's own tests are written against, published so a
//! consumer's tests build recordings the same way — and read the clips cut from
//! them back the same way.
//!
//! Every scan and cut test needs a recording of a known shape — a finished one,
//! a chunked one, a half-written one, a deliberately malformed one — and the
//! writers below are where those shapes are defined. They are the twin of the
//! scan: a fixture whose framing drifts from what [`crate::index`] expects makes
//! the test that uses it prove nothing, so a downstream crate that hand-rolls
//! its own copy of these writers puts that drift somewhere the format layer's
//! tests cannot catch it, and every shape has to be discovered twice.
//!
//! Compiled under this crate's own `cfg(test)` and, for a downstream test, under
//! the `test-support` feature. See that feature's note in `Cargo.toml`: it is a
//! dev-only opt-in, and a release build of a consumer compiles none of this.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use crossbeam_channel::Sender;

use crate::TimeSource;
use crate::index::{MAGIC, RecordingIndex, ScanProgress, ScanSeed, WindowPlan, op, scan_available};
use crate::manifest::{CutRequest, Planned, Producer, WindowCoverage};
use crate::trigger::{Stamp, Trigger, TriggerRecord};

/// The producer a test's clips are stamped with: a mode name no real binary
/// runs under, so a manifest written by a fixture is never mistaken for one a
/// recorder wrote.
pub const TEST_PRODUCER: Producer = Producer {
    program: "clipper",
    mode: "test",
};

/// The [`CutRequest`] a test cuts the window `[start_ns, end_ns]` on `source`
/// with.
///
/// A request derives its window from the trigger that asked for it, so a test
/// naming bounds directly gets a trigger built to resolve to exactly them: the
/// anchor sits at the window end, with the whole width as preroll. Tests about
/// the *trigger* build their own; this serves the many that only need some
/// window.
pub fn window_request(start_ns: u64, end_ns: u64, source: TimeSource) -> CutRequest {
    CutRequest::new(
        TEST_PRODUCER,
        Trigger {
            name: "test".to_string(),
            description: String::new(),
            trigger_time: Stamp { sec: 0, nanosec: 0 },
            preroll: end_ns.saturating_sub(start_ns),
            postroll: 0,
        },
        end_ns,
        source,
    )
}

/// The [`Planned`] facts of a window one recording covered end to end — what a
/// test staging a single segment out of a finished fixture recording is looking
/// at. Tests about the empty and short cases state their own.
pub fn planned_one_file() -> Planned {
    Planned {
        files: 1,
        coverage: WindowCoverage::Covered,
    }
}

/// Read a finished clip back as its `(topic, log_time)` pairs. `MessageStream`
/// insists on a complete summary/footer/magic, so this doubles as a validity
/// check: a clip that does not parse here was never publishable.
pub fn read_clip(path: &Path) -> Result<Vec<(String, u64)>> {
    let buf = std::fs::read(path)?;
    mcap::MessageStream::new(&buf)?
        .map(|msg| {
            let msg = msg?;
            Ok((msg.channel.topic.clone(), msg.log_time))
        })
        .collect()
}

/// Write a finished recording with one message per `(topic, log_time)`.
pub fn write_recording(path: &Path, chunked: bool, stamps: &[(&str, u64)]) -> Result<()> {
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
pub fn write_recording_opts(
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
                let schema = writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data")?;
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

/// A fresh scratch directory under the system temp dir, named for the test and
/// made unique by pid and nanosecond, so tests that run concurrently (and reruns
/// of a test that left its directory behind on failure) never collide.
pub fn test_dir(name: &str) -> Result<PathBuf> {
    let nanos = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
    let path = std::env::temp_dir().join(format!("clipper-{name}-{}-{nanos}", std::process::id()));
    std::fs::create_dir_all(&path)?;
    Ok(path)
}

/// Open `path` and index it past the 8 magic bytes — the state a tailer
/// reaches once it has verified the magic, and the setup every
/// single-recording scan test shares. The returned handle is a second
/// reference to the same file, for the scan and for length assertions.
pub fn index_file(path: &Path) -> Result<(RecordingIndex, Arc<File>)> {
    let file = Arc::new(File::open(path)?);
    let mut index = RecordingIndex::new(path.to_path_buf(), file.clone());
    index.offset = MAGIC.len() as u64;
    index.magic_ok = true;
    Ok((index, file))
}

/// The seed a resuming pass takes from `index`: its open extent and its
/// known trigger channels, plus `tap` — `None` for the timestamp-only scan
/// most tests drive, `Some((topic, tx))` for the trigger-tap ones. Public for
/// the tests that call [`scan_available`] a pass at a time rather than through
/// [`scan_passes`].
pub fn seed_with(index: &RecordingIndex, tap: Option<(&str, &Sender<TriggerRecord>)>) -> ScanSeed {
    ScanSeed {
        open: index.open,
        tap: tap.map(|(topic, tx)| (topic.to_string(), tx.clone())),
        trigger_channels: index.trigger_channels.clone(),
    }
}

/// Drive scan passes into `index` the way a tailer does — apply each delta,
/// advance the resume offset — until the recording ends, a pass faults, or a
/// pass makes no progress (the file stopped growing). Stays a `Result` only
/// because reading the file length can fail; the scan itself does not return
/// one. Stops on a fault without retrying — retry and backoff belong to the
/// caller driving the tail.
///
/// The general driver: [`scan_to_end`] is this with the tap disabled, and a
/// test watching a trigger topic passes `Some((topic, tx))`.
pub fn scan_passes(
    index: &mut RecordingIndex,
    file: &File,
    tap: Option<(&str, &Sender<TriggerRecord>)>,
) -> Result<ScanProgress> {
    loop {
        let file_len = file.metadata().context("stat of the scanned file")?.len();
        let offset = index.offset;
        let (delta, progress) = scan_available(file, offset, file_len, seed_with(index, tap));
        index.advance(delta, &progress);
        if progress.ended || progress.fault.is_some() || progress.offset == offset {
            return Ok(progress);
        }
    }
}

/// [`scan_passes`] with the trigger tap disabled: the timestamp-only scan.
pub fn scan_to_end(index: &mut RecordingIndex, file: &File) -> Result<ScanProgress> {
    scan_passes(index, file, None)
}

/// The single plan a one-recording test cuts from on the `log` domain — the
/// domain almost every test windows on. [`RecordingIndex::plan`] yields
/// `None` when no extent overlaps, which becomes an empty plan here.
pub fn plan_one(index: &RecordingIndex, start_ns: u64, end_ns: u64) -> WindowPlan {
    index
        .plan(start_ns, end_ns, TimeSource::Log)
        .unwrap_or_else(WindowPlan::empty)
}

/// A length-prefixed top-level record as the writer lays it down.
pub fn raw_record(opcode: u8, body: &[u8]) -> Vec<u8> {
    let mut rec = vec![opcode];
    rec.extend_from_slice(&(body.len() as u64).to_le_bytes());
    rec.extend_from_slice(body);
    rec
}

/// A conformant `Message` record body (22 fixed bytes + payload) whose
/// `publish_time` equals its `log_time` — the common case for tests that do
/// not exercise the log/publish split.
pub fn message_body(channel_id: u16, sequence: u32, log_time: u64, payload: &[u8]) -> Vec<u8> {
    message_body_pub(channel_id, sequence, log_time, log_time, payload)
}

/// A conformant `Message` record body with an independent `publish_time`,
/// for tests asserting the tail carries both stamps.
pub fn message_body_pub(
    channel_id: u16,
    sequence: u32,
    log_time: u64,
    publish_time: u64,
    payload: &[u8],
) -> Vec<u8> {
    let mut body = Vec::new();
    body.extend_from_slice(&channel_id.to_le_bytes());
    body.extend_from_slice(&sequence.to_le_bytes());
    body.extend_from_slice(&log_time.to_le_bytes());
    body.extend_from_slice(&publish_time.to_le_bytes());
    body.extend_from_slice(payload);
    body
}

/// A `Channel` record body (id, schema_id, topic, encoding, empty metadata).
pub fn channel_body(id: u16, schema_id: u16, topic: &str, encoding: &str) -> Vec<u8> {
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

/// A `Metadata` record body (name, then the key/value map as a
/// byte-length-prefixed run of length-prefixed strings) — the record shape a
/// `ros2 bag record` MCAP carries its own `rosbag2` metadata in, and the one a
/// clip's manifest is written as.
pub fn metadata_body(name: &str, entries: &[(&str, &str)]) -> Vec<u8> {
    fn put_str(out: &mut Vec<u8>, s: &str) {
        out.extend_from_slice(&(s.len() as u32).to_le_bytes());
        out.extend_from_slice(s.as_bytes());
    }
    let mut map = Vec::new();
    for (k, v) in entries {
        put_str(&mut map, k);
        put_str(&mut map, v);
    }
    let mut body = Vec::new();
    put_str(&mut body, name);
    body.extend_from_slice(&(map.len() as u32).to_le_bytes());
    body.extend_from_slice(&map);
    body
}

/// A `Schema` record body (id, name, encoding, length-prefixed data).
pub fn schema_body(id: u16, name: &str, encoding: &str, data: &[u8]) -> Vec<u8> {
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

/// An uncompressed `Chunk` record body wrapping `records` (each a raw
/// length-prefixed interior record), with a caller-supplied
/// `uncompressed_crc`. `mcap::read::ChunkReader` yields the interior
/// records as it walks and verifies the CRC only at the end of iteration,
/// so a deliberately wrong CRC lets a test absorb the messages and then
/// fail. `compression` is the chunk's algorithm string (empty for none);
/// an unknown string fails `ChunkReader` construction outright.
pub fn chunk_body(compression: &str, uncompressed_crc: u32, records: &[Vec<u8>]) -> Vec<u8> {
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
pub fn write_raw(path: &Path, records: &[Vec<u8>]) -> Result<()> {
    let mut bytes = MAGIC.to_vec();
    for rec in records {
        bytes.extend_from_slice(rec);
    }
    std::fs::write(path, bytes)?;
    Ok(())
}

/// A recording that is still being written: one schemaless channel and its
/// messages, with no DataEnd/Footer — exactly the shape a live tail sees.
pub fn write_unfinished_recording(path: &Path, topic: &str, stamps: &[u64]) -> Result<()> {
    let mut records = vec![raw_record(op::CHANNEL, &channel_body(1, 0, topic, "cdr"))];
    for (seq, t) in stamps.iter().enumerate() {
        records.push(raw_record(
            op::MESSAGE,
            &message_body(1, seq as u32, *t, b"payload"),
        ));
    }
    write_raw(path, &records)
}
