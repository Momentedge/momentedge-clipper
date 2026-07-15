//! Minimal example: write a **chunked, zstd-compressed MCAP file that clipper
//! can still tail live**. This is the second producer path that satisfies
//! clipper's tailability contract — the first is
//! `examples/custom-mcap-writer`'s unchunked output — and this program's only
//! job is to show the `mcap` crate configuration that makes it work.
//!
//! ## Buffered chunks: why this is tailable
//!
//! A producer clipper can tail live appends complete top-level records only
//! and never seeks back to rewrite one (`ARCHITECTURE.md`, "Tailing a live
//! MCAP"). The `mcap` crate's *default* chunked writer violates that: it
//! writes each `Chunk` header with a placeholder length (`u64::MAX`) and
//! seeks back to patch in the true length when the chunk closes, so the file
//! is unparseable mid-write. `use_chunks(true)` combined with
//! `disable_seeking(true)` selects the crate's buffered chunk mode instead
//! ([`chunked_writer`]): each chunk is assembled in an in-memory buffer and
//! appended as one *complete* `Chunk` record — real length, real CRC — so no
//! placeholder ever reaches disk and the writer never seeks back. The
//! contract is "append complete records", not "never chunk". The price is
//! visibility latency: a tailer sees a chunk's messages only when the chunk
//! closes (see "Chunk-close latency" in the README for the `--grace-secs`
//! guidance).
//!
//! ## Time model: everything lives on log time
//!
//! Every record carries `publish_time = log_time` (wall clock at write).
//! There is no capture-time machinery here — owning `publish_time` as a
//! capture stamp is `examples/custom-mcap-writer`'s story. The synthetic
//! trigger's JSON payload leaves `trigger_time` at `{sec: 0, nanosec: 0}`:
//! under `--interface mcap` the field is inert — clipper anchors the clip
//! window on the trigger *record's own* MCAP stamp (here its `log_time`) —
//! and a non-zero value in that cell is a mis-anchoring hazard clipper
//! rejects (see "Why `trigger_time` is zero" in custom-mcap-writer's README).
//!
//! Run: `cargo run -p chunked-mcap-writer -- --out demo.mcap`
//! (writes `chunked.mcap` until Ctrl+C by default; `--out` is the only flag).

use std::collections::BTreeMap;
use std::io::{Seek, Write};
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::Parser;
use serde::Serialize;

/// Uncompressed bytes a chunk accumulates before it closes and is appended to
/// the file as one complete record. The gate is on *uncompressed* accumulated
/// record bytes, so the chunk-close cadence — the tailer's visibility latency
/// — is independent of the codec. 4 KiB keeps that cadence under a second
/// (~0.8 s) at this program's data rate (see "Chunk-close latency" in the
/// README for the arithmetic).
const CHUNK_SIZE: u64 = 4 * 1024;

/// Sample period of the `/pose` channel: one message every 20 ms (50 Hz).
const PERIOD_MS: u64 = 20;

/// One synthetic trigger is emitted this often; the first fires this long
/// after startup.
const TRIGGER_PERIOD: Duration = Duration::from_secs(10);

/// Nanoseconds either side of the trigger's anchor its clip window keeps —
/// wider than the chunk-close cadence, so a cut around a live trigger always
/// spans closed chunks with real data.
const TRIGGER_PREROLL_NS: u64 = 3_000_000_000;
const TRIGGER_POSTROLL_NS: u64 = 3_000_000_000;

/// The topic clipper's MCAP interface taps for triggers
/// (`crates/clipper/src/interface.rs`).
const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";

/// CLI: where to write. Everything else — rate, chunk size, trigger cadence —
/// is a hardcoded const above; this example demonstrates a writer
/// configuration, not a tuning surface.
#[derive(Parser)]
#[command(
    about = "Write a tailable chunked+zstd MCAP file with the mcap crate (buffered chunks, no ROS)."
)]
struct Config {
    /// Output MCAP file path.
    #[arg(long, default_value = "chunked.mcap")]
    out: PathBuf,
}

/// A plain struct, serialized to JSON for the `/pose` channel.
#[derive(Serialize)]
struct Pose {
    x: f64,
    y: f64,
    theta: f64,
}

/// JSON Schema for `/pose`, so a viewer can render the struct.
const POSE_SCHEMA: &[u8] = br#"{
  "type": "object",
  "properties": {
    "x": { "type": "number" },
    "y": { "type": "number" },
    "theta": { "type": "number" }
  }
}"#;

/// `builtin_interfaces/Time` flattened to its two fields — the JSON shape
/// `crates/clipper/src/trigger.rs::Stamp` derives `Deserialize` for.
#[derive(Serialize)]
struct StampMsg {
    sec: i32,
    nanosec: u32,
}

/// The JSON payload for `/events/momentedge/trigger` — field-for-field the
/// shape `crates/clipper/src/trigger.rs::Trigger` decodes. `trigger_time`
/// stays `{sec: 0, nanosec: 0}`: inert under the mcap interface, where the
/// anchor is the trigger record's own MCAP stamp (see the module docs).
#[derive(Serialize)]
struct TriggerMsg {
    name: String,
    description: String,
    trigger_time: StampMsg,
    preroll: u64,
    postroll: u64,
}

/// Build the writer this example exists to demonstrate. `use_chunks(true)` +
/// `disable_seeking(true)` selects the `mcap` crate's buffered chunk mode:
/// each chunk is assembled in an in-memory buffer and appended as one
/// complete `Chunk` record (real length, real CRC), so the output is
/// zstd-compressed yet still satisfies the tailability contract — no
/// placeholder length ever reaches the sink and the writer never seeks back.
fn chunked_writer<W: Write + Seek>(sink: W) -> Result<mcap::Writer<W>> {
    Ok(mcap::WriteOptions::new()
        .use_chunks(true)
        .disable_seeking(true)
        .compression(Some(mcap::Compression::Zstd))
        .chunk_size(Some(CHUNK_SIZE))
        .create(sink)?)
}

/// Register the two channels, returning `(pose_id, trigger_id)`. Ids are
/// assigned per file, so this runs once before the loop.
fn register<W: Write + Seek>(writer: &mut mcap::Writer<W>) -> Result<(u16, u16)> {
    let schema_id = writer.add_schema("Pose", "jsonschema", POSE_SCHEMA)?;
    let pose_id = writer.add_channel(schema_id, "/pose", "json", &BTreeMap::new())?;
    // Schemaless, schema id 0 — clipper's MCAP interface decodes triggers by
    // `message_encoding` alone, not a schema (see crates/clipper/src/decode.rs).
    let trigger_id = writer.add_channel(0, TRIGGER_TOPIC, "json", &BTreeMap::new())?;
    Ok((pose_id, trigger_id))
}

/// Write one `/pose` sample with `publish_time = log_time = now_ns` — every
/// record this program writes lives on log time (see the module docs).
fn write_pose<W: Write + Seek>(
    writer: &mut mcap::Writer<W>,
    channel_id: u16,
    seq: u32,
    now_ns: u64,
) -> Result<()> {
    let a = seq as f64 * 0.05;
    let payload = serde_json::to_vec(&Pose {
        x: a.cos(),
        y: a.sin(),
        theta: a,
    })
    .expect("Pose serializes");
    writer.write_to_known_channel(
        &mcap::records::MessageHeader {
            channel_id,
            sequence: seq,
            log_time: now_ns,
            publish_time: now_ns,
        },
        &payload,
    )?;
    Ok(())
}

/// Write one trigger message stamped `publish_time = log_time = now_ns`.
/// Under the mcap interface the clip window's anchor is this record's own
/// stamp; the payload's `trigger_time` field is inert there and stays zero.
fn write_trigger<W: Write + Seek>(
    writer: &mut mcap::Writer<W>,
    channel_id: u16,
    now_ns: u64,
) -> Result<()> {
    let trigger = TriggerMsg {
        name: "chunked-mcap-writer-example".to_string(),
        description: "synthetic trigger emitted by chunked-mcap-writer".to_string(),
        trigger_time: StampMsg { sec: 0, nanosec: 0 },
        preroll: TRIGGER_PREROLL_NS,
        postroll: TRIGGER_POSTROLL_NS,
    };
    let payload = serde_json::to_vec(&trigger).expect("TriggerMsg serializes");
    writer.write_to_known_channel(
        &mcap::records::MessageHeader {
            channel_id,
            sequence: 0,
            log_time: now_ns,
            publish_time: now_ns,
        },
        &payload,
    )?;
    println!(
        "trigger emitted on {TRIGGER_TOPIC}: anchor={now_ns}ns preroll={TRIGGER_PREROLL_NS}ns postroll={TRIGGER_POSTROLL_NS}ns"
    );
    Ok(())
}

fn now_ns() -> Result<u64> {
    Ok(SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64)
}

fn main() -> Result<()> {
    let cfg = Config::parse();

    // Write straight to the file: no BufWriter, so nothing depends on Drop for
    // flushing. `writer.finish()` below writes the summary + footer. Until a
    // chunk closes its messages exist only in the writer's in-memory buffer —
    // the file on disk is always a clean run of complete records.
    let file = std::fs::File::create(&cfg.out)
        .with_context(|| format!("creating {}", cfg.out.display()))?;
    let mut writer = chunked_writer(file)?;
    let (pose_id, trigger_id) = register(&mut writer)?;

    // Ctrl+C just flips the flag (the handler suppresses the default kill), so
    // the loop can stop and reach finish() rather than dying mid-write.
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;

    println!(
        "recording /pose to {} every {PERIOD_MS} ms (zstd chunks close every {CHUNK_SIZE} uncompressed bytes), \
         one trigger every {}s — Ctrl+C to stop",
        cfg.out.display(),
        TRIGGER_PERIOD.as_secs(),
    );

    let period = Duration::from_millis(PERIOD_MS);
    let mut next_trigger = Instant::now() + TRIGGER_PERIOD;
    let mut seq = 0u32;
    loop {
        let now = now_ns()?;
        write_pose(&mut writer, pose_id, seq, now)?;
        seq += 1;

        if Instant::now() >= next_trigger {
            write_trigger(&mut writer, trigger_id, now)?;
            // Skip periods missed during a stall (suspend, scheduler pause):
            // one trigger per period, never a catch-up burst.
            while next_trigger <= Instant::now() {
                next_trigger += TRIGGER_PERIOD;
            }
        }

        std::thread::sleep(period);
        // Stop after the sleep, before writing the next sample.
        if stop.load(Ordering::Relaxed) {
            break;
        }
    }

    writer.finish().context("finalising mcap")?;
    println!("wrote {seq} /pose messages to {}", cfg.out.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Synthetic stamps for the sample recording: an arbitrary absolute base
    /// (2023-11-14T22:13:20Z) advancing one `PERIOD_MS` per message.
    const BASE_NS: u64 = 1_700_000_000_000_000_000;
    const STEP_NS: u64 = PERIOD_MS * 1_000_000;

    /// A `Write + Seek` sink that panics if any write lands below the current
    /// end of the buffer — i.e. it *proves*, at the sink level, that the
    /// writer only ever appends and never rewrites a byte already written
    /// (the tailability contract). Position queries (`stream_position`) and
    /// seeks are tolerated; only a write below the end faults.
    struct AppendOnly {
        buf: Vec<u8>,
        pos: u64,
    }

    impl Write for AppendOnly {
        fn write(&mut self, data: &[u8]) -> std::io::Result<usize> {
            assert_eq!(
                self.pos,
                self.buf.len() as u64,
                "append-only contract violated: write at offset {} would rewrite \
                 bytes below the current end {}",
                self.pos,
                self.buf.len(),
            );
            self.buf.extend_from_slice(data);
            self.pos += data.len() as u64;
            Ok(data.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    impl Seek for AppendOnly {
        fn seek(&mut self, from: std::io::SeekFrom) -> std::io::Result<u64> {
            use std::io::SeekFrom;
            let new = match from {
                SeekFrom::Start(p) => p as i128,
                SeekFrom::End(off) => self.buf.len() as i128 + off as i128,
                SeekFrom::Current(off) => self.pos as i128 + off as i128,
            };
            self.pos = u64::try_from(new).expect("seek before start of file");
            Ok(self.pos)
        }
    }

    /// Drive the real write path — the same [`chunked_writer`] construction
    /// `main` uses — into an [`AppendOnly`] sink, so every test below also
    /// proves the writer never rewrote a byte. Writes `n_pose` `/pose`
    /// messages stamped `BASE_NS + i * STEP_NS`, then one trigger stamped
    /// [`trigger_ns`], then finalises.
    fn sample_recording(n_pose: u32) -> Vec<u8> {
        let sink = AppendOnly {
            buf: Vec::new(),
            pos: 0,
        };
        let mut writer = chunked_writer(sink).expect("writer");
        let (pose_id, trigger_id) = register(&mut writer).expect("register");
        for seq in 0..n_pose {
            write_pose(&mut writer, pose_id, seq, BASE_NS + seq as u64 * STEP_NS).expect("pose");
        }
        write_trigger(&mut writer, trigger_id, trigger_ns(n_pose)).expect("trigger");
        writer.finish().expect("finish");
        writer.into_inner().buf
    }

    /// The stamp [`sample_recording`] puts on its trigger: one step past the
    /// last `/pose` message.
    fn trigger_ns(n_pose: u32) -> u64 {
        BASE_NS + n_pose as u64 * STEP_NS
    }

    /// Walk the top-level record framing by hand, exactly the way clipper's
    /// tail does: 8-byte leading magic, then records of 1-byte opcode +
    /// u64le length. Returns `(complete_records, final_offset)` for the
    /// records that fit entirely below `end`, panicking on any length no
    /// valid record can have — the `u64::MAX` back-patch placeholder above
    /// all.
    fn walk_complete_records(bytes: &[u8], end: usize) -> (usize, usize) {
        /// Far above any record this program writes; a length past this is
        /// framing corruption, not data.
        const MAX_RECORD_LEN: u64 = 1 << 30;

        assert!(end >= mcap::MAGIC.len() && end <= bytes.len());
        assert_eq!(&bytes[..mcap::MAGIC.len()], mcap::MAGIC, "leading magic");
        let mut pos = mcap::MAGIC.len();
        let mut records = 0;
        while pos + 9 <= end {
            let len = u64::from_le_bytes(bytes[pos + 1..pos + 9].try_into().unwrap());
            assert_ne!(
                len,
                u64::MAX,
                "u64::MAX placeholder on disk at offset {pos} — a back-patching \
                 writer's unpatched chunk length"
            );
            assert!(
                len <= MAX_RECORD_LEN,
                "implausible record length {len} at offset {pos}"
            );
            let next = pos + 9 + len as usize;
            if next > end {
                break; // in-progress record — what a live tailer waits out
            }
            pos = next;
            records += 1;
        }
        (records, pos)
    }

    /// Mirrors `crates/clipper/src/trigger.rs::Stamp`'s `Deserialize` shape —
    /// a local copy so this crate stays free of any dependency on
    /// `crates/clipper` (which pulls in `r2r`/ROS), while still proving the
    /// JSON this program writes decodes into that exact field shape.
    #[derive(serde::Deserialize, Debug, PartialEq)]
    struct TestStamp {
        sec: i32,
        nanosec: u32,
    }

    /// Mirrors `crates/clipper/src/trigger.rs::Trigger`'s `Deserialize` shape
    /// (`description` optional, defaulting empty; the rest required; unknown
    /// fields ignored) — see that module's doc comment and
    /// `crates/clipper/src/decode.rs`'s `json_decodes_the_msg_shape` test.
    #[derive(serde::Deserialize, Debug, PartialEq)]
    struct TestTrigger {
        name: String,
        #[serde(default)]
        description: String,
        trigger_time: TestStamp,
        preroll: u64,
        postroll: u64,
    }

    /// The output really is chunked: enough messages to overflow
    /// `CHUNK_SIZE` several times over must close several `Chunk` records —
    /// this example is pointless if its writer quietly falls back to
    /// unchunked output.
    #[test]
    fn output_contains_chunk_records() {
        let bytes = sample_recording(200);
        let mut chunks = 0;
        for record in mcap::read::LinearReader::new(&bytes).expect("reader") {
            if matches!(record.expect("record"), mcap::records::Record::Chunk { .. }) {
                chunks += 1;
            }
        }
        assert!(
            chunks >= 2,
            "expected several closed chunks from ~200 messages against a 4 KiB \
             chunk size, got {chunks}"
        );
    }

    /// Every top-level record on disk is complete: a `LinearReader` walks the
    /// whole file without a framing fault, and a hand-rolled walk of the
    /// record framing (the same 1-byte-opcode + u64le-length scan clipper's
    /// tail performs) consumes every byte up to the closing magic without
    /// ever meeting a `u64::MAX` placeholder or an implausible length.
    #[test]
    fn every_top_level_record_is_complete() {
        let bytes = sample_recording(200);

        for record in mcap::read::LinearReader::new(&bytes).expect("reader") {
            record.expect("every record parses without a framing fault");
        }

        let (records, final_pos) = walk_complete_records(&bytes, bytes.len());
        assert!(records > 0);
        assert_eq!(
            final_pos,
            bytes.len() - mcap::MAGIC.len(),
            "the record walk must land exactly on the closing magic"
        );
        assert_eq!(&bytes[final_pos..], mcap::MAGIC, "closing magic");
    }

    /// The data survives the chunk + zstd round trip: every `/pose` message
    /// decodes with its stamps intact (`publish_time = log_time`, in write
    /// order), and the trigger record decodes into the local mirror of
    /// clipper's `Trigger` shape with `trigger_time` at zero and the record's
    /// own stamps carrying the anchor.
    #[test]
    fn messages_decode_with_stamps_intact_and_trigger_matches_clippers_shape() {
        let n = 200u32;
        let bytes = sample_recording(n);

        let mut pose_idx = 0u64;
        let mut saw_trigger = false;
        for msg in mcap::MessageStream::new(&bytes).expect("stream") {
            let msg = msg.expect("message");
            if msg.channel.topic == TRIGGER_TOPIC {
                assert_eq!(msg.channel.message_encoding, "json");
                assert_eq!(msg.log_time, trigger_ns(n));
                assert_eq!(msg.publish_time, msg.log_time, "trigger lives on log time");

                let decoded: TestTrigger =
                    serde_json::from_slice(&msg.data).expect("trigger payload decodes");
                assert_eq!(
                    decoded,
                    TestTrigger {
                        name: "chunked-mcap-writer-example".to_string(),
                        description: "synthetic trigger emitted by chunked-mcap-writer".to_string(),
                        trigger_time: TestStamp { sec: 0, nanosec: 0 },
                        preroll: TRIGGER_PREROLL_NS,
                        postroll: TRIGGER_POSTROLL_NS,
                    }
                );
                saw_trigger = true;
            } else {
                assert_eq!(msg.channel.topic, "/pose");
                assert_eq!(msg.log_time, BASE_NS + pose_idx * STEP_NS, "write order");
                assert_eq!(msg.publish_time, msg.log_time, "data lives on log time");
                serde_json::from_slice::<serde_json::Value>(&msg.data).expect("payload is JSON");
                pose_idx += 1;
            }
        }
        assert_eq!(pose_idx, n as u64, "every /pose message decodes");
        assert!(saw_trigger, "the trigger record must be present");
    }

    /// A truncated prefix — what a live tailer sees mid-write — is always a
    /// clean run of complete records followed by at most one in-progress
    /// record, never a framing fault. Cuts land at arbitrary byte positions,
    /// including mid-record.
    #[test]
    fn truncated_prefix_scans_as_a_clean_run_of_complete_records() {
        let bytes = sample_recording(200);
        let len = bytes.len();
        for cut in [len / 4, len / 2, len * 3 / 4, len - 9] {
            let (records, final_pos) = walk_complete_records(&bytes, cut);
            assert!(
                records > 0,
                "cut at {cut}/{len} must still yield complete records"
            );
            assert!(final_pos <= cut);
        }
    }
}
