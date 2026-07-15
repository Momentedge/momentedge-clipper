//! Minimal example: write an MCAP file directly with the `mcap` crate, owning
//! `publish_time` as a capture timestamp — the one thing no ROS publisher can
//! set on the wire on any distro (see "The publish_time contract" below).
//!
//! Each data channel is an iterator that yields one generated JSON payload per
//! `.next()`. Every `--period-ms` the loop pulls one sample from every channel
//! and writes it, until Ctrl+C or `--duration` elapses. No ROS, no CDR — MCAP
//! stores opaque bytes and a channel's `message_encoding` (`"json"` here) is
//! the only thing that says how to read them, so the simplest possible payload
//! works. Modeled on Foxglove's quickstart writer, extended with the
//! `publish_time`/trigger machinery `crates/clipper`'s MCAP interface needs.
//!
//! Run: `cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5`
//! (all flags optional; writes `quickstart.mcap` until Ctrl+C by default).
//!
//! ## The `publish_time` contract
//!
//! Every data message's record carries a capture timestamp in `publish_time`,
//! offset from `log_time` by `--publish-offset-ms`: `publish_time = log_time -
//! offset`, i.e. capture happened before the write that stamps `log_time`.
//! That timestamp **must be absolute wall-clock nanoseconds since the Unix
//! epoch, PTP-disciplined** — the same scale as `log_time`, `now()`, and
//! `Trigger.trigger_time`. A relative or monotonic capture stamp is
//! incomparable with the anchor a clip window is cut around and silently
//! breaks every window downstream. [`assert_absolute_unix_ns`] enforces this
//! on every `publish_time` this program writes rather than assuming it.
//!
//! ## The trigger payload's `trigger_time`
//!
//! Which timestamp clipper resolves a clip window's anchor from depends on
//! its interface and time source. Under `--interface mcap`, on either time
//! source, the anchor is the trigger *record's own* MCAP stamp (`log_time` or
//! `publish_time`) — the JSON payload's `trigger_time` field is read only
//! under `--interface ros` with the `publish` time source, standing in there
//! for the `publish_time` a ROS publisher cannot set on the wire. A non-zero
//! `trigger_time` in any other cell is a mis-anchoring hazard clipper rejects.
//! So this program leaves the payload's `trigger_time` at `{sec: 0, nanosec:
//! 0}` by default — the record's `publish_time` above already carries the
//! anchor — and only stamps it with the anchor when
//! `--stamp-payload-trigger-time` is passed, to exercise the rejection path or
//! produce a ros+publish-shaped payload.
//!
//! ## Tailability: unchunked output
//!
//! A producer clipper can tail live appends complete top-level records only
//! and never seeks back to rewrite one. MCAP's chunked writer — the `mcap`
//! crate's default — buffers messages into a `Chunk` record whose header is
//! written with a placeholder length and back-patched once the chunk closes,
//! so a chunk is only parseable after that close; read mid-write, its
//! placeholder length reads as an enormous, framing-breaking record. This
//! program builds its writer with `.use_chunks(false)`, so every message is
//! its own complete top-level `Message` record, appended once and never
//! rewritten — readable the instant it hits disk, the same append-only
//! contract rosbag2's fastwrite profile gives clipper in production. Unchunked
//! output carries no compression and no chunk-level index; `mcap recover` can
//! add indexes to a finished file if needed.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::Parser;
use serde::Serialize;

/// Nanoseconds since the Unix epoch at 2020-01-01T00:00:00Z — the floor
/// [`assert_absolute_unix_ns`] checks every `publish_time` against. Anything
/// below it is not a plausible PTP-disciplined wall-clock stamp: a relative
/// offset, a monotonic-clock reading, or a unit mistake (e.g. milliseconds
/// where nanoseconds are expected) all land far below this floor.
const EPOCH_FLOOR_NS: u64 = 1_577_836_800_000_000_000;

/// Panic with a clear message unless `ns` is plausibly an absolute
/// Unix-epoch-nanosecond wall-clock stamp (`>= EPOCH_FLOOR_NS`). Called on
/// every `publish_time` this program writes — see "The `publish_time`
/// contract" above for why this must be asserted, not assumed.
fn assert_absolute_unix_ns(label: &str, ns: u64) {
    assert!(
        ns >= EPOCH_FLOOR_NS,
        "{label}: publish_time must be absolute Unix-epoch nanoseconds on a \
         PTP-disciplined wall clock — the same scale as log_time, now(), and \
         Trigger.trigger_time — but got {ns} ns, which is before \
         2020-01-01T00:00:00Z ({EPOCH_FLOOR_NS} ns). A relative or monotonic \
         capture stamp is incomparable with the anchor and would silently \
         break every clip window."
    );
}

/// Nanoseconds either side of the synthetic trigger's anchor its window keeps
/// — generous enough that even a short `--duration` run produces a non-empty
/// clip once clipper cuts around it.
const TRIGGER_PREROLL_NS: u64 = 2_000_000_000;
const TRIGGER_POSTROLL_NS: u64 = 2_000_000_000;

/// The topic clipper's MCAP interface taps for triggers
/// (`crates/clipper/src/interface.rs`).
const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";

/// CLI: where to write, how fast, and how long.
#[derive(Parser)]
#[command(about = "Write an MCAP file directly with the mcap crate (JSON channels, no ROS).")]
struct Config {
    /// Output MCAP file path.
    #[arg(long, default_value = "quickstart.mcap")]
    out: PathBuf,

    /// Sample period in milliseconds: how often every channel is written.
    #[arg(long, default_value_t = 20)]
    period_ms: u64,

    /// Stop after this many seconds; run until Ctrl+C if unset.
    #[arg(long)]
    duration: Option<f64>,

    /// Milliseconds subtracted from `log_time` to produce every data
    /// message's `publish_time`: capture happens this long before the message
    /// is written, so `publish_time = log_time - publish_offset_ms`. See "The
    /// `publish_time` contract" in the module docs.
    #[arg(long, default_value_t = 50)]
    publish_offset_ms: u64,

    /// Emit one `/events/momentedge/trigger` message this many milliseconds
    /// after startup, anchored at the same capture-time offset as the data
    /// channels. No trigger is emitted if unset.
    #[arg(long)]
    trigger_after_ms: Option<u64>,

    /// Stamp the trigger JSON payload's `trigger_time` field with the capture
    /// anchor instead of leaving it `{sec: 0, nanosec: 0}`. See "The trigger
    /// payload's `trigger_time`" in the module docs for why the default is
    /// zero.
    #[arg(long)]
    stamp_payload_trigger_time: bool,
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

impl StampMsg {
    /// Split an absolute Unix-epoch-nanosecond anchor into `{sec, nanosec}`.
    /// `sec` is `i32` (matching `builtin_interfaces/Time`, so it inherits that
    /// message's year-2038 rollover) — safe for any anchor before then.
    fn from_ns(ns: u64) -> Self {
        StampMsg {
            sec: (ns / 1_000_000_000) as i32,
            nanosec: (ns % 1_000_000_000) as u32,
        }
    }
}

/// The JSON payload for `/events/momentedge/trigger`. Field-for-field the
/// shape `crates/clipper/src/trigger.rs::Trigger` derives `Deserialize` for
/// (verified against its doc comments and `crates/clipper/src/decode.rs`'s
/// `json_decodes_the_msg_shape` test): `description` is optional there but
/// always emitted here, `trigger_time` mirrors `builtin_interfaces/Time`, and
/// `preroll`/`postroll` are nanoseconds either side of `trigger_time`.
/// `trigger_time` is `{sec: 0, nanosec: 0}` unless
/// `--stamp-payload-trigger-time` asks otherwise — see "The trigger payload's
/// `trigger_time`" in the module docs.
#[derive(Serialize)]
struct TriggerMsg {
    name: String,
    description: String,
    trigger_time: StampMsg,
    preroll: u64,
    postroll: u64,
}

/// An MCAP channel: how to register it, plus the iterator that generates its
/// payloads. `source.next()` yields the next message's bytes.
struct Channel {
    topic: &'static str,
    message_encoding: &'static str,
    schema_name: &'static str,
    schema_encoding: &'static str,
    /// JSON Schema, or empty for a schemaless channel.
    schema: &'static [u8],
    source: Box<dyn Iterator<Item = Vec<u8>>>,
}

/// The channels to record. Each `source` is an endless iterator of generated
/// JSON: a typed struct via serde, and a raw JSON string built by hand.
fn channels() -> Vec<Channel> {
    vec![
        Channel {
            topic: "/pose",
            message_encoding: "json",
            schema_name: "Pose",
            schema_encoding: "jsonschema",
            schema: POSE_SCHEMA,
            source: Box::new((0u64..).map(|i| {
                let a = i as f64 * 0.05;
                serde_json::to_vec(&Pose {
                    x: a.cos(),
                    y: a.sin(),
                    theta: a,
                })
                .expect("Pose serializes")
            })),
        },
        Channel {
            topic: "/size",
            message_encoding: "json",
            schema_name: "",
            schema_encoding: "",
            schema: b"", // schemaless — the most basic payload there is
            source: Box::new((0u64..).map(|i| {
                let size = 1.0 + (i as f64 * 0.05).sin().abs();
                format!("{{\"size\": {size}}}").into_bytes()
            })),
        },
    ]
}

/// Register every channel into `writer`, returning their assigned ids (ids are
/// assigned per file, so this runs once before the loop).
fn register<W: std::io::Write + std::io::Seek>(
    writer: &mut mcap::Writer<W>,
    channels: &[Channel],
) -> Result<Vec<u16>> {
    channels
        .iter()
        .map(|ch| {
            let schema_id = if ch.schema.is_empty() {
                0 // schemaless
            } else {
                writer.add_schema(ch.schema_name, ch.schema_encoding, ch.schema)?
            };
            Ok(writer.add_channel(schema_id, ch.topic, ch.message_encoding, &BTreeMap::new())?)
        })
        .collect()
}

/// Pull one sample from every channel and write it: `log_time` is `now_ns`
/// (wall clock at write), `publish_time` is `now_ns` minus `publish_offset_ns`
/// (the capture instant — see "The `publish_time` contract" in the module
/// docs), asserted absolute before it is written.
fn write_tick<W: std::io::Write + std::io::Seek>(
    writer: &mut mcap::Writer<W>,
    channels: &mut [Channel],
    ids: &[u16],
    seq: u32,
    now_ns: u64,
    publish_offset_ns: u64,
) -> Result<()> {
    let publish_time = now_ns.saturating_sub(publish_offset_ns);
    assert_absolute_unix_ns("data channel publish_time", publish_time);
    for (ch, &channel_id) in channels.iter_mut().zip(ids) {
        if let Some(payload) = ch.source.next() {
            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence: seq,
                    log_time: now_ns,
                    publish_time,
                },
                &payload,
            )?;
        }
    }
    Ok(())
}

/// Write one trigger message: `log_time` is wall-now at emission, and
/// `publish_time` is `anchor_ns` — the capture-domain instant clipper's clip
/// window is centred on under the mcap interface, computed by the caller the
/// same way as the data channels' `publish_time` (`now_ns -
/// publish_offset_ns`), so both are directly comparable. The JSON payload's
/// `trigger_time` field is `{sec: 0, nanosec: 0}` unless
/// `stamp_payload_trigger_time` is set, in which case it carries the same
/// `anchor_ns` — see "The trigger payload's `trigger_time`" in the module
/// docs for why the default is zero.
fn write_trigger<W: std::io::Write + std::io::Seek>(
    writer: &mut mcap::Writer<W>,
    trigger_channel_id: u16,
    log_time_ns: u64,
    anchor_ns: u64,
    stamp_payload_trigger_time: bool,
) -> Result<()> {
    assert_absolute_unix_ns("trigger publish_time", anchor_ns);
    let trigger_time = if stamp_payload_trigger_time {
        StampMsg::from_ns(anchor_ns)
    } else {
        StampMsg { sec: 0, nanosec: 0 }
    };
    let trigger = TriggerMsg {
        name: "custom-mcap-writer-example".to_string(),
        description: "synthetic trigger emitted by custom-mcap-writer".to_string(),
        trigger_time,
        preroll: TRIGGER_PREROLL_NS,
        postroll: TRIGGER_POSTROLL_NS,
    };
    let payload = serde_json::to_vec(&trigger).expect("TriggerMsg serializes");
    writer.write_to_known_channel(
        &mcap::records::MessageHeader {
            channel_id: trigger_channel_id,
            sequence: 0,
            log_time: log_time_ns,
            publish_time: anchor_ns,
        },
        &payload,
    )?;
    println!(
        "trigger emitted on {TRIGGER_TOPIC}: anchor={anchor_ns}ns preroll={TRIGGER_PREROLL_NS}ns postroll={TRIGGER_POSTROLL_NS}ns"
    );
    Ok(())
}

fn now_ns() -> Result<u64> {
    Ok(SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64)
}

fn main() -> Result<()> {
    let cfg = Config::parse();

    // Write straight to the file: no BufWriter, so there is no userspace buffer
    // whose flush depends on Drop. `writer.finish()` below writes the summary +
    // footer and flushes the writer's internal buffers to the file descriptor.
    // `use_chunks(false)` is what makes the output tailable live — see
    // "Tailability: unchunked output" in the module docs.
    let file = std::fs::File::create(&cfg.out)
        .with_context(|| format!("creating {}", cfg.out.display()))?;
    let mut writer = mcap::WriteOptions::new().use_chunks(false).create(file)?;

    let mut channels = channels();
    let ids = register(&mut writer, &channels)?;
    // Schemaless, like `/size` — clipper's MCAP interface decodes triggers by
    // `message_encoding` alone, not a schema (see crates/clipper/src/decode.rs).
    let trigger_channel_id = writer.add_channel(0, TRIGGER_TOPIC, "json", &BTreeMap::new())?;

    let publish_offset_ns = cfg.publish_offset_ms * 1_000_000;

    // Ctrl+C just flips the flag (the handler suppresses the default kill), so
    // the loop can stop and reach finish() rather than dying mid-write.
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;

    let period = Duration::from_millis(cfg.period_ms);
    let start = Instant::now();
    let deadline = cfg
        .duration
        .map(|s| Instant::now() + Duration::from_secs_f64(s));
    println!(
        "recording {} channels to {} every {} ms (publish_offset {} ms) — Ctrl+C to stop",
        channels.len(),
        cfg.out.display(),
        cfg.period_ms,
        cfg.publish_offset_ms,
    );

    let mut seq = 0u32;
    let mut trigger_emitted = false;
    loop {
        let now = now_ns()?;
        write_tick(
            &mut writer,
            &mut channels,
            &ids,
            seq,
            now,
            publish_offset_ns,
        )?;
        seq += 1;

        if !trigger_emitted
            && cfg
                .trigger_after_ms
                .is_some_and(|t| start.elapsed().as_millis() as u64 >= t)
        {
            write_trigger(
                &mut writer,
                trigger_channel_id,
                now,
                now.saturating_sub(publish_offset_ns),
                cfg.stamp_payload_trigger_time,
            )?;
            trigger_emitted = true;
        }

        std::thread::sleep(period);
        // Stop after the sleep, before writing the next sample: on Ctrl+C, or
        // once the optional duration has elapsed.
        if stop.load(Ordering::Relaxed) || deadline.is_some_and(|dl| Instant::now() >= dl) {
            break;
        }
    }

    writer.finish().context("finalising mcap")?;
    println!("wrote {seq} messages per channel to {}", cfg.out.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use super::*;

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
    /// `crates/clipper/src/decode.rs`'s `json_decodes_the_msg_shape` test,
    /// which this mirrors field-for-field.
    #[derive(serde::Deserialize, Debug, PartialEq)]
    struct TestTrigger {
        name: String,
        #[serde(default)]
        description: String,
        trigger_time: TestStamp,
        preroll: u64,
        postroll: u64,
    }

    #[test]
    fn writes_a_readable_mcap_with_json_on_every_channel() -> Result<()> {
        let mut writer = mcap::Writer::new(Cursor::new(Vec::new()))?;
        let mut chans = channels();
        let ids = register(&mut writer, &chans)?;
        for seq in 0..5 {
            write_tick(
                &mut writer,
                &mut chans,
                &ids,
                seq,
                EPOCH_FLOOR_NS + seq as u64 * 20_000_000,
                0,
            )?;
        }
        writer.finish()?;
        let bytes = writer.into_inner().into_inner();

        let mut topics = std::collections::BTreeSet::new();
        let mut count = 0;
        for msg in mcap::MessageStream::new(&bytes)? {
            let msg = msg?;
            topics.insert(msg.channel.topic.clone());
            // Whatever channel it came from, the payload is valid JSON.
            serde_json::from_slice::<serde_json::Value>(&msg.data).expect("payload is JSON");
            count += 1;
        }
        assert_eq!(count, 10, "5 ticks x 2 channels");
        assert!(topics.contains("/pose") && topics.contains("/size"));
        Ok(())
    }

    #[test]
    fn epoch_floor_boundary_is_accepted() {
        // The boundary itself (2020-01-01T00:00:00Z exactly) must not panic.
        assert_absolute_unix_ns("test", EPOCH_FLOOR_NS);
    }

    #[test]
    #[should_panic(expected = "absolute Unix-epoch nanoseconds")]
    fn one_nanosecond_below_epoch_floor_panics() {
        assert_absolute_unix_ns("test", EPOCH_FLOOR_NS - 1);
    }

    /// End-to-end through the real write path: data channels carry a
    /// `publish_time` trailing `log_time` by the configured offset, and the
    /// trigger record is present on the expected topic with the expected
    /// `log_time`/`publish_time` pair (the record's `publish_time` is the
    /// anchor) and a JSON payload that decodes into clipper's `Trigger` shape
    /// — with `trigger_time` left at `{sec: 0, nanosec: 0}`, the default: the
    /// mcap interface's anchor is the record's own `publish_time` above, not
    /// this field (see "The trigger payload's `trigger_time`" in the module
    /// docs).
    #[test]
    fn publish_time_is_offset_and_trigger_payload_time_defaults_to_zero() -> Result<()> {
        let offset_ns = 50_000_000; // 50 ms, matching --publish-offset-ms's default
        let base = EPOCH_FLOOR_NS + 1_000_000_000; // a full second above the floor

        let mut writer = mcap::Writer::new(Cursor::new(Vec::new()))?;
        let mut chans = channels();
        let ids = register(&mut writer, &chans)?;
        let trigger_channel_id = writer.add_channel(0, TRIGGER_TOPIC, "json", &BTreeMap::new())?;

        for seq in 0..5u32 {
            let log_time = base + seq as u64 * 20_000_000;
            write_tick(&mut writer, &mut chans, &ids, seq, log_time, offset_ns)?;
        }
        let trigger_log_time = base + 2 * 20_000_000;
        let anchor_ns = trigger_log_time.saturating_sub(offset_ns);
        write_trigger(
            &mut writer,
            trigger_channel_id,
            trigger_log_time,
            anchor_ns,
            false,
        )?;

        writer.finish()?;
        let bytes = writer.into_inner().into_inner();

        let mut data_msgs = 0;
        let mut saw_trigger = false;
        for msg in mcap::MessageStream::new(&bytes)? {
            let msg = msg?;
            if msg.channel.topic == TRIGGER_TOPIC {
                saw_trigger = true;
                assert_eq!(msg.channel.message_encoding, "json");
                assert_eq!(msg.log_time, trigger_log_time);
                assert_eq!(msg.publish_time, anchor_ns);
                assert_ne!(
                    msg.publish_time, msg.log_time,
                    "trigger publish_time and log_time must be distinct"
                );

                let decoded: TestTrigger = serde_json::from_slice(&msg.data)?;
                assert_eq!(
                    decoded,
                    TestTrigger {
                        name: "custom-mcap-writer-example".to_string(),
                        description: "synthetic trigger emitted by custom-mcap-writer".to_string(),
                        trigger_time: TestStamp { sec: 0, nanosec: 0 },
                        preroll: TRIGGER_PREROLL_NS,
                        postroll: TRIGGER_POSTROLL_NS,
                    }
                );
            } else {
                data_msgs += 1;
                assert_eq!(
                    msg.publish_time,
                    msg.log_time - offset_ns,
                    "data publish_time must trail log_time by the configured offset"
                );
                assert_ne!(
                    msg.publish_time, msg.log_time,
                    "data publish_time and log_time must be distinct"
                );
            }
        }
        assert_eq!(data_msgs, 10, "5 ticks x 2 data channels");
        assert!(saw_trigger, "the trigger record must be present");
        Ok(())
    }

    /// Mirrors `main()`'s writer construction (`WriteOptions::use_chunks(false)`)
    /// and asserts the output contains no top-level `Chunk` record — a live
    /// tailer only ever sees complete `Message` records, never a chunk whose
    /// length is back-patched on close. See "Tailability: unchunked output" in
    /// the module docs for why that back-patch is unreadable mid-write.
    #[test]
    fn writer_emits_unchunked_output_for_live_tailing() -> Result<()> {
        let mut writer = mcap::WriteOptions::new()
            .use_chunks(false)
            .create(Cursor::new(Vec::new()))?;
        let mut chans = channels();
        let ids = register(&mut writer, &chans)?;
        for seq in 0..5 {
            write_tick(
                &mut writer,
                &mut chans,
                &ids,
                seq,
                EPOCH_FLOOR_NS + seq as u64 * 20_000_000,
                0,
            )?;
        }
        writer.finish()?;
        let bytes = writer.into_inner().into_inner();

        let mut message_count = 0;
        for record in mcap::read::LinearReader::new(&bytes)? {
            match record? {
                mcap::records::Record::Chunk { .. } => {
                    panic!("unchunked output must contain no Chunk records")
                }
                mcap::records::Record::Message { .. } => message_count += 1,
                _ => {}
            }
        }
        assert_eq!(
            message_count, 10,
            "5 ticks x 2 channels, each a top-level Message record"
        );
        Ok(())
    }

    /// `--stamp-payload-trigger-time` opts back into stamping the JSON
    /// payload's `trigger_time` with the anchor, for exercising a
    /// ros+publish-shaped payload or the mis-anchoring rejection path.
    #[test]
    fn stamp_payload_trigger_time_flag_stamps_the_anchor() -> Result<()> {
        let anchor_ns = EPOCH_FLOOR_NS + 1_000_000_000;
        let log_time_ns = anchor_ns + 50_000_000;

        let mut writer = mcap::Writer::new(Cursor::new(Vec::new()))?;
        let trigger_channel_id = writer.add_channel(0, TRIGGER_TOPIC, "json", &BTreeMap::new())?;
        write_trigger(
            &mut writer,
            trigger_channel_id,
            log_time_ns,
            anchor_ns,
            true,
        )?;
        writer.finish()?;
        let bytes = writer.into_inner().into_inner();

        let msg = mcap::MessageStream::new(&bytes)?
            .next()
            .expect("one trigger message written")?;
        let decoded: TestTrigger = serde_json::from_slice(&msg.data)?;
        assert_eq!(
            decoded.trigger_time,
            TestStamp {
                sec: (anchor_ns / 1_000_000_000) as i32,
                nanosec: (anchor_ns % 1_000_000_000) as u32,
            }
        );
        Ok(())
    }
}
