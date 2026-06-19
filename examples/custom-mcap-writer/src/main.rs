//! Minimal example: write an MCAP file directly with the `mcap` crate.
//!
//! Each channel is an iterator that yields one generated JSON payload per
//! `.next()`. Every 20 ms the loop pulls one sample from every channel and
//! writes it, until Ctrl+C. No ROS, no CDR — MCAP stores opaque bytes and a
//! channel's `message_encoding` (`"json"` here) is the only thing that says how
//! to read them, so the simplest possible payload works. Modeled on Foxglove's
//! quickstart writer.
//!
//! Run: `cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5`
//! (all flags optional; writes `quickstart.mcap` until Ctrl+C by default).

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use clap::{Parser, ValueEnum};
use serde::Serialize;

/// CLI: where to write, how fast, how long, and how to compress.
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

    /// Per-chunk compression.
    #[arg(long, value_enum, default_value_t = CompressionArg::Zstd)]
    compression: CompressionArg,
}

#[derive(Clone, Copy, ValueEnum)]
enum CompressionArg {
    None,
    Zstd,
    Lz4,
}

impl CompressionArg {
    fn to_mcap(self) -> Option<mcap::Compression> {
        match self {
            CompressionArg::None => None,
            CompressionArg::Zstd => Some(mcap::Compression::Zstd),
            CompressionArg::Lz4 => Some(mcap::Compression::Lz4),
        }
    }
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

/// Pull one sample from every channel and write it stamped at `now_ns`.
fn write_tick<W: std::io::Write + std::io::Seek>(
    writer: &mut mcap::Writer<W>,
    channels: &mut [Channel],
    ids: &[u16],
    seq: u32,
    now_ns: u64,
) -> Result<()> {
    for (ch, &channel_id) in channels.iter_mut().zip(ids) {
        if let Some(payload) = ch.source.next() {
            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id,
                    sequence: seq,
                    log_time: now_ns,
                    publish_time: now_ns,
                },
                &payload,
            )?;
        }
    }
    Ok(())
}

fn now_ns() -> Result<u64> {
    Ok(SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u64)
}

fn main() -> Result<()> {
    let cfg = Config::parse();

    // Write straight to the file: no BufWriter, so there is no userspace buffer
    // whose flush depends on Drop. `writer.finish()` below writes the summary +
    // footer and flushes the mcap chunk buffer to the file descriptor.
    let file = std::fs::File::create(&cfg.out)
        .with_context(|| format!("creating {}", cfg.out.display()))?;
    let mut writer = mcap::WriteOptions::new()
        .compression(cfg.compression.to_mcap())
        .create(file)?;

    let mut channels = channels();
    let ids = register(&mut writer, &channels)?;

    // Ctrl+C just flips the flag (the handler suppresses the default kill), so
    // the loop can stop and reach finish() rather than dying mid-write.
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;

    let period = Duration::from_millis(cfg.period_ms);
    let deadline = cfg
        .duration
        .map(|s| Instant::now() + Duration::from_secs_f64(s));
    println!(
        "recording {} channels to {} every {} ms — Ctrl+C to stop",
        channels.len(),
        cfg.out.display(),
        cfg.period_ms,
    );

    let mut seq = 0u32;
    loop {
        write_tick(&mut writer, &mut channels, &ids, seq, now_ns()?)?;
        seq += 1;

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

    #[test]
    fn writes_a_readable_mcap_with_json_on_every_channel() -> Result<()> {
        let mut writer = mcap::Writer::new(Cursor::new(Vec::new()))?;
        let mut chans = channels();
        let ids = register(&mut writer, &chans)?;
        for seq in 0..5 {
            write_tick(&mut writer, &mut chans, &ids, seq, seq as u64 * 20_000_000)?;
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
}
