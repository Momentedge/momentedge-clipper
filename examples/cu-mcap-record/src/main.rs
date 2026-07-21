//! A copper (cu29) `CuSinkTask` that appends routed task outputs to an MCAP
//! Recording `clipper --interface mcap` can tail live — the copper producer
//! path beside the two plain-`mcap`-crate writer examples
//! (`examples/custom-mcap-writer`, `examples/chunked-mcap-writer`). No ROS
//! stack is involved anywhere: the Trigger travels in-band, written into the
//! Recording on clipper's default trigger topic, and the clip's appearance in
//! clipper's output directory is the only completion signal.
//!
//! The task graph is one ~50 Hz synthetic sensor source, one periodic (~10 s)
//! trigger source, and this sink. The sink receives both as two typed inputs
//! and appends each to the Recording, mirroring copper's MCAP-exporter
//! conventions (a `copper.<task>` jsonschema schema and a `/<task>` JSON
//! channel per data input; the export envelope `payload`/`tov`/`process_time`/
//! `status_txt`; payload-less messages on a `/<task>/__meta` channel under a
//! shared `copper.meta` schema) with two deliberate divergences documented in
//! `README.md`: the writer is unchunked (append-only, tailable) and the record
//! stamps live in the Unix-epoch domain (so clipper's retention, restart
//! recovery, and Coverage behave, and capture-time windowing works under
//! `--time-source publish`).
//!
//! Everything lives in this one file, sibling-style: payload types and the
//! three tasks in `mod tasks`, the `Recording<W>` writer core (the test seam)
//! and the JSON envelope at crate root, `main`, and the inline test module.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{BufWriter, Seek, Write};
use std::path::Path;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result, anyhow};
use clap::Parser;
use cu29::prelude::*;
use serde::Serialize;

/// The topic clipper's MCAP interface taps for triggers
/// (`crates/clipper/src/interface.rs`); the mcap interface reads no
/// `--trigger-topic` flag, so this constant is the contract.
const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";

/// Default output directory when `--out` is not given; also the value baked
/// into `copperconfig.ron` before `main` overrides it.
const DEFAULT_OUT_DIR: &str = "out";

/// Nanoseconds either side of a trigger's anchor its clip window keeps.
/// Wide enough that clipper cuts a non-empty clip around a live trigger; the
/// sibling writer examples use the same 3 s.
const TRIGGER_PREROLL_NS: u64 = 3_000_000_000;
const TRIGGER_POSTROLL_NS: u64 = 3_000_000_000;

/// The trigger source fires this often. Its next-fire instant starts at the
/// clock origin, so the first trigger fires on the first iteration (a clip
/// appears promptly in a demo) and every `TRIGGER_PERIOD_SECS` thereafter.
const TRIGGER_PERIOD_SECS: u64 = 10;

/// The bounded run loop paces itself to this period so the graph advances at
/// roughly `rate_target_hz: 50` (copperconfig.ron). The rate limiter cu29
/// applies inside `app.run()` does not run under a manual `run_one_iteration`
/// loop, which is used here so the sink's `stop()`/`finish()` always runs on
/// Ctrl+C — so the loop sleeps this long between iterations itself.
const LOOP_PERIOD: Duration = Duration::from_millis(20);

/// Nanoseconds since the Unix epoch at 2020-01-01T00:00:00Z — the floor every
/// record stamp is checked against. A value below it is not a plausible
/// PTP-disciplined wall-clock stamp (a relative offset, a monotonic reading,
/// or a unit mistake), and would be incomparable with the anchor a clip window
/// is cut around, silently breaking every window downstream.
const EPOCH_FLOOR_NS: u64 = 1_577_836_800_000_000_000;

/// Panic unless `ns` is plausibly an absolute Unix-epoch-nanosecond wall-clock
/// stamp (`>= EPOCH_FLOOR_NS`). Called on every `log_time` and `publish_time`
/// [`Recording`] writes: the epoch time domain is load-bearing (see the
/// `README.md` divergences), so it is asserted, not assumed.
fn assert_absolute_unix_ns(label: &str, ns: u64) {
    assert!(
        ns >= EPOCH_FLOOR_NS,
        "{label}: MCAP record stamps must be absolute Unix-epoch nanoseconds on \
         a PTP-disciplined wall clock — the same scale as clipper's anchor, \
         log_time, and Trigger.trigger_time — but got {ns} ns, which is before \
         2020-01-01T00:00:00Z ({EPOCH_FLOOR_NS} ns). A relative or monotonic \
         stamp is incomparable with the anchor and would silently break every \
         clip window."
    );
}

/// Wall-clock nanoseconds since the Unix epoch, sampled now.
fn now_ns() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is before the Unix epoch")
        .as_nanos() as u64
}

// ---- The JSON envelope (copper MCAP-exporter parity) -----------------------

/// The `tov` field of the export envelope. Copper's exporter serializes a
/// friendly `{kind, time_ns, start_ns, end_ns}` struct here rather than
/// `Tov`'s native form, so the raw robot-clock time-of-validity travels inside
/// the JSON untouched while the record's own stamps carry the epoch domain.
#[derive(Serialize)]
struct TovEnvelope {
    kind: &'static str,
    time_ns: u64,
    start_ns: u64,
    end_ns: u64,
}

impl From<Tov> for TovEnvelope {
    fn from(tov: Tov) -> Self {
        match tov {
            Tov::None => TovEnvelope {
                kind: "none",
                time_ns: 0,
                start_ns: 0,
                end_ns: 0,
            },
            Tov::Time(t) => {
                let n = t.as_nanos();
                TovEnvelope {
                    kind: "time",
                    time_ns: n,
                    start_ns: n,
                    end_ns: n,
                }
            }
            Tov::Range(r) => TovEnvelope {
                kind: "range",
                time_ns: 0,
                start_ns: r.start.as_nanos(),
                end_ns: r.end.as_nanos(),
            },
        }
    }
}

/// The export envelope for a message that carries a payload. Field order —
/// `payload`, `tov`, `process_time`, `status_txt` — matches copper's exporter.
/// `process_time` is cu29's `PartialCuTimeRange` serialized directly (its
/// derive emits `{start, end}` as integer nanos, with the u64 sentinel
/// `18446744073709551615` for an absent bound), giving exact parity.
#[derive(Serialize)]
struct DataEnvelope<'a, P: Serialize> {
    payload: &'a P,
    tov: TovEnvelope,
    process_time: PartialCuTimeRange,
    status_txt: &'a str,
}

/// The export envelope for a payload-less message, written on a `__meta`
/// channel. It carries no `payload` key; `payload_missing` is the constant
/// `true` marker copper's exporter writes.
#[derive(Serialize)]
struct MetaEnvelope<'a> {
    tov: TovEnvelope,
    process_time: PartialCuTimeRange,
    status_txt: &'a str,
    payload_missing: bool,
}

/// `builtin_interfaces/Time` flattened to its two fields — the shape
/// `crates/clipper/src/trigger.rs::Stamp` decodes.
#[derive(Serialize)]
struct TriggerStampWire {
    sec: i32,
    nanosec: u32,
}

/// The wire JSON for the trigger channel: the momentedge `Trigger`, written
/// **unwrapped** (not in the copper envelope) because clipper's decoder reads
/// these fields at the top level. This is the one channel with no exporter
/// parity. `trigger_time` is the constant zero: under `--interface mcap`
/// clipper anchors on the trigger record's own MCAP stamp, and its admission
/// gate rejects a non-zero payload `trigger_time`.
#[derive(Serialize)]
struct TriggerWire {
    name: String,
    description: String,
    trigger_time: TriggerStampWire,
    preroll: u64,
    postroll: u64,
}

impl TriggerWire {
    fn new(name: String, description: String, preroll: u64, postroll: u64) -> Self {
        TriggerWire {
            name,
            description,
            trigger_time: TriggerStampWire { sec: 0, nanosec: 0 },
            preroll,
            postroll,
        }
    }
}

// ---- Hand-written jsonschema schemas (exporter-shape parity) ---------------
//
// Copper's exporter reflection-generates these; its generator and envelope
// helpers are module-private, so the schemas are hand-written here to the same
// draft-07 shape rather than pulling in the cu29-export + bevy_reflect tree.

/// Schema for `/sensor`: the wrapped envelope around `SensorSample`.
const SENSOR_SCHEMA: &[u8] = br#"{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "payload": {
      "type": "object",
      "properties": {
        "seq": { "type": "integer", "minimum": 0 },
        "value": { "type": "number" }
      },
      "additionalProperties": false,
      "required": ["seq", "value"]
    },
    "tov": {
      "type": "object",
      "properties": {
        "kind": { "type": "string", "enum": ["none", "time", "range"] },
        "time_ns": { "type": "integer", "minimum": 0 },
        "start_ns": { "type": "integer", "minimum": 0 },
        "end_ns": { "type": "integer", "minimum": 0 }
      },
      "required": ["kind", "time_ns", "start_ns", "end_ns"],
      "additionalProperties": false
    },
    "process_time": {
      "type": "object",
      "properties": {
        "start": { "type": "integer", "minimum": 0 },
        "end": { "type": "integer", "minimum": 0 }
      },
      "additionalProperties": false,
      "required": ["start", "end"]
    },
    "status_txt": { "type": "string" }
  },
  "required": ["payload", "tov", "process_time", "status_txt"],
  "additionalProperties": false
}"#;

/// Shared schema for every `__meta` channel: the payload-less envelope with
/// the `payload_missing` const-true marker.
const META_SCHEMA: &[u8] = br#"{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "tov": {
      "type": "object",
      "properties": {
        "kind": { "type": "string", "enum": ["none", "time", "range"] },
        "time_ns": { "type": "integer", "minimum": 0 },
        "start_ns": { "type": "integer", "minimum": 0 },
        "end_ns": { "type": "integer", "minimum": 0 }
      },
      "required": ["kind", "time_ns", "start_ns", "end_ns"],
      "additionalProperties": false
    },
    "process_time": {
      "type": "object",
      "properties": {
        "start": { "type": "integer", "minimum": 0 },
        "end": { "type": "integer", "minimum": 0 }
      },
      "additionalProperties": false,
      "required": ["start", "end"]
    },
    "status_txt": { "type": "string" },
    "payload_missing": { "type": "boolean", "const": true }
  },
  "required": ["tov", "process_time", "status_txt", "payload_missing"],
  "additionalProperties": false
}"#;

/// Schema for the trigger channel: the unwrapped momentedge `Trigger` shape.
const TRIGGER_SCHEMA: &[u8] = br#"{
  "$schema": "https://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "name": { "type": "string" },
    "description": { "type": "string" },
    "trigger_time": {
      "type": "object",
      "properties": {
        "sec": { "type": "integer" },
        "nanosec": { "type": "integer", "minimum": 0 }
      },
      "additionalProperties": false,
      "required": ["sec", "nanosec"]
    },
    "preroll": { "type": "integer", "minimum": 0 },
    "postroll": { "type": "integer", "minimum": 0 }
  },
  "required": ["name", "description", "trigger_time", "preroll", "postroll"],
  "additionalProperties": false
}"#;

// ---- The Recording writer core (the test seam) -----------------------------

/// The MCAP Recording the sink appends to — the seam every test drives.
///
/// Its methods take plain arguments (a serializable payload, a `Tov`, a
/// `PartialCuTimeRange`, a `&str` status), never a `CuMsg`, so tests exercise
/// the whole writer over a `Cursor` or an append-only sink with no copper
/// runtime. The sink task adapts each `CuMsg` into these arguments.
///
/// The writer is unchunked (`use_chunks(false)`): every message is one
/// complete top-level record, appended once and never rewritten — the
/// tailability contract clipper requires. Record stamps are computed here, the
/// single tested place: `log_time` is the wall clock at write, and
/// `publish_time` is the epoch-translated time-of-validity.
struct Recording<W: Write + Seek> {
    writer: mcap::Writer<W>,
    /// Robot-clock(ns) → Unix-epoch(ns) offset, sampled once at sink start.
    epoch_offset_ns: i128,
    sensor_channel: u16,
    sensor_meta_channel: u16,
    trigger_channel: u16,
    /// One monotonic sequence across all channels (exporter parity).
    sequence: u32,
}

impl<W: Write + Seek> Recording<W> {
    /// Open a Recording over `sink`, registering every schema and channel up
    /// front. `epoch_offset_ns` translates a robot-clock time-of-validity into
    /// the epoch domain for `publish_time`.
    fn new(sink: W, epoch_offset_ns: i128) -> Result<Self> {
        let mut writer = mcap::WriteOptions::new()
            .use_chunks(false)
            .create(sink)
            .context("creating unchunked mcap writer")?;

        let sensor_schema = writer.add_schema("copper.sensor", "jsonschema", SENSOR_SCHEMA)?;
        let meta_schema = writer.add_schema("copper.meta", "jsonschema", META_SCHEMA)?;
        let trigger_schema =
            writer.add_schema("momentedge.trigger", "jsonschema", TRIGGER_SCHEMA)?;

        let sensor_channel =
            writer.add_channel(sensor_schema, "/sensor", "json", &BTreeMap::new())?;
        let sensor_meta_channel =
            writer.add_channel(meta_schema, "/sensor/__meta", "json", &BTreeMap::new())?;
        let trigger_channel =
            writer.add_channel(trigger_schema, TRIGGER_TOPIC, "json", &BTreeMap::new())?;

        Ok(Recording {
            writer,
            epoch_offset_ns,
            sensor_channel,
            sensor_meta_channel,
            trigger_channel,
            sequence: 0,
        })
    }

    /// Compute the `(log_time, publish_time)` pair for a record, in the
    /// Unix-epoch domain. `log_time` is the wall clock at write. `publish_time`
    /// is the epoch-translated `tov`: `Tov::Time(t)` and `Tov::Range` map to
    /// `t`/`start` plus the offset; `Tov::None` falls back to `log_time`, so
    /// every record stays windowable on either time source. Both stamps are
    /// asserted absolute.
    fn stamps(&self, tov: Tov) -> (u64, u64) {
        let log_time = now_ns();
        assert_absolute_unix_ns("log_time", log_time);

        let publish = match tov {
            Tov::Time(t) => t.as_nanos() as i128 + self.epoch_offset_ns,
            Tov::Range(r) => r.start.as_nanos() as i128 + self.epoch_offset_ns,
            Tov::None => log_time as i128,
        };
        let publish_time = u64::try_from(publish)
            .expect("publish_time resolved to a negative epoch nanosecond value");
        assert_absolute_unix_ns("publish_time", publish_time);

        (log_time, publish_time)
    }

    /// Take the next global sequence number.
    fn next_seq(&mut self) -> u32 {
        let seq = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        seq
    }

    /// Append one data message to `/sensor`: the payload wrapped in the export
    /// envelope, stamped from `tov`.
    fn write_sensor<P: Serialize>(
        &mut self,
        payload: &P,
        tov: Tov,
        process_time: PartialCuTimeRange,
        status_txt: &str,
    ) -> Result<()> {
        let (log_time, publish_time) = self.stamps(tov);
        let envelope = DataEnvelope {
            payload,
            tov: TovEnvelope::from(tov),
            process_time,
            status_txt,
        };
        let data = serde_json::to_vec(&envelope).context("serializing sensor envelope")?;
        let seq = self.next_seq();
        self.writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: self.sensor_channel,
                sequence: seq,
                log_time,
                publish_time,
            },
            &data,
        )?;
        Ok(())
    }

    /// Append one payload-less message to `/sensor/__meta` (exporter parity for
    /// an iteration where the sensor produced nothing).
    fn write_sensor_meta(
        &mut self,
        tov: Tov,
        process_time: PartialCuTimeRange,
        status_txt: &str,
    ) -> Result<()> {
        let (log_time, publish_time) = self.stamps(tov);
        let envelope = MetaEnvelope {
            tov: TovEnvelope::from(tov),
            process_time,
            status_txt,
            payload_missing: true,
        };
        let data = serde_json::to_vec(&envelope).context("serializing meta envelope")?;
        let seq = self.next_seq();
        self.writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: self.sensor_meta_channel,
                sequence: seq,
                log_time,
                publish_time,
            },
            &data,
        )?;
        Ok(())
    }

    /// Append one trigger message on the trigger topic: the unwrapped momentedge
    /// `Trigger` JSON, stamped from `tov`. Its record stamps are what clipper
    /// anchors the clip window on.
    fn write_trigger(&mut self, trigger: &TriggerWire, tov: Tov) -> Result<()> {
        let (log_time, publish_time) = self.stamps(tov);
        let data = serde_json::to_vec(trigger).context("serializing trigger")?;
        let seq = self.next_seq();
        self.writer.write_to_known_channel(
            &mcap::records::MessageHeader {
                channel_id: self.trigger_channel,
                sequence: seq,
                log_time,
                publish_time,
            },
            &data,
        )?;
        Ok(())
    }

    /// Flush buffered bytes through to the sink. Unchunked, this is a no-op
    /// chunk-finish followed by a passthrough flush of the underlying writer,
    /// so a `BufWriter<File>` reaches the file — the tail's visibility latency
    /// equals the flush cadence.
    fn flush(&mut self) -> Result<()> {
        self.writer.flush().context("flushing mcap writer")?;
        Ok(())
    }

    /// Write the summary + footer + closing magic and finalise the file.
    fn finish(mut self) -> Result<()> {
        self.writer.finish().context("finalising mcap")?;
        Ok(())
    }

    /// Finalise, then reclaim the underlying sink so tests can read the
    /// produced bytes back.
    #[cfg(test)]
    fn finish_into_inner(mut self) -> Result<W> {
        self.writer.finish().context("finalising mcap")?;
        Ok(self.writer.into_inner())
    }
}

// ---- The task graph --------------------------------------------------------

pub mod tasks {
    use bincode::{Decode, Encode};

    use super::*;

    /// The synthetic sensor payload. The `CuMsgPayload` bound is the blanket
    /// `Default + Debug + Clone + Encode + Decode + Serialize + DeserializeOwned
    /// + Reflect`; `Reflect` is free in the default feature set and its derive
    /// (from the prelude) is a no-op. `Encode`/`Decode` come from the
    /// `cu-bincode` fork imported as `bincode`.
    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct SensorSample {
        pub seq: u32,
        pub value: f64,
    }

    /// The trigger command routed to the sink. It deliberately has no
    /// `trigger_time` field: the mcap interface anchors on the trigger record's
    /// own stamp and rejects a non-zero payload stamp, so the sink writes a
    /// constant-zero `trigger_time` on the wire. `preroll`/`postroll` are
    /// nanoseconds either side of the anchor.
    #[derive(Default, Debug, Clone, Encode, Decode, Serialize, Deserialize, Reflect)]
    pub struct TriggerCmd {
        pub name: String,
        pub description: String,
        pub preroll: u64,
        pub postroll: u64,
    }

    /// Source #1: a ~50 Hz synthetic sensor. Every iteration it stamps the
    /// robot-clock capture time into `tov` and emits an incrementing sample.
    #[derive(Default, Reflect)]
    pub struct SensorSource {
        seq: u32,
    }

    impl Freezable for SensorSource {}

    impl CuSrcTask for SensorSource {
        type Output<'m> = output_msg!(SensorSample);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self::default())
        }

        fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            self.seq = self.seq.wrapping_add(1);
            new_msg.tov = Tov::Time(ctx.clock.now());
            new_msg.set_payload(SensorSample {
                seq: self.seq,
                value: (self.seq as f64 * 0.05).sin(),
            });
            Ok(())
        }
    }

    /// Source #2: a periodic trigger. The graph runs at 50 Hz, so this task
    /// decimates internally — on every non-fire iteration it clears its payload
    /// (the sink then sees `None` for the trigger slot and writes nothing);
    /// once a period elapses it fires with a fresh `TriggerCmd`.
    #[derive(Reflect)]
    pub struct TriggerSource {
        #[reflect(ignore)]
        next_fire: CuTime,
        #[reflect(ignore)]
        period: CuDuration,
        count: u32,
    }

    impl Default for TriggerSource {
        fn default() -> Self {
            TriggerSource {
                next_fire: CuTime::from_nanos(0),
                period: CuDuration::from_secs(TRIGGER_PERIOD_SECS),
                count: 0,
            }
        }
    }

    impl Freezable for TriggerSource {}

    impl CuSrcTask for TriggerSource {
        type Output<'m> = output_msg!(TriggerCmd);
        type Resources<'r> = ();

        fn new(_config: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            Ok(Self::default())
        }

        fn process(&mut self, ctx: &CuContext, new_msg: &mut Self::Output<'_>) -> CuResult<()> {
            let now = ctx.clock.now();
            if now >= self.next_fire {
                self.next_fire = now + self.period;
                self.count += 1;
                new_msg.tov = Tov::Time(now);
                new_msg.set_payload(TriggerCmd {
                    name: format!("periodic-{}", self.count),
                    description: "periodic demo trigger from cu-mcap-record".into(),
                    preroll: TRIGGER_PREROLL_NS,
                    postroll: TRIGGER_POSTROLL_NS,
                });
            } else {
                new_msg.clear_payload();
            }
            Ok(())
        }
    }

    /// The sink: two typed inputs (sensor, then trigger — the cnx order in
    /// `copperconfig.ron`), appended to the [`Recording`]. It opens the file in
    /// `start` (where the clock, and thus the epoch offset, is first reachable)
    /// and finalises it in `stop`.
    #[derive(Reflect)]
    pub struct RecordSink {
        #[reflect(ignore)]
        out_dir: String,
        #[reflect(ignore)]
        recording: Option<Recording<BufWriter<File>>>,
    }

    impl Freezable for RecordSink {}

    impl CuSinkTask for RecordSink {
        type Input<'m> = input_msg!('m, SensorSample, TriggerCmd);
        type Resources<'r> = ();

        fn new(config: Option<&ComponentConfig>, _res: Self::Resources<'_>) -> CuResult<Self>
        where
            Self: Sized,
        {
            let out_dir = config
                .and_then(|c| c.get::<String>("out_dir").ok().flatten())
                .unwrap_or_else(|| DEFAULT_OUT_DIR.to_string());
            Ok(RecordSink {
                out_dir,
                recording: None,
            })
        }

        fn start(&mut self, ctx: &CuContext) -> CuResult<()> {
            // The clock reaches the task only through CuContext, never new().
            // Sample the robot-clock -> Unix-epoch offset here, once.
            let robot_now_ns = ctx.clock.now().as_nanos() as i128;
            let epoch_now = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map_err(|e| CuError::new_with_cause("system clock before Unix epoch", e))?;
            let epoch_offset_ns = epoch_now.as_nanos() as i128 - robot_now_ns;

            std::fs::create_dir_all(&self.out_dir)
                .map_err(|e| CuError::new_with_cause("creating output directory", e))?;
            // A per-run timestamped name so repeated runs never collide.
            let path =
                Path::new(&self.out_dir).join(format!("recording-{}.mcap", epoch_now.as_secs()));
            let file = File::create(&path)
                .map_err(|e| CuError::new_with_cause("creating recording file", e))?;

            let recording = Recording::new(BufWriter::new(file), epoch_offset_ns)
                .map_err(|e| CuError::from(format!("opening recording: {e:#}")))?;
            self.recording = Some(recording);
            println!("cu-mcap-record: writing Recording to {}", path.display());
            Ok(())
        }

        fn process(&mut self, _ctx: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
            let (sensor_msg, trigger_msg) = input;
            let recording = self
                .recording
                .as_mut()
                .ok_or_else(|| CuError::from("recording is not open"))?;

            // Sensor slot: a payload lands on /sensor, its absence on
            // /sensor/__meta (in practice the 50 Hz sensor always has a payload,
            // but the meta path exists for exporter parity and is tested).
            match sensor_msg.payload() {
                Some(sample) => recording
                    .write_sensor(
                        sample,
                        sensor_msg.tov,
                        sensor_msg.metadata.process_time,
                        sensor_msg.metadata.status_txt.0.as_str(),
                    )
                    .map_err(|e| CuError::from(format!("writing sensor: {e:#}")))?,
                None => recording
                    .write_sensor_meta(
                        sensor_msg.tov,
                        sensor_msg.metadata.process_time,
                        sensor_msg.metadata.status_txt.0.as_str(),
                    )
                    .map_err(|e| CuError::from(format!("writing sensor meta: {e:#}")))?,
            }

            // Trigger slot: a fired trigger is written and flushed immediately
            // so it reaches the tail without waiting for the postprocess flush;
            // a cleared slot writes nothing (the one channel with no meta path).
            if let Some(cmd) = trigger_msg.payload() {
                let wire = TriggerWire::new(
                    cmd.name.clone(),
                    cmd.description.clone(),
                    cmd.preroll,
                    cmd.postroll,
                );
                recording
                    .write_trigger(&wire, trigger_msg.tov)
                    .map_err(|e| CuError::from(format!("writing trigger: {e:#}")))?;
                recording
                    .flush()
                    .map_err(|e| CuError::from(format!("flushing after trigger: {e:#}")))?;
            }
            Ok(())
        }

        fn postprocess(&mut self, _ctx: &CuContext) -> CuResult<()> {
            if let Some(recording) = self.recording.as_mut() {
                recording
                    .flush()
                    .map_err(|e| CuError::from(format!("flushing recording: {e:#}")))?;
            }
            Ok(())
        }

        fn stop(&mut self, _ctx: &CuContext) -> CuResult<()> {
            if let Some(recording) = self.recording.take() {
                recording
                    .finish()
                    .map_err(|e| CuError::from(format!("finalising recording: {e:#}")))?;
            }
            Ok(())
        }
    }
}

// `copper_runtime` reads `copperconfig.ron` at compile time and generates the
// `RecorderApp` struct and its builder. (The attribute macro rejects a doc
// comment on the item it rewrites, so this note stays a plain comment.)
#[copper_runtime(config = "copperconfig.ron")]
struct RecorderApp {}

/// CLI: the output directory. Everything else — the graph, rates, cadence — is
/// a hardcoded constant or lives in `copperconfig.ron`; this example
/// demonstrates a producer path, not a tuning surface.
#[derive(Parser)]
#[command(about = "Copper CuSinkTask writing a clipper-tailable MCAP Recording (no ROS).")]
struct Args {
    /// Directory the timestamped `recording-<unix-seconds>.mcap` is written
    /// into (created if absent).
    #[arg(long, default_value = DEFAULT_OUT_DIR)]
    out: String,
}

fn main() -> Result<()> {
    let args = Args::parse();

    // Override the sink's out_dir from --out so the value is visible in the
    // effective config, then build with the default NoopLogger — the Recording
    // is the only on-disk artifact (no `.copper` unified-log slab).
    let mut cfg = CuConfig::deserialize_ron(include_str!("../copperconfig.ron"))
        .map_err(|e| anyhow!("parsing embedded config: {e}"))?;
    let graph = cfg
        .get_graph_mut(None)
        .map_err(|e| anyhow!("reading config graph: {e}"))?;
    let sink_id = graph
        .get_node_id_by_name("sink")
        .ok_or_else(|| anyhow!("no 'sink' node in copperconfig.ron"))?;
    graph
        .get_node_mut(sink_id)
        .ok_or_else(|| anyhow!("'sink' node vanished from the graph"))?
        .set_param::<String>("out_dir", args.out.clone());

    let mut app = RecorderApp::builder()
        .with_config(cfg)
        .build()
        .map_err(|e| anyhow!("building copper app: {e}"))?;

    // Ctrl+C flips the flag so the bounded loop stops and reaches stop_all_tasks
    // (and the sink's finish()) rather than dying mid-write.
    let stop = Arc::new(AtomicBool::new(false));
    signal_hook::flag::register(signal_hook::consts::SIGINT, Arc::clone(&stop))?;

    app.start_all_tasks()
        .map_err(|e| anyhow!("starting tasks: {e}"))?;
    println!(
        "cu-mcap-record: recording to {}/ — Ctrl+C to stop",
        args.out
    );
    loop {
        app.run_one_iteration()
            .map_err(|e| anyhow!("running iteration: {e}"))?;
        // Pace the loop, then stop after the sleep (before the next iteration)
        // so Ctrl+C lands between iterations and reaches stop_all_tasks below.
        std::thread::sleep(LOOP_PERIOD);
        if stop.load(Ordering::Relaxed) {
            break;
        }
    }
    app.stop_all_tasks()
        .map_err(|e| anyhow!("stopping tasks: {e}"))?;
    println!("cu-mcap-record: stopped, Recording finalised");
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io::Cursor;

    use serde_json::Value;

    use super::*;

    /// A nanosecond value comfortably above the epoch floor, so test stamps are
    /// accepted by `assert_absolute_unix_ns`.
    const BASE_NS: u64 = 1_700_000_000_000_000_000; // 2023-11-14T22:13:20Z

    /// Build a `PartialCuTimeRange` from two concrete robot-clock nanosecond
    /// bounds (the sink passes the message's own `process_time`; tests supply
    /// their own).
    fn process_time(start_ns: u64, end_ns: u64) -> PartialCuTimeRange {
        PartialCuTimeRange {
            start: CuTime::from_nanos(start_ns).into(),
            end: CuTime::from_nanos(end_ns).into(),
        }
    }

    /// A `Write + Seek` sink that panics if any write lands below the current
    /// end of the buffer — it *proves*, at the sink level, that the writer only
    /// ever appends and never rewrites a byte already written (the tailability
    /// contract). Position queries and seeks are tolerated; only a write below
    /// the end faults.
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

    /// Walk the top-level record framing by hand, exactly the way clipper's
    /// tail does: 8-byte leading magic, then records of 1-byte opcode + u64le
    /// length. Returns `(complete_records, final_offset)` for the records that
    /// fit entirely below `end`, panicking on any length no valid record can
    /// have — the `u64::MAX` back-patch placeholder above all.
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

    /// Mirrors `crates/clipper/src/trigger.rs::Stamp`'s `Deserialize` shape — a
    /// local copy so this crate stays free of any dependency on
    /// `crates/clipper` (which pulls in `r2r`/ROS), while still proving the JSON
    /// this program writes decodes into that exact field shape.
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

    /// Drive a Recording over a `Cursor`: `n` sensor samples on `Tov::Time`,
    /// then one trigger, then finalise; return the produced bytes.
    fn sample_recording(n: u32) -> Vec<u8> {
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        for seq in 0..n {
            let t = BASE_NS + seq as u64 * 20_000_000;
            let sample = tasks::SensorSample {
                seq,
                value: (seq as f64 * 0.05).sin(),
            };
            rec.write_sensor(
                &sample,
                Tov::Time(CuTime::from_nanos(t)),
                process_time(t, t + 1_000),
                "",
            )
            .expect("write sensor");
        }
        let trigger = TriggerWire::new(
            "periodic-1".to_string(),
            "periodic demo trigger from cu-mcap-record".to_string(),
            TRIGGER_PREROLL_NS,
            TRIGGER_POSTROLL_NS,
        );
        rec.write_trigger(&trigger, Tov::Time(CuTime::from_nanos(BASE_NS)))
            .expect("write trigger");
        rec.finish_into_inner().expect("finish").into_inner()
    }

    /// A produced Recording reads back with copper's MCAP-exporter conventions:
    /// the `/sensor` data channel and the trigger channel are `json`-encoded;
    /// the sensor envelope carries exactly `payload`/`tov`/`process_time`/
    /// `status_txt` with the friendly `tov` shape; and the registered schemas
    /// are named `copper.sensor` and `momentedge.trigger`.
    #[test]
    fn writes_a_readable_mcap_with_exporter_conventions() {
        let v = BASE_NS + 40_000_000;
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        let sample = tasks::SensorSample { seq: 2, value: 0.5 };
        rec.write_sensor(
            &sample,
            Tov::Time(CuTime::from_nanos(v)),
            process_time(v, v + 500),
            "",
        )
        .expect("write sensor");
        let trigger = TriggerWire::new(
            "periodic-1".to_string(),
            "demo".to_string(),
            TRIGGER_PREROLL_NS,
            TRIGGER_POSTROLL_NS,
        );
        rec.write_trigger(&trigger, Tov::Time(CuTime::from_nanos(BASE_NS)))
            .expect("write trigger");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        let mut topics = std::collections::BTreeSet::new();
        let mut saw_sensor = false;
        let mut saw_trigger = false;
        for msg in mcap::MessageStream::new(&bytes).expect("stream") {
            let msg = msg.expect("message");
            topics.insert(msg.channel.topic.clone());
            assert_eq!(msg.channel.message_encoding, "json");
            let schema = msg.channel.schema.as_ref().expect("channel has a schema");

            if msg.channel.topic == "/sensor" {
                saw_sensor = true;
                assert_eq!(schema.name, "copper.sensor");
                let body: Value = serde_json::from_slice(&msg.data).expect("sensor json");
                let obj = body.as_object().expect("sensor body is an object");
                let keys: std::collections::BTreeSet<&str> =
                    obj.keys().map(String::as_str).collect();
                assert_eq!(
                    keys,
                    ["payload", "process_time", "status_txt", "tov"]
                        .into_iter()
                        .collect(),
                    "sensor envelope keys"
                );
                assert_eq!(body["tov"]["kind"], "time");
                assert_eq!(body["tov"]["time_ns"], v);
                assert_eq!(body["tov"]["start_ns"], v);
                assert_eq!(body["tov"]["end_ns"], v);
                assert_eq!(body["payload"]["seq"], 2);
            } else if msg.channel.topic == TRIGGER_TOPIC {
                saw_trigger = true;
                assert_eq!(schema.name, "momentedge.trigger");
            }
        }
        assert!(topics.contains("/sensor"), "the /sensor topic is present");
        assert!(
            topics.contains(TRIGGER_TOPIC),
            "the trigger topic is present"
        );
        assert!(saw_sensor && saw_trigger);
    }

    /// The writer emits no top-level `Chunk` record: a live tailer only ever
    /// sees complete `Message` records, never a chunk whose length is
    /// back-patched on close. This is the mutation check on `use_chunks(false)`.
    #[test]
    fn writer_emits_unchunked_output_for_live_tailing() {
        let bytes = sample_recording(5);
        let mut messages = 0;
        for record in mcap::read::LinearReader::new(&bytes).expect("reader") {
            match record.expect("record") {
                mcap::records::Record::Chunk { .. } => {
                    panic!("unchunked output must contain no Chunk records")
                }
                mcap::records::Record::Message { .. } => messages += 1,
                _ => {}
            }
        }
        assert_eq!(messages, 6, "5 sensor messages + 1 trigger, each a Message");
    }

    /// A truncated prefix — what a live tailer sees mid-write — is always a
    /// clean run of complete records followed by at most one in-progress
    /// record, never a framing fault. Cuts land at arbitrary byte positions,
    /// including mid-record. Includes the `u64::MAX` placeholder assertion in
    /// `walk_complete_records`.
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

    /// A whole Recording session — schemas, channels, data, a trigger, a flush,
    /// and `finish()` — runs through the append-only sink without ever writing
    /// below the current end of the buffer, proving the writer only appends.
    #[test]
    fn a_full_session_only_ever_appends() {
        let sink = AppendOnly {
            buf: Vec::new(),
            pos: 0,
        };
        let mut rec = Recording::new(sink, 0).expect("open recording");
        for seq in 0..50u32 {
            let t = BASE_NS + seq as u64 * 20_000_000;
            let sample = tasks::SensorSample { seq, value: 0.0 };
            rec.write_sensor(
                &sample,
                Tov::Time(CuTime::from_nanos(t)),
                process_time(t, t + 1_000),
                "",
            )
            .expect("write sensor");
        }
        // Also exercise the meta path and the mid-stream flush inside the guard.
        rec.write_sensor_meta(Tov::None, PartialCuTimeRange::default(), "")
            .expect("write meta");
        rec.flush().expect("flush");
        let trigger = TriggerWire::new(
            "periodic-1".to_string(),
            "demo".to_string(),
            TRIGGER_PREROLL_NS,
            TRIGGER_POSTROLL_NS,
        );
        rec.write_trigger(&trigger, Tov::Time(CuTime::from_nanos(BASE_NS)))
            .expect("write trigger");

        let sink = rec.finish_into_inner().expect("finish");
        let (records, final_pos) = walk_complete_records(&sink.buf, sink.buf.len());
        assert!(records > 0);
        assert_eq!(
            &sink.buf[final_pos..],
            mcap::MAGIC,
            "the record walk lands on the closing magic"
        );
    }

    /// The trigger record decodes into the local mirror of clipper's `Trigger`
    /// shape: `trigger_time` is the constant `{sec: 0, nanosec: 0}`, the
    /// preroll/postroll survive, and the description round-trips.
    #[test]
    fn trigger_decodes_into_clippers_shape() {
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        let trigger = TriggerWire::new(
            "periodic-1".to_string(),
            "periodic demo trigger from cu-mcap-record".to_string(),
            TRIGGER_PREROLL_NS,
            TRIGGER_POSTROLL_NS,
        );
        rec.write_trigger(&trigger, Tov::Time(CuTime::from_nanos(BASE_NS)))
            .expect("write trigger");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        let msg = mcap::MessageStream::new(&bytes)
            .expect("stream")
            .next()
            .expect("one message")
            .expect("message");
        assert_eq!(msg.channel.topic, TRIGGER_TOPIC);
        assert_eq!(msg.channel.message_encoding, "json");
        let decoded: TestTrigger = serde_json::from_slice(&msg.data).expect("trigger decodes");
        assert_eq!(
            decoded,
            TestTrigger {
                name: "periodic-1".to_string(),
                description: "periodic demo trigger from cu-mcap-record".to_string(),
                trigger_time: TestStamp { sec: 0, nanosec: 0 },
                preroll: TRIGGER_PREROLL_NS,
                postroll: TRIGGER_POSTROLL_NS,
            }
        );
    }

    /// Read back the single message's `(log_time, publish_time)` pair from a
    /// one-shot Recording, for the timestamp-convention tests below.
    fn one_shot_stamps(
        offset: i128,
        write: impl FnOnce(&mut Recording<Cursor<Vec<u8>>>),
    ) -> (u64, u64) {
        let mut rec = Recording::new(Cursor::new(Vec::new()), offset).expect("open recording");
        write(&mut rec);
        let bytes = rec.finish_into_inner().expect("finish").into_inner();
        let msg = mcap::MessageStream::new(&bytes)
            .expect("stream")
            .next()
            .expect("one message")
            .expect("message");
        (msg.log_time, msg.publish_time)
    }

    /// `Tov::Time`: `publish_time` is the time-of-validity translated through
    /// the epoch offset, distinct from `log_time` (the wall clock at write).
    #[test]
    fn publish_time_maps_tov_time_through_the_offset() {
        let offset: i128 = 1_000_000_000;
        let (log_time, publish_time) = one_shot_stamps(offset, |rec| {
            let sample = tasks::SensorSample { seq: 1, value: 0.0 };
            rec.write_sensor(
                &sample,
                Tov::Time(CuTime::from_nanos(BASE_NS)),
                PartialCuTimeRange::default(),
                "",
            )
            .expect("write sensor");
        });
        assert_eq!(publish_time, (BASE_NS as i128 + offset) as u64);
        assert_ne!(publish_time, log_time);
    }

    /// `Tov::None`: `publish_time` falls back to `log_time`, so the message
    /// stays windowable on the publish source.
    #[test]
    fn publish_time_falls_back_to_log_time_for_tov_none() {
        let (log_time, publish_time) = one_shot_stamps(0, |rec| {
            let sample = tasks::SensorSample { seq: 1, value: 0.0 };
            rec.write_sensor(&sample, Tov::None, PartialCuTimeRange::default(), "")
                .expect("write sensor");
        });
        assert_eq!(publish_time, log_time);
    }

    /// `Tov::Range`: `publish_time` is the range start translated through the
    /// offset, and the envelope's `tov` carries the `range` mapping
    /// (`kind: "range"`, `time_ns: 0`, and the range bounds in `start_ns`/
    /// `end_ns`) — guarding the `From<Tov>` Range arm's exact shape.
    #[test]
    fn publish_time_maps_tov_range_start_through_the_offset() {
        let offset: i128 = 500_000_000;
        let start = BASE_NS;
        let end = BASE_NS + 2_000;
        let mut rec = Recording::new(Cursor::new(Vec::new()), offset).expect("open recording");
        let sample = tasks::SensorSample { seq: 1, value: 0.0 };
        rec.write_sensor(
            &sample,
            Tov::Range(CuTimeRange {
                start: CuTime::from_nanos(start),
                end: CuTime::from_nanos(end),
            }),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write sensor");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        let msg = mcap::MessageStream::new(&bytes)
            .expect("stream")
            .next()
            .expect("one message")
            .expect("message");
        assert_eq!(msg.publish_time, (start as i128 + offset) as u64);
        let body: Value = serde_json::from_slice(&msg.data).expect("sensor json");
        assert_eq!(body["tov"]["kind"], "range");
        assert_eq!(body["tov"]["time_ns"], 0);
        assert_eq!(body["tov"]["start_ns"], start);
        assert_eq!(body["tov"]["end_ns"], end);
    }

    /// `log_time` is the wall clock sampled at write: it lands within a bracket
    /// taken immediately before and after the write.
    #[test]
    fn log_time_is_wall_clock_at_write() {
        let before = now_ns();
        let (log_time, _publish_time) = one_shot_stamps(0, |rec| {
            let sample = tasks::SensorSample { seq: 1, value: 0.0 };
            rec.write_sensor(&sample, Tov::None, PartialCuTimeRange::default(), "")
                .expect("write sensor");
        });
        let after = now_ns();
        assert!(
            before <= log_time && log_time <= after,
            "log_time {log_time} must fall within [{before}, {after}]"
        );
    }

    /// The epoch floor (2020-01-01T00:00:00Z exactly) is accepted.
    #[test]
    fn epoch_floor_boundary_is_accepted() {
        assert_absolute_unix_ns("test", EPOCH_FLOOR_NS);
    }

    /// One nanosecond below the floor panics.
    #[test]
    #[should_panic(expected = "absolute Unix-epoch nanoseconds")]
    fn one_nanosecond_below_epoch_floor_panics() {
        assert_absolute_unix_ns("test", EPOCH_FLOOR_NS - 1);
    }

    /// A payload-less sensor write is routed to `/sensor/__meta` under the
    /// `copper.meta` schema, its body carrying `payload_missing: true` and no
    /// `payload` key.
    #[test]
    fn payload_less_sensor_write_routes_to_meta_channel() {
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        rec.write_sensor_meta(
            Tov::Time(CuTime::from_nanos(BASE_NS)),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write meta");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        let msg = mcap::MessageStream::new(&bytes)
            .expect("stream")
            .next()
            .expect("one message")
            .expect("message");
        assert_eq!(msg.channel.topic, "/sensor/__meta");
        assert_eq!(
            msg.channel.schema.as_ref().expect("schema").name,
            "copper.meta"
        );
        let body: Value = serde_json::from_slice(&msg.data).expect("meta json");
        let obj = body.as_object().expect("meta body is an object");
        let keys: std::collections::BTreeSet<&str> = obj.keys().map(String::as_str).collect();
        assert_eq!(
            keys,
            ["payload_missing", "process_time", "status_txt", "tov"]
                .into_iter()
                .collect(),
            "meta envelope keys (no payload key)"
        );
        assert_eq!(obj["payload_missing"], Value::Bool(true));
    }

    /// The registered schemas parse as JSON and carry the exporter's required
    /// fields: the `copper.sensor` envelope requires
    /// `payload`/`tov`/`process_time`/`status_txt`, and `copper.meta` requires
    /// `payload_missing` and pins it to the const `true`.
    #[test]
    fn registered_schemas_match_exporter_shape() {
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        let sample = tasks::SensorSample { seq: 1, value: 0.0 };
        rec.write_sensor(
            &sample,
            Tov::Time(CuTime::from_nanos(BASE_NS)),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write sensor");
        rec.write_sensor_meta(Tov::None, PartialCuTimeRange::default(), "")
            .expect("write meta");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        let mut sensor_schema = None;
        let mut meta_schema = None;
        for msg in mcap::MessageStream::new(&bytes).expect("stream") {
            let msg = msg.expect("message");
            let schema = msg.channel.schema.as_ref().expect("schema");
            let json: Value = serde_json::from_slice(&schema.data).expect("schema json");
            match schema.name.as_str() {
                "copper.sensor" => sensor_schema = Some(json),
                "copper.meta" => meta_schema = Some(json),
                _ => {}
            }
        }

        let sensor = sensor_schema.expect("copper.sensor registered");
        let required: std::collections::BTreeSet<&str> = sensor["required"]
            .as_array()
            .expect("required array")
            .iter()
            .map(|v| v.as_str().expect("string"))
            .collect();
        assert_eq!(
            required,
            ["payload", "process_time", "status_txt", "tov"]
                .into_iter()
                .collect()
        );

        let meta = meta_schema.expect("copper.meta registered");
        assert_eq!(
            meta["properties"]["payload_missing"]["const"],
            Value::Bool(true)
        );
        let meta_required: std::collections::BTreeSet<&str> = meta["required"]
            .as_array()
            .expect("required array")
            .iter()
            .map(|v| v.as_str().expect("string"))
            .collect();
        assert!(meta_required.contains("payload_missing"));
    }

    /// One global sequence number advances by exactly one per record across
    /// every channel, in write order: read a mixed sensor / meta / trigger
    /// stream back and assert the sequences are `0, 1, 2, …`. This is the
    /// exporter-parity contract — a per-channel or constant sequence fails it.
    #[test]
    fn sequence_numbers_increase_monotonically_across_all_channels() {
        let mut rec = Recording::new(Cursor::new(Vec::new()), 0).expect("open recording");
        // A deliberately mixed write order over all three channels.
        let sample = tasks::SensorSample { seq: 0, value: 0.0 };
        rec.write_sensor(
            &sample,
            Tov::Time(CuTime::from_nanos(BASE_NS)),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write sensor");
        rec.write_sensor_meta(Tov::None, PartialCuTimeRange::default(), "")
            .expect("write meta");
        rec.write_sensor(
            &sample,
            Tov::Time(CuTime::from_nanos(BASE_NS)),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write sensor");
        let trigger = TriggerWire::new(
            "periodic-1".to_string(),
            "demo".to_string(),
            TRIGGER_PREROLL_NS,
            TRIGGER_POSTROLL_NS,
        );
        rec.write_trigger(&trigger, Tov::Time(CuTime::from_nanos(BASE_NS)))
            .expect("write trigger");
        rec.write_sensor(
            &sample,
            Tov::Time(CuTime::from_nanos(BASE_NS)),
            PartialCuTimeRange::default(),
            "",
        )
        .expect("write sensor");
        let bytes = rec.finish_into_inner().expect("finish").into_inner();

        // Unchunked output is a run of top-level Message records in write order,
        // so the stream yields them in that order.
        let sequences: Vec<u32> = mcap::MessageStream::new(&bytes)
            .expect("stream")
            .map(|m| m.expect("message").sequence)
            .collect();
        assert_eq!(
            sequences,
            vec![0, 1, 2, 3, 4],
            "one global sequence, +1 per record in write order across all channels"
        );
    }

    /// The `TestTrigger` mirror's `#[serde(default)]` on `description` matches
    /// clipper's optional field: a trigger JSON with `description` omitted
    /// decodes with an empty description, the rest required.
    #[test]
    fn trigger_mirror_defaults_description_when_omitted() {
        let json = br#"{"name":"periodic-1","trigger_time":{"sec":0,"nanosec":0},"preroll":1,"postroll":2}"#;
        let decoded: TestTrigger =
            serde_json::from_slice(json).expect("decodes without description");
        assert_eq!(
            decoded,
            TestTrigger {
                name: "periodic-1".to_string(),
                description: String::new(),
                trigger_time: TestStamp { sec: 0, nanosec: 0 },
                preroll: 1,
                postroll: 2,
            }
        );
    }
}
