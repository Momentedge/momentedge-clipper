# cu-mcap-record — a copper CuSinkTask writing a clipper-tailable Recording

A standalone [copper (cu29)](https://github.com/copper-project/copper-rs)
application whose sink task appends routed task outputs to an MCAP **Recording**
that `clipper --interface mcap` tails live — the copper **Producer** path beside
the two plain-`mcap`-crate writer examples,
[`custom-mcap-writer`](../custom-mcap-writer/README.md) (unchunked) and
[`chunked-mcap-writer`](../chunked-mcap-writer/README.md) (buffered chunks +
zstd). No ROS stack is involved anywhere: the **Trigger** travels in-band,
written into the Recording on clipper's default trigger topic, and the **Clip**'s
appearance in clipper's output directory is the only completion signal.

The task graph is three tasks wired in `copperconfig.ron`: a ~50 Hz synthetic
sensor source, a periodic (~10 s) trigger source, and the `RecordSink`. The sink
receives both sources as two typed inputs and appends each to the Recording,
mirroring copper's MCAP-exporter conventions — a `copper.<task>` jsonschema
schema and a `/<task>` JSON channel per data input, the export envelope
(`payload`, `tov`, `process_time`, `status_txt`), and payload-less messages on a
`/<task>/__meta` channel under a shared `copper.meta` schema. The whole Producer
path is one file, `src/main.rs`: the `Recording<W>` writer core, the JSON
envelope, the three tasks, and `main`.

## Two divergences from copper's MCAP exporter

Copper ships an offline MCAP exporter that runs over a *finished* unified log.
This sink follows its schema and envelope conventions so Recordings and Clips
render in [Foxglove](https://foxglove.dev) exactly like copper's offline
exports, but diverges from it in two deliberate ways a live clipper Producer
requires.

### 1. An unchunked, append-only writer

A Producer clipper can tail live appends complete top-level records only and
never seeks back to rewrite one ([ARCHITECTURE.md, "Tailing a live
MCAP"](../../ARCHITECTURE.md#tailing-a-live-mcap)). Copper's exporter uses the
`mcap` crate's default chunked writer, which back-patches each chunk's length on
close by seeking back — unparseable mid-write. This sink builds its writer with

```rust
mcap::WriteOptions::new().use_chunks(false)
```

so every message is one complete top-level `Message` record, appended once and
never rewritten — readable the instant it reaches the file. The sink wraps the
file in a `BufWriter` and flushes explicitly (through `mcap::Writer::flush`,
which in unchunked mode passes straight through to the file) at the end of every
iteration and immediately after each Trigger, so the tail's visibility latency
equals the flush cadence rather than the OS buffer's.

### 2. The Unix-epoch time domain

Copper's exporter stamps records with raw robot-clock nanoseconds. Clipper's
retention, restart recovery, and **Coverage** semantics are defined on absolute
wall-clock time, and a cross-run robot clock resets to zero — so this sink
stamps in the Unix-epoch domain instead, computed in one tested place inside
`Recording`:

- **Log time** is the system wall clock sampled at write. A single writer stamps
  the Recording, so log times are approximately non-decreasing — which is what
  makes Coverage on the `log` source a completeness proof.
- **Publish time** carries the capture time: the message's `tov` translated
  through a robot-clock→epoch offset sampled once at sink start. `Tov::Time(t)`
  maps to `t + offset`, `Tov::Range` to its start `+ offset`, and `Tov::None`
  falls back to `log_time`, so every message stays windowable on either time
  source. This is what lets clipper window Clips on capture time with
  `--time-source publish`.

The raw robot-clock `tov` still travels untouched inside the JSON envelope. Both
stamps are asserted to be absolute Unix-epoch nanoseconds (a 2020-01-01 floor)
before they are written, so a monotonic or unit-mistaken stamp fails loudly
rather than silently producing empty Clips.

## What it writes

| topic | payload | message encoding | schema | cadence |
|---|---|---|---|---|
| `/sensor` | a `SensorSample { seq, value }` wrapped in the copper export envelope → JSON | json | `copper.sensor` (jsonschema) | every ~20 ms (~50 Hz) |
| `/sensor/__meta` | the payload-less export envelope (`payload_missing: true`), for an iteration the sensor produced nothing | json | `copper.meta` (jsonschema) | only when the sensor slot is empty |
| `/events/momentedge/trigger` | a `momentedge_msgs/Trigger`-shaped JSON object, written **unwrapped** (not in the envelope) | json | `momentedge.trigger` (jsonschema) | every ~10 s (first on startup) |

The trigger channel is the one channel with no exporter parity: clipper's decoder
reads the `Trigger` fields at the top level, so the sink writes them unwrapped.
The payload carries `preroll` = `postroll` = 3 s and a constant `trigger_time` of
`{"sec": 0, "nanosec": 0}`: under `--interface mcap` clipper resolves the
**Anchor** from the Trigger record's own MCAP stamp, and its admission gate
rejects a non-zero payload `trigger_time`.

## Run

Run it from the repo root with `-p` — it is a workspace member:

```bash
cargo run -p cu-mcap-record -- --out out   # writes out/recording-<unix-seconds>.mcap until Ctrl+C
```

`--out` is the only flag (default `out`); it names the directory the Recording is
written into, created if absent. The file name carries the start time in Unix
seconds so repeated runs never collide. The sensor rate, trigger cadence, and
preroll/postroll are hardcoded constants in `src/main.rs` — this example
demonstrates a Producer path, not a tuning surface. On Ctrl+C the bounded run
loop stops between iterations and the sink finalises the file (summary + footer +
closing magic). The app runs with copper's no-op unified logger, so the MCAP
Recording is the only artifact on disk — no `.copper` slab.

## Tailing it with clipper

Point unmodified clipper at the same directory with `--record-dir` (the tailed
Recording directory) and `--out-dir` (where finished Clips land):

```bash
clipper --interface mcap --record-dir examples/cu-mcap-record/out --out-dir clips
```

Clipper tails the growing Recording, lifts each Trigger out of it, cuts a Clip
around the Trigger's **Window**, and writes the Clip into its `--out-dir` — no
ROS, no extra IPC. To window Clips on capture time instead of log time, add
`--time-source publish`. Clipper's retention flags (see the [repo
README](../../README.md)) bound how much of the Recording is kept; this example
writes one continuous Recording and leaves pruning to clipper.

Clipper's live e2e suite drives this binary as its copper Producer fixture: the
`copper_sink_recording_produces_clip` test runs this app and unmodified clipper
together on `--interface mcap` and asserts a Clip appears and parses. The suite
provisions the binary beside the clipper binary under test (`CU_MCAP_RECORD_BIN`,
or an on-demand `-p cu-mcap-record` build).

## Production notes

- **Epoch stamping is a requirement, not a demo convenience.** On a real robot
  the sink's `publish_time` must be an absolute, PTP-disciplined wall-clock
  instant on the same scale as clipper's Anchor. The robot-clock→epoch offset
  sampled at sink start is exact on std targets, where copper's `RobotClock` is
  an offset view of the same real-time clock.
- **A mock clock breaks the offset.** Under copper's simulation or replay modes
  the `RobotClock` is driven by a mock source decoupled from the wall clock, so
  the single start-time offset no longer tracks it. Sample and apply the offset
  only where the clock is the real hardware clock.
- **Keep copper's unified logger alongside in production.** This example uses the
  default no-op logger so the Recording is unambiguous. A production app builds
  with `RecorderApp::builder().with_log_path(path, slab_size)` to retain copper's
  full unified log next to the Recording clipper tails.
