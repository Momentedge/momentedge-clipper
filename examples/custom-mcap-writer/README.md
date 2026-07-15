# custom-mcap-writer — MCAP writer with capture-time `publish_time`

A standalone program that writes an MCAP file directly with the
[`mcap`](https://crates.io/crates/mcap) crate — no ROS, no CDR. It exists to
make **capture-time windowing** real: `crates/clipper` can cut a clip on either
`log_time` or `publish_time`, but no ROS publisher, on any distro, can set
`publish_time` on the wire — rosbag2 fills it from the DDS source timestamp.
Writing MCAP records directly is the only way to put a genuine capture instant
into `publish_time`. This program is that producer, and the fixture the live
e2e suite drives clipper against for capture-time windowing.

Each data channel is an iterator that yields a generated JSON payload; every
`--period-ms` the loop pulls one sample from each channel and writes it, until
Ctrl+C or `--duration` elapses, at which point the file is finalised. Modeled
on [Foxglove's quickstart writer](https://docs.foxglove.dev/docs/sdk/example).

## Tailability: unchunked output

This writer's output is meant to be tailed **live** by `clipper --interface
mcap`, not just read after the fact — that is the deployment story capture-time
windowing is built for. A producer clipper can tail live appends complete
top-level records only and never seeks back to rewrite one. MCAP's chunked
writer (the `mcap` crate's default) buffers messages into a `Chunk` record
whose header is written with a placeholder length and back-patched once the
chunk closes, so a chunk is parseable only after that close: read mid-write,
the placeholder reads as an enormous, framing-breaking record length, and a
live tailer sees it as corruption rather than an in-progress write. This
program builds its writer with `mcap::WriteOptions::new().use_chunks(false)`,
so every message is its own complete top-level `Message` record — appended
once and never rewritten, readable the instant it hits disk. That is the same
append-only contract rosbag2's fastwrite profile gives clipper in production.
Unchunked output carries no per-chunk compression and no chunk-level index;
`mcap recover` can add indexes to a finished file if needed.

## What it writes

| topic | payload | message encoding | schema |
|---|---|---|---|
| `/pose` | a `Pose { x, y, theta }` struct → JSON (`serde_json`) | json | jsonschema |
| `/size` | a raw `{"size": N}` JSON string | json | *(schemaless)* |
| `/events/momentedge/trigger` | a `momentedge_msgs/Trigger`-shaped JSON object (optional, one-shot) | json | *(schemaless)* |

Every data message's record carries `log_time` = wall clock at write, and
`publish_time` = `log_time - publish_offset_ms` — the capture instant, offset
deliberately before receipt. The trigger message (if enabled) carries
`log_time` = wall clock at emission and `publish_time` = the capture-time
anchor; its JSON `trigger_time` field is `{"sec": 0, "nanosec": 0}` unless
`--stamp-payload-trigger-time` asks otherwise — see "Why `trigger_time` is
zero" below.

## The `publish_time` contract

**`publish_time` must be absolute wall-clock nanoseconds since the Unix epoch,
PTP-disciplined — the same scale as `log_time`, `now()`, and
`Trigger.trigger_time`.** This is load-bearing: clipper's clip window is
`[anchor − preroll, anchor + postroll]`, resolved either from a message's
`publish_time` or from a `Trigger.trigger_time`, on one time source. A relative
offset, a monotonic-clock reading (`Instant`), or a value in the wrong unit is
incomparable with that anchor and silently produces an empty or wrong-window
clip — there is no error, just data quietly missing. This program asserts the
contract rather than assuming it: every `publish_time` it writes is checked
against a 2020-01-01T00:00:00Z floor (`assert_absolute_unix_ns` in
`src/main.rs`), and panics with a clear message if it is not plausibly
absolute.

## Why `trigger_time` is zero

Which timestamp clipper resolves a clip window's anchor from is a matrix of
its interface (`ros` / `mcap`) and its time source (`log` / `publish`). Under
`--interface mcap`, on either time source, the anchor is the trigger record's
own MCAP stamp — its `log_time` or `publish_time` — never the JSON payload.
The payload's `trigger_time` field is read only under `--interface ros` with
the `publish` time source, where it stands in for the `publish_time` a ROS
publisher cannot set on the wire. A non-zero `trigger_time` in any other cell
of that matrix is a mis-anchoring hazard clipper rejects outright.

So this writer leaves the trigger payload's `trigger_time` at `{"sec": 0,
"nanosec": 0}` by default: the record's own `publish_time` (above) already
declares the anchor for the mcap interface, and a non-zero payload value would
only be misread as one. `--stamp-payload-trigger-time` opts back into stamping
it with the anchor, for producing a ros+publish-shaped payload or exercising
the rejection path directly.

## Run

```bash
cargo run -p custom-mcap-writer                       # writes ./quickstart.mcap until Ctrl+C
cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5
cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5 --trigger-after-ms 1000
cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5 --trigger-after-ms 1000 --stamp-payload-trigger-time
```

| flag | default | meaning |
|---|---|---|
| `--out <file>` | `quickstart.mcap` | output MCAP file |
| `--period-ms <ms>` | `20` | how often every data channel is sampled (50 Hz) |
| `--duration <secs>` | run until Ctrl+C | stop after this long |
| `--publish-offset-ms <ms>` | `50` | milliseconds subtracted from `log_time` to produce every data message's `publish_time` |
| `--trigger-after-ms <ms>` | unset (no trigger) | emit one `/events/momentedge/trigger` message this many milliseconds after startup |
| `--stamp-payload-trigger-time` | off (payload `trigger_time` is zero) | stamp the trigger JSON payload's `trigger_time` with the anchor instead — see "Why `trigger_time` is zero" |

No ROS environment is needed — the crate carries no r2r dependency and builds
with the system toolchain. Open the result in [Foxglove](https://foxglove.dev)
or inspect it with the [`mcap` CLI](https://github.com/foxglove/mcap); feed it
to `clipper --interface mcap` to exercise capture-time clip windows end to end.

## How it works

MCAP stores opaque message bytes and labels each channel with a
`message_encoding`; it does no serialization itself. So the writer just picks a
format and writes bytes:

- **`/pose`** — a `#[derive(Serialize)]` struct turned into JSON with
  `serde_json::to_vec`, on a channel declared `message_encoding = "json"` with a
  JSON Schema so viewers can render it. The typed way.
- **`/size`** — a raw JSON string built with `format!`, on a schemaless channel.
  The most basic payload there is: no schema, no serializer, just bytes.
- **`/events/momentedge/trigger`** — a JSON object matching the shape
  `crates/clipper` decodes for a `json`-encoded trigger
  (`crates/clipper/src/trigger.rs::Trigger` / `crates/clipper/src/decode.rs`):
  `{"name", "description", "trigger_time": {"sec", "nanosec"}, "preroll",
  "postroll"}`, `preroll`/`postroll` in nanoseconds either side of the anchor.
  `trigger_time` is `{"sec": 0, "nanosec": 0}` unless
  `--stamp-payload-trigger-time` is set (see "Why `trigger_time` is zero"
  above) — the record's own `publish_time` carries the anchor regardless.
  Written once, `--trigger-after-ms` milliseconds after startup, on a
  schemaless channel — clipper's MCAP interface decodes triggers by
  `message_encoding` alone, never a schema.

Each data channel is an `Iterator<Item = Vec<u8>>` that generates its next
payload on `.next()`. The program registers every channel once, then every
`--period-ms` pulls one sample from each data channel and writes it with the
current time as `log_time` and `log_time - publish_offset_ms` as `publish_time`
(asserted absolute first). On Ctrl+C it stops *after* the current sleep, before
the next write, and calls `Writer::finish()` — which writes the summary +
footer and flushes the writer's internal buffers to the file. The file is
written straight to a `File` (no `BufWriter`), so finalisation never depends on
`Drop`; unchunked output (see "Tailability" above) means each message is
already durable as its own record before `finish()` ever runs.
