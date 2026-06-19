# custom-mcap-writer — minimal MCAP writer

A tiny standalone program that writes an MCAP file directly with the
[`mcap`](https://crates.io/crates/mcap) crate — no ROS, no CDR. Each channel is
an iterator that yields a generated JSON payload; every 20 ms the loop pulls one
sample from each channel and writes it, until Ctrl+C finalises the file. Modeled
on [Foxglove's quickstart writer](https://docs.foxglove.dev/docs/sdk/example).

## What it writes

| topic | payload | message encoding | schema |
|---|---|---|---|
| `/pose` | a `Pose { x, y, theta }` struct → JSON (`serde_json`) | json | jsonschema |
| `/size` | a raw `{"size": N}` JSON string | json | *(schemaless)* |

## Run

```bash
cargo run -p custom-mcap-writer                       # writes ./quickstart.mcap until Ctrl+C
cargo run -p custom-mcap-writer -- --out demo.mcap --duration 5 --compression lz4
```

| flag | default | meaning |
|---|---|---|
| `--out <file>` | `quickstart.mcap` | output MCAP file |
| `--period-ms <ms>` | `20` | how often every channel is sampled (50 Hz) |
| `--duration <secs>` | run until Ctrl+C | stop after this long |
| `--compression <none\|zstd\|lz4>` | `zstd` | per-chunk compression |

No ROS environment is needed — the crate carries no r2r dependency and builds
with the system toolchain. Open the result in [Foxglove](https://foxglove.dev)
or inspect it with the [`mcap` CLI](https://github.com/foxglove/mcap).

## How it works

MCAP stores opaque message bytes and labels each channel with a
`message_encoding`; it does no serialization itself. So the writer just picks a
format and writes bytes:

- **`/pose`** — a `#[derive(Serialize)]` struct turned into JSON with
  `serde_json::to_vec`, on a channel declared `message_encoding = "json"` with a
  JSON Schema so viewers can render it. The typed way.
- **`/size`** — a raw JSON string built with `format!`, on a schemaless channel.
  The most basic payload there is: no schema, no serializer, just bytes.

Each channel is an `Iterator<Item = Vec<u8>>` that generates its next payload on
`.next()`. The program registers the channels once, then every `--period-ms`
pulls one sample from each and writes it with the current time as the message
`log_time`/`publish_time`. On Ctrl+C it stops *after* the current sleep, before
the next write, and calls `Writer::finish()` — which writes the summary + footer
and flushes the chunk buffer to the file. The file is written straight to a
`File` (no `BufWriter`), so finalisation never depends on `Drop`.
