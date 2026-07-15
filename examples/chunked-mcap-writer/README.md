# chunked-mcap-writer — tailable chunked + compressed MCAP

A standalone program that writes a **chunked, zstd-compressed MCAP file that
`clipper --interface mcap` can still tail live** — the second producer path
that satisfies clipper's tailability contract, next to
[`custom-mcap-writer`](../custom-mcap-writer/README.md)'s unchunked output.
Its only job is to show the [`mcap`](https://crates.io/crates/mcap) crate
configuration that makes this work; everything else is deliberately minimal:
one JSON data channel, one synthetic trigger channel, every timestamp on log
time, and a single `--out` flag.

## Buffered chunks: why this is tailable

A producer clipper can tail live appends complete top-level records only and
never seeks back to rewrite one ([ARCHITECTURE.md, "Tailing a live
MCAP"](../../ARCHITECTURE.md#tailing-a-live-mcap)). The `mcap` crate's
*default* chunked writer violates that: it writes each `Chunk` record's
header with a placeholder length (`u64::MAX`) and seeks back to patch in the
true length once the chunk closes — read mid-write, the placeholder is a
framing-breaking record length, and clipper's tail faults on it by design.

This program builds its writer with

```rust
mcap::WriteOptions::new()
    .use_chunks(true)
    .disable_seeking(true)              // <- the load-bearing line
    .compression(Some(mcap::Compression::Zstd))
    .chunk_size(Some(4 * 1024))
```

`use_chunks(true)` + `disable_seeking(true)` selects the crate's **buffered
chunk mode**: each chunk is assembled in an in-memory buffer and appended to
the file as one *complete* `Chunk` record — real length, real CRC, no
placeholder, no seek-back. The bytes on disk are always a clean run of
complete records, which is exactly what clipper's tail requires: the
contract is "append complete records", not "never chunk". In return the
output carries zstd-compressed chunks — the trade
[`custom-mcap-writer`](../custom-mcap-writer/README.md)'s unchunked (and
therefore uncompressed, but lowest-latency) output cannot make.

## Chunk-close latency and `--grace-secs`

A tailer sees a chunk's messages only when the chunk closes and hits the
file; until then they exist solely in the writer's in-memory buffer. The
close cadence is the chunk size divided by the byte rate — and the writer
gates chunks on *uncompressed* accumulated record bytes (~31 B of record
framing plus the payload, per message), so the cadence is independent of
the codec:

```
close cadence ≈ CHUNK_SIZE / (record bytes × message rate)
              ≈ 4096 B / (~100 B × 50 Hz)  ≈  ~0.8 s
```

(each `/pose` JSON payload is ~70 B, plus the fixed per-record overhead).

That latency is what clipper's grace timeout must absorb: the grace must
exceed the recorder's flush latency, roughly one chunk fill for any chunked
producer (the same guidance as rosbag2's chunked profiles). The rule of
thumb is at least twice the expected chunk fill — ~2 s for this program's
constants; `--grace-secs 4` gives comfortable headroom. An undersized grace
does not lose the recording; it degrades clips whose window end waits on an
unclosed chunk to grace-timeout cuts.

## What it writes

| topic | payload | message encoding | schema | cadence |
|---|---|---|---|---|
| `/pose` | a `Pose { x, y, theta }` struct → JSON (`serde_json`) | json | jsonschema | every 20 ms (50 Hz) |
| `/events/momentedge/trigger` | a `momentedge_msgs/Trigger`-shaped JSON object | json | *(schemaless)* | every 10 s (first at 10 s) |

Every record — data and trigger alike — carries `publish_time = log_time` =
wall clock at write: this example lives entirely on log time, and the
capture-time `publish_time` story belongs to
[`custom-mcap-writer`](../custom-mcap-writer/README.md). The trigger payload
carries `preroll` = `postroll` = 3 s and leaves `trigger_time` at `{"sec": 0,
"nanosec": 0}`: under `--interface mcap` that field is inert — clipper
anchors the clip window on the trigger *record's own* MCAP stamp (here its
`log_time`) — and a non-zero value in that cell is a mis-anchoring hazard
clipper rejects (see ["Why `trigger_time` is
zero"](../custom-mcap-writer/README.md#why-trigger_time-is-zero)).

## Run

```bash
cargo run -p chunked-mcap-writer                    # writes ./chunked.mcap until Ctrl+C
cargo run -p chunked-mcap-writer -- --out demo.mcap
```

`--out` is the only flag; the sample rate, chunk size, and trigger cadence
are hardcoded constants in `src/main.rs` — this example demonstrates a
writer configuration, not a tuning surface. On Ctrl+C the program stops
cleanly and finalises the file (summary + footer).

No ROS environment is needed — the crate carries no r2r dependency and
builds with the system toolchain. Point clipper at the growing file with
`--interface mcap` and `--grace-secs 4` (see the [repo
README](../../README.md)) to cut clips from it live, or open the finished
file in [Foxglove](https://foxglove.dev).
