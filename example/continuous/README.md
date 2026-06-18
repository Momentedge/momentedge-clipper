# Continuous recording (the clipper pairing)

clipper tails **one growing MCAP file**. A continuous `ros2 bag record` — no
splits — is what pairs with it: the recorder appends to a single file, clipper
keeps that file open and cuts clips from it on each trigger.

```
ros2 bag record --all ──one growing mcap──▶ ./record/<bag>_0.mcap
       ▲ kept open + tailed
clipper ◀── /events/momentedge/trigger
       ├──▶ ./clipped/<trigger_ns>_<name>.mcap
       └──▶ /events/momentedge/recorded
```

Run each in its own shell with a sourced ROS2 environment (the dev shell, or
`/opt/ros/<distro>/setup.bash`), sharing `RMW_IMPLEMENTATION` and
`ROS_DOMAIN_ID`.

## Record

```bash
# the convenience script (storage-efficient defaults):
./scripts/record.sh                 # --all → ./record, zstd_fast, 100 MiB cache

# or spelled out:
ros2 bag record --all \
  --storage mcap \
  --storage-preset-profile zstd_fast \
  --max-cache-size 104857600 \
  --output ./record
```

## Run clipper

```bash
./scripts/run.sh                    # clipper --record-dir ./record …
# or, in the dev shell without an install:
cargo run -p clipper -- --record-dir ./record --out-dir ./clipped --grace-secs 30
```

Fire test triggers with `cargo run -p trigger-pub` (or publish
`momentedge_msgs/Trigger` on `/events/momentedge/trigger` yourself).

## The two latency knobs

clipper can only cut a clip once the window's messages are physically on disk.
Two recorder settings decide how long that takes — the gap clipper's
`--grace-secs` has to cover:

### `--storage-preset-profile`

The mcap storage plugin's write profile. Valid values: `none`, `fastwrite`,
`zstd_fast`, `zstd_small`.

| profile      | on disk                          | tail latency                | file size |
|--------------|----------------------------------|-----------------------------|-----------|
| `fastwrite`  | unchunked, no CRC, write-through | minimal — per message       | largest   |
| `none`       | stock chunked, uncompressed      | ~one chunk fill             | large     |
| `zstd_fast`  | chunked + zstd (fast)            | ~one chunk fill             | small     |
| `zstd_small` | chunked + zstd (max)            | ~one chunk fill             | smallest  |

A chunked message is visible to the tail only after its chunk is flushed, which
happens roughly every *chunk size / aggregate data rate* seconds. clipper reads
chunked files fine — it decompresses zstd/lz4 chunks during the tail — so this
is purely a latency-vs-size trade, not a correctness one.

- **Lowest clip latency:** `fastwrite` — every record lands top-level, visible
  immediately. Pair with `--max-cache-size 0` and a small `--grace-secs`.
- **Smallest recording:** `zstd_fast` / `zstd_small` — pay one chunk-fill of
  latency; size `--grace-secs` above it.

### `--max-cache-size`

Bytes rosbag2 buffers in memory before writing through (default
`104857600` = 100 MiB; double-buffered, so peak memory is ~2×). The cache
drains eagerly, so in steady state it adds little latency either way; `0`
forces every message straight to disk (no buffering) and is the companion to
`fastwrite` for minimal tail latency.

### Matching clipper's grace

`--grace-secs` (default 30) is how long past a window's end clipper waits for
the recording to cover it before cutting from whatever is on disk. Keep it
**above the recorder's flush latency**: near-zero for `fastwrite`/`cache 0`,
roughly one chunk fill for the chunked profiles. Too low against a chunked
profile and clips get cut short at the grace timeout.

## Low-latency variant

```bash
ros2 bag record --all \
  --storage mcap \
  --storage-preset-profile fastwrite \
  --max-cache-size 0 \
  --output ./record

clipper --record-dir ./record --grace-secs 2
```

## Retention

A continuous recording is one file that grows until you stop it; there is no
built-in retention (in-place hole-punching is the planned follow-up, beads
`clipper-wkg`). When unbounded growth is a problem, see
[`../split-bags`](../split-bags/README.md) for split recording with pruning;
clipper follows the rollovers, at the cost of the split-boundary clip trade-off
documented there.
