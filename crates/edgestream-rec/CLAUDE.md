# edgestream-rec

A *triggered* recorder, distinct from `r2r-sub`/`rclrs-sub`. It does not
subscribe to sensor topics or build an in-memory index. Instead it cuts clips
out of a continuous on-disk rosbag2 recording on demand, driven by ROS2 events.
Built on [r2r](https://github.com/sequenceplanner/r2r) with tokio.

## The pipeline it sits in

```
ros2 bag record (scripts/record.sh) ──5 s mcap splits──▶ ./record/*.mcap
        │ publishes /events/write_split on each split boundary
        ▼
edgestream-rec ◀── /events/edgestream/trigger ── trigger-pub (or any publisher)
        │ cuts [trigger_time-preroll, trigger_time+postroll]
        ├──▶ ./triggered/<trigger_ns>_<name>.mcap
        └──▶ /events/edgestream/recorded
```

`record.sh` is a standalone `ros2 bag record` — edgestream-rec never spawns it.
The two communicate only through `./record` and `/events/write_split`.

## Per-trigger flow (`handle_trigger`)

Each `edgestream_msgs/Trigger` is handled in its own tokio task, so overlapping
windows are cut concurrently:

1. **Wait out the postroll.** Sleep until the system clock passes
   `trigger_time + postroll`.
2. **Wait for the next split.** Block until a `WriteSplitEvent` is observed
   at/after the window end, so the split holding the tail of the window has been
   finalised on disk before it is read. `write_split` events fan out over a
   `tokio::sync::broadcast`; a handler subscribes *before* it starts waiting, so
   events during the wait are captured, then consumes until it sees one
   timestamped at/after the window end.
3. **Extract** the window into `./triggered/<trigger_ns>_<name>.mcap` via
   `mcap_copy::extract_clip` (blocking IO, run on `spawn_blocking`).
4. **Announce** by publishing `edgestream_msgs/Recorded`.

## The copy is direct (`mcap_copy.rs`)

`extract_clip` lists `./record/*.mcap`, skips any split whose summary time range
cannot overlap the window, and for the rest streams messages with
`mcap::MessageStream`, keeping those whose `log_time` is in the window.
`mcap::Writer::write` re-emits each message's **raw serialized bytes** and
deduplicates channels/schemas by content, so a topic spread across several splits
collapses to one output channel. The CDR message bodies are never decoded — only
each record's `log_time` is read. This is the same "take raw, don't decode the
body" spirit as the other recorders, applied to file-to-file copy. A split that
has no summary yet (the one rosbag2 still holds open) is scanned linearly, and a
read error at the truncated tail ends that split's scan with the already-copied
messages kept.

## Time base

`log_time`, the trigger stamp, and the wait clock are all nanoseconds on the
system clock — this assumes the default (no `use_sim_time`). The
`builtin_interfaces/Time` stamp is flattened with `time_to_ns`
(`sec * 1e9 + nanosec`), matching MCAP `log_time` and the workspace's
`cdr_header_stamp_ns` convention.

## Topics and types

| Direction | Topic | Type |
|---|---|---|
| in | `/events/edgestream/trigger` | `edgestream_msgs/Trigger` |
| in | `/events/write_split` | `rosbag2_interfaces/WriteSplitEvent` |
| out | `/events/edgestream/recorded` | `edgestream_msgs/Recorded` |

`edgestream_msgs` is the local interface package (`../../edgestream_msgs`);
`rosbag2_interfaces` and both message packages are on the flake's
`IDL_PACKAGE_FILTER` so r2r generates bindings for them. QoS is the reliable
default, matching rosbag2's event publishers.

## Concurrency

Same single-owner node model as `r2r-sub`: one `spawn_blocking` thread spins the
node; the typed subscription streams are consumed on tokio tasks and never touch
the node. The `Recorded` publisher is `Clone` and is shared into each handler.

## Run

```bash
nix develop --command cargo run -p edgestream-rec     # --record-dir ./record --out-dir ./triggered
```

Needs `scripts/record.sh` running (for `./record` and `/events/write_split`) and
a trigger publisher (`trigger-pub`). `RUST_LOG=debug` logs every `write_split`.
