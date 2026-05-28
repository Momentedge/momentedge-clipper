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
   timestamped at/after the window end. That event's `closed_file` becomes the
   newest split the extraction is allowed to read.
3. **Extract** the window into `./triggered/<trigger_ns>_<name>.mcap` via
   `mcap_copy::extract_clip` (blocking IO, run on `spawn_blocking`).
4. **Announce** by publishing `edgestream_msgs/Recorded`.

## The copy is direct (`mcap_copy.rs`)

`extract_clip` lists `./record/*.mcap`, orders them by file modification time
(`sort_by_modified_time` — rosbag2 writes splits sequentially, so write order is
captured without parsing the `record_<n>.mcap` names), and drops everything newer
than the split the triggering `write_split` reported closed
(`truncate_after_closed_file`). The still-open split rosbag2 has not finalised is
therefore never read. Of the remaining splits it skips any whose summary time
range cannot overlap the window, and for the rest streams messages with
`mcap::MessageStream`, keeping those whose `log_time` is in the window.

The output channel for each message is mapped by **content**, not by the source
file's IDs: separate split files assign their own numeric schema/channel IDs, so
`output_channel_id` registers each distinct channel in the writer once
(`add_schema`/`add_channel`) and caches the resulting ID, then messages are
emitted with `write_to_known_channel` carrying their **raw serialized bytes**. A
topic spread across several splits collapses to one output channel. The CDR
message bodies are never decoded — only each record's `log_time` is read, the
same "take raw, don't decode the body" spirit as the other recorders applied to a
file-to-file copy. A read or write error on a closed input aborts the clip and is
returned to the trigger handler rather than being silently truncated.

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
