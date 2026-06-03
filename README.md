# ros2_subscribe

A testing pad for **ROS2-based recorders** written in Rust. It holds two
families of tools:

- **All-topic indexers** (`r2r-sub`, `rclrs-sub`) — attach to *every* live ROS2
  topic, take each message as **raw serialized CDR** (one copy out of the
  middleware, with no field decoding), and index it by the message's own header
  timestamp. A bench for comparing ROS2 Rust client libraries and runtime models
  for a minimal-copy recorder front-end.
- **Triggered clip recorder** (`edgestream-rec` + `trigger-pub`) — cuts clips out
  of a continuous `ros2 bag record` on demand, copying MCAP messages straight
  through without decoding their bodies. See [Triggered recording](#triggered-recording).

It is a testing pad, not a finished tool.

## The all-topic indexers

Two binaries implement the same behaviour two different ways:

| Binary | Client library | Runtime model | Raw-CDR source |
|---|---|---|---|
| `r2r-sub` | [r2r](https://github.com/sequenceplanner/r2r) | tokio; a central writer task collects every topic | `subscribe_raw` |
| `rclrs-sub` | [rclrs](https://github.com/ros2-rust/ros2_rust) | no async runtime; one OS thread per topic, each with its own index | `SerializedSubscription::take` |

Both decode **only** the leading `builtin_interfaces/Time` stamp out of the CDR
buffer (never the message body) and key each message by nanoseconds since the
Unix epoch — the same flat clock MCAP uses for `log_time`. Messages with no
leading header stamp (such as `/tf`) are counted but not indexed.

## Prerequisites

- **Nix** with flakes enabled. The dev shell provides ROS2 Jazzy via
  [`nix-ros-overlay`](https://github.com/lopsided98/nix-ros-overlay) and pulls
  prebuilt packages from `ros.cachix.org` (nixpkgs has no ROS2).
- **System Rust** (`cargo`/`rustc` on your `PATH`). The flake intentionally does
  not provide a Rust toolchain.
- **A data source** — the sibling repo `../ros2_sources`, which replays a bag of
  recorded sensor data onto live ROS2 topics.

## Quickstart

```bash
# 1. Enter the ROS2 + C-toolchain shell (direnv users: `direnv allow`)
nix develop

# 2. Build both recorders
cargo build
```

To see a recorder work, you need something publishing. In one terminal, replay
the bag from `../ros2_sources` (see its `REPLAY.md`):

```bash
cd ../ros2_sources
nix develop
ros2 bag play --loop bags/example-011-ugv-ds.mcap
```

In another terminal, run a recorder against it:

```bash
cd ros2_subscribe
nix develop
cargo run -p rclrs-sub      # or:  cargo run -p r2r-sub
```

Both the replay and the recorder must share the same middleware and domain
(`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`); each repo's dev shell
sets these for you.

### What you should see

The UGV sample bag carries ~28 channels (camera, LIDAR packets, IMU, GPS,
odometry, tf) over ~16 seconds. A healthy run subscribes to all of them and logs
a periodic per-topic index summary. `/tf` and the bag player's
`/events/read_split` are header-less, so they are counted but not indexed. For a
high-rate topic
like `/sensor/camera/vi_sensor/imu`, the indexed time span converges on ~16 s,
which is the bag's length and confirms the timestamps are being parsed
correctly. Replaying with `--loop` resends identical stamps, reported as
"collisions".

Set `RUST_LOG=debug` for a line per received message; the default `info` level
logs startup, subscriptions, and the periodic index stats.

## Triggered recording

Instead of indexing every topic in memory, this workflow keeps a continuous
on-disk recording and extracts short clips from it on demand.

```
ros2 bag record ──5 s mcap splits──▶ ./record/*.mcap
       │ /events/write_split on each split boundary
       ▼
edgestream-rec ◀── /events/edgestream/trigger ── trigger-pub
       ├──▶ ./triggered/<trigger_ns>_<name>.mcap
       └──▶ /events/edgestream/recorded
```

- **`ros2 bag record`** (via `scripts/record.sh`) records all topics into 5 s
  MCAP splits under `./record`, publishing a `rosbag2_interfaces/WriteSplitEvent`
  on `/events/write_split` at each split boundary. It runs standalone —
  `edgestream-rec` never starts it. `./record` is gitignored and not pruned, so
  it grows until you stop recording or clear it.
- **`edgestream-rec`** listens on `/events/edgestream/trigger`
  (`edgestream_msgs/Trigger`: `name`, `description`, `trigger_time`, and the
  `preroll`/`postroll` windows in nanoseconds). For each trigger it waits until
  the clock passes `trigger_time + postroll` *and* the next split is finalised,
  then bulk-copies every message in `[trigger_time - preroll, trigger_time +
  postroll]` into `./triggered/<trigger_ns>_<name>.mcap` and publishes
  `edgestream_msgs/Recorded` on `/events/edgestream/recorded`. The copy re-emits
  raw MCAP message bytes — channels and schemas are carried over, message bodies
  are never decoded.
- **`trigger-pub`** publishes a trigger every 1 s (configurable), stamping
  `trigger_time` with the current time — a development stand-in for a real
  trigger source. With no `--preroll`/`--postroll` flags it draws each side a
  random 1–10 s window per trigger; pass either flag to pin it.

Run it with the bag replay from `../ros2_sources` as the data source, one
process per shell (all inside `nix develop`, sharing RMW + `ROS_DOMAIN_ID`):

```bash
# 1. data source — replay a bag (see ../ros2_sources/REPLAY.md)
cd ../ros2_sources && nix develop --command ros2 bag play --loop bags/example-011-ugv-ds.mcap

# 2. continuous recorder → ./record (5 s splits)
nix develop --command ./scripts/record.sh

# 3. triggered extractor → ./triggered
nix develop --command cargo run -p edgestream-rec

# 4. fire a trigger every 1 s (random 1-10 s preroll/postroll per trigger)
nix develop --command cargo run -p trigger-pub
```

Clips land in `./triggered` (gitignored); inspect one with `ros2 bag info
triggered/<file>.mcap`.

## Layout

```
crates/r2r-sub/        # r2r all-topic indexer
crates/rclrs-sub/      # rclrs all-topic indexer
crates/edgestream-rec/ # r2r+tokio triggered clip recorder
crates/trigger-pub/    # r2r periodic trigger publisher
edgestream_msgs/       # local ROS2 interface package (Trigger, Recorded)
scripts/record.sh      # standalone continuous `ros2 bag record`
flake.nix              # ROS2 Jazzy dev shell (system Rust)
```

`rclrs-sub` depends on a fork of `ros2_rust` that adds a raw serialized
subscription API (not yet available in any released rclrs) and corrects its type
support handling. The fork is fetched over git during the build; the sibling
checkout `../ros2_rust` is its working tree, needed only when changing it.

## For contributors and agents

Implementation rationale, concurrency design, build mechanics, and the fork
details live in the `CLAUDE.md` files: [`CLAUDE.md`](CLAUDE.md) for the shared
model and workspace, and one per crate
([`r2r-sub`](crates/r2r-sub/CLAUDE.md), [`rclrs-sub`](crates/rclrs-sub/CLAUDE.md),
[`edgestream-rec`](crates/edgestream-rec/CLAUDE.md),
[`trigger-pub`](crates/trigger-pub/CLAUDE.md)).
