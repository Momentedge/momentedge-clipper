# ros2_subscribe

A testing pad for **ROS2-based recorders** written in Rust. It holds two
families of tools:

- **All-topic indexers** (`r2r-sub`, `rclrs-sub`) — attach to *every* live ROS2
  topic, take each message as **raw serialized CDR** (one copy out of the
  middleware, with no field decoding), and index it by the message's own header
  timestamp. A bench for comparing ROS2 Rust client libraries and runtime models
  for a minimal-copy recorder front-end.
- **Triggered clip recorders** (`edgestream-rec`, `edgestream-rec-cont` +
  `trigger-pub`) — cut clips out of a continuous `ros2 bag record` on demand,
  copying MCAP messages straight through without decoding their bodies.
  `edgestream-rec` reads closed 5 s bag splits; `edgestream-rec-cont` tails one
  growing MCAP file. See [Triggered recording](#triggered-recording).

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
- **A data source** — the in-repo synthetic camera ([`sim/`](sim/README.md)),
  which publishes a gscam test pattern raw + H.265, or the sibling repo
  `../ros2_sources`, which replays a bag of recorded sensor data onto live
  ROS2 topics.

## Quickstart

```bash
# 1. Enter the ROS2 + C-toolchain shell (direnv users: `direnv allow`)
nix develop

# 2. Build both recorders
cargo build
```

To see a recorder work, you need something publishing. In one terminal, either
start the in-repo sim camera (`sim/cam_sim.sh`, see
[`sim/README.md`](sim/README.md)) or replay the bag from `../ros2_sources`
(see its `REPLAY.md`):

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

- **`ros2 bag record`** (via `scripts/record.sh`) records into 5 s MCAP splits
  under `./record`, publishing a `rosbag2_interfaces/WriteSplitEvent` on
  `/events/write_split` at each split boundary. It runs standalone —
  `edgestream-rec` never starts it. `./record` is gitignored and not pruned, so
  it grows until you stop recording or clear it. By default every live topic is
  recorded; pass a rosbag2 recorder-parameters YAML to select topics:

  ```bash
  ./scripts/record.sh                       # all topics → ./record
  ./scripts/record.sh config/cam_sim.yaml   # only the sim camera topics (sim/)
  ./scripts/record.sh my.yaml /tmp/record   # optional 2nd arg: output dir
  ```

  The config uses the standard `rosbag2_transport` Recorder node-parameters
  schema (the same file a composable Recorder node accepts);
  [`config/cam_sim.yaml`](config/cam_sim.yaml) is the example, listing the four
  `/camera/...` topics published by the in-repo sim camera
  ([`sim/`](sim/README.md)). The script honours the
  `record.*` topic-selection keys (`topics`, `all`/`all_topics`, `regex`,
  `exclude_regex`, `exclude_topics`); storage settings (MCAP, 5 s splits) are
  fixed by the script because `edgestream-rec` depends on them.
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

### Continuous single-file variant

`edgestream-rec-cont` cuts the same clips out of **one growing MCAP file**
instead of 5 s splits — no split boundaries and no `/events/write_split`. It
keeps the recording open and tails it, so clip latency is bounded by the
recorder's write-through latency rather than the split duration.

```
ros2 bag record ──one growing mcap──▶ ./record-cont/<bag>_0.mcap
       ▲ kept open + tailed
edgestream-rec-cont ◀── /events/edgestream/trigger ── trigger-pub
       ├──▶ ./triggered-cont/<trigger_ns>_<name>.mcap
       └──▶ /events/edgestream/recorded
```

Run it like the split-based pipeline, swapping steps 2–3:

```bash
# 2. continuous recorder → ./record-cont (one file, fastwrite profile)
nix develop --command ./scripts/record-continuous.sh   # optional: config/cam_sim.yaml

# 3. tailing extractor → ./triggered-cont
nix develop --command cargo run -p edgestream-rec-cont
```

`record-continuous.sh` records unchunked with the rosbag2 message cache
disabled (`--storage-preset-profile fastwrite --max-cache-size 0`), so each
message is visible to the tail as soon as it is written; the extractor also
reads chunked recordings. Clips have the same form as `edgestream-rec`'s and
land in `./triggered-cont`. The single recording file has no retention — it
grows until you stop recording (hole-punch retention is tracked in beads:
`ros2_subscribe-wkg`).

## Deployment (Jetson / native build)

The triggered recorder ships to an edge target — a Jetson running ROS2 Humble.
The target runs the same ROS2 distro the recorder is built against, so the two
binaries are compiled **natively on the target against its own ROS2 install**.
There is no container and no bundled ROS: only `edgestream-rec` and `trigger-pub`
are deployed, and rosbag2 with MCAP storage and the `WriteSplitEvent` on
`/events/write_split` come from the host's ROS2 (`ros-humble-rosbag2-storage-mcap`),
which the recorder reuses rather than shipping its own.

What gets built is only the Rust code and the local `edgestream_msgs` interface
package; `rcl`, `rmw_fastrtps_cpp`, the standard message packages, and rosbag2
are the host's ROS2. Building against the host's own libraries is what makes the
binaries ABI-compatible with the rest of its ROS graph (the camera node, rosbag2,
…) — a binary built against a different ROS2 build would link different
typesupport and not interoperate.

### Build on the target

Prerequisites (Ubuntu 22.04 / ROS2 Humble):

```bash
sudo apt install ros-humble-ros-base ros-humble-rosbag2-storage-mcap \
                 ros-humble-rosbag2-transport ros-humble-ros2bag \
                 ros-dev-tools clang libclang-dev
# plus a Rust toolchain (rustup, or the distro cargo/rustc) on PATH
```

Then, from a checkout of this repo on the target:

```bash
./scripts/build-on-target.sh
```

It builds `edgestream_msgs` into a colcon overlay (`./install`) and compiles
`edgestream-rec` + `trigger-pub` (`./target/release`) against `/opt/ros/humble`.
Override the ROS install with `ROS_SETUP=/opt/ros/<distro>/setup.bash`.

### Run

```bash
./scripts/start_recorder.sh           # ros2 bag record + edgestream-rec, detached
./scripts/start_demo_trigger_pub.sh   # demo trigger source (foreground)
```

`start_recorder.sh` launches the continuous `ros2 bag record` and the extractor
as two detached processes, each with a pidfile and log under the data dir. Splits
land in `~/edgestream-rec/recordings/`, clips in `~/edgestream-rec/captured/`
(override with `EDGESTREAM_DIR`). Running natively, every ROS2 process shares the
host `/dev/shm`, so FastDDS shared-memory transport works and discovery + data
interop with the host's other Humble nodes is direct.

### Retention

`scripts/record.sh`-style continuous recording has no built-in retention, so the
directories grow unbounded. `scripts/prune-recordings/` is a systemd user timer
that deletes files older than 24 h; install it on the target with its
`install-remote.sh` (see [`prune-recordings/README.md`](scripts/prune-recordings/README.md)).

## Layout

```
crates/r2r-sub/         # r2r all-topic indexer
crates/rclrs-sub/       # rclrs all-topic indexer
crates/edgestream-rec/  # r2r+tokio triggered clip recorder (5 s bag splits)
crates/edgestream-rec-cont/  # triggered clip recorder tailing one continuous mcap
crates/trigger-pub/     # r2r periodic trigger publisher
edgestream_msgs/        # local ROS2 interface package (Trigger, Recorded)
sim/                    # synthetic gscam camera, raw + H.265 (sim/cam_sim.sh) — see sim/README.md
nix/                    # flake package defs: edgestream-msgs, ros-env, binaries
config/                 # rosbag2 recorder-params YAMLs for record.sh (topic selection)
scripts/record.sh       # standalone `ros2 bag record`, 5 s splits (dev)
scripts/record-continuous.sh  # standalone `ros2 bag record`, one growing file (dev)
scripts/build-on-target.sh  # native target build (edgestream_msgs overlay + binaries)
scripts/start_recorder.sh, start_demo_trigger_pub.sh  # run the deployed binaries natively
scripts/prune-recordings/  # retention loop for the target
flake.nix               # ROS2 Jazzy dev shell + nix-built binaries/images
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
