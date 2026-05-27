# ros2_subscribe

A testing pad for **ROS2-based recorders** written in Rust. Each binary attaches
to *every* live ROS2 topic, takes each message as **raw serialized CDR** (one
copy out of the middleware, with no field decoding), and indexes it by the
message's own header timestamp. It is a bench for comparing ROS2 Rust client
libraries and runtime models for a minimal-copy recorder front-end — not a
finished tool.

## The recorders

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
odometry, tf) over ~16 seconds. A healthy run subscribes to ~27 of them — one
topic (`rosbag2_interfaces/ReadSplitEvent`) has no installed type support and is
skipped — and logs a periodic per-topic index summary. For a high-rate topic
like `/sensor/camera/vi_sensor/imu`, the indexed time span converges on ~16 s,
which is the bag's length and confirms the timestamps are being parsed
correctly. Replaying with `--loop` resends identical stamps, reported as
"collisions".

Set `RUST_LOG=debug` for a line per received message; the default `info` level
logs startup, subscriptions, and the periodic index stats.

## Layout

```
crates/r2r-sub/      # r2r-based recorder
crates/rclrs-sub/    # rclrs-based recorder
flake.nix            # ROS2 Jazzy dev shell (system Rust)
```

`rclrs-sub` depends on a fork of `ros2_rust` that adds a raw serialized
subscription API (not yet available in any released rclrs) and corrects its type
support handling. The fork is fetched over git during the build; the sibling
checkout `../ros2_rust` is its working tree, needed only when changing it.

## For contributors and agents

Implementation rationale, concurrency design, build mechanics, and the fork
details live in the `CLAUDE.md` files: [`CLAUDE.md`](CLAUDE.md) for the shared
model and workspace, and one per crate
([`r2r-sub`](crates/r2r-sub/CLAUDE.md), [`rclrs-sub`](crates/rclrs-sub/CLAUDE.md)).
