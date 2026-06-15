# ros2_subscribe

A **triggered clip recorder** for ROS2, written in Rust. It keeps a continuous
on-disk `ros2 bag record` running and, on each trigger event, cuts a short clip
out of it — copying MCAP messages straight through without decoding their
bodies. Two crates make up the pad:

- **`edgestream-rec-cont`** — the recorder. It tails one growing MCAP file and,
  for each trigger, bulk-copies the messages in the trigger's pre/post window
  into a standalone clip, then announces the result. See
  [Triggered recording](#triggered-recording).
- **`trigger-pub`** — a development stand-in for a real trigger source. It
  periodically publishes `edgestream_msgs/Trigger` so you can exercise the
  recorder end to end.

It is a testing pad, not a finished tool.

## Prerequisites

- **Nix** with flakes enabled. The dev shell provides ROS2 via
  [`nix-ros-overlay`](https://github.com/lopsided98/nix-ros-overlay) and pulls
  prebuilt packages from `ros.cachix.org` (nixpkgs has no ROS2). One distro per
  shell, selected at the command line — `nix develop` is Jazzy (the default);
  `nix develop .#humble`, `.#lyrical`, and `.#rolling` select the others. The
  `nix develop` and `cargo` commands below take a `.#<distro>` selector to pick
  one. The recorder builds and the e2e suite pass fully on **humble** and
  **jazzy**; **lyrical** builds and passes 12/14 (two recorder-restart tests
  trip lyrical's timestamped rosbag2 filenames — a harness assumption, beads
  `ros2_subscribe-7ys`); **rolling** gets a working ROS2 shell but cannot build
  the Rust crates (an `r2r` limitation — see "Integration tests" below).
- **System Rust** (`cargo`/`rustc` on your `PATH`). The flake intentionally does
  not provide a Rust toolchain.
- **A data source** — the in-repo synthetic camera ([`sim/`](sim/README.md)),
  which publishes a gscam test pattern raw + H.265, or the sibling repo
  `../ros2_sources`, which replays a bag of recorded sensor data onto live
  ROS2 topics. The sim camera rides in the Jazzy shell only; the overlay
  packages its gscam/ffmpeg stack for Jazzy but not Humble, Lyrical, or Rolling,
  whose shells carry the recorder/e2e core without it. Use `../ros2_sources`
  replay as the data source under those distros. The recorder, the e2e suite,
  and deployment never touch the sim camera, so they are unaffected.

## Quickstart

```bash
# 1. Enter the ROS2 + C-toolchain shell (direnv users: `direnv allow`)
nix develop

# 2. Build the recorder
cargo build
```

To see the recorder work you need a data source plus a continuous recording and
a trigger source. Run each in its own shell (all inside `nix develop`, sharing
RMW + `ROS_DOMAIN_ID`):

```bash
# 1. data source — replay a bag (see ../ros2_sources/REPLAY.md)
cd ../ros2_sources && nix develop --command ros2 bag play --loop bags/example-011-ugv-ds.mcap

# 2. continuous recording → ./record-cont (one growing file, fastwrite profile)
nix develop --command ./scripts/record-continuous.sh

# 3. tailing recorder → ./triggered-cont
nix develop --command cargo run -p edgestream-rec-cont

# 4. fire a trigger every 1 s (random 1-10 s preroll/postroll per trigger)
nix develop --command cargo run -p trigger-pub
```

The data source, the recording, and the recorder must share the same middleware
and domain (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`); each
repo's dev shell sets these for you.

A healthy run drops a clip into `./triggered-cont` for each trigger
(`<trigger_ns>_<name>.mcap`) and publishes an `edgestream_msgs/Recorded`
announcement on `/events/edgestream/recorded` naming the file just written.
Inspect a clip with `ros2 bag info triggered-cont/<file>.mcap`.

## Triggered recording

The recorder keeps a continuous on-disk recording open and extracts short clips
from it on demand, tailing **one growing MCAP file** — no split boundaries and
no per-split events. Clip latency is bounded by the recorder's write-through
latency rather than any split duration.

```
ros2 bag record ──one growing mcap──▶ ./record-cont/<bag>_0.mcap
       ▲ kept open + tailed
edgestream-rec-cont ◀── /events/edgestream/trigger ── trigger-pub
       ├──▶ ./triggered-cont/<trigger_ns>_<name>.mcap
       └──▶ /events/edgestream/recorded
```

- **`ros2 bag record`** (via `scripts/record-continuous.sh`) records into one
  growing MCAP file under `./record-cont`. It runs standalone —
  `edgestream-rec-cont` never starts it. `./record-cont` is gitignored and not
  pruned, so the file grows until you stop recording or clear it. By default
  every live topic is recorded; pass a rosbag2 recorder-parameters YAML as the
  first argument to select topics (e.g. `config/cam_sim.yaml` for the sim camera
  topics).

  The script uses the **fastwrite** storage profile (`--storage-preset-profile
  fastwrite --max-cache-size 0`) so each message is visible to the tail
  immediately after the recorder writes it.
- **`edgestream-rec-cont`** listens on `/events/edgestream/trigger`
  (`edgestream_msgs/Trigger`: `name`, `description`, `trigger_time`, and the
  `preroll`/`postroll` windows in nanoseconds). For each trigger it waits until
  the recording covers the window `[trigger_time - preroll, trigger_time +
  postroll]` (or the grace timeout elapses), then bulk-copies every message in
  that window into `./triggered-cont/<trigger_ns>_<name>.mcap` and publishes
  `edgestream_msgs/Recorded` on `/events/edgestream/recorded`. The copy re-emits
  raw MCAP message bytes — channels and schemas are carried over, message bodies
  are never decoded.
- **`trigger-pub`** publishes a trigger every 1 s (configurable), stamping
  `trigger_time` with the current time — a development stand-in for a real
  trigger source. With no `--preroll`/`--postroll` flags it draws each side a
  random 1–10 s window per trigger; pass either flag to pin it.

Ctrl-C stops the recorder cleanly (exit zero); any internal fault exits non-zero
for a supervisor to restart.

### Configuration

`edgestream-rec-cont` takes no CLI args. It reads `edgestream-rec-cont.toml` from
the working directory (or the path in `$EDGESTREAM_CONFIG`; a missing file is
fine), overridable per key by `EDGESTREAM_*` environment variables (e.g.
`EDGESTREAM_GRACE_SECS=60`). All keys are optional:

```toml
record_dir = "./record-cont"   # bag directory of the continuous recording
out_dir = "./triggered-cont"   # where clips are written
grace_secs = 30                # wait past the window end for coverage before cutting
extract_parallelism = 1        # concurrent clip copies (1 = one at a time, FIFO)
```

At most 16 triggers are handled concurrently — a fixed bound, not a config key.
A trigger that arrives while all 16 handler slots are occupied is rejected: a
logged error is emitted and the trigger produces no clip and no
`/events/edgestream/recorded` announcement.

The recorder also reads chunked recordings (override `STORAGE_PRESET` /
`MAX_CACHE_SIZE` on `record-continuous.sh`), but `grace_secs` must then be sized
to the resulting flush latency (roughly chunk size / aggregate data rate). Every
file visible in `out_dir` is a complete, crash-durable clip; the
`/events/edgestream/recorded` announce always names an already-written file. The
single recording file has no retention — it grows until you stop recording.

For the internal design — thread model, tailing mechanics, atomic clip
publication, recorder restart handling, and damage tolerance — see
[`crates/edgestream-rec-cont/ARCHITECTURE.md`](crates/edgestream-rec-cont/ARCHITECTURE.md).

## Tests and coverage

Tests run with plain `cargo test` inside the dev shell. Line/branch coverage
comes from [`cargo-llvm-cov`](https://github.com/taiki-e/cargo-llvm-cov). Like
Rust itself it is not part of this flake — it and the `llvm-tools` toolchain
component it needs (`llvm-cov`/`llvm-profdata`, discovered via the rustc
sysroot) come from the system:

```bash
# Summary table on stdout (edgestream-rec-cont carries the test suite)
nix develop --command cargo llvm-cov -p edgestream-rec-cont

# Browsable HTML report → target/llvm-cov/html/index.html
nix develop --command cargo llvm-cov -p edgestream-rec-cont --html

# lcov output for editor/CI integration
nix develop --command cargo llvm-cov -p edgestream-rec-cont --lcov --output-path lcov.info
```

Coverage builds are instrumented and use their own target directory
(`target/llvm-cov-target`), so they neither clobber nor reuse the normal
`cargo build` cache — expect a full rebuild on the first run. `trigger-pub` has
no tests, so the suite is `edgestream-rec-cont`'s.

### Integration tests (live ROS2 e2e)

`crates/edgestream-rec-cont/tests/e2e.rs` drives the real stack end to end: a
real `ros2 bag record` (the production `scripts/record-continuous.sh`
invocation), CLI-published triggers, and a `ros2 topic echo` listener for the
`Recorded` announcements. The matrix covers the fastwrite and zstd_fast
storage profiles; recorder restarts both between and inside an open trigger
window; deletion of the recording with and without a subsequent restart;
grace-timeout degraded paths; offline/live corruption of the recording; and
the most-recent-file-only semantics — when a recording is replaced, data from
the previous file is never recovered into a clip, even if that file still
exists on disk.

The suite is gated on `EDGESTREAM_E2E`: unset (plain `cargo test`,
`cargo llvm-cov`) every e2e test prints a skip notice and passes, so the
commands above are unaffected. With the gate set, a missing prerequisite (no
`ros2` on PATH, `edgestream_msgs` not resolvable) fails loudly instead of
skipping.

[cargo-nextest](https://nexte.st/) is a hard prerequisite for the gated run —
like Rust itself it comes from the system, not this flake. Its `e2e` profile
(`.config/nextest.toml`) runs each test in its own process, serializes the
suite (one DDS graph and one disk at a time), and enforces per-test timeouts:

```bash
nix develop --command bash -c \
  'EDGESTREAM_E2E=1 cargo nextest run -p edgestream-rec-cont --profile e2e -E "binary(e2e)"'
```

Each test runs in its own `ROS_DOMAIN_ID` (band 80–101) with its own temp
dirs, so a recorder already running on the host's domain 0 is unaffected.
Expect a few minutes of wall clock: the tests sleep out real trigger windows
against a live recording.

The suite builds only `edgestream-rec-cont` and drives the `ros2` CLI from the
shell, so it runs under any distro whose shell builds. Select the distro's shell
and give each its own target directory, since the r2r build artifacts link that
distro's `rcl`/`rmw` and must not collide:

```bash
for d in humble jazzy lyrical; do
  nix develop ".#$d" --command bash -c \
    "CARGO_TARGET_DIR=target/e2e-$d EDGESTREAM_E2E=1 \
     cargo nextest run -p edgestream-rec-cont --profile e2e -E 'binary(e2e)'"
done
```

The suite passes 14/14 on **humble** and **jazzy**. **Lyrical** builds (the
workspace pins [`r2r`](https://github.com/sequenceplanner/r2r) to its `0.9.6` git
tag, which supports lyrical) and passes 12/14: two recorder-restart tests
(`old_recording_on_disk_is_not_recovered_after_restart` and
`corrupt_tail_health_live`) fail because lyrical's rosbag2 names bag files with a
timestamp, breaking the test harness's stable-filename re-attach check — the
recorder itself is unaffected (beads `ros2_subscribe-7ys`). **Rolling** selects
and builds its ROS2 closure (`nix develop .#rolling` works) but cannot build the
Rust crates: even r2r `0.9.6` references the `RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE`
rmw enum variant that the current rolling has dropped (beads `ros2_subscribe-2xb`).
The r2r pin returns to a crates.io release once `0.9.6` is published there (beads
`ros2_subscribe-4rw`).

## Deployment (Jetson / native build)

The recorder ships to an edge target — a Jetson running ROS2 Humble. The target
runs the same ROS2 distro the recorder is built against, so the two binaries are
compiled **natively on the target against its own ROS2 install**. There is no
container and no bundled ROS: only `edgestream-rec-cont` and `trigger-pub` are
deployed, and rosbag2 with MCAP storage comes from the host's ROS2
(`ros-humble-rosbag2-storage-mcap`), which the recorder reuses rather than
shipping its own.

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
`edgestream-rec-cont` + `trigger-pub` (`./target/release`) against
`/opt/ros/humble`. Override the ROS install with
`ROS_SETUP=/opt/ros/<distro>/setup.bash`.

### Run

The recorder itself — `record-continuous.sh` plus `edgestream-rec-cont` — is
wired up per deployment; there is no turnkey run script. The trigger publisher
has one:

```bash
./scripts/start_demo_trigger_pub.sh   # demo trigger source (foreground)
```

Running natively, every ROS2 process shares the host `/dev/shm`, so FastDDS
shared-memory transport works and discovery + data interop with the host's other
Humble nodes is direct.

### Retention

The continuous recording has no built-in retention, so the data directory grows
unbounded. `scripts/prune-recordings/` deletes files older than 24 h from the
captured-clips directory (`~/edgestream-rec/captured`); install it on the target
with its `install-remote.sh` (see
[`prune-recordings/README.md`](scripts/prune-recordings/README.md)).

## Layout

```
crates/edgestream-rec-cont/  # triggered clip recorder tailing one continuous mcap
crates/trigger-pub/     # r2r periodic trigger publisher
edgestream_msgs/        # local ROS2 interface package (Trigger, Recorded)
sim/                    # synthetic gscam camera, raw + H.265 (sim/cam_sim.sh) — see sim/README.md
nix/                    # flake package defs: edgestream-msgs, ros-env, binaries
config/                 # rosbag2 recorder-params YAMLs for record.sh (topic selection)
scripts/record.sh       # standalone `ros2 bag record`, 5 s splits (general/sim use)
scripts/record-continuous.sh  # standalone `ros2 bag record`, one growing file (recorder pipeline)
scripts/build-on-target.sh  # native target build (edgestream_msgs overlay + binaries)
scripts/start_demo_trigger_pub.sh  # run the deployed trigger publisher natively
scripts/prune-recordings/  # retention loop for the target
flake.nix               # per-distro ROS2 dev shells (humble/jazzy/lyrical/rolling) + nix-built binaries
```

`scripts/record.sh` is a standalone `ros2 bag record` producing 5 s MCAP splits
into `./record`. It is a general-purpose / sim recorder — the in-repo sim camera
records its topics through it ([`config/cam_sim.yaml`](config/cam_sim.yaml)) — and
is not part of the triggered-recording pipeline, which uses
`record-continuous.sh`. By default it records every live topic; pass a rosbag2
recorder-parameters YAML to select topics:

```bash
./scripts/record.sh                       # all topics → ./record
./scripts/record.sh config/cam_sim.yaml   # only the sim camera topics (sim/)
./scripts/record.sh my.yaml /tmp/record   # optional 2nd arg: output dir
```

## For contributors and agents

Implementation rationale, concurrency design, and build mechanics live in the
`CLAUDE.md` files: [`CLAUDE.md`](CLAUDE.md) for the shared model and workspace,
and one per crate
([`edgestream-rec-cont`](crates/edgestream-rec-cont/CLAUDE.md),
[`trigger-pub`](crates/trigger-pub/CLAUDE.md)).
