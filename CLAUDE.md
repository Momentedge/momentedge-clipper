# ros2_subscribe — contributor notes

[README.md](README.md) is the canonical overview: what this testing pad is, the
`r2r-sub` vs `rclrs-sub` comparison, the triggered-recording workflow,
prerequisites, and the build/run/replay quickstart. Start there. This file covers
what a contributor or agent needs *beyond* the README — the shared recorder
internals, build mechanics, and conventions — without repeating it. Each crate
has its own CLAUDE.md: [`r2r-sub`](crates/r2r-sub/CLAUDE.md),
[`rclrs-sub`](crates/rclrs-sub/CLAUDE.md),
[`edgestream-rec`](crates/edgestream-rec/CLAUDE.md),
[`trigger-pub`](crates/trigger-pub/CLAUDE.md).

## Two recorder families

The crates split into two unrelated models, and the "shared recorder model" below
applies only to the first:

- **All-topic indexers** — `r2r-sub` and `rclrs-sub`. Subscribe to every live
  topic, take raw CDR, index by header stamp. The section below is about these.
- **Triggered clip recorder** — `edgestream-rec` (+ `trigger-pub`). Owns no
  sensor subscriptions and no index; it cuts clips out of a continuous on-disk
  `ros2 bag record` on ROS2 trigger events, copying MCAP messages straight
  through. Its internals live in [`edgestream-rec`](crates/edgestream-rec/CLAUDE.md);
  the only shared idea is decoding nothing but the timestamp.

## The shared recorder model

Both binaries do the same three things per message; these design choices are the
point of the bench:

1. **Take raw CDR, never decode the body.** A message arrives as a `Vec<u8>` of
   serialized CDR.
2. **Decode only the header stamp.** A 4-byte CDR encapsulation header precedes
   the body; byte 1's low bit selects endianness. For a message whose first field
   is a `std_msgs/Header` (or a bare `builtin_interfaces/Time`), `sec` (`i32`) is
   at offset 4 and `nanosec` (`u32`) at offset 8. The index key is
   `sec * 1e9 + nanosec`, nanoseconds since the Unix epoch. Both crates carry an
   identical `cdr_header_stamp_ns` helper — keep them in step.
3. **Index per topic, keyed by stamp.** A header-less message (e.g.
   `tf2_msgs/TFMessage`, first field a sequence) fails the sanity gate
   (`sec >= 0`, `nanosec < 1e9`) and is counted but not indexed.

`rosbag2_interfaces` is on the env and the `IDL_PACKAGE_FILTER`, so the split
event types have type support: the indexers subscribe to `/events/read_split`
(replay) and `/events/write_split` (record) rather than skipping them. Both
events are header-less, so like `/tf` they are counted but not indexed.

## Build and environment mechanics

Setup is in the README; the parts that matter when changing the build:

- Builds run **inside the dev shell**: `nix develop --command cargo build`
  (likewise `clippy`). Rust is the system toolchain, not the flake's.
- The shellHook exports `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`,
  and `ROS_DISTRO=jazzy`.
- The flake's message-package list serves **both** build models from one
  `AMENT_PREFIX_PATH`: r2r generates bindings at build time, gated by
  `IDL_PACKAGE_FILTER` + bindgen (`LIBCLANG_PATH`); rclrs uses pre-generated
  bindings selected by `ROS_DISTRO` and needs neither. `example-interfaces` and
  `test-msgs` are present only as an rclrs link requirement
  ([#557](https://github.com/ros2-rust/ros2_rust/issues/557)). Each crate's
  CLAUDE.md has the details.
- `edgestream_msgs/` is a **local ament_cmake interface package** built by the
  flake via `ros.buildRosPackage` (mirroring upstream `example-interfaces`) and
  added to both the env and `IDL_PACKAGE_FILTER`, so its `Trigger`/`Recorded`
  types get r2r bindings like any other message package. Flakes only see
  git-tracked files: a newly added or renamed file under `edgestream_msgs/` must
  be `git add`ed before `nix develop`/`cargo build`, or the eval fails with "Path
  … is not tracked by Git".
- `ros2bag` + `rosbag2-transport` + `rosbag2-storage-mcap` provide the standalone
  `ros2 bag record` (`scripts/record.sh`) the triggered recorder reads from;
  `rosbag2-interfaces` provides `WriteSplitEvent` on `/events/write_split`.

## Workspace layout

A virtual workspace (no root package), so `resolver = "3"` (the edition-2024
resolver) is set explicitly — a virtual workspace does not infer the resolver
from member editions and otherwise falls back to `"1"` with a warning. Members
are the four crates (`r2r-sub`, `rclrs-sub`, `edgestream-rec`, `trigger-pub`);
shared metadata is in `[workspace.package]`. `edgestream_msgs/` is a ROS2
interface package, not a Cargo member.

## Sibling repositories

- `../ros2_sources` — the bag replay that feeds the recorders (README has the
  workflow).
- `../ros2_rust` — the working tree of the rclrs fork `rclrs-sub` depends on;
  relevant only when changing that fork (see [`rclrs-sub`](crates/rclrs-sub/CLAUDE.md)).

## Keeping docs in sync

`README.md` (human-facing) and the `CLAUDE.md` files (contributor/agent-facing)
describe the same system from two angles. **After completing a task that changes
behaviour, build or run steps, dependencies, or layout, update both in the same
change** so they stay accurate and consistent:

- Put overview, quickstart, and anything a user runs in `README.md`.
- Put rationale, internals, and conventions in `CLAUDE.md` (root for shared
  concerns, the crate's own for crate-specific ones).
- Do not duplicate — cross-reference. If a fact would otherwise appear in both,
  it belongs in `README.md` and the `CLAUDE.md` links to it.
