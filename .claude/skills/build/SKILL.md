---
name: build
description: >
  Dev-shell and build mechanics for the clipper workspace — the per-distro Nix
  ROS 2 shells, the system Rust toolchain, the r2r/IDL codegen model, which
  distros the crates build on, and how to run unit tests, coverage, and the live
  e2e suite. Use when building or testing clipper, entering the dev shell, adding
  a ROS distro, editing flake.nix / nix/, or running cargo-llvm-cov or the gated
  e2e tests.
---

# Build & dev environment

The Nix flake is for **development** (the dev shell, and `nix build .#clipper`
as a build check) and CI. Deployment is a native build on the target — see the
`packaging` skill and [ARCHITECTURE.md § Deployment](ARCHITECTURE.md#deployment).

## Dev shell and toolchain

Builds run **inside the dev shell**, with the **system** Rust toolchain (the
flake deliberately provides no Rust):

```bash
nix develop --command cargo build        # likewise clippy, test, run
```

- **Coverage** is `cargo-llvm-cov`, also from the system, not the flake. The
  system toolchain ships the `llvm-tools` component, so `cargo-llvm-cov` finds
  `llvm-cov`/`llvm-profdata` through the rustc sysroot, version-matched to
  rustc's LLVM by construction. The flake exports nothing for coverage —
  `LLVM_COV`/`LLVM_PROFDATA` stay unset on purpose (setting them would override
  the sysroot tools and force a hand-maintained LLVM-major constraint).
- **nextest** (the e2e runner) is likewise a system prerequisite, not flaked.

## One ROS 2 distro per shell

`flake.nix` carries a `rosDistros` list (`humble`, `jazzy`, `lyrical`,
`rolling`; `kilted` is available too) and `defaultDistro` (`jazzy`). `mkDistro`
builds the whole per-distro closure — `momentedge-msgs`, `rosEnv`
([`nix/ros-env.nix`](nix/ros-env.nix)), the nix-built binaries, the dev shell —
once per distro:

```bash
nix develop            # jazzy (the default)
nix develop .#humble   # or .#lyrical / .#rolling
nix build .#clipper            # default distro
nix build .#clipper-rolling    # per-distro; also .#rosEnv-humble, etc.
```

The attrset is lazy: selecting one distro never forces the others. Adding a
distro is one entry in `rosDistros`. The shellHook exports
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, and
`ROS_DISTRO=<selected>`. The single `IDL_PACKAGE_FILTER` and the
`nix/ros-env.nix` package list serve every distro unchanged.

## r2r / IDL build model and distro support

The crates use one build model: r2r generates bindings at build time from
`AMENT_PREFIX_PATH`, gated by `IDL_PACKAGE_FILTER`
(`builtin_interfaces;momentedge_msgs` — the only packages the crates decode) plus
bindgen (`LIBCLANG_PATH`).

r2r support gates **which distros the crates build on**: r2r references the
`RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE` rmw enum variant that distros after
jazzy have removed. The workspace pins r2r to its `0.9.6` git tag
(`Cargo.toml`), which adds `lyrical` and cfg-gates that variant for it, so the
crates build on **humble, jazzy, lyrical** — but **not rolling**, which r2r
`0.9.6` still references the variant for (beads `clipper-2xb`). The pin returns
to crates.io once `0.9.6` ships there (beads `clipper-4rw`). `rolling` still gets
a working ROS 2 shell for everything but the Rust build.

`momentedge_msgs/` is a **local `ament_cmake` interface package** built by the
flake via `ros.buildRosPackage` and added to both the env and
`IDL_PACKAGE_FILTER`, so its `Trigger`/`Recorded` types get r2r bindings like any
other message package. **Flakes only see git-tracked files**: a newly added or
renamed file under `momentedge_msgs/` must be `git add`ed before
`nix develop`/`cargo build`, or eval fails with "Path … is not tracked by Git".

`ros2bag` + `rosbag2-transport` + `rosbag2-storage-mcap` provide the standalone
`ros2 bag record` that `scripts/record.sh` runs as the recording clipper tails.
rosbag2 publishes `WriteSplitEvent` on `/events/write_split` at a split, but
clipper discovers splits by watching the directory for new `*.mcap` files and
consumes no split events.

## Tests and coverage

Unit/integration tests run with plain `cargo test` in the dev shell. Coverage:

```bash
nix develop --command cargo llvm-cov -p clipper                          # summary table
nix develop --command cargo llvm-cov -p clipper --html                   # target/llvm-cov/html/index.html
nix develop --command cargo llvm-cov -p clipper --lcov --output-path lcov.info
```

Coverage builds use their own target dir (`target/llvm-cov-target`), so the
first run is a full rebuild. `trigger-pub` has no tests; the suite is
`clipper`'s.

### Live ROS 2 e2e suite

`crates/clipper/tests/e2e.rs` drives the real stack: a real `ros2 bag record`
(matching `scripts/record.sh`), CLI-published triggers, and `ros2 topic echo`
for `Recorded`. It is gated on `CLIPPER_E2E`: unset, every e2e test prints a
skip notice and passes (so `cargo test`/`llvm-cov` are unaffected); set, a
missing prerequisite fails loudly. [cargo-nextest](https://nexte.st/) is required
for the gated run — its `e2e` profile (`.config/nextest.toml`) runs each test in
its own process, serializes the suite, and enforces per-test timeouts:

```bash
nix develop --command bash -c \
  'CLIPPER_E2E=1 cargo nextest run -p clipper --profile e2e -E "binary(e2e)"'
```

Each test runs in its own `ROS_DOMAIN_ID` (band 80–101) with its own temp dirs,
so a recorder already on domain 0 is unaffected. Expect a few minutes of wall
clock (the tests sleep out real trigger windows). Run across the working distros
with a per-distro target dir (the r2r artifacts link that distro's `rcl`/`rmw`
and must not collide):

```bash
for d in humble jazzy lyrical; do
  nix develop ".#$d" --command bash -c \
    "CARGO_TARGET_DIR=target/e2e-$d CLIPPER_E2E=1 \
     cargo nextest run -p clipper --profile e2e -E 'binary(e2e)'"
done
```

The recovery tests discover recordings by mtime and assert on path-free log
needles, so lyrical's timestamped rosbag2 filenames do not break them (beads
`clipper-7ys`). CI runs build + unit + e2e on humble/jazzy/lyrical — see the
`ci` skill for the matrix, skip rules, and `act` notes.

## Binary cache

The flake registers two substituters (`nixConfig`): `ros.cachix.org` (upstream
ROS 2) and `https://cache.stfl.dev/momentedge` (a self-hosted attic cache of the
project's ROS 2 closure). Both are public — **pulling needs no credentials**, so
a fresh checkout substitutes the closure instead of compiling it. The
substituters apply only once nix accepts the flake's `nixConfig`: trusted users
get this automatically; others pass `--accept-flake-config`. Pushing is for
maintainers/CI and needs an attic token (`ATTIC_TOKEN` repo secret in CI):

```bash
attic login momentedge https://cache.stfl.dev <token>
attic push momentedge <store-path>
```
