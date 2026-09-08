---
name: build
description: >
  Dev-shell and build mechanics for the clipper workspace — the per-distro Nix
  ROS 2 shells, the system Rust toolchain, the recorder's two builds (ROS-free by
  default, `--features ros` for the device), the ROS-free `clip` and `tail`
  libraries, the r2r/IDL codegen model, which distros the crates build on, and
  how to run unit tests, coverage, and the live e2e suite. Use when building or
  testing clipper, entering the dev shell, adding a ROS distro, editing
  flake.nix / nix/, or running cargo-llvm-cov or the gated e2e tests.
---

# Build & dev environment

The Nix flake is for **development** (the dev shell, and
`nix build .#clipper` as a build check) and CI. Deployment is a native
build on the target — see the `packaging` skill and
[ARCHITECTURE.md § Deployment](ARCHITECTURE.md#deployment).

## Dev shell and toolchain

Builds run **inside the dev shell**, with the **system** Rust toolchain (the
flake deliberately provides no Rust):

```bash
nix develop --command cargo build        # likewise clippy, test, run
```

The dev shell is what a build that links ROS needs. `crates/clip`,
`crates/tail`, and the recorder's own default build need neither it nor nix
(below).

- **Coverage** is `cargo-llvm-cov`, also from the system, not the flake. The
  system toolchain ships the `llvm-tools` component, so `cargo-llvm-cov` finds
  `llvm-cov`/`llvm-profdata` through the rustc sysroot, version-matched to
  rustc's LLVM by construction. The flake exports nothing for coverage —
  `LLVM_COV`/`LLVM_PROFDATA` stay unset on purpose (setting them would override
  the sysroot tools and force a hand-maintained LLVM-major constraint).
- **nextest** (the e2e runner) is likewise a system prerequisite, not flaked.

### The recorder's two builds

ROS is the `ros` cargo feature of `crates/clipper`, and it is **off by default**:

```bash
cargo build -p clipper                        # ROS-free: no r2r, no ROS install needed
nix develop --command \
  cargo build -p clipper --features ros       # the device build
```

Both produce the same one binary with the same subcommand (`clipper tail`) and
the same flags. The feature buys exactly one thing: the `ros` interface — a live
`momentedge_msgs/Trigger` subscription on a node, the `Recorded` publish that
answers it, and `clip`'s CDR trigger decoder underneath. So:

|  | default build | `--features ros` |
|---|---|---|
| links r2r / needs a ROS install | no | yes |
| needs the dev shell | no | yes |
| `--interface` accepts | `mcap` | `ros`, `mcap` |
| default `--interface` | `mcap` | `ros` |
| `cdr` triggers in the recording | skipped, with an error naming the feature | decoded |

Every packaging path asks for the feature, so a shipped binary is always the
device build: `nix/binaries.nix` passes `--features ros`, `scripts/build-on-target.sh`
passes `--features clipper/ros` (the package-qualified spelling, since it may
select `trigger-pub` too), and the CI recorder matrix passes it to build, unit
tests and e2e alike.

### `clip`, `tail`, and the default recorder build without the dev shell

`crates/clip` — the MCAP format layer and recording index, the cut path, the
neutral trigger contract — and `crates/tail` — discovery, the recording
collection and its lifecycle, coverage, retention, and the waits before a cut —
pull no r2r in their default feature sets, and neither does `clipper` with its
`ros` feature off. All three need no ROS installation and no nix realization:

```bash
cargo clippy -p clip -p tail -p clipper --all-targets
cargo test -p clip -p tail -p clipper
```

Straight from the repo root, on the system toolchain. That is the fast inner
loop for anything in the index, the cut, the trigger types, the tail, or the
recorder's own CLI, supervision and MCAP interface. Only what actually links r2r
goes back through `nix develop`: `--features ros` on the recorder, `clip` with
`ros` on, and the e2e suite (which drives the binary on `--interface ros`).

`clip` carries three features, `tail` one, and `clipper` one, all off by
default, so nothing a consumer has not asked for gets linked:

- **`clip/ros`** — the CDR arm of the trigger decoder and the two r2r message
  conversions. They live in `clip` rather than in the recorder because the
  orphan rule forbids a downstream crate from writing `From` between two foreign
  types.
- **`clip/clap`** — `TimeSource` as a `ValueEnum`, for a binary that takes the
  clock domain on its command line.
- **`clip/test-support`** — publishes `clip::testing`, the MCAP fixture writers,
  so a consumer's tests build recordings the way `clip`'s own do instead of
  keeping a copy that drifts from what the scan expects. A dev-only opt-in:
  declared under `[dev-dependencies]`, where the v2+ resolver keeps it out of a
  release build.
- **`tail/test-support`** — publishes `Watch`'s test-only `get` and
  `send_replace`, the unconditional reader and setter the tail itself never
  calls (its own coverage updates go through `send_if_modified`); the recorder's
  admission-gate test drives waiters with them. The same dev-only opt-in, under
  `[dev-dependencies]`.
- **`clipper/ros`** — the ROS interface, and with it `dep:r2r`, `dep:futures`
  and `clip/ros`. The one feature that decides which of the recorder's two
  builds you get.

The recorder takes `clap` on its normal dependency on `clip` and adds `clip`'s
`ros` through its own, plus both crates' `test-support` under
`[dev-dependencies]`. The featureless build is the one CI holds down: the
`libraries` job asserts `cargo tree` names no r2r for `clip`, `tail` or
`clipper` before compiling anything, then clippies and tests all three on a
stock toolchain with no ROS on the machine at all — see the `ci` skill.

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

The flake outputs are named for the binary, which for the recorder is also the
cargo package name: `cargo build -p clipper` produces `target/release/clipper`,
whose modes are subcommands (`cargo run -p clipper -- tail`).

The attrset is lazy: selecting one distro never forces the others. Adding a
distro is one entry in `rosDistros`. The shellHook exports
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, and
`ROS_DISTRO=<selected>`. The single `IDL_PACKAGE_FILTER` and the
`nix/ros-env.nix` package list serve every distro unchanged.

## r2r / IDL build model and distro support

Every crate that links r2r — the recorder with `ros` on, `trigger-pub`, and
`clip` with `ros` on — uses one build model: r2r generates bindings at build time from
`AMENT_PREFIX_PATH`, gated by `IDL_PACKAGE_FILTER`
(`builtin_interfaces;momentedge_msgs` — the only packages the crates decode) plus
bindgen (`LIBCLANG_PATH`).

r2r support gates **which distros those crates build on**: r2r references the
`RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE` rmw enum variant that distros after
jazzy have removed. The workspace pins r2r to its `0.9.6` git tag
(`Cargo.toml`), which adds `lyrical` and cfg-gates that variant for it, so they
build on **humble, jazzy, lyrical** — but **not rolling**, which r2r `0.9.6`
still references the variant for (beads `clipper-2xb`). The pin returns to
crates.io once `0.9.6` ships there (beads `clipper-4rw`). `rolling` still gets a
working ROS 2 shell for everything but the Rust build. None of this reaches the
default builds: `clip` and `clipper` link r2r only with `ros` on and `tail` never
links it at all, so none of the three answers to a distro until the feature is on.

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

Unit/integration tests run with plain `cargo test` in the dev shell; everything
ROS-free runs outside it (above). The suite spans all three crates, so coverage
names all three, and `--features clipper/ros` is what puts the ROS interface's
lines in the report at all:

```bash
cargo llvm-cov -p clip -p tail -p clipper                                       # everything ROS-free, no shell
nix develop --command cargo llvm-cov --features clipper/ros -p clip -p tail -p clipper                 # summary table
nix develop --command cargo llvm-cov --features clipper/ros -p clip -p tail -p clipper --html          # target/llvm-cov/html/index.html
nix develop --command cargo llvm-cov --features clipper/ros -p clip -p tail -p clipper --lcov --output-path lcov.info
```

`-p clipper` alone builds no test binary for `clip` or `tail`, so the libraries'
own tests never run and their lines — instrumented all the same, as path
dependencies — report only what the recorder's tests happen to reach.

Coverage builds use their own target dir (`target/llvm-cov-target`), so the
first run is a full rebuild. The report is assembled from every test binary
under that dir's `debug/deps`, stale ones included: a binary left behind by an
earlier build contributes 0.00% rows for source files that are not on disk,
and `cargo llvm-cov clean --workspace` leaves it in place. When the report
names a file you cannot find, delete the stale binaries and re-run.

`trigger-pub` has no tests. The mcap-writer and copper examples carry their own,
which the explicit `-p` list keeps out of the recorder's coverage.

### Live ROS 2 e2e suite

`crates/clipper/tests/e2e.rs` drives the real stack: a real `ros2 bag record`
(matching `scripts/record.sh`), CLI-published triggers, and `ros2 topic echo`
for `Recorded`. It is gated on `CLIPPER_E2E`: unset, every e2e test prints a
skip notice and passes (so `cargo test`/`llvm-cov` are unaffected, and the test
binary compiles in the ROS-free lane too — it links no ROS itself, it drives the
`ros2` CLI); set, a missing prerequisite fails loudly. [cargo-nextest](https://nexte.st/) is required
for the gated run — its `e2e` profile (`.config/nextest.toml`) runs each test in
its own process, serializes the suite, and enforces per-test timeouts:

```bash
nix develop --command bash -c \
  'CLIPPER_E2E=1 cargo nextest run -p clipper --features ros --profile e2e -E "binary(e2e)"'
```

`--features ros` is not optional here: the suite starts the recorder on
`--interface ros` and reads its `Recorded` announcements, and only the device
build has either.

The copper live e2e (`copper_sink_recording_produces_clip`) drives the
`cu-mcap-record` member binary as its Producer fixture, building it on demand
with `-p cu-mcap-record` into the active `target/`. The first cold run compiles
the cu29 tree and takes minutes — the `e2e` profile grants that one test a
longer terminate-after so it does not trip the per-test timeout. Prebuild it
with `cargo build --locked -p cu-mcap-record`, or set `CU_MCAP_RECORD_BIN` to a
prebuilt binary, to skip the wait. As a workspace member it builds into whatever
target dir is active, so the per-distro loop below compiles it once per leg (into
that leg's `target/e2e-$d`) alongside the rest of the crates.

Each test runs in its own `ROS_DOMAIN_ID` (band 80–101) with its own temp dirs,
so a recorder already on domain 0 is unaffected. Expect a few minutes of wall
clock (the tests sleep out real trigger windows). Run across the working distros
with a per-distro target dir (the r2r artifacts link that distro's `rcl`/`rmw`
and must not collide):

```bash
for d in humble jazzy lyrical; do
  nix develop ".#$d" --command bash -c \
    "CARGO_TARGET_DIR=target/e2e-$d CLIPPER_E2E=1 \
     cargo nextest run -p clipper --features ros --profile e2e -E 'binary(e2e)'"
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
