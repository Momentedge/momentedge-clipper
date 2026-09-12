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

The Nix flake is for **development** (the dev shell, and the per-distro
`nix build .#clipper` as a build check), CI, and one shipping artefact:
`nix build .#clipper-ros-free`, the recorder without ROS. The device deploys as a
native build on the target instead — see the `packaging` skill and
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

Both produce the same one binary with the same subcommands (`clipper tail`,
`clipper clip`) and the same flags. The feature buys exactly one thing: the `ros`
trigger source — a live `momentedge_msgs/Trigger` subscription on a node, the
`Recorded` publish that answers it, and `clip`'s CDR trigger decoder underneath.
`clipper clip` cuts one window out of one finished recording, which has no live
topic, so it never offers `ros` in either build; only a `cdr`-encoded trigger it
reads out of the recording asks for the feature's decoder. So:

|  | default build | `--features ros` |
|---|---|---|
| links r2r / needs a ROS install | no | yes |
| needs the dev shell | no | yes |
| `clipper tail --trigger-source` accepts | `mcap` | `ros`, `mcap` |
| default `clipper tail --trigger-source` | `mcap` | `ros` |
| `cdr` triggers in the recording | skipped, with an error naming the feature | decoded |

Every packaging path that ships to a ROS 2 device asks for the feature:
`nix/binaries.nix` passes `--features ros`, `scripts/build-on-target.sh` passes
`--features clipper/ros` (the package-qualified spelling, since it may select
`trigger-pub` too), and the CI recorder matrix passes it to build, unit tests and
e2e alike. The default build ships too, as its own nix package — see
[The two nix packages](#the-two-nix-packages).

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
recorder's own CLI, supervision and `mcap` trigger source. Only what actually
links r2r goes back through `nix develop`: `--features ros` on the recorder,
`clip` with `ros` on, and the e2e suite (which drives the binary on
`--trigger-source ros`).

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
- **`clipper/ros`** — the `ros` trigger source and the `Recorded` publish that
  answers it, and with it `dep:r2r`, `dep:futures` and `clip/ros`. The one
  feature that decides which of the recorder's two builds you get.

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
```

The packages that closure also produces — `.#clipper-<distro>`,
`.#rosEnv-<distro>`, `.#trigger-pub-<distro>` — are in
[The two nix packages](#the-two-nix-packages). `clipper-ros-free` is outside all
of this: with the `ros` feature off there is no ROS closure to build against, and
so no distro to select.

The flake outputs are named for the binary, which for the recorder is also the
cargo package name: `cargo build -p clipper` produces `target/release/clipper`,
whose modes are subcommands (`cargo run -p clipper -- tail`).

The attrset is lazy: selecting one distro never forces the others. Adding a
distro is one entry in `rosDistros`. The shellHook exports
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`, and
`ROS_DISTRO=<selected>`. The single `IDL_PACKAGE_FILTER` and the
`nix/ros-env.nix` package list serve every distro unchanged.

## The two nix packages

The flake's `packages` split the same way the cargo builds do, and only one half
is shippable:

| | `clipper-ros-free` | `clipper` / `clipper-<distro>` |
|---|---|---|
| defined in | [`nix/clipper-ros-free.nix`](nix/clipper-ros-free.nix) | [`nix/binaries.nix`](nix/binaries.nix) |
| cargo features | none (the default build) | `ros` |
| ROS 2 distro | none — one package, no suffix | one package per `rosDistros` entry |
| built against | nothing but its own closure | `rosEnv`, the nix ROS 2 closure |
| what it is for | a shippable artefact | a build check |

```bash
nix build .#clipper-ros-free   # the ROS-free binary, distro-independent
nix build .#clipper            # the device build, default distro (jazzy)
nix build .#clipper-rolling    # per-distro; also .#rosEnv-humble, etc.
```

Both install the same one executable, `bin/clipper`, whose modes are
subcommands.

**Why only one of them ships.** A `clipper-<distro>` binary links the nix ROS
closure and bakes `/nix/store` RPATHs, so it would load that closure instead of
the target's own apt ROS 2 and break ABI compatibility with the rest of the
host's ROS graph — which is why the device builds natively on the target
(`packaging` skill). `clipper-ros-free` links no ROS at all: its only dynamic
dependencies are libc and libgcc, there is no distro to answer to, and it runs
anywhere the store path is available.

**What the package build checks.** `doCheck = false` — unit tests are CI's
ROS-free lane, far cheaper there than a release-profile rebuild in the sandbox.
The derivation runs an `installCheckPhase` instead, asserting the one property
that makes it this artefact: the sandbox holds no ROS 2 of any kind, so
`clipper --help` and `clipper tail --trigger-source mcap --help` running at all
prove the binary needs none, and `clipper tail --trigger-source ros` must fail
with clap's `invalid value 'ros'` — that parse error, not merely a non-zero
exit, since a build that leaked the feature would accept the flag and start a
recorder, whose own exit status says nothing about which trigger sources the
binary offers.

**Both take the r2r vendor hash.** `Cargo.lock` carries r2r's git source
regardless of features, and nix vendors the whole lockfile before cargo picks a
feature set. So `cargoOutputHashes` in `flake.nix` — one entry, shared by both
package definitions — is needed by the ROS-free build too, which fetches r2r and
never compiles it. `flake.nix` reads the package `version` from
`[workspace.package]` in `Cargo.toml` for the same reason: one value, both
packages.

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

## The quality gate

`justfile` is the entry point; `just --list` names every recipe. Each one runs
the *caller's* toolchain and enters no nix shell of its own, so the two that link
ROS — `check-ros` and `e2e` — are invoked from inside the dev shell, and
everything else runs straight from the repo root:

| recipe | what it runs |
|---|---|
| `just fmt` / `just fmt-check` | `cargo fmt --all`, writing or checking |
| `just check` | `fmt-check`, then clippy and rustdoc with warnings denied |
| `just test` | nextest over the three crates, then the doctests |
| `just cov` | one instrumented run, rendered three ways (below) |
| `nix develop --command just check-ros` | the same clippy gate with `--features clipper/ros` |
| `nix develop --command just e2e` | the live ROS 2 suite (`CLIPPER_E2E=1`) |
| `just test-examples` | the example crates CI splits into their own jobs |

Three files hold the policy, and all three want a **nightly** toolchain (the dev
box's system Rust is one, as is the CI `fmt` job's):

- **`rustfmt.toml`** — `edition`, the std/external/own import grouping, one `use`
  per module path, formatted doc-comment code, and `hex_literal_case = "Upper"`
  for the MCAP magic and opcodes. Most of those keys are unstable, which is what
  makes nightly a requirement rather than a preference.
- **`[workspace.lints]` in `Cargo.toml`** — the lint levels every member inherits.
  `-D warnings` in `just check` is what makes the `warn` levels a gate; without
  it clippy exits zero on all of them. `RUSTDOCFLAGS="-D warnings" cargo doc` is
  the same promotion for the `rustdoc` block, and building the docs is the only
  thing that runs those lints at all.
- **`clippy.toml`** — the thresholds for the size and shape lints
  (`too_many_lines` at 60, `too_many_arguments` at 5, `cognitive_complexity` at
  15, `excessive_nesting` at 4, one bool per struct and per parameter list). The
  defaults are loose enough never to fire, so the file *is* the lint config. A
  hit is a cleanup item: fix it, or `#[expect(lint, reason = "…")]` at the
  narrowest scope with a reason a reviewer would accept.

`[lints]` is per package, not per target, so `unwrap_used`, `expect_used`,
`indexing_slicing` and the shape lints all fire inside `#[cfg(test)] mod tests`
and in `tests/*.rs` too. Each test module carries one inner `#![allow(…, reason
= "…")]` for them rather than an attribute per case.

## Tests and coverage

Unit/integration tests run with `just test` (or plain `cargo test`) in the dev
shell; everything ROS-free runs outside it (above). The suite spans all three
crates, so coverage names all three, and `--features clipper/ros` is what puts
the `ros` trigger source's lines in the report at all:

```bash
just cov                                      # the three crates, branches included, no shell
cargo llvm-cov -p clip -p tail -p clipper     # the same run without the renderings
nix develop --command cargo llvm-cov --features clipper/ros -p clip -p tail -p clipper          # summary table
nix develop --command cargo llvm-cov --features clipper/ros -p clip -p tail -p clipper --html   # target/llvm-cov/html/index.html
```

`just cov` instruments once and renders that one run three ways — `--no-report`
leaves the raw profile behind, so the cobertura file
(`target/llvm-cov/coverage.cobertura.xml`), the browsable HTML
(`target/llvm-cov/html/index.html`, `just cov-open`) and the printed table all
describe the same execution. `--branch` adds the branch columns; it rests on
rustc's `-Z coverage-options=branch`, so it is nightly-only and prints an
unstable warning. LLVM counts a branch where control flow forks on a condition —
`if`, `&&`, `||`, `while let` — and a `match` arm is a *region*, not a branch, so
code that decides by matching reports few branches and a high region count
rather than a gap in the tests. Doctests are outside the measurement: the test
runner does not carry them.

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
`--trigger-source ros` and reads its `Recorded` announcements, and only the
device build has either.

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
and must not collide), from the repo root and spelled absolute — nextest runs
each test with its cwd at `crates/clipper/`, so a relative `CARGO_TARGET_DIR`
names one directory to the build and a different one to every process the tests
spawn:

```bash
for d in humble jazzy lyrical; do
  nix develop ".#$d" --command bash -c \
    "CARGO_TARGET_DIR=$PWD/target/e2e-$d CLIPPER_E2E=1 \
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
