# ros2_subscribe — contributor notes

[README.md](README.md) is the canonical overview: what this testing pad is, the
triggered-recording workflow, prerequisites, and the build/run/replay quickstart.
Start there. This file covers what a contributor or agent needs *beyond* the
README — build mechanics and conventions — without repeating it. Each crate has
its own CLAUDE.md:
[`clipper`](crates/clipper/CLAUDE.md),
[`trigger-pub`](crates/trigger-pub/CLAUDE.md); the sim camera (`sim/`) has
[`sim/CLAUDE.md`](sim/CLAUDE.md), and CI mechanics live in the `/ci` repo skill (`.claude/skills/ci/SKILL.md`). The nested files load on demand when
you work in their directory, keeping their detail out of context otherwise.

## The recorders

`clipper` is a triggered clip recorder, and `trigger-pub` is the
periodic `Trigger` publisher that drives it. The recorder owns no sensor
subscriptions and no message index; it cuts clips out of a continuous on-disk
`ros2 bag record` on ROS2 trigger events, copying MCAP messages straight through.
It tails one growing MCAP file, decoding nothing but each message's MCAP log
time. Internals live in
[`clipper`](crates/clipper/CLAUDE.md).

## The sim camera (`sim/`)

`sim/` is the in-repo data source: a synthetic gscam camera (`videotestsrc` →
raw + H.265 topics) driven by `sim/cam_sim.sh` (`run` / `record` / `stop`). It
is a launch/config tree, not a Cargo crate. Its ROS packages (`gscam`, the
image_transport plugins, `ffmpeg-image-transport[-msgs]`, `rclcpp-components`)
are the `simPaths` half of `nix/ros-env.nix`, gated by `withSim` and pulled into
the closure only for the distros in the flake's `simDistros` list. The dev shell
adds the GStreamer plugin set (`GST_PLUGIN_SYSTEM_PATH_1_0` exported by the
shellHook) on those same distros. Overview and usage:
[`sim/README.md`](sim/README.md); gotchas: [`sim/CLAUDE.md`](sim/CLAUDE.md).

`simDistros` is `jazzy`, `humble`, `lyrical`, and `rolling`. Jazzy, Lyrical, and
Rolling build the sim stack (gscam, `ffmpeg_image_transport`,
`ffmpeg_image_transport_msgs`) from source as packaged — all ament-2.x, with
`pkg-config` on the build `PATH`. Humble needs a packaging fix, carried by
`simOverlays` in `flake.nix`: its older ament-1.x closure leaves `pkg-config`
off the build `PATH`, so the overlay adds `pkg-config` to the `nativeBuildInputs`
of `gscam` and `ffmpeg_image_transport`, plus a `-DFFMPEG_PKGCONFIG=…` flag on
the latter (its `ffmpeg_encoder_decoder` dependency exports an extras.cmake that
clobbers `PKG_CONFIG_PATH`). Rolling's sim shell is camera-only: its recorder
crates don't build (r2r references an rmw QoS variant rolling removed, beads
`clipper-2xb`), but the sim stack is env-only and unaffected. The
recorder, the e2e suite, and deployment use none of the sim stack, so the
`corePaths` half — the recorder/CLI/rosbag2/message closure — builds and tests on
every distro regardless. Extend `simDistros` (and `simOverlays`, only if the
distro's closure needs the pkg-config fix) to put sim in another distro's shell.

## Build and environment mechanics

Setup is in the README; the parts that matter when changing the build:

- Builds run **inside the dev shell**: `nix develop --command cargo build`
  (likewise `clippy`, `test`, and `llvm-cov`). Rust is the system toolchain,
  not the flake's.
- Test coverage is `cargo-llvm-cov`, which — like Rust itself — comes from
  the system, not this flake (commands in README "Tests and coverage"). The
  system toolchain includes the `llvm-tools` component, so `cargo-llvm-cov`
  finds `llvm-cov`/`llvm-profdata` through the rustc sysroot, version-matched
  to rustc's own LLVM by construction. The flake exports nothing for coverage;
  in particular `LLVM_COV`/`LLVM_PROFDATA` stay unset — they would override
  the sysroot tools with an LLVM whose major version then has to be kept
  **>= the system rustc's LLVM major** by hand.
- **The flake targets one ROS2 distro per shell, chosen at the command line.**
  `flake.nix` carries a `rosDistros` list (`humble`, `jazzy`, `lyrical`,
  `rolling` — all packaged by `nix-ros-overlay`; `kilted` is available too) and a
  `defaultDistro` (`jazzy`). A `mkDistro` function builds the whole per-distro
  closure — `momentedge-msgs`, `rosEnv` ([`nix/ros-env.nix`](nix/ros-env.nix)),
  the nix-built binaries, and the dev shell — once for each. So `nix develop`
  (default) and `nix develop .#humble` / `.#lyrical` / `.#rolling` select the
  distro; packages come both unsuffixed (default distro, e.g.
  `nix build .#clipper`) and per-distro
  (`.#clipper-rolling`, `.#rosEnv-humble`). The attrset is lazy:
  selecting one distro never forces the others. Adding a distro is one entry in
  `rosDistros`.
- The shellHook exports `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_DOMAIN_ID=0`,
  and `ROS_DISTRO=<selected distro>` (the default shell is `jazzy`). The single
  `IDL_PACKAGE_FILTER` and `nix/ros-env.nix` package list serve every distro
  unchanged — every listed package exists under all of them.
- The crates use a single build model: r2r generates bindings at build time from
  the `AMENT_PREFIX_PATH`, gated by `IDL_PACKAGE_FILTER` + bindgen
  (`LIBCLANG_PATH`). `IDL_PACKAGE_FILTER` is `builtin_interfaces;momentedge_msgs`
  — the only packages the crates decode via r2r. r2r support gates which distros
  the crates (the deployables and the e2e suite) build under, because r2r
  references the `RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE` rmw enum variant that
  distros after jazzy have removed. The workspace pins r2r to its `0.9.6` git tag
  (`Cargo.toml`), which adds `lyrical` and cfg-gates that variant for it, so the
  crates build on `humble`, `jazzy`, and `lyrical` — but not `rolling`, which
  r2r `0.9.6` still references the variant for (beads `clipper-2xb`); the
  pin returns to crates.io once `0.9.6` ships there (beads `clipper-4rw`).
  So the live e2e suite passes fully on `humble` and `jazzy`, and 12/14 on
  `lyrical` (two recorder-restart tests trip over lyrical's timestamped rosbag2
  bag filenames — a harness assumption, not a recorder bug, beads
  `clipper-7ys`); `rolling` still gets a working ROS2 shell for everything
  but the Rust build. The sim camera's stack (`ros-core`, `gscam`, the
  image_transport plugins, `rclcpp-components`) serves only `sim/`, with
  `ffmpeg-image-transport-msgs` doubling as the type support `ros2 bag record`
  needs to capture the H.265 topic (`sim/config/recorder_params.yaml`). Env-only packages
  like these stay out of `IDL_PACKAGE_FILTER` — no Rust crate decodes them. Each
  crate's CLAUDE.md has the details.
- `momentedge_msgs/` is a **local ament_cmake interface package** built by the
  flake via `ros.buildRosPackage` (mirroring upstream `example-interfaces`) and
  added to both the env and `IDL_PACKAGE_FILTER`, so its `Trigger`/`Recorded`
  types get r2r bindings like any other message package. Flakes only see
  git-tracked files: a newly added or renamed file under `momentedge_msgs/` must
  be `git add`ed before `nix develop`/`cargo build`, or the eval fails with "Path
  … is not tracked by Git".
- `ros2bag` + `rosbag2-transport` + `rosbag2-storage-mcap` provide the standalone
  `ros2 bag record`. `scripts/record.sh` runs it as the one growing MCAP file
  `clipper` tails (started with `scripts/run.sh`); the [`example/`](example/)
  guides cover the continuous, split-bag, and `ros2 launch` setups. rosbag2 publishes
  `WriteSplitEvent` on `/events/write_split` when a bag splits, but the recorder
  tails a continuous file and consumes no split events.

## Continuous integration

GitHub Actions CI ([`.github/workflows/ci.yml`](.github/workflows/ci.yml))
builds, unit-tests, and runs the live e2e suite for `clipper` on
`humble`, `jazzy`, and `lyrical` on every push, with the nix store cached
through the GitHub Actions cache (no external cache service). A separate `fmt`
job checks workspace formatting (`cargo fmt --all --check`) — nix- and
distro-independent, since rustfmt only parses source. It runs on **nightly**
rustfmt (`dtolnay/rust-toolchain@nightly`, rustfmt component only): the
workspace `rustfmt.toml` sets unstable options that only nightly honours, which
also matches the dev box's nightly system rustfmt. Mechanics,
rationale, and the local-`act` caveats live in the `/ci` repo skill.

## Deployment build model

The nix flake above is for **development** (the dev shell, and `nix build
.#clipper`/`trigger-pub` as build checks). **Deployment is a native
build on the target**, not nix or Docker — see README "Deployment". The reasoning
that shapes it:

- The edge target runs a full ROS2 install (Humble) of the **same distro** the
  recorder is built against, so `rosbag2` (MCAP storage), `rcl`,
  `rmw_fastrtps_cpp`, and the standard message packages all come from the host.
  Only `clipper` + `trigger-pub` (which link just `rcl`/`rmw` + the
  `builtin_interfaces`/`momentedge_msgs` types) and the `momentedge_msgs` overlay
  are built — `scripts/build-on-target.sh` does both.
- They are built **against the host's own ROS2 libraries** for ABI compatibility:
  a nix-built binary bakes `/nix/store` RPATHs and would load the nix closure
  rather than the host's ROS, defeating the point. So the build runs on the
  target (or an ABI-identical apt box of the same arch+distro), not cross- or
  nix-built.
- Native (no container) means all ROS2 processes share the host `/dev/shm`, so
  FastDDS shared-memory transport works and DDS interop with the host's other
  nodes is direct — no per-container `/dev/shm` split and no UDP-only workaround.

### Debian packaging and CI release

clipper ships as **two Debian packages**, each built the way its kind is built:
`ros-<distro>-momentedge-msgs` (the `ament_cmake` interface package) with **bloom**
into `/opt/ros/<distro>`, and `momentedge-clipper` (the Rust/r2r binary) with
**cargo-deb**, declaring `Depends:` on the msgs package. bloom has no cargo build
type, so the two tools are not interchangeable; and because the msgs package is a
first-class ROS deb, the clipper binary ships **no bundled overlay and no baked
rpath** — it resolves its typesupport through the standard
`/opt/ros/<distro>/setup.bash`, like every ROS executable. `clipper` is the only
binary packaged (`trigger-pub` is a dev stand-in; the recording is the host's own
`ros2 bag record`).

`.github/workflows/release.yml` builds both packages per distro on native arm64
runners (ABI-compatible with the targets); a `v*` tag publishes, `workflow_dispatch`
builds without publishing. The packaging scripts, the run model, the build and
on-target/`act` verification recipes, and the gotchas live in the **`packaging`
skill** (`.claude/skills/packaging/SKILL.md`); the workflow/CI mechanics and the
`act` recipe live in the **`/ci` skill**.

## Workspace layout

A virtual workspace (no root package), so `resolver = "3"` (the edition-2024
resolver) is set explicitly — a virtual workspace does not infer the resolver
from member editions and otherwise falls back to `"1"` with a warning. Members
are the two crates (`clipper`, `trigger-pub`); shared metadata is in
`[workspace.package]`. `momentedge_msgs/` (ROS2 interface package) and `sim/`
(the sim camera's launch/config tree) are not Cargo members.

## Sibling repositories

- `../ros2_sources` — the bag replay that feeds the recorder (README has the
  workflow).

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


<!-- BEGIN BEADS INTEGRATION v:1 profile:minimal hash:6cd5cc61 -->
## Beads Issue Tracker

This project uses **bd (beads)** for issue tracking. Run `bd prime` to see full workflow context and commands.

### Quick Reference

```bash
bd ready              # Find available work
bd show <id>          # View issue details
bd update <id> --claim  # Claim work
bd close <id>         # Complete work
```

### Rules

- Use `bd` for ALL task tracking — do NOT use TodoWrite, TaskCreate, or markdown TODO lists
- Run `bd prime` for detailed command reference and session close protocol
- Use `bd remember` for persistent knowledge — do NOT use MEMORY.md files

**Architecture in one line:** issues live in a local Dolt DB; sync uses `refs/dolt/data` on your git remote; `.beads/issues.jsonl` is a passive export. See https://github.com/gastownhall/beads/blob/main/docs/SYNC_CONCEPTS.md for details and anti-patterns.

## Agent Context Profiles

The managed Beads block is task-tracking guidance, not permission to override repository, user, or orchestrator instructions.

- **Conservative (default)**: Use `bd` for task tracking. Do not run git commits, git pushes, or Dolt remote sync unless explicitly asked. At handoff, report changed files, validation, and suggested next commands.
- **Minimal**: Keep tool instruction files as pointers to `bd prime`; use the same conservative git policy unless active instructions say otherwise.
- **Team-maintainer**: Only when the repository explicitly opts in, agents may close beads, run quality gates, commit, and push as part of session close. A current "do not commit" or "do not push" instruction still wins.

## Session Completion

This protocol applies when ending a Beads implementation workflow. It is subordinate to explicit user, repository, and orchestrator instructions.

1. **File issues for remaining work** - Create beads for anything that needs follow-up
2. **Run quality gates** (if code changed) - Tests, linters, builds
3. **Update issue status** - Close finished work, update in-progress items
4. **Handle git/sync by active profile**:
   ```bash
   # Conservative/minimal/default: report status and proposed commands; wait for approval.
   git status

   # Team-maintainer opt-in only, unless current instructions forbid it:
   git pull --rebase
   git push
   git status
   ```
5. **Hand off** - Summarize changes, validation, issue status, and any blocked sync/commit/push step

**Critical rules:**
- Explicit user or orchestrator instructions override this Beads block.
- Do not commit or push without clear authority from the active profile or the current user request.
- If a required sync or push is blocked, stop and report the exact command and error.
<!-- END BEADS INTEGRATION -->
