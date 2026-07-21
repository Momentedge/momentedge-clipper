---
name: ci
description: >
  CI/CD knowledge for this repo — design rationale, known skips, and local
  testing with act. Use when editing .github/workflows/, diagnosing CI failures,
  adding distros, or adjusting the e2e skip list.
---

# CI — design rationale and gotchas

The workflow files are the source of truth for *what* runs:
[`ci.yml`](.github/workflows/ci.yml) and
[`release.yml`](.github/workflows/release.yml). This skill covers the *why*
behind non-obvious choices, the skip rules, and local testing.

## ci.yml — key decisions

**`fmt` is a standalone nightly job, outside the distro matrix.**
`rustfmt.toml` uses unstable options (`group_imports`, `imports_granularity`)
that only nightly rustfmt honours. Running it per-distro would be three
identical copies behind a heavy nix realization.

**`cu-mcap-record` has its own standalone ROS-free job, plus a fixture build in
the matrix.** The copper example crate is workspace-excluded (root `Cargo.toml`
`exclude`) with its own committed `Cargo.lock`, so the exclusion keeps its cu29
dependency tree out of the workspace dependency graph and the ROS dev shells —
`cargo build -p clipper` and the unit lanes never resolve cu29. Its dedicated
job runs plain-toolchain `cargo fmt` (nightly, same rustfmt.toml reason as
above) plus `clippy`/`test` on stable with `--locked`, inside
`examples/cu-mcap-record` — no nix, no ROS. Separately, the per-distro
`recorder` matrix `Build` step builds the example **binary** by
`--manifest-path` (its own lockfile, never `-p`) as the Producer fixture the
live copper e2e (`copper_sink_recording_produces_clip`) drives; its `target/`
is a second `Swatinem/rust-cache` workspace so the cu29 tree caches per leg.
Prebuilding it there keeps the cu29 compile out of the e2e test's own timeout —
the test's on-demand `cargo build --locked` then finds it up to date.

**One matrix leg per distro; three steps share one nix shell.**
Build → unit → e2e reuse the same realized nix closure and compiled artifacts.
Only `humble`, `jazzy`, and `lyrical` are in the matrix — rolling can't build
the crates (r2r QoS variant issue, beads `clipper-2xb`).

**`on: push` only — no `pull_request:`.**
Same-repo PRs would run the heavy matrix twice (push + PR events). Add
`pull_request:` if fork contributions ever need their own CI run.

**Nix store cached via `cache-nix-action`, Cargo via `Swatinem/rust-cache`.**
No external cache service needed. `ros.cachix.org` is a read-only substituter
(named in `nixConfig`); nothing is pushed to it and no token is required.

## e2e skip rules

`CLIPPER_E2E_SKIP_FLAKY=1` is set on every CI leg. The `skip_flaky()` gate in
`tests/harness/mod.rs` reads it to skip (and pass) `corrupt_tail_health_live` —
a timing-sensitive corruption race that reliably fails under CI-grade scheduling.
The skip lives in the test next to the reason; a local run (env unset) still
exercises it.

Every distro runs the full e2e binary: the recovery suite discovers recordings
by mtime and asserts on path-free log needles, so lyrical's timestamped rosbag2
filenames do not break it (beads `clipper-7ys`).

Result: every distro runs the whole binary minus the one flaky-skipped test.

## release.yml — key decisions

**Native arm64 runners** (`ubuntu-22.04-arm` / `ubuntu-24.04-arm`) produce
binaries ABI-compatible with the deployment targets by construction — same
rationale as `build-on-target.sh`.

**apt is repointed at the Azure ports mirror before `setup-ros`.** The arm64
runner image fetches base Ubuntu packages from `ports.ubuntu.com`, which is
intermittently unroutable from the runner network (IPv6 has no route, IPv4 times
out) and fails the `setup-ros` apt install with `Unable to fetch some archives`.
A step rewrites the `ports.ubuntu.com` host to `azure.ports.ubuntu.com` (the
mirror the Azure-hosted runners reach over the backbone) in the apt sources and
drops an `apt.conf.d` file forcing IPv4 plus retries. The rewrite is idempotent
and leaves `packages.ros.org` alone; only release.yml needs it (ci.yml is Nix).

**Two packages, two tools.** The `deb` job builds `ros-<distro>-momentedge-msgs`
with bloom and `momentedge-clipper` with cargo-deb (bloom has no cargo build
type). `ros-tooling/setup-ros@v0.7` (desktop) covers `ros-base`,
`rmw_fastrtps_cpp`, rosdep, and the `ament_cmake`/`rosidl` generators; the inline
`apt install` adds `clang libclang-dev` (r2r bindgen) plus `python3-bloom fakeroot
debhelper dpkg-dev` (bloom + the msgs deb), and `cargo install cargo-deb` follows.
The job ends with a smoke-test that installs both `.deb` files and runs `clipper`,
confirming the `Depends` chain resolves. The packaging steps themselves live in the
`packaging` skill (`.claude/skills/packaging/SKILL.md`).

**Tag-gated publish.** A `v*` tag publishes; `workflow_dispatch` has a `publish`
boolean (default `false`) so packaging can be exercised without touching a
release. Both `artifact upload` and the release-attach step are gated on
`!env.ACT` so `act` runs never publish.

## Local testing with `act`

```bash
# List the matrix
act -l

# Run one distro leg of ci.yml
act push -j recorder --matrix distro:jazzy \
  --container-options "--shm-size=2g"

# Exercise the release packaging pipeline on amd64 (arm64 logic, amd64 binary):
# builds both .debs (bloom msgs + cargo-deb clipper) and smoke-tests the install.
act workflow_dispatch -j deb --matrix distro:humble \
  -P ubuntu-22.04-arm=catthehacker/ubuntu:act-22.04 \
  -P ubuntu-24.04-arm=catthehacker/ubuntu:act-24.04
```

The `--shm-size=2g` matters: FastDDS shared-memory transport needs more than
act's default 64 MB `/dev/shm`. The artifact upload and release steps are
skipped automatically (`!env.ACT`).

Nix is installed via `cachix/install-nix-action` (fetches from `releases.nixos.org`),
not the Determinate installer — the Determinate CDN times out inside act's containers.

**Lyrical fails under `act`** (purity check: `home directory /homeless-shelter
exists`). act's single-user Nix store is non-sandboxed; GitHub-hosted runners
use a proper daemon. Verify lyrical locally with `nix develop .#lyrical` instead.

humble and jazzy substitute most of their closure from `ros.cachix.org` and run
end to end under `act`.
