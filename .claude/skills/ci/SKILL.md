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

Lyrical additionally skips `old_recording_on_disk_is_not_recovered_after_restart`
via a nextest filterset (`not test(...)`), not an env gate — its timestamped
rosbag2 filenames break a harness assumption (beads `clipper-7ys`), so it's a
distro incompatibility, not flakiness.

Result: humble/jazzy run 13/14, lyrical 12/14.

## release.yml — key decisions

**Native arm64 runners** (`ubuntu-22.04-arm` / `ubuntu-24.04-arm`) produce
binaries ABI-compatible with the deployment targets by construction — same
rationale as `build-on-target.sh`.

**`setup-ros` (desktop) + one `apt install clang libclang-dev` step.**
`ros-tooling/setup-ros@v0.7` covers `ros-base`, `rmw_fastrtps_cpp`, and the
`ament_cmake`/`rosidl` generators. It omits `libclang`; the inline apt step
fills the gap.

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

# Exercise the release packaging pipeline on amd64 (arm64 logic, amd64 binary)
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
