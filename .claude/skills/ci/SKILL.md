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

**`cu-mcap-record` has a lean ROS-free job, plus a fixture build in the matrix.**
The copper example is a workspace member, but its cu29 deps are declared
crate-local (not in `[workspace.dependencies]`), so the cu29 tree stays scoped
to that one member: `cargo build -p clipper` and the `-p clipper` unit lane
never resolve or compile it. Its dedicated job runs from the repo root on the
plain system toolchain — `cargo clippy --locked --all-targets -p cu-mcap-record`
and `cargo test --locked -p cu-mcap-record`, no nix, no ROS — where `-p` pulls
only the example and its cu29 tree, never clipper's r2r closure. Formatting is
covered by the `fmt` job's `cargo fmt --all --check`, which includes the member.
Separately, the per-distro `recorder` matrix `Build` step adds `-p
cu-mcap-record` to its build line to prebuild the Producer fixture the live
copper e2e (`copper_sink_recording_produces_clip`) drives; prebuilding it keeps
the cu29 compile out of the e2e test's own timeout — the test's on-demand `-p
cu-mcap-record` build then finds it up to date.

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

**Every job using `rust-cache` first runs `mkdir -p target/tests/target`.**
Any `target/` that has compiled cu29 carries a `tests` entry, because
cu29-derive's build script copies its trybuild fixtures into
`<target>/tests/trybuild`. rust-cache special-cases a `tests` entry and recurses
into both `<target>/tests/target` and `<target>/tests/trybuild`, but the first
call is not awaited (`src/cleanup.ts`), so the ENOENT for the directory that is
never created escapes its `try`/`catch` as an unhandled rejection and kills the
action mid-clean. What is left is a `target/` whose `.fingerprint` entries
outlive the build-script binaries already deleted beside them, and the next
cargo build fails on a build script it believes is current:

```
error: failed to run custom build command for `bindgen v0.71.1`
  could not execute process .../build-script-build (never executed)
  No such file or directory (os error 2)
```

The clean fires on a partial cache-key match, so the symptom is intermittent and
lands on whichever crate cargo reaches first, unrelated to any source change.
Pre-creating the directory turns that recursion into a no-op; it precedes the
rust-cache step so the guard covers the restore-time clean, and the directory is
cached in turn, so an already-poisoned cache heals without a key bump. The
`release.yml` deb legs need no guard — `build-on-target.sh` builds only
`BUILD_PACKAGES` (`clipper`), so cu29 never compiles there.

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

**Tag-gated publish, in a single `release` job.** A `v*` tag publishes;
`workflow_dispatch` has a `publish` boolean (default `false`) so packaging can be
exercised without touching a release. The `deb` matrix legs only upload artifacts;
the `release` job (`needs: deb`, plain `ubuntu-latest` — it renders Markdown and
moves files, so it needs no ROS and no arm64) downloads them with
`merge-multiple: true` and writes the release once. One writer is a requirement,
not a preference: a release has a single body, so generating notes from a 2-leg
matrix would have the legs race to overwrite each other. `needs: deb` also means a
failed distro blocks the release rather than publishing half of it. The artifact
upload, the download, and the publish step are gated on `!env.ACT` so `act` runs
never publish.

**Release notes come from the commit history, not from PRs.** Work lands directly
on `main` as conventional commits, so GitHub's native `generate_release_notes`
(which enumerates merged PRs) would yield an empty list — and being a flat list
itself, it would not summarise anything either. `git-cliff` reads the commits
between the previous tag and this one instead, grouping them by conventional-commit
type; [`cliff.toml`](cliff.toml) in the repo root holds the template and carries its
own rationale. `orhun/git-cliff-action@v4` is a composite action that downloads a
prebuilt binary, so it also runs under `act`.

**Cut releases with an annotated tag — `git tag -a v0.2.0`, never `git tag v0.2.0`.**
GitHub does not copy a tag's annotation into the release body (no setting changes
that), but git-cliff exposes it to the template as `message`, and `cliff.toml`
renders its first line as the release headline and the remainder as the overview
paragraph. That is the only place a human explains what the release is *for*: the
generated groups below it say what landed, never why it matters. Highlights
(`feat`/`fix`/`perf`) stay open beneath it and the mechanical types fold into a
`<details>`. A lightweight tag degrades cleanly to the changelog with no overview —
which is what every tag through `v0.1.1` is, so the first annotated tag is also the
first release with a headline.

Two details the config depends on: the `release` job checks out with
`fetch-depth: 0` (a shallow checkout has none of the commits to walk), and the
git-cliff invocation switches on the trigger — `--latest` on a tag renders that
tag's section, `--unreleased` otherwise previews commits since the last tag. The
notes are written to the job summary on *every* run, publishing or not, so an
unpublished `workflow_dispatch` is how you preview what a tag would say before
cutting it. Locally the same preview is `nix run nixpkgs#git-cliff -- --unreleased`,
plus `--with-tag-message "$(cat draft.txt)"` to try out the headline and overview
before committing to the annotation.

**The rendered notes must end in a newline**, which `cliff.toml`'s `\s*\z` tail
postprocessor guarantees. `git-cliff-action`'s `run.sh` appends the file to
`$GITHUB_OUTPUT` between a `content<<EOF` line and a closing `EOF` line using
`cat`, which adds nothing of its own; a body ending flush against its last
character therefore absorbs the delimiter into that line, no line equals `EOF`,
and the runner kills the step before any release is written:

```
##[error]Unable to process file command 'output' successfully.
##[error]Invalid value. Matching delimiter not found 'EOF'
```

The workflow itself never reads that `content` output — it passes
`RELEASE_NOTES.md` as `body_path` — but the failed output write fails the step
regardless, skipping publish entirely. Keep the tail postprocessor as `\s*\z`
(matches the empty string at end of input, so the replacement always lands) and
not `\s+\z`, which cannot match a body with no trailing whitespace to consume.

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
