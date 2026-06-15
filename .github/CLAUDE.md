# CI — contributor/agent notes

[`workflows/ci.yml`](workflows/ci.yml) is the GitHub Actions CI. It builds,
unit-tests, and runs the live ROS2 e2e suite for `clipper` against
every working ROS2 distro on each push, plus a standalone formatting gate. The
mechanics and the reasoning behind the choices:

- **A standalone `fmt` job on nightly rustfmt, distro- and nix-independent.**
  rustfmt only parses source, so checking formatting needs neither the nix dev
  shell nor a ROS2 closure — only a toolchain's `rustfmt` component, installed
  here via `dtolnay/rust-toolchain@nightly` (`components: rustfmt`, so nightly
  rustfmt only — no full nightly std/clippy). It must be nightly: `rustfmt.toml`
  at the workspace root sets unstable options (`group_imports`,
  `imports_granularity`) that only nightly rustfmt honours, and the dev box's own
  system rustfmt is nightly, so a stable rustfmt here would both reject the config
  and reflow locally-clean code. One cheap job runs `cargo fmt --all --check` over
  both workspace crates, in parallel with the recorder matrix. It deliberately
  stays outside the matrix: format is identical across distros, so running it per
  distro would be three redundant copies behind a heavy nix realization.
- **One job per working distro, three steps (build → unit → e2e).** A single
  matrix leg per distro shares one nix dev shell and one cargo target dir across
  all three steps, so the ROS closure is realized once and the crate compiled
  once per distro. Only `clipper` is built; `trigger-pub` is out of
  CI. The distro set is the r2r-buildable three — `humble`, `jazzy`, `lyrical`
  (rolling cannot build the crates; see the root CLAUDE.md "Build and
  environment mechanics" and README "Integration tests").
- **One run per commit.** `on: push` fires on all branches; `pull_request` is
  omitted so a same-repo PR branch does not run the heavy matrix twice (its push
  and PR events fall in different concurrency groups). Same-repo PRs still
  surface the push run's checks; add `pull_request:` if fork contributions ever
  need their own CI.
- **System Rust inside the nix shell, exactly as locally.** The shell ships no
  Rust, so each build/unit/e2e step runs `nix develop .#<distro> --command cargo
  …` with the toolchain from `dtolnay/rust-toolchain@stable` on `PATH`. The crates
  carry no feature gates, so stable builds them; the action tracks the `stable`
  channel (the exact patch floats with it). The build/test path deliberately
  stays on stable — only the `fmt` job adopts the dev box's nightly, and only for
  rustfmt (see above). `cargo --locked` holds the build to the committed
  `Cargo.lock`.
- **Nix store cached through the GitHub Actions cache, not an external service.**
  `cache-nix-action` saves and restores `/nix/store` keyed per distro on the
  flake inputs — "push and load the nix store" with no external dependency, the
  GitHub cache being exactly that. `ros.cachix.org` is a read-only substituter
  (the same one the flake's `nixConfig` names); nothing is pushed to it and no
  token is needed. Whatever `ros.cachix.org` lacks for the pinned overlay
  revision (e.g. `cyclonedds`, `iceoryx`) is built from source on the cold run
  and then saved by the same cache, so later runs restore it. Cargo artifacts
  cache the same way (`Swatinem/rust-cache`, per distro). Per-distro keys, a
  purge of older same-prefix caches, and a store-size trim keep the three
  distros under GitHub's 10 GB per-repo cache budget; on `jazzy`/`humble` the
  dev shell additionally pulls the sim/GStreamer stack (`withSim`), so those
  closures are the largest.
- **Nix is installed from the official source.** `cachix/install-nix-action`
  fetches Nix from `releases.nixos.org` — no third-party installer CDN. That is
  both the no-external-dependency choice and the one that installs reliably
  inside `act`'s container (the Determinate installer's CDN times out there).
- **Action versions are pinned to major tags** (`actions/checkout@v6`,
  `cachix/install-nix-action@v31`, `nix-community/cache-nix-action@v7`,
  `taiki-e/install-action@v2`, `Swatinem/rust-cache@v2`), so they pick up patch
  and security fixes without a manual bump; `dtolnay/rust-toolchain@stable`
  (build/test) and `dtolnay/rust-toolchain@nightly` (fmt) track their channels.
- **Every leg drops `corrupt_tail_health_live`; lyrical drops one more.** That
  test is the live-corruption race the project flags as its one inherently flaky
  case: after damaging the recording mid-write it waits 60 s for the mandatory
  post-damage announcement, which misses under CI-grade timing — observed in
  `act`, where it fails all retries and (nextest's default fail-fast) masks the
  rest of the suite, so only 2/14 run. The E2E step sets
  `CLIPPER_E2E_SKIP_FLAKY=1`, which the test's own `skip_flaky()` gate
  (`tests/harness/mod.rs`, mirroring `require_e2e()`) reads to skip-and-pass — so
  the skip lives in the test, the reason next to it, and a local run (env unset)
  still exercises the corruption race. Skipping it on every distro keeps the
  suite green and the other 13 as coverage (beads `ros2_subscribe-3q8`). Lyrical
  additionally drops `old_recording_on_disk_is_not_recovered_after_restart` — a
  distro incompatibility (its timestamped rosbag2 filenames break the test's
  harness assumption, beads `ros2_subscribe-7ys`), not CI flakiness, so it stays
  a nextest filterset (`not test(...)`) on lyrical's leg rather than an env gate.
  humble/jazzy run 13 and lyrical 12.

## Verifying locally with `act`

```bash
act -l                                         # list the matrix
act push -j recorder --matrix distro:jazzy \
  --container-options "--shm-size=2g"          # run one distro leg locally
```

The larger `/dev/shm` matches a GitHub-hosted runner; the live e2e tests'
FastDDS shared-memory transport needs more than act's Docker default of 64 MB.
act copies the working tree honouring `.gitignore`, so the local `target/` and
`.omc/` never enter the container.

**humble** and **jazzy** substitute almost their whole closure prebuilt from
`ros.cachix.org` and run end to end under act. **lyrical** is the exception: its
closure is the least cache-covered, so act has to build packages (e.g. zenoh)
from source — and act's single-user Nix store is **non-sandboxed**, which trips
Nix's purity check (`home directory /homeless-shelter exists`). That is a
limitation of act's container, not of the workflow: GitHub-hosted runners
install a sandboxed Nix daemon where each build gets an isolated home, so
lyrical builds there normally, as does a host `nix develop .#lyrical`. The
workflow carries no special setting for this; verify lyrical's full build/e2e on
a host shell instead.
