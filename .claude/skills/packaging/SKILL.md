---
name: packaging
description: >
  How clipper ships as Debian packages — the bloom (momentedge_msgs) + cargo-deb
  (clipper) two-deb pipeline, its run model, and the gotchas. Use when editing the
  packaging scripts, crates/clipper/Cargo.toml [package.metadata.deb], the
  release.yml deb job, or building/verifying the .debs on the target or under act.
---

# Packaging — two debs, two tools

clipper deploys as **two Debian packages**, each built the way the ROS2 ecosystem
builds its kind. This is the rule, not an accident: **bloom only understands ament
build types (`ament_cmake`/`ament_python`/`cmake`/`catkin`) — it has no cargo build
type**, so it cannot build the Rust crate; **cargo-deb is the idiomatic deb tool for
a Cargo binary**. So:

| Package | Source | Tool | Script | Installs to |
|---|---|---|---|---|
| `ros-<distro>-momentedge-msgs` | `momentedge_msgs/` (ament_cmake) | **bloom** | `scripts/package-msgs-deb.sh` | `/opt/ros/<distro>` |
| `momentedge-clipper` | `crates/clipper` (Rust/r2r) | **cargo-deb** | `scripts/package-clipper-deb.sh` | `/opt/momentedge-clipper/bin` |

`momentedge-clipper` declares `Depends: ros-<distro>-momentedge-msgs,
ros-<distro>-rmw-fastrtps-cpp, ros-<distro>-ros-base`. The msgs package is a
first-class ROS deb under `/opt/ros/<distro>`, so the clipper binary needs **no
bundled overlay** — its message typesupport resolves through the distro's own
`setup.bash`, like every ROS executable.

## Run model: no baked rpath, source setup.bash

The clipper binary is built **without `MOMENTEDGE_RPATH`** (the optional rpath knob
in `ros-cargo.sh` stays unset), so it carries **no RUNPATH**. It resolves `rcl`/`rmw`
and the `momentedge_msgs` typesupport — including the dlopen'd rmw-specific
`libmomentedge_msgs__rosidl_typesupport_fastrtps_c.so` — through
`LD_LIBRARY_PATH`/`AMENT_PREFIX_PATH` set by sourcing `/opt/ros/<distro>/setup.bash`.
The msgs package puts those `.so` files in `/opt/ros/<distro>/lib`, which the distro
`setup.bash` already covers. Run it:

```bash
sudo apt install ./ros-humble-momentedge-msgs_*.deb ./momentedge-clipper_*.deb
source /opt/ros/humble/setup.bash
/opt/momentedge-clipper/bin/clipper --help
```

A systemd unit does the same: source `/opt/ros/<distro>/setup.bash` in `ExecStart`,
or set the equivalent `Environment=`/`LD_LIBRARY_PATH`. There is **no overlay
`setup.bash` and no launcher wrapper** in the clipper package — only the binary.

## Build both debs

Build the binary first (the only step that needs the ROS env), then package.
On the target or a matching CI runner, with cargo + cargo-deb on PATH:

```bash
ROS_DISTRO=humble ./scripts/package-msgs-deb.sh        # bloom -> ros-humble-momentedge-msgs deb
BUILD_PACKAGES=clipper ./scripts/build-on-target.sh    # colcon overlay (r2r codegen) + cargo build
ROS_DISTRO=humble ./scripts/package-clipper-deb.sh     # cargo-deb --no-build -> momentedge-clipper deb
```

Both `.deb` files land in `dist/`. `release.yml`'s `deb` job runs exactly this per
distro (humble/jazzy) on native arm64 runners, then a smoke-test that installs both
and runs `clipper`. `cargo install cargo-deb --locked` and `python3-bloom fakeroot
debhelper dpkg-dev` are the build prerequisites beyond `setup-ros` + `libclang`.

**One naming convention, one version.** Both scripts emit
`<pkg>_<VERSION>_ubuntu<YY.MM>-<distro>_<arch>.deb` (e.g.
`ros-humble-momentedge-msgs_0.0.3_ubuntu22.04-humble_arm64.deb` and
`momentedge-clipper_0.0.3_ubuntu22.04-humble_arm64.deb`). `VERSION` is one value for
both: the release tag (`release.yml` sets it from `v*`), or the workspace
`Cargo.toml` version for a dev build. cargo-deb takes `VERSION` via `--deb-version`;
the msgs script overrides `momentedge_msgs/package.xml`'s `<version>` with it for the
bloom build (bloom versions the deb straight from package.xml, so without the override
the msgs deb tracks package.xml while clipper tracks the tag) and renames bloom's
native `…_<ver>-0<codename>_<arch>.deb` to the shared shape. The `<version>` literal in
package.xml is just the in-source default, kept in step with the workspace version.

## Verify under act (amd64 pipeline shape)

```bash
act workflow_dispatch -j deb --matrix distro:humble \
  -P ubuntu-22.04-arm=catthehacker/ubuntu:act-22.04 \
  -P ubuntu-24.04-arm=catthehacker/ubuntu:act-24.04 \
  --container-options "--shm-size=2g"
```

A throwaway amd64 container: validates that both debs build, install (Depends
resolve), and clipper runs — but the binary is amd64, not the target's arm64. The
artifact/release steps are `!env.ACT`-gated, so act never publishes.

## Verify on the target (authoritative arm64)

The deployment box is the Jetson (`momentedge@momentedge-desktop`, aarch64 Ubuntu
22.04 / ROS2 Humble). Build there for a real arm64/Humble proof. The strong test is
to install **both** debs into a clean state and run clipper sourcing **only**
`/opt/ros/<distro>/setup.bash` (not the build overlay), proving the typesupport
comes from the installed msgs package:

```bash
sudo dpkg --purge momentedge-clipper ros-humble-momentedge-msgs   # clean slate
sudo apt install ./dist/ros-humble-momentedge-msgs_*.deb ./dist/momentedge-clipper_*.deb
readelf -d /opt/momentedge-clipper/bin/clipper | grep -i runpath   # expect: none
source /opt/ros/humble/setup.bash
ldd /opt/momentedge-clipper/bin/clipper | grep momentedge          # => /opt/ros/humble/lib/...
RUST_LOG=info MOMENTEDGE_RECORD_DIR=$(mktemp -d) timeout -s INT 6 /opt/momentedge-clipper/bin/clipper
```

Success: `ldd` resolves the typesupport from `/opt/ros/<distro>/lib` (the apt
package, not the build overlay or a baked rpath), clipper logs `clipper up: ...`,
idles, then `SIGINT received; shutting down`.

## Gotchas (each cost real time)

- **Jetson `apt-get update` is blocked.** A wrapper refuses it to protect the
  L4T/BSP on the Seeed carrier (escape: `sudo /usr/bin/apt-get.real` — do **not** use
  it on the production board). `apt-get install` works from the existing package
  cache, so install `debhelper dpkg-dev` without an update.
- **bloom needs rosdep + debhelper + fakeroot.** `sudo rosdep init` (once) +
  `rosdep update` populate the keys bloom resolves from `package.xml`
  (`ament_cmake`, `rosidl_default_generators`, `builtin_interfaces`,
  `rosidl_default_runtime` — all public). bloom 0.11+/0.14 handle the format-3
  `package.xml` fine for `bloom-generate rosdebian` (no rosdistro index needed).
- **bloom emits a `-dbgsym .ddeb`** into the repo root. `package-msgs-deb.sh` sets
  `DEB_BUILD_OPTIONS=noautodbgsym` to suppress it and produce one clean `.deb`.
- **cargo-deb per-distro variants.** `[package.metadata.deb.variants.<distro>]` in
  `crates/clipper/Cargo.toml` supplies the distro-specific `Depends`; each variant
  repeats `name = "momentedge-clipper"` so cargo-deb does **not** append `-<distro>`
  to the package name (the distro lives in the `.deb` filename instead, via
  `--output`). Select with `cargo deb --variant <distro>`.
- **cargo-deb summary comes from `[package].description`.** Without it the deb
  Description is `[generated from Rust crate clipper]`; clipper sets `description`
  (and `license`) in its `[package]` so the control file reads properly.
- **`timeout -s INT N cmd` exits 124** when it fires the signal after the full
  window — the *healthy* case for the clipper live-run. Absorb it (`|| code=$?`),
  don't let `set -e` treat it as a failure.
- **ssh quoting.** Don't nest single quotes inside a single-quoted remote command —
  the inner quote closes the outer one. For anything non-trivial, write a local
  script and pipe it: `ssh host bash -s < script.sh`. cargo lives in `~/.cargo` on
  the target, so a build over ssh needs `source ~/.cargo/env` (login shells don't
  always have it).

## Why not bloom-everything / cargo-deb-everything

bloom cannot build the Rust crate (no `ament_cargo` generator;
`colcon-ros-cargo`/`cargo-ament-build` are build-only, no deb path). cargo-deb could
package the binary but not the ament interface package. The split is the
state-of-the-art answer for a mixed repo: a proper ROS deb for the messages, an
idiomatic Cargo deb for the binary, joined by an apt `Depends`. Bundling everything
into one hand-rolled `dpkg-deb` (a copied `momentedge_msgs` overlay + a custom
`setup.bash` under `/opt/momentedge-clipper`) is the shortcut to avoid: it
duplicates the typesupport, couples the clipper and msgs versions at build time, and
forgoes apt's dependency tracking.
