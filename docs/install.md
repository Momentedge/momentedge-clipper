# Installing and building clipper

Every way to get a clipper binary — the Debian packages a release attaches, the
two builds ROS as a cargo feature gives you, the nix package, a native build on
the target — plus which ROS 2 distros each covers and what a device already
running clipper needs before the new package lands. For what clipper is and how
to cut a first clip, start at the [README](../README.md).

## From a release

Each [GitHub release](https://github.com/Momentedge/momentedge-clipper/releases)
attaches two arm64 Debian packages for **Humble** (Ubuntu 22.04) and **Jazzy**
(Ubuntu 24.04) — the deployment target is a Jetson-class board. Install both on a
host running the matching distro:

```bash
sudo apt install ./ros-humble-momentedge-msgs_*.deb ./momentedge-clipper_*.deb
source /opt/ros/humble/setup.bash
/opt/momentedge-clipper/bin/clipper --help
```

`ros-<distro>-momentedge-msgs` is the ament interface package carrying `Trigger`
and `Recorded`; `momentedge-clipper` is the binary, and it `Depends` on that
package plus `ros-<distro>-ros-base` and `ros-<distro>-rmw-fastrtps-cpp` —
which is why both `.deb`s go on one `apt install` line. The binary lands at
`/opt/momentedge-clipper/bin/clipper` (the `clipper` name in `/usr/bin` belongs
to an unrelated Debian package) and resolves its message types through the
ordinary distro `setup.bash`, with no build overlay.

## From source

ROS is a cargo feature, and it decides which of two builds you get:

```bash
cargo build -p clipper                                       # ROS-free
nix build .#clipper-ros-free                                 # …or as a nix package
nix develop --command cargo build -p clipper --features ros  # the device build
```

The ROS-free build needs no ROS installation, no dev shell and no environment;
it reads triggers out of the recording it tails, and its `clipper tail
--trigger-source` offers `mcap` alone. `--features ros` adds the live trigger
subscription and the `Recorded` publish, and is what every Debian artefact is
built with. Everything else — the tail, the window, the cut — is the same code in
both.

`./scripts/build-on-target.sh` builds the device half natively on a target,
against the host's apt ROS 2 rather than a cross toolchain;
[ARCHITECTURE.md](../ARCHITECTURE.md#deployment) says why.

## Which ROS 2 distros

CI builds, lints and runs the unit and live end-to-end suites on **Humble, Jazzy
and Lyrical** on every push; Jazzy is the default dev shell. Humble and Jazzy are
the two that ship `.deb`s. Rolling is not supported: r2r references an rmw QoS
variant Rolling has removed, so the device build does not compile there — the
ROS-free build, which links no r2r, is unaffected.

## Upgrading a device already running clipper

Every unit file needs rewriting for this release: the recorder is a subcommand
(`clipper tail …`) and its trigger source is spelled `--trigger-source`. Read
[the upgrade note](configuration.md#upgrading-a-deployment-configured-for-an-earlier-release)
before the new package lands — one of the stale spellings starts the process and
silently changes nothing, so it has to be hunted rather than waited for.
