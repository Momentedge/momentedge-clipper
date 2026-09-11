#!/usr/bin/env bash
# Build the clipper recorder natively on the deployment target, against the
# host's own ROS2 (apt). No nix, no Docker: when the target runs the same ROS2
# distro we build against, the two Rust binaries link the exact rcl/rmw/message
# libraries that load them at runtime, so they are ABI-compatible with the rest
# of the host's ROS graph (rosbag2, the camera node, …). rosbag2 with MCAP
# storage and WriteSplitEvent comes from the apt install — only our code is built
# here.
#
# Prerequisites on the target (Ubuntu 22.04 / ROS2 Humble):
#   sudo apt install ros-humble-ros-base ros-humble-rosbag2-storage-mcap \
#                    ros-humble-rosbag2-transport ros-humble-ros2bag \
#                    ros-dev-tools clang libclang-dev
#   # plus a Rust toolchain (rustup, or the distro cargo/rustc) on PATH
#
# Produces, under the repo root:
#   install/                  colcon overlay carrying momentedge_msgs typesupport
#   target/release/clipper, target/release/trigger-pub
# trigger-pub is run by examples/trigger-pub/start_demo_trigger_pub.sh;
# `clipper tail` is run by scripts/run.sh, against the recording scripts/record.sh
# writes.
#
# Override the ROS install with ROS_SETUP=/opt/ros/<distro>/setup.bash.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# 1. Build the momentedge_msgs interface package into a colcon overlay.
#    momentedge_msgs is not in apt, so its Trigger/Recorded typesupport is
#    generated here; the overlay feeds both r2r's codegen and the binaries at
#    runtime. (Both sub-scripts source the ROS env via scripts/ros-setup.sh, which
#    honours ROS_SETUP / ROS_DISTRO.)
"$REPO_ROOT/scripts/ros-build-messages.sh"

# 2. Build the binaries through scripts/ros-cargo.sh, which sources ROS + the
#    just-built overlay and adds the r2r codegen env (IDL_PACKAGE_FILTER) and any
#    MOMENTEDGE_RPATH. Both deployables by default; the .deb build sets
#    BUILD_PACKAGES=clipper, the only one it packages.
#
#    --features clipper/ros is what makes clipper the device build. ROS is a
#    cargo feature of the recorder and it is off by default, so without this flag
#    the binary would link no ROS and offer no `--trigger-source ros` — exactly
#    the wrong artefact for a target set up with an apt ROS2 above. The flag is
#    added only when clipper is selected: cargo rejects a `<pkg>/<feature>`
#    naming a package no `-p` picked. trigger-pub has no such feature; it links
#    r2r unconditionally.
read -ra _build_pkgs <<< "${BUILD_PACKAGES:-clipper trigger-pub}"
_pkg_flags=()
_feature_flags=()
for _p in "${_build_pkgs[@]}"; do
  _pkg_flags+=(-p "$_p")
  if [[ "$_p" == clipper ]]; then
    _feature_flags=(--features clipper/ros)
  fi
done
"$REPO_ROOT/scripts/ros-cargo.sh" build --release "${_feature_flags[@]}" "${_pkg_flags[@]}"

echo
echo "built:"
# Each package here builds one binary of the same name.
for _p in "${_build_pkgs[@]}"; do
  echo "  $REPO_ROOT/target/release/$_p"
done
echo "run the recorder: scripts/record.sh + target/release/clipper tail"
echo "run the demo trigger: examples/trigger-pub/start_demo_trigger_pub.sh (needs trigger-pub built)"
