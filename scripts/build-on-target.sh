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
# trigger-pub is run by start_demo_trigger_pub.sh; clipper is run
# with scripts/record-continuous.sh per deployment.
#
# Override the ROS install with ROS_SETUP=/opt/ros/<distro>/setup.bash.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
echo "building against ROS2 ${ROS_DISTRO} ($ROS_SETUP)"

# 1. Build the local interface package into a colcon overlay. momentedge_msgs is
#    not in apt, so its Trigger/Recorded typesupport must be generated here; the
#    overlay feeds both r2r's codegen below and the binaries at runtime.
colcon build --packages-select momentedge_msgs
# shellcheck disable=SC1091
source install/setup.bash

# 2. Build the two binaries. r2r regenerates its Rust bindings from the message
#    IDL on AMENT_PREFIX_PATH at build time; restrict codegen to exactly the
#    packages the deployables reference — builtin_interfaces and momentedge_msgs
#    (r2r does no dependency resolution).
export LIBCLANG_PATH="${LIBCLANG_PATH:-$(llvm-config --libdir 2>/dev/null || echo /usr/lib/llvm-14/lib)}"
export IDL_PACKAGE_FILTER="builtin_interfaces;momentedge_msgs"
cargo build --release -p clipper -p trigger-pub

echo
echo "built:"
echo "  $REPO_ROOT/target/release/clipper"
echo "  $REPO_ROOT/target/release/trigger-pub"
echo "run the recorder: scripts/record-continuous.sh + target/release/clipper"
echo "run the demo trigger: scripts/start_demo_trigger_pub.sh"
