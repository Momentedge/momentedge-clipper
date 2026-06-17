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

# ROS2's setup.bash references unset vars (AMENT_TRACE_SETUP_FILES, …); it is not
# nounset-clean, so relax -u only around sourcing it.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
echo "building against ROS2 ${ROS_DISTRO} ($ROS_SETUP)"

# 1. Build the local interface package into a colcon overlay. momentedge_msgs is
#    not in apt, so its Trigger/Recorded typesupport must be generated here; the
#    overlay feeds both r2r's codegen below and the binaries at runtime.
colcon build --packages-select momentedge_msgs
# The colcon overlay's setup is likewise not nounset-clean.
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u

# 2. Build the two binaries. r2r regenerates its Rust bindings from the message
#    IDL on AMENT_PREFIX_PATH at build time; restrict codegen to exactly the
#    packages the deployables reference — builtin_interfaces and momentedge_msgs
#    (r2r does no dependency resolution).
# bindgen (r2r's build script) needs libclang. Ubuntu's libclang-dev installs it
# under a versioned llvm dir whose number tracks the release (llvm-14 on 22.04,
# llvm-18 on 24.04), so prefer llvm-config and fall back to a search rather than
# pinning one version.
if [[ -z "${LIBCLANG_PATH:-}" ]]; then
  LIBCLANG_PATH="$(llvm-config --libdir 2>/dev/null || true)"
  if [[ -z "$LIBCLANG_PATH" ]]; then
    libclang="$(find /usr/lib /usr/lib64 -name 'libclang.so*' 2>/dev/null | head -n1)"
    [[ -n "$libclang" ]] && LIBCLANG_PATH="$(dirname "$libclang")"
  fi
fi
export LIBCLANG_PATH
export IDL_PACKAGE_FILTER="builtin_interfaces;momentedge_msgs"

# Optional: bake RUNPATHs into the binaries so they resolve rcl/rmw and the
# momentedge_msgs typesupport at their *installed* locations, independent of any
# sourced environment. scripts/package-deb.sh sets this to the target ROS lib dir
# and the package's own lib dir; left unset (the default native on-target build)
# the binaries resolve via the sourced overlay as before.
if [[ -n "${MOMENTEDGE_RPATH:-}" ]]; then
  IFS=':' read -ra _rpath_dirs <<< "$MOMENTEDGE_RPATH"
  for _d in "${_rpath_dirs[@]}"; do
    [[ -n "$_d" ]] && RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-rpath,$_d"
  done
  export RUSTFLAGS
fi

# Which workspace crates to build. Default: both deployables. scripts/package-deb.sh
# sets BUILD_PACKAGES=clipper, the only binary it packages.
read -ra _build_pkgs <<< "${BUILD_PACKAGES:-clipper trigger-pub}"
_pkg_flags=()
for _p in "${_build_pkgs[@]}"; do _pkg_flags+=(-p "$_p"); done
cargo build --release "${_pkg_flags[@]}"

echo
echo "built:"
for _p in "${_build_pkgs[@]}"; do echo "  $REPO_ROOT/target/release/$_p"; done
echo "run the recorder: scripts/record-continuous.sh + target/release/clipper"
echo "run the demo trigger: scripts/start_demo_trigger_pub.sh (needs trigger-pub built)"
