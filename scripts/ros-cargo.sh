#!/usr/bin/env bash
# Run cargo for the recorder crates against an apt ROS2 install plus the local
# momentedge_msgs overlay, then exec `cargo "$@"`. scripts/ros-setup.sh sources
# the ROS distro and the overlay (so AMENT_PREFIX_PATH resolves the message
# packages); this script adds the r2r codegen environment on top:
#   - IDL_PACKAGE_FILTER restricting codegen to builtin_interfaces;momentedge_msgs
#   - optional MOMENTEDGE_RPATH baked as RUNPATHs into the linked binaries
#
# The momentedge_msgs overlay must already be built (scripts/ros-build-messages.sh,
# which scripts/build-on-target.sh runs first). scripts/build-on-target.sh and the
# release CI's build and unit-test steps all go through here. The caller provides
# the rest of PATH (cargo and the C/Rust toolchain).
#
# Env:
#   ROS_SETUP         ROS setup.bash to source  (default /opt/ros/${ROS_DISTRO:-humble})
#   MOMENTEDGE_RPATH  colon-separated RUNPATH dirs to bake in   (optional)
#   LIBCLANG_PATH     point bindgen at a non-standard libclang   (optional)
#
# Usage: scripts/ros-cargo.sh build --release -p clipper
#        scripts/ros-cargo.sh test  --release -p clipper --bins
set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ros-setup.sh
source "$_here/ros-setup.sh"
cd "$_here/.."

# bindgen (r2r's build script) needs libclang at build time. clang-sys, the
# loader bindgen uses, finds it without help: it searches the standard Ubuntu
# llvm locations, including the versioned dir libclang-dev installs
# (/usr/lib/llvm-*/lib — llvm-14 on 22.04, llvm-18 on 24.04, …), so the lookup is
# release-agnostic with no path or version to maintain here. Export LIBCLANG_PATH
# before this script only to point bindgen at a non-standard libclang.
export IDL_PACKAGE_FILTER="builtin_interfaces;momentedge_msgs"

# Optional: bake RUNPATHs into the binaries so they resolve rcl/rmw and the
# momentedge_msgs typesupport at their *installed* locations, independent of any
# sourced environment. The release build sets this to the target ROS lib dir and
# the package's own lib dir; left unset (the default native on-target build) the
# binaries resolve via the sourced overlay. Build and test pass the same value so
# their RUSTFLAGS match and cargo reuses the compiled artifacts across the steps.
if [[ -n "${MOMENTEDGE_RPATH:-}" ]]; then
  IFS=':' read -ra _rpath_dirs <<< "$MOMENTEDGE_RPATH"
  for _d in "${_rpath_dirs[@]}"; do
    [[ -n "$_d" ]] && RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-rpath,$_d"
  done
  export RUSTFLAGS
fi

exec cargo "$@"
