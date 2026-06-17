#!/usr/bin/env bash
# Run cargo for the recorder crates against an apt ROS2 install. Sets up the one
# environment r2r's build needs and then execs `cargo "$@"`:
#   - the ROS distro setup, and the momentedge_msgs colcon overlay when built
#     (install/setup.bash), so AMENT_PREFIX_PATH resolves the message packages
#   - libclang for r2r's bindgen, and IDL_PACKAGE_FILTER restricting codegen to
#     builtin_interfaces;momentedge_msgs
#   - optional MOMENTEDGE_RPATH baked as RUNPATHs into the linked binaries
#
# scripts/build-on-target.sh and the release CI's build and unit-test steps all
# go through here, so that environment is defined in exactly one place. The
# caller provides the rest of PATH (cargo and the C/Rust toolchain).
#
# Env:
#   ROS_SETUP         ROS setup.bash to source  (default /opt/ros/${ROS_DISTRO:-humble})
#   MOMENTEDGE_RPATH  colon-separated RUNPATH dirs to bake in   (optional)
#   LIBCLANG_PATH     override the detected libclang dir         (optional)
#
# Usage: scripts/ros-cargo.sh build --release -p clipper
#        scripts/ros-cargo.sh test  --release -p clipper --bins
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"

# ROS2's setup.bash and the colcon overlay reference unset vars (they are not
# nounset-clean), so relax -u only around sourcing them. The overlay is sourced
# only once built (the build step produces it).
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
[[ -f install/setup.bash ]] && source install/setup.bash
set -u

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
