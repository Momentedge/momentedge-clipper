#!/usr/bin/env bash
# Assemble a Debian package for the clipper recorder from a native build against
# the host's apt ROS2. Not a cross-build and not nix: run it on (or in a
# container matching) the deployment target — Ubuntu 22.04/Humble or 24.04/Jazzy
# — with that distro's ROS2 installed. It builds the clipper binary and the
# momentedge_msgs typesupport overlay (scripts/build-on-target.sh), lays them out
# under /opt/momentedge-clipper with a thin /usr/bin/momentedge-clipper wrapper,
# and writes a DEBIAN/control that declares clipper's ROS2 runtime packages as
# Depends so apt pulls them on install.
#
# clipper is the only binary packaged: trigger-pub is a dev stand-in and the
# record scripts run from the host's ROS2, so neither ships here. The binary
# carries RUNPATHs to the target ROS lib dir and the package's own lib dir, and
# the wrapper sources the ROS and overlay environments so AMENT_PREFIX_PATH and
# LD_LIBRARY_PATH are set for the dlopen'd typesupport.
#
# Inputs (env):
#   ROS_DISTRO   humble | jazzy            (default: derived from the sourced ROS)
#   VERSION      package version           (default: workspace version, Cargo.toml)
#   OUT_DIR      where the .deb is written (default: ./dist)
# Tools required on PATH: dpkg-deb, colcon, cargo, a C/Rust toolchain.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

PREFIX="/opt/momentedge-clipper"
ROS_DISTRO="${ROS_DISTRO:-}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"

# Derive the distro from the ROS install when not given explicitly. ROS2's
# setup.bash references unset vars, so relax -u only around sourcing it.
if [[ -z "$ROS_DISTRO" ]]; then
  set +u; source "$ROS_SETUP"; set -u
  : "${ROS_DISTRO:?could not determine ROS_DISTRO — set ROS_DISTRO or ROS_SETUP}"
fi

VERSION="${VERSION:-$(grep -m1 '^version' Cargo.toml | sed -E 's/.*"([^"]+)".*/\1/')}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/dist}"
ARCH="$(dpkg --print-architecture)"
# Read VERSION_ID in a subshell so sourcing os-release does not clobber $VERSION.
UBUNTU_VERSION="$(. /etc/os-release && echo "$VERSION_ID")"

echo "packaging momentedge-clipper $VERSION for $ROS_DISTRO (ubuntu $UBUNTU_VERSION, $ARCH)"

# 1. Native build of the clipper binary with RUNPATHs pointing at the installed
#    locations, so it resolves rcl/rmw and the directly-linked momentedge_msgs
#    typesupport even without a sourced environment. The wrapper adds
#    AMENT_PREFIX_PATH and LD_LIBRARY_PATH on top, which the dlopen'd rmw-specific
#    typesupport still needs. BUILD_PACKAGES=clipper builds only what is packaged.
MOMENTEDGE_RPATH="/opt/ros/${ROS_DISTRO}/lib:${PREFIX}/lib" \
ROS_SETUP="$ROS_SETUP" \
BUILD_PACKAGES="clipper" \
  "$REPO_ROOT/scripts/build-on-target.sh"

# 2. Stage the package tree.
STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT
install -d "$STAGE$PREFIX/bin" "$STAGE/usr/bin" "$STAGE/DEBIAN"

# The clipper binary — the only binary packaged.
install -m 0755 target/release/clipper "$STAGE$PREFIX/bin/clipper"

# Bundled momentedge_msgs overlay: the Trigger/Recorded types clipper subscribes
# and publishes are not in apt, so the package carries their typesupport libs
# (lib/) and ament index (share/). The generated C headers under include/ are
# build-time only and stay out of the package.
cp -a install/momentedge_msgs/lib   "$STAGE$PREFIX/"
cp -a install/momentedge_msgs/share "$STAGE$PREFIX/"

# Relocatable overlay environment: put the bundled overlay on the ament/library
# search paths. The literal ${VAR:+...} forms are expanded on the target at
# source time; $PREFIX is baked in here.
cat > "$STAGE$PREFIX/setup.bash" <<EOF
# momentedge-clipper overlay environment. Source the matching ROS2 distro setup
# (/opt/ros/$ROS_DISTRO/setup.bash) first; this adds the bundled momentedge_msgs
# typesupport.
export AMENT_PREFIX_PATH="$PREFIX\${AMENT_PREFIX_PATH:+:\$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PREFIX/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
EOF

# /usr/bin/momentedge-clipper wrapper: source ROS + the overlay, then exec the
# binary. The binary carries RUNPATHs too, but the wrapper's LD_LIBRARY_PATH and
# AMENT_PREFIX_PATH are what the dlopen'd rmw-specific typesupport needs. It
# forwards its args to clipper.
cat > "$STAGE/usr/bin/momentedge-clipper" <<EOF
#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash
source $PREFIX/setup.bash
exec $PREFIX/bin/clipper "\$@"
EOF
chmod 0755 "$STAGE/usr/bin/momentedge-clipper"

# 3. DEBIAN/control with clipper's ROS2 runtime packages as Depends, so apt pulls
#    them on `apt install ./momentedge-clipper_*.deb`. rmw_fastrtps_cpp is named
#    explicitly: it is the RMW the clipper binary and the bundled typesupport
#    load, so naming it directly does not rely on ros-base keeping it the default.
INSTALLED_SIZE="$(du -sk "$STAGE$PREFIX" "$STAGE/usr" | awk '{s+=$1} END {print s}')"
cat > "$STAGE/DEBIAN/control" <<EOF
Package: momentedge-clipper
Version: $VERSION
Architecture: $ARCH
Maintainer: Stefan Lendl <s@stfl.dev>
Section: utils
Priority: optional
Installed-Size: $INSTALLED_SIZE
Depends: ros-$ROS_DISTRO-ros-base, ros-$ROS_DISTRO-rmw-fastrtps-cpp
Description: Triggered MCAP clip recorder for ROS2 ($ROS_DISTRO)
 clipper tails one continuous ros2 bag MCAP recording and cuts clips around
 momentedge_msgs/Trigger events, announcing each via momentedge_msgs/Recorded.
 Bundles the momentedge_msgs typesupport (not packaged in apt). Built natively
 against ROS2 $ROS_DISTRO for ABI compatibility with the host's ROS graph.
EOF

# 4. Build the .deb. --root-owner-group keeps installed files root:root without
#    fakeroot. The distro is in the filename so both per-distro packages can
#    attach to one release without colliding.
mkdir -p "$OUT_DIR"
DEB="$OUT_DIR/momentedge-clipper_${VERSION}_ubuntu${UBUNTU_VERSION}-${ROS_DISTRO}_${ARCH}.deb"
dpkg-deb --root-owner-group --build "$STAGE" "$DEB"
echo
dpkg-deb --info "$DEB"
dpkg-deb --contents "$DEB"
echo
echo "built $DEB"
