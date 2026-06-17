#!/usr/bin/env bash
# Build the ros-<distro>-momentedge-msgs Debian package from the momentedge_msgs
# ament_cmake interface package using bloom — the official ROS deb generator.
# bloom expands package.xml into a debian/ tree and debhelper builds it, installing
# the generated typesupport to /opt/ros/<distro> exactly like any ros-<distro>-*
# package. clipper's own .deb then Depends on this one, so the message typesupport
# resolves through the standard /opt/ros/<distro>/setup.bash with no bundled overlay.
#
# This is the ament half of the packaging pipeline; clipper (a Rust crate, which
# bloom cannot build) is packaged with cargo-deb in scripts/package-clipper-deb.sh.
#
# Tools required on PATH: bloom-generate, fakeroot, dpkg/debhelper, and an
# initialised rosdep (rosdep update) to resolve the package.xml depend keys. The
# build deps (ament_cmake, rosidl generators, builtin_interfaces) come from the
# host ROS install. bloom bakes the distro into the generated debian/rules, which
# sources /opt/ros/<distro>/setup.sh itself, so this script need not source ROS.
#
# Env:
#   ROS_DISTRO   target ROS2 distro            (required, e.g. humble)
#   OS_VERSION   ubuntu codename for bloom     (default: from /etc/os-release)
#   OUT_DIR      where the .deb is written     (default: ./dist)
set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$_here/.." && pwd)"

: "${ROS_DISTRO:?set ROS_DISTRO (e.g. humble) — the distro to package against}"
OS_VERSION="${OS_VERSION:-$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/dist}"
PKG_DIR="$REPO_ROOT/momentedge_msgs"

# bloom and debhelper write into the package dir and the repo root. Clear any
# leftovers from a previous run (the generated debian/ tree, the cmake build dir,
# and any stray build products in the repo root) so this build starts clean.
clean() {
  rm -rf "$PKG_DIR/debian" "$PKG_DIR"/.obj-* "$PKG_DIR"/obj-*
  rm -f "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.ddeb \
        "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.buildinfo \
        "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.changes
}
clean
trap clean EXIT

cd "$PKG_DIR"
echo "bloom-generate rosdebian: momentedge_msgs ($ROS_DISTRO / ubuntu $OS_VERSION)"
bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO"

# debhelper builds the binary .deb into the parent dir (the repo root). `binary`
# does the cmake/ament build, installs to debian/<pkg>/opt/ros/<distro>, and runs
# dh_builddeb — no source package or signing. noautodbgsym suppresses the separate
# -dbgsym .ddeb: this is typesupport for a tiny interface package, not something
# worth shipping debug symbols for.
DEB_BUILD_OPTIONS="${DEB_BUILD_OPTIONS:+$DEB_BUILD_OPTIONS }noautodbgsym" \
  fakeroot debian/rules binary

mkdir -p "$OUT_DIR"
mv "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs_*.deb "$OUT_DIR"/

echo
echo "built:"
for _deb in "$OUT_DIR"/ros-"$ROS_DISTRO"-momentedge-msgs_*.deb; do
  echo "  $_deb"
  dpkg-deb --info "$_deb"
done
