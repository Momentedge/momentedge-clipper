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
# Both packages share one artefact naming convention and one version: the .deb is
# named <pkg>_<VERSION>_ubuntu<YY.MM>-<distro>_<arch>.deb (matching the clipper deb),
# and VERSION is the same value clipper uses — the release tag, or the workspace
# Cargo.toml version for a dev build. The literal <version> in package.xml is only
# the in-source default; this script overrides it for the build so the msgs and
# clipper debs never drift apart on a release (bloom would otherwise version the deb
# straight from package.xml).
#
# Tools required on PATH: bloom-generate, fakeroot, dpkg/debhelper, and an
# initialised rosdep (rosdep update) to resolve the package.xml depend keys. The
# build deps (ament_cmake, rosidl generators, builtin_interfaces) come from the
# host ROS install. bloom bakes the distro into the generated debian/rules, which
# sources /opt/ros/<distro>/setup.sh itself, so this script need not source ROS.
#
# Env:
#   ROS_DISTRO   target ROS2 distro            (required, e.g. humble)
#   VERSION      package version               (default: workspace version from Cargo.toml)
#   OS_VERSION   ubuntu codename for bloom     (default: from /etc/os-release)
#   OUT_DIR      where the .deb is written     (default: ./dist)
set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$_here/.." && pwd)"

: "${ROS_DISTRO:?set ROS_DISTRO (e.g. humble) — the distro to package against}"
# VERSION: the single source clipper also uses — the release tag (passed in env) or,
# for a dev build, the workspace version from the root Cargo.toml.
VERSION="${VERSION:-$(grep -m1 '^version' "$REPO_ROOT/Cargo.toml" | sed -E 's/.*"([^"]+)".*/\1/')}"
# bloom's --os-version wants the ubuntu codename (jammy/noble); the artefact filename
# wants the numeric release (22.04/24.04), matching the clipper deb. Read both.
OS_VERSION="${OS_VERSION:-$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")}"
UBUNTU_VERSION="$(. /etc/os-release && echo "$VERSION_ID")"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/dist}"
ARCH="$(dpkg --print-architecture)"
PKG_DIR="$REPO_ROOT/momentedge_msgs"

# bloom and debhelper write into the package dir and the repo root. Clear any
# leftovers from a previous run (the generated debian/ tree, the cmake build dir,
# stray build products in the repo root) so this build starts clean, and restore the
# package.xml the version override below edits in place.
clean() {
  rm -rf "$PKG_DIR/debian" "$PKG_DIR"/.obj-* "$PKG_DIR"/obj-*
  rm -f "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.deb \
        "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.ddeb \
        "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.buildinfo \
        "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs*.changes
  [[ -f "$PKG_DIR/package.xml.orig" ]] && mv -f "$PKG_DIR/package.xml.orig" "$PKG_DIR/package.xml"
  return 0
}
clean
trap clean EXIT

# Override the package.xml version with VERSION so the generated deb tracks the
# release/tag rather than the literal in package.xml. bloom derives the changelog,
# control, and deb version all from package.xml, so this is the one knob; clean()
# restores the original on exit.
cp "$PKG_DIR/package.xml" "$PKG_DIR/package.xml.orig"
sed -i -E "s|<version>[^<]+</version>|<version>${VERSION}</version>|" "$PKG_DIR/package.xml"

cd "$PKG_DIR"
echo "bloom-generate rosdebian: momentedge_msgs $VERSION ($ROS_DISTRO / ubuntu $OS_VERSION)"
bloom-generate rosdebian --os-name ubuntu --os-version "$OS_VERSION" --ros-distro "$ROS_DISTRO"

# debhelper builds the binary .deb into the parent dir (the repo root). `binary`
# does the cmake/ament build, installs to debian/<pkg>/opt/ros/<distro>, and runs
# dh_builddeb — no source package or signing. noautodbgsym suppresses the separate
# -dbgsym .ddeb: this is typesupport for a tiny interface package, not something
# worth shipping debug symbols for.
DEB_BUILD_OPTIONS="${DEB_BUILD_OPTIONS:+$DEB_BUILD_OPTIONS }noautodbgsym" \
  fakeroot debian/rules binary

# Rename bloom's native output (ros-<distro>-momentedge-msgs_<ver>-0<codename>_<arch>.deb)
# to the shared convention the clipper deb uses, so a release carries one consistent
# set of artefact names.
mkdir -p "$OUT_DIR"
_src="$(echo "$REPO_ROOT"/ros-"$ROS_DISTRO"-momentedge-msgs_*.deb)"
DEB="$OUT_DIR/ros-${ROS_DISTRO}-momentedge-msgs_${VERSION}_ubuntu${UBUNTU_VERSION}-${ROS_DISTRO}_${ARCH}.deb"
mv "$_src" "$DEB"

echo
echo "built:"
echo "  $DEB"
dpkg-deb --info "$DEB"
