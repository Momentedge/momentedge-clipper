#!/usr/bin/env bash
# Package the clipper binary into a Debian package with cargo-deb, reading
# [package.metadata.deb] from crates/clipper/Cargo.toml. clipper is a Rust/r2r
# crate with no ament build type, so bloom cannot build it; cargo-deb is the
# idiomatic deb tool for a Cargo binary and assembles the package from the
# already-built binary.
#
# This packages only — build first (scripts/build-on-target.sh, or
# scripts/ros-cargo.sh build --release -p clipper), which is the step that needs
# the ROS environment. cargo-deb here neither compiles nor links, so it needs no
# sourced ROS: --no-build takes target/release/clipper as-is.
#
# --variant $ROS_DISTRO selects the matching ros-<distro>-* Depends; the .deb is
# named with the ubuntu+distro tag so the per-distro packages don't collide.
#
# Env:
#   ROS_DISTRO   target ROS2 distro  (required; selects the cargo-deb variant)
#   VERSION      package version     (default: workspace version from Cargo.toml)
#   OUT_DIR      where the .deb goes  (default: ./dist)
# Tools required on PATH: cargo, cargo-deb, dpkg.
set -euo pipefail

_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$_here/.." && pwd)"
cd "$REPO_ROOT"

: "${ROS_DISTRO:?set ROS_DISTRO (humble|jazzy) — selects the cargo-deb variant}"
VERSION="${VERSION:-$(grep -m1 '^version' Cargo.toml | sed -E 's/.*"([^"]+)".*/\1/')}"
OUT_DIR="${OUT_DIR:-$REPO_ROOT/dist}"
ARCH="$(dpkg --print-architecture)"
UBUNTU_VERSION="$(. /etc/os-release && echo "$VERSION_ID")"

[[ -x target/release/clipper ]] || {
  echo "missing target/release/clipper — run scripts/build-on-target.sh first" >&2
  exit 1
}

mkdir -p "$OUT_DIR"
DEB="$OUT_DIR/momentedge-clipper_${VERSION}_ubuntu${UBUNTU_VERSION}-${ROS_DISTRO}_${ARCH}.deb"

echo "cargo-deb: clipper -> momentedge-clipper $VERSION ($ROS_DISTRO, ubuntu $UBUNTU_VERSION, $ARCH)"
cargo deb -p clipper --no-build --variant "$ROS_DISTRO" --deb-version "$VERSION" --output "$DEB"

echo
dpkg-deb --info "$DEB"
dpkg-deb --contents "$DEB"
echo
echo "built $DEB"
