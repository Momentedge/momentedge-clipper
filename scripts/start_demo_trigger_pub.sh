#!/usr/bin/env bash
# Demo trigger publisher (trigger-pub) — a stand-in that publishes
# momentedge_msgs/Trigger on /events/momentedge/trigger so clipper produces
# clips. A real trigger source replaces it in production; the recorder (clipper)
# does not depend on this demo publisher.
#
# Runs in the foreground (Ctrl-C to stop). Extra args are forwarded; the
# preroll/postroll windows are in nanoseconds, e.g. a 2 s / 3 s window:
#   ./scripts/start_demo_trigger_pub.sh --preroll 2000000000 --postroll 3000000000
# With no flags each trigger draws a random 1–10 s window.
#
# Build the binary first with scripts/build-on-target.sh, and run inside a
# sourced ROS2 environment (e.g. . /opt/ros/<distro>/setup.bash).
set -euo pipefail

# trigger-pub links rcl/rmw and resolves momentedge_msgs typesupport from the
# ROS2 environment; setup.bash exports ROS_DISTRO, so its absence means nothing
# is sourced yet — fail fast rather than emit an opaque rcl error.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is unset — source a ROS2 environment first:" >&2
  echo "  . /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN_DIR="${BIN_DIR:-$REPO_ROOT/target/release}"

# momentedge_msgs typesupport comes from the local build overlay (not apt); the
# colcon overlay's setup.bash is not nounset-clean, so relax -u while sourcing.
set +u
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"
set -u

exec "$BIN_DIR/trigger-pub" "$@"
