#!/usr/bin/env bash
# Demo trigger publisher (trigger-pub) for the native Humble target — a stand-in
# that publishes momentedge_msgs/Trigger on /events/clipper/trigger ~1/s so
# clipper produces clips. A real trigger source replaces it in
# production; the recorder (clipper) does not depend on this demo
# publisher.
#
# Runs in the foreground (Ctrl-C to stop). Extra args are forwarded; the
# preroll/postroll windows are in nanoseconds, e.g. a 2 s / 3 s window:
#   ./scripts/start_demo_trigger_pub.sh --preroll 2000000000 --postroll 3000000000
# With no flags each trigger draws a random 1–10 s window. Build it first with
# scripts/build-on-target.sh.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BIN_DIR="${BIN_DIR:-$REPO_ROOT/target/release}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"   # momentedge_msgs typesupport

exec "$BIN_DIR/trigger-pub" "$@"
