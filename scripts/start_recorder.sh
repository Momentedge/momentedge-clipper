#!/usr/bin/env bash
# Launch the edgestream recorder stack natively on the Humble target (no Docker).
# Two long-running processes make up "the recorder":
#
#   ros2 bag record  → <data>/recordings   every live topic in 5 s mcap splits;
#                                           each split boundary publishes a
#                                           WriteSplitEvent on /events/write_split
#   edgestream-rec   → <data>/captured      cuts a clip per /events/edgestream/trigger,
#                                           gated on /events/write_split
#
# Running natively (not containerised) means every ROS2 process shares the host's
# /dev/shm, so FastDDS shared-memory transport works and discovery/data interop
# with the host's other Humble nodes is direct — none of the container-era
# FASTDDS_BUILTIN_TRANSPORTS=UDPv4 workaround is needed.
#
# Both processes are started detached (setsid + nohup), each with a pidfile and
# log under <data>, mirroring scripts/prune-recordings. Re-running stops the
# previous pair first. Build the binaries and the edgestream_msgs overlay with
# scripts/build-on-target.sh before the first run.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
DATA_DIR="${EDGESTREAM_DIR:-$HOME/edgestream-rec}"
BIN_DIR="${BIN_DIR:-$REPO_ROOT/target/release}"
SPLIT_SECONDS="${SPLIT_SECONDS:-5}"

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$REPO_ROOT/install/setup.bash"   # edgestream_msgs typesupport

mkdir -p "$DATA_DIR/captured"

stop_prev() {  # $1: pidfile
  [[ -f "$1" ]] || return 0
  kill "$(cat "$1")" 2>/dev/null || true
  rm -f "$1"
}

# Continuous recorder. rosbag2 refuses to write into an existing bag dir, so the
# recordings dir is wiped on each (re)start.
stop_prev "$DATA_DIR/record.pid"
rm -rf "$DATA_DIR/recordings"
setsid nohup ros2 bag record -a --storage mcap \
  --max-bag-duration "$SPLIT_SECONDS" --output "$DATA_DIR/recordings" \
  >> "$DATA_DIR/record.log" 2>&1 &
echo $! > "$DATA_DIR/record.pid"

# Triggered extractor, reading the splits above.
stop_prev "$DATA_DIR/rec.pid"
setsid nohup "$BIN_DIR/edgestream-rec" \
  --record-dir "$DATA_DIR/recordings" --out-dir "$DATA_DIR/captured" \
  >> "$DATA_DIR/rec.log" 2>&1 &
echo $! > "$DATA_DIR/rec.pid"

echo "recorder stack up (native, ROS2 ${ROS_DISTRO}):"
echo "  ros2 bag record  pid $(cat "$DATA_DIR/record.pid")  → $DATA_DIR/recordings  (log: record.log)"
echo "  edgestream-rec   pid $(cat "$DATA_DIR/rec.pid")  → $DATA_DIR/captured  (log: rec.log)"
echo "stop with: kill \$(cat \"$DATA_DIR/record.pid\" \"$DATA_DIR/rec.pid\")"
