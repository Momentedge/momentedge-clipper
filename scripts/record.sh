#!/usr/bin/env bash
# Example continuous `ros2 bag record` for clipper to tail.
#
# Records every live topic (`--all`) into one growing MCAP file under ./record
# — the single file clipper keeps open and tails (scripts/run.sh runs clipper
# against the same directory). There are no bag splits: clipper follows one file,
# so a continuous recording is what pairs with it.
#
# This is a minimal example invocation, not a deployment entry point. The full
# setups — low-latency tuning, split bags with retention, and a systemd unit
# layout — live under example/:
#   example/continuous/   --max-cache-size / --storage-preset-profile trade-offs
#   example/split-bags/   --max-bag-size / --max-bag-duration (and clipper's limits)
#   example/systemd/      rosbag + clipper + pruning as services
#
# rosbag2 refuses to record into an existing bag directory, so ./record is
# wiped on start. ./record is gitignored.
#
# Run inside a sourced ROS2 environment (e.g. . /opt/ros/<distro>/setup.bash):
#   ./scripts/record.sh
set -euo pipefail

# rosbag2 needs a sourced ROS2 environment. setup.bash exports ROS_DISTRO, so
# its absence means nothing is sourced yet — fail fast rather than emit an
# opaque "ros2: command not found".
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is unset — source a ROS2 environment first:" >&2
  echo "  . /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

OUT_DIR="${OUT_DIR:-./record}"

# Storage defaults: regular caching and the stock chunked+compressed profile.
# These are the rosbag2 defaults — a clip's data becomes visible to the tail
# only once a chunk fills and the cache drains, so clipper's --grace-secs must
# exceed that flush latency (see example/continuous for the fastwrite profile,
# which writes every message straight through for minimal tail latency).
MAX_CACHE_SIZE="${MAX_CACHE_SIZE:-104857600}"     # 100 MiB, rosbag2 default
STORAGE_PRESET="${STORAGE_PRESET:-zstd_fast}"      # none | fastwrite | zstd_fast | zstd_small

rm -rf "$OUT_DIR"
echo "recording all topics into $OUT_DIR (one continuous mcap, preset=$STORAGE_PRESET cache=$MAX_CACHE_SIZE) — Ctrl-C to stop"
exec ros2 bag record --all \
  --storage mcap \
  --storage-preset-profile "$STORAGE_PRESET" \
  --max-cache-size "$MAX_CACHE_SIZE" \
  --output "$OUT_DIR"
