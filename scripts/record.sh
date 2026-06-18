#!/usr/bin/env bash
# Example split-bag `ros2 bag record` for clipper to tail.
#
# Records every live topic (`--all`) into ./record, split into a sequence of
# `<bag>_<n>.mcap` files: rosbag2 rolls over to a fresh file each time a size or
# duration cap is hit. clipper tails the newest split and follows each rollover
# (scripts/run.sh runs clipper against the same directory), so clips are cut
# live from the current split.
#
# Splitting bounds each file; disk is bounded only once old splits are pruned,
# which is a separate job — this script only records. See example/split-bags
# for a delete-by-age timer and example/systemd for the recorder + clipper +
# pruner as services.
#
# This is a minimal example invocation, not a deployment entry point. The full
# setups — low-latency tuning, split bags with retention, and a systemd unit
# layout — live under example/:
#   example/continuous/   one growing file (no splits) + latency trade-offs
#   example/split-bags/   --max-bag-size / --max-bag-duration + pruning
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

# Split controls — the bag rolls over when either cap is hit; 0 disables that
# cap. Keep the active split well longer than clipper's pre+post-roll window: a
# trigger whose window reaches back across a boundary captures only the part in
# the current split, since clipper drops a split once it advances past it.
MAX_BAG_DURATION="${MAX_BAG_DURATION:-300}"       # seconds; 0 = no duration split
MAX_BAG_SIZE="${MAX_BAG_SIZE:-0}"                 # bytes;   0 = no size split

rm -rf "$OUT_DIR"
echo "recording all topics into $OUT_DIR (split bags, preset=$STORAGE_PRESET cache=$MAX_CACHE_SIZE dur=${MAX_BAG_DURATION}s size=${MAX_BAG_SIZE}B) — Ctrl-C to stop"
exec ros2 bag record --all \
  --storage mcap \
  --storage-preset-profile "$STORAGE_PRESET" \
  --max-cache-size "$MAX_CACHE_SIZE" \
  --max-bag-duration "$MAX_BAG_DURATION" \
  --max-bag-size "$MAX_BAG_SIZE" \
  --output "$OUT_DIR"
