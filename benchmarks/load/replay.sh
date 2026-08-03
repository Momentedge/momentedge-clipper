#!/usr/bin/env bash
# Replay the benchmark bag as live ROS2 topics, looped, at a calibrated rate.
#
# Stages --bag into /dev/shm on first use (skipped on later runs once a copy
# of the right size is already there), then execs `ros2 bag play` against the
# staged copy — playback reads never touch the disk the benchmark measures.
# Execing means SIGINT/SIGTERM sent to this script's process reach the player
# directly, so it stops the same way `ros2 bag play` itself would.
#
# --rate is a raw `ros2 bag play --rate` multiplier, not a MB/s figure — see
# calibrate.py for turning a target MB/s into one, against this same bag.
#
# Run inside a sourced ROS2 environment (e.g. . /opt/ros/<distro>/setup.bash):
#   ./benchmarks/load/replay.sh --bag ~/bench/bags/example-011-ugv-ds.mcap \
#     --rate 0.32 --loop
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: replay.sh --bag PATH --rate FLOAT [--loop] [--topics "a b c"]

  --bag PATH        mcap file to replay
  --rate FLOAT      playback rate multiplier for `ros2 bag play --rate`
                     (see calibrate.py to derive this from a target MB/s)
  --loop            loop playback indefinitely (`ros2 bag play --loop`)
  --topics "a b c"  restrict playback to these topics (space-separated, quoted)
  -h, --help        print this message
EOF
}

BAG=""
RATE=""
LOOP=0
TOPICS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      BAG="${2:?--bag needs an argument}"
      shift 2
      ;;
    --rate)
      RATE="${2:?--rate needs an argument}"
      shift 2
      ;;
    --loop)
      LOOP=1
      shift
      ;;
    --topics)
      TOPICS="${2:?--topics needs an argument}"
      shift 2
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      echo "replay.sh: unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ -z "$BAG" || -z "$RATE" ]]; then
  echo "replay.sh: --bag and --rate are required" >&2
  usage >&2
  exit 1
fi

# rosbag2 needs a sourced ROS2 environment. setup.bash exports ROS_DISTRO, so
# its absence means nothing is sourced yet — fail fast rather than emit an
# opaque "ros2: command not found".
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is unset — source a ROS2 environment first:" >&2
  echo "  . /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not on PATH even though ROS_DISTRO=$ROS_DISTRO — check the sourced environment" >&2
  exit 1
fi

if [[ ! -f "$BAG" ]]; then
  echo "replay.sh: --bag $BAG: no such file" >&2
  exit 1
fi

# A zero, negative, or non-numeric --rate would otherwise reach `ros2 bag
# play` and fail there confusingly (or hang) well after staging the bag —
# catch it here instead, before doing any of that work.
if ! awk -v r="$RATE" 'BEGIN { exit !(r + 0 > 0) }'; then
  echo "replay.sh: --rate $RATE must be a positive number" >&2
  exit 1
fi

SHM_DIR="/dev/shm/momentedge-bench"
SHM_BAG="$SHM_DIR/$(basename "$BAG")"
SRC_SIZE=$(stat -c%s "$BAG")

need_copy=1
if [[ -f "$SHM_BAG" ]]; then
  DST_SIZE=$(stat -c%s "$SHM_BAG")
  if [[ "$DST_SIZE" == "$SRC_SIZE" ]]; then
    need_copy=0
  fi
fi

if [[ "$need_copy" == 1 ]]; then
  AVAIL=$(df --output=avail -B1 /dev/shm | tail -n1 | tr -d ' ')
  # /dev/shm holds nothing else of ours; the margin beyond the bag's own bytes
  # just keeps a copy from wedging other shared-memory users on the host.
  HEADROOM=$((100 * 1024 * 1024))
  NEEDED=$((SRC_SIZE + HEADROOM))
  if ((AVAIL < NEEDED)); then
    echo "replay.sh: /dev/shm has $AVAIL bytes free, need ~$NEEDED" \
      "to stage $(basename "$BAG") ($SRC_SIZE bytes + $((HEADROOM / 1024 / 1024)) MiB headroom)" >&2
    exit 1
  fi
  mkdir -p "$SHM_DIR"
  echo "replay.sh: staging $BAG -> $SHM_BAG ($((SRC_SIZE / 1024 / 1024)) MiB)" >&2
  cp "$BAG" "$SHM_BAG.tmp"
  mv "$SHM_BAG.tmp" "$SHM_BAG"
else
  echo "replay.sh: reusing already-staged $SHM_BAG" >&2
fi

ARGS=(bag play "$SHM_BAG" --rate "$RATE")
((LOOP)) && ARGS+=(--loop)
if [[ -n "$TOPICS" ]]; then
  # Intentional word-splitting of a space-separated topic list into --topics'
  # nargs='+'.
  # shellcheck disable=SC2206
  TOPIC_ARR=($TOPICS)
  ARGS+=(--topics "${TOPIC_ARR[@]}")
fi

echo "replay.sh: exec ros2 ${ARGS[*]}" >&2
exec ros2 "${ARGS[@]}"
