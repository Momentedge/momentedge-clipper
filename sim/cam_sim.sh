#!/usr/bin/env bash
# The synthetic gscam camera: publish it, publish + record it in-process, or
# stop a stray run. One entry point, three subcommands:
#
#   sim/cam_sim.sh run [launch args]     # publish raw + H.265 (the default)
#   sim/cam_sim.sh record [launch args]  # publish AND record in one process
#   sim/cam_sim.sh stop                  # force-stop orphaned nodes
#
# A bare launch-arg list implies `run`:
#
#   sim/cam_sim.sh width:=1280 height:=720 fps:=60
#   sim/cam_sim.sh pattern:=ball ns:=/sim
#   sim/cam_sim.sh encoder:=hevc_nvenc           # NVIDIA hardware HEVC
#   sim/cam_sim.sh record out:=/tmp/simbag       # bag output directory
#
# `record` composes gscam with rosbag2_transport::Recorder in one
# component_container_mt (launch/sim_camera_record.launch.py), MCAP output.
# Topic selection and recorder settings live in sim/config/recorder_params.yaml.
# rosbag2 refuses an existing bag directory, so the output dir is wiped on
# start. Ctrl-C stops cleanly; inspect with `ros2 bag info <out>`.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # sim/

usage() {
  sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

CMD="run"
if [[ $# -gt 0 ]]; then
  case "$1" in
    run|record|stop) CMD="$1"; shift ;;
    *:=*)            ;;                 # bare launch args -> run
    -h|--help)       usage; exit 0 ;;
    *)               echo "unknown command: $1" >&2; usage >&2; exit 2 ;;
  esac
fi

# stop matches by process NAME with -x (never `pkill -f`, which would also
# match this script and the invoking shell). run.sh-style runs leave a
# gscam_node; record runs leave a component_container_mt — note the latter
# matches ANY multithreaded component container on the machine, not just ours.
if [[ "$CMD" == "stop" ]]; then
  for name in gscam_node component_container_mt; do
    if pgrep -x "$name" >/dev/null 2>&1; then
      echo "stopping $name ..."
      pkill -INT -x "$name" 2>/dev/null || true
    fi
  done
  sleep 1
  for name in gscam_node component_container_mt; do
    if pgrep -x "$name" >/dev/null 2>&1; then
      echo "force-killing $name ..."
      pkill -KILL -x "$name" 2>/dev/null || true
    fi
  done
  echo "done — remaining:"
  pgrep -ax '(gscam_node|component_container_mt)' || echo "  (none)"
  exit 0
fi

# `run` and `record` need the ROS2 env on PATH (the repo dev shell provides
# it; direnv loads it on cd).
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 not on PATH — run inside the dev shell:" >&2
  echo "  nix develop --command ${BASH_SOURCE[0]} $CMD $*" >&2
  exit 1
fi

case "$CMD" in
  run)
    exec ros2 launch "${HERE}/launch/sim_camera.launch.py" "$@"
    ;;
  record)
    OUT_DIR="./record"
    for arg in "$@"; do
      case "$arg" in
        out:=*) OUT_DIR="${arg#out:=}" ;;
      esac
    done
    rm -rf "$OUT_DIR"
    exec ros2 launch "${HERE}/launch/sim_camera_record.launch.py" "$@"
    ;;
esac
