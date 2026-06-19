#!/usr/bin/env bash
# Example clipper invocation matching scripts/record.sh.
#
# clipper tails the split recording scripts/record.sh writes under ./record,
# following each rollover to the newest `<bag>_<n>.mcap`, and cuts a clip per
# trigger into ./clipped. The --record-dir here must match record.sh's output
# directory, and --grace-secs must exceed the recorder's flush latency: with
# record.sh's stock chunked+compressed profile a clip's data is visible to the
# tail only once a chunk fills, so the default 30 s grace leaves headroom (drop
# it toward 0 only with the fastwrite profile — see examples/continuous).
#
# This is a minimal example invocation. examples/systemd shows the same as a
# long-running service.
#
# Run inside a sourced ROS2 environment (e.g. . /opt/ros/<distro>/setup.bash):
#   ./scripts/run.sh
set -euo pipefail

# clipper links rcl/rmw and resolves momentedge_msgs typesupport from the ROS2
# environment; setup.bash exports ROS_DISTRO, so its absence means nothing is
# sourced yet.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is unset — source a ROS2 environment first:" >&2
  echo "  . /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

# The installed binary (the momentedge-clipper deb, or `cargo install`). Without
# an install, run the crate from the workspace instead:
#   cargo run -p clipper -- --record-dir ./record
if ! command -v clipper >/dev/null 2>&1; then
  echo "clipper not on PATH — install the momentedge-clipper package, or run" >&2
  echo "from the workspace: cargo run -p clipper -- --record-dir ./record" >&2
  exit 1
fi

# Every flag also reads a MOMENTEDGE_* env var (CLI > env > default); keep these
# defaults aligned with scripts/record.sh's OUT_DIR.
exec clipper \
  --record-dir "${MOMENTEDGE_RECORD_DIR:-./record}" \
  --out-dir "${MOMENTEDGE_OUT_DIR:-./clipped}" \
  --grace-secs "${MOMENTEDGE_GRACE_SECS:-30}"
