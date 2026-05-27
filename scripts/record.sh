#!/usr/bin/env bash
# Continuous rosbag2 recorder feeding the triggered extractor (edgestream-rec).
#
# Records every live topic into 5-second MCAP splits under ./record. Each split
# boundary makes rosbag2 publish a rosbag2_interfaces/WriteSplitEvent on
# /events/write_split, which edgestream-rec waits on before cutting a clip.
#
# rosbag2 refuses to record into an existing bag directory, so ./record is wiped
# on start. ./record is gitignored. There is no automatic retention — the splits
# accumulate until you stop recording or clear ./record yourself.
#
# Run inside the dev shell (provides ros2bag + the mcap storage plugin):
#   nix develop --command ./scripts/record.sh
set -euo pipefail

OUT_DIR="${1:-./record}"

rm -rf "$OUT_DIR"
echo "recording all topics into $OUT_DIR (5 s mcap splits) — Ctrl-C to stop"
exec ros2 bag record -a \
  --storage mcap \
  --max-bag-duration 5 \
  --output "$OUT_DIR"
