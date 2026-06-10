#!/usr/bin/env bash
# Continuous single-file rosbag2 recorder feeding the tailing extractor
# (edgestream-rec-cont).
#
# Records ONE growing MCAP file under ./record-cont — no splits, no
# WriteSplitEvent. The extractor keeps the file open and tails it, so the
# storage settings here are chosen for tail latency:
#
#   --storage-preset-profile fastwrite   no chunking, no CRC: every record is a
#                                        top-level MCAP record, visible to the
#                                        reader as soon as it is written
#   --max-cache-size 0                   no rosbag2 message cache: each message
#                                        is written through to the file directly
#
# The extractor also reads chunked (zstd/lz4) files, so these settings are a
# latency choice, not a correctness requirement.
#
# Topic selection comes from an optional rosbag2 recorder-parameters YAML — the
# rosbag2_transport Recorder node schema, the same file scripts/record.sh
# accepts (see config/cam_sim.yaml). Only the record.* topic-selection keys are
# read: topics, all/all_topics, regex, exclude_regex, exclude_topics. Without a
# config, every live topic is recorded.
#
# rosbag2 refuses to record into an existing bag directory, so OUT_DIR is wiped
# on start. ./record-cont is gitignored. There is no retention — the single
# file grows until you stop recording (hole-punch retention is tracked in
# beads: ros2_subscribe-wkg).
#
# Run inside the dev shell (provides ros2bag + the mcap storage plugin):
#   nix develop --command ./scripts/record-continuous.sh                       # all topics
#   nix develop --command ./scripts/record-continuous.sh config/cam_sim.yaml   # cam_sim topics
set -euo pipefail

CONFIG="${1:-}"
OUT_DIR="${2:-./record-cont}"

SELECT_ARGS=(--all-topics)
if [[ -n "$CONFIG" ]]; then
  [[ -f "$CONFIG" ]] || { echo "config not found: $CONFIG" >&2; exit 1; }
  selection="$(python3 - "$CONFIG" <<'EOF'
import sys
import yaml

with open(sys.argv[1]) as f:
    doc = yaml.safe_load(f) or {}
# Top-level key is the (arbitrary) recorder node name.
params = next(iter(doc.values()), {}).get("ros__parameters", {})
rec = params.get("record", {})

args = []
if rec.get("all") or rec.get("all_topics"):
    args.append("--all-topics")
if rec.get("topics"):
    args += ["--topics", *rec["topics"]]
if rec.get("regex"):
    args += ["--regex", rec["regex"]]
if rec.get("exclude_regex"):
    args += ["--exclude-regex", rec["exclude_regex"]]
if rec.get("exclude_topics"):
    args += ["--exclude-topics", *rec["exclude_topics"]]

if not any(a in ("--all-topics", "--topics", "--regex") for a in args):
    sys.exit("config selects no topics: set record.topics, record.regex "
             "or record.all_topics")
print("\n".join(args))
EOF
)"
  mapfile -t SELECT_ARGS <<< "$selection"
  echo "topic selection from $CONFIG: ${SELECT_ARGS[*]}"
fi

rm -rf "$OUT_DIR"
echo "recording into $OUT_DIR (one continuous mcap, fastwrite) — Ctrl-C to stop"
exec ros2 bag record "${SELECT_ARGS[@]}" \
  --storage mcap \
  --storage-preset-profile fastwrite \
  --max-cache-size 0 \
  --output "$OUT_DIR"
