#!/usr/bin/env bash
# Continuous rosbag2 recorder feeding the triggered extractor (edgestream-rec).
#
# Records into 5-second MCAP splits under ./record. Each split boundary makes
# rosbag2 publish a rosbag2_interfaces/WriteSplitEvent on /events/write_split,
# which edgestream-rec waits on before cutting a clip.
#
# Topic selection comes from an optional rosbag2 recorder-parameters YAML — the
# rosbag2_transport Recorder node schema, the same file a composable Recorder
# node accepts (see config/cam_sim.yaml). Only the record.* topic-selection keys
# are read: topics, all/all_topics, regex, exclude_regex, exclude_topics.
# Storage settings (mcap, 5 s splits, output dir) stay fixed here because
# edgestream-rec depends on them. Without a config, every live topic is
# recorded.
#
# rosbag2 refuses to record into an existing bag directory, so OUT_DIR is wiped
# on start. ./record is gitignored. There is no automatic retention — the splits
# accumulate until you stop recording or clear the directory yourself.
#
# Run inside the dev shell (provides ros2bag + the mcap storage plugin):
#   nix develop --command ./scripts/record.sh                       # all topics
#   nix develop --command ./scripts/record.sh config/cam_sim.yaml   # cam_sim topics
set -euo pipefail

CONFIG="${1:-}"
OUT_DIR="${2:-./record}"

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
echo "recording into $OUT_DIR (5 s mcap splits) — Ctrl-C to stop"
exec ros2 bag record "${SELECT_ARGS[@]}" \
  --storage mcap \
  --max-bag-duration 5 \
  --output "$OUT_DIR"
