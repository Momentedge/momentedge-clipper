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

# `--all` (not `--all-topics`) records every live topic on every supported
# distro: Humble names this flag `-a`/`--all` and has no `--all-topics`, while
# Jazzy and newer keep `--all` as a superset (topics + service-event topics, no
# service events in this bench). The config-path flags below (`--topics`,
# `--exclude-regex`, …) are Jazzy+ syntax used only by the Jazzy-only sim config.
SELECT_ARGS=(--all)
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
selected = False
if rec.get("all") or rec.get("all_topics"):
    args.append("--all")
    selected = True
if rec.get("topics"):
    # An explicit topic list goes through --regex (anchored alternation), the
    # only selection form ros2 bag record accepts on every supported distro:
    # Humble has no --topics (positional only), Lyrical has no positional topics
    # (--topics only), and --regex works on all of them.
    import re
    alt = "|".join(re.escape(t) for t in rec["topics"])
    args += ["--regex", "^(" + alt + ")$"]
    selected = True
if rec.get("regex"):
    args += ["--regex", rec["regex"]]
    selected = True
if rec.get("exclude_regex"):
    args += ["--exclude-regex", rec["exclude_regex"]]
if rec.get("exclude_topics"):
    args += ["--exclude-topics", *rec["exclude_topics"]]

if not selected:
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
