#!/usr/bin/env bash
# Continuous single-file rosbag2 recorder feeding the tailing extractor
# (clipper).
#
# Records ONE growing MCAP file under ./record-cont — no splits, no
# WriteSplitEvent. The extractor keeps the file open and tails it, so the
# storage defaults here are chosen for tail latency:
#
#   --storage-preset-profile fastwrite   no chunking, no CRC: every record is a
#                                        top-level MCAP record, visible to the
#                                        reader as soon as it is written
#   --max-cache-size 0                   no rosbag2 message cache: each message
#                                        is written through to the file directly
#
# The extractor also reads chunked (zstd/lz4) files, so these settings are a
# latency choice, not a correctness requirement. Override per run via env:
#
#   STORAGE_PRESET   rosbag2 mcap preset (fastwrite | none | zstd_fast |
#                    zstd_small); "none" is the stock rosbag2 chunked profile,
#                    whose on-disk visibility lags by roughly one chunk fill
#                    (chunk size / aggregate data rate)
#   MAX_CACHE_SIZE   rosbag2 message cache in bytes (104857600 is the rosbag2
#                    default; the cache drains eagerly, so it adds little
#                    steady-state latency either way)
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
# beads: clipper-wkg).
#
# Run inside the dev shell (provides ros2bag + the mcap storage plugin):
#   nix develop --command ./scripts/record-continuous.sh                       # all topics
#   nix develop --command ./scripts/record-continuous.sh config/cam_sim.yaml   # cam_sim topics
set -euo pipefail

CONFIG="${1:-}"
OUT_DIR="${2:-./record-cont}"
STORAGE_PRESET="${STORAGE_PRESET:-fastwrite}"
MAX_CACHE_SIZE="${MAX_CACHE_SIZE:-0}"

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
echo "recording into $OUT_DIR (one continuous mcap, preset=$STORAGE_PRESET cache=$MAX_CACHE_SIZE) — Ctrl-C to stop"
exec ros2 bag record "${SELECT_ARGS[@]}" \
  --storage mcap \
  --storage-preset-profile "$STORAGE_PRESET" \
  --max-cache-size "$MAX_CACHE_SIZE" \
  --output "$OUT_DIR"
