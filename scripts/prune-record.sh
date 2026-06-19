#!/usr/bin/env bash
# Prune old split recordings so the record directory stays bounded.
#
# scripts/record.sh writes a sequence of `<bag>_<n>.mcap` split files and never
# deletes them: splitting caps each file's size, not the directory's total. This
# is the retention half — `find -mmin` deletes any split last modified more than
# MAX_AGE_MIN minutes ago. The split rosbag2 is actively writing keeps a fresh
# mtime, so it is never matched as long as MAX_AGE_MIN stays well above
# record.sh's --max-bag-duration (default 300 s). See example/split-bags.
#
# Two modes, by PRUNE_INTERVAL:
#   PRUNE_INTERVAL=0  (default)  one sweep, then exit — for cron / a systemd timer
#   PRUNE_INTERVAL=N  (seconds)  sweep every N s until stopped — standalone / tmux
#
# Env (all optional):
#   RECORD_DIR      directory of split mcaps                 (default ./record)
#   MAX_AGE_MIN     delete splits older than this many min   (default 30)
#   PRUNE_INTERVAL  loop period in seconds; 0 = single sweep (default 0)
#
# Disk sizing: record grows at the camera's full bitrate, so the retained bytes
# are roughly bitrate x MAX_AGE_MIN. Needs no ROS environment.
set -euo pipefail

RECORD_DIR="${RECORD_DIR:-./record}"
MAX_AGE_MIN="${MAX_AGE_MIN:-30}"
PRUNE_INTERVAL="${PRUNE_INTERVAL:-0}"

prune_once() {
  [[ -d "$RECORD_DIR" ]] || { echo "prune-record: $RECORD_DIR does not exist yet, skipping" >&2; return 0; }
  find "$RECORD_DIR" -mindepth 1 -maxdepth 1 -type f -name '*.mcap' \
    -mmin "+$MAX_AGE_MIN" -print -delete
}

if (( PRUNE_INTERVAL > 0 )); then
  echo "pruning $RECORD_DIR every ${PRUNE_INTERVAL}s (deleting *.mcap older than ${MAX_AGE_MIN}m) — Ctrl-C to stop"
  while true; do
    prune_once
    sleep "$PRUNE_INTERVAL"
  done
else
  prune_once
fi
