#!/usr/bin/env bash
# Continuously delete recordings older than one day from a captured directory.
#
# Loops forever: prune, sleep 1 min, repeat. The directory to prune is taken from
# $PRUNE_DIR, then the first positional argument, then ~/clipper-rec/captured.
# Each pruned path and a summary count are logged to stdout — redirect it to a
# file when running detached. A missing directory is skipped, not fatal — the
# loop keeps running so it picks up once the first recording lands.
#
# Run it directly; for a persistent background run on a host:
#   setsid nohup ~/.local/bin/clipper-prune.sh >> ~/clipper-rec/prune.log 2>&1 &
#
# On start it writes its own PID to $PRUNE_PIDFILE (default ~/clipper-rec/
# prune.pid) so a redeploy can stop the previous run with `kill $(cat …)`.
set -euo pipefail

PRUNE_DIR="${PRUNE_DIR:-${1:-$HOME/clipper-rec/captured}}"
PRUNE_PIDFILE="${PRUNE_PIDFILE:-$HOME/clipper-rec/prune.pid}"

echo "$$" > "$PRUNE_PIDFILE"

prune_once() {
  if [[ ! -d "$PRUNE_DIR" ]]; then
    echo "prune: directory does not exist, nothing to do: $PRUNE_DIR"
    return
  fi

  # -mmin +1440 = modified more than 1440 minutes (24 h) ago. -print before
  # -delete so each path is logged before it is removed. Files first, then the
  # now-empty directories they leave behind.
  local count=0 f
  while IFS= read -r f; do
    echo "pruned: $f"
    count=$((count + 1))
  done < <(find "$PRUNE_DIR" -mindepth 1 -type f -mmin +1440 -print -delete)

  find "$PRUNE_DIR" -mindepth 1 -type d -empty -delete

  if ((count > 0)); then
    echo "pruned $count file(s) older than 24h from $PRUNE_DIR"
  else
    echo "nothing to prune in $PRUNE_DIR"
  fi
}

while true; do
  prune_once
  sleep 60
done
