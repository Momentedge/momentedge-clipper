#!/usr/bin/env bash
# Delete recordings older than one day from a captured directory.
#
# The directory to prune is taken from $PRUNE_DIR, then the first positional
# argument, then a default of ~/edgestream-rec/captured. Regular files with an
# mtime older than 24 h are removed and any empty subdirectories left behind are
# cleaned up. A missing directory is not an error — the script exits 0 so the
# systemd timer driving it stays green before the first recording lands.
set -euo pipefail

PRUNE_DIR="${PRUNE_DIR:-${1:-$HOME/edgestream-rec/captured}}"

if [[ ! -d "$PRUNE_DIR" ]]; then
  echo "prune: directory does not exist, nothing to do: $PRUNE_DIR"
  exit 0
fi

# -mmin +1440 = modified more than 1440 minutes (24 h) ago. -print before -delete
# so each path is logged before it is removed; stdout is captured by the systemd
# service journal. Files first, then the now-empty directories they leave behind.
count=0
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
