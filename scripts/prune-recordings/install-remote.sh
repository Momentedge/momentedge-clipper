#!/usr/bin/env bash
# Deploy the prune loop to a remote edgestream host and start it in the background.
#
# Copies prune.sh to ~/.local/bin/edgestream-prune.sh, stops any previous run,
# and relaunches it detached with setsid+nohup so it survives the SSH session.
# Output goes to ~/edgestream-rec/prune.log.
#
# Usage:
#   ./install-remote.sh [user@host]
# Defaults to melikag@100.67.74.107. The pruned directory defaults to
# ~/edgestream-rec/captured; override it by exporting PRUNE_DIR for the launch
# (edit the ExecStart line below) or pass it as the script's first argument.
set -euo pipefail

TARGET="${1:-melikag@100.67.74.107}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "deploying prune loop to $TARGET"

ssh "$TARGET" 'mkdir -p ~/.local/bin ~/edgestream-rec/captured'
scp "$HERE/prune.sh" "$TARGET:.local/bin/edgestream-prune.sh"

ssh "$TARGET" '
  set -euo pipefail
  chmod +x ~/.local/bin/edgestream-prune.sh

  # Remove any earlier systemd-based install.
  systemctl --user disable --now edgestream-prune.service edgestream-prune.timer 2>/dev/null || true
  rm -f ~/.config/systemd/user/edgestream-prune.service ~/.config/systemd/user/edgestream-prune.timer
  systemctl --user daemon-reload 2>/dev/null || true

  # Stop the previous loop via its PID file (the script records its own PID).
  # Avoid pkill -f: this very SSH command line contains the script name.
  pidfile=~/edgestream-rec/prune.pid
  [ -f "$pidfile" ] && kill "$(cat "$pidfile")" 2>/dev/null || true
  sleep 1

  # Relaunch detached: fds redirected away from the SSH channel so it returns.
  setsid nohup ~/.local/bin/edgestream-prune.sh >> ~/edgestream-rec/prune.log 2>&1 < /dev/null &
  sleep 2
  pid=$(cat "$pidfile" 2>/dev/null || true)
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    echo "--- running (pid $pid) ---"
    tail -n 3 ~/edgestream-rec/prune.log
  else
    echo "FAILED to start"; exit 1
  fi
'

echo "done. follow pruned-file logs with:"
echo "  ssh $TARGET 'tail -f ~/edgestream-rec/prune.log'"
