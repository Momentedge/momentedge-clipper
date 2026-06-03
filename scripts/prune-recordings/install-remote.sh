#!/usr/bin/env bash
# Install the prune script + systemd user timer onto a remote edgestream host.
#
# Copies prune.sh to ~/.local/bin/edgestream-prune.sh and the .service/.timer
# units to ~/.config/systemd/user/, enables user lingering (so the timer runs
# without an active login), then enables and starts the timer.
#
# Usage:
#   ./install-remote.sh [user@host]
# Defaults to melikag@100.67.74.107. The pruned directory defaults to
# ~/edgestream-rec/captured (set in the .service); override it on the host with
#   systemctl --user edit edgestream-prune.service
set -euo pipefail

TARGET="${1:-melikag@100.67.74.107}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "installing edgestream prune timer on $TARGET"

# Stage files into place, creating the directories they need.
ssh "$TARGET" 'mkdir -p ~/.local/bin ~/.config/systemd/user ~/edgestream-rec/captured'
scp "$HERE/prune.sh" "$TARGET:.local/bin/edgestream-prune.sh"
scp "$HERE/edgestream-prune.service" "$HERE/edgestream-prune.timer" \
  "$TARGET:.config/systemd/user/"

# Linger lets the user manager (and thus the timer) keep running without a login
# session. Then load the new units and start the timer.
ssh "$TARGET" '
  set -euo pipefail
  chmod +x ~/.local/bin/edgestream-prune.sh
  loginctl enable-linger "$USER"
  systemctl --user daemon-reload
  systemctl --user enable --now edgestream-prune.timer
  echo "--- timer ---"
  systemctl --user list-timers edgestream-prune.timer --no-pager
'

echo "done. follow pruned-file logs with:"
echo "  ssh $TARGET 'journalctl --user-unit=edgestream-prune.service -f'"
