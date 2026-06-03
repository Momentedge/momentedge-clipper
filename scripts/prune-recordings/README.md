# prune-recordings

Retention for captured recordings: a small helper that deletes files older than
one day, driven by a systemd **user** timer that fires every minute.

## Files

- `prune.sh` — deletes regular files with an mtime older than 24 h under the
  target directory, then removes the empty subdirectories left behind. Each
  pruned path and a summary count are logged to stdout, which the systemd
  service records in the journal. The directory comes from `$PRUNE_DIR`, then the
  first argument, then `~/edgestream-rec/captured`. A missing directory exits 0
  (the timer stays green before the first recording lands).
- `edgestream-prune.service` — `Type=oneshot` unit that runs `prune.sh`. The
  pruned path is set via `Environment=PRUNE_DIR=%h/edgestream-rec/captured`.
- `edgestream-prune.timer` — fires 1 min after boot and every 1 min thereafter.
- `install-remote.sh` — copies the script and units to a remote host, enables
  lingering, and starts the timer.

## Install on the edgestream Orin

```sh
./install-remote.sh                       # melikag@100.67.74.107
./install-remote.sh user@other-host       # any other host
```

This prunes `~/edgestream-rec/captured/` every minute. To prune a different
directory on the host, override the environment in a drop-in:

```sh
systemctl --user edit edgestream-prune.service
# [Service]
# Environment=PRUNE_DIR=/some/other/path
```

## Checking it

```sh
systemctl --user list-timers edgestream-prune.timer
journalctl --user-unit=edgestream-prune.service -f
```

On a host with no per-user persistent journal, user-service output goes to the
system journal — query it with `--user-unit=` (above), not `--user -u`, which
looks for a separate per-user journal and reports "No journal files were found".
