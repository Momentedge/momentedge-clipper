# prune-recordings

Retention for captured recordings: a small bash script that loops forever,
deleting files older than one day once a minute.

## Files

- `prune.sh` — loops: prune, `sleep 60`, repeat. Each pass deletes regular files
  with an mtime older than 24 h under the target directory, then removes the
  empty subdirectories left behind. Each pruned path and a summary count are
  logged to stdout. The directory comes from `$PRUNE_DIR`, then the first
  argument, then `~/clipper-rec/captured`. A missing directory is skipped (not
  fatal), so the loop keeps running until the first recording lands.
- `install-remote.sh` — copies the script to a remote host and launches it
  detached (`setsid nohup`), logging to `~/clipper-rec/prune.log`.

## Run it

Directly, pruning the default directory:

```sh
./prune.sh                          # ~/clipper-rec/captured
PRUNE_DIR=/some/path ./prune.sh     # any directory
```

Detached on a host (survives the SSH session):

```sh
setsid nohup ~/.local/bin/clipper-prune.sh >> ~/clipper-rec/prune.log 2>&1 &
```

## Deploy on the Orin

```sh
./install-remote.sh                 # melikag@100.67.74.107
./install-remote.sh user@other-host # any other host
```

This prunes `~/clipper-rec/captured/` every minute. Check on it with:

```sh
ssh melikag@100.67.74.107 'pgrep -af clipper-prune.sh; tail -f ~/clipper-rec/prune.log'
```
