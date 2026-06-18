# Setup examples

Each subdirectory is a self-contained guide for one way to run the recording +
clipper stack. The `scripts/record.sh` / `scripts/run.sh` pair in the repo root
is the minimal continuous setup; these expand on it.

- [`continuous/`](continuous/README.md) — one growing MCAP file tailed by
  clipper (the pairing clipper is built for). Covers the `--max-cache-size` and
  `--storage-preset-profile` latency/size trade-offs.
- [`split-bags/`](split-bags/README.md) — `--max-bag-size` / `--max-bag-duration`
  split recording with pruning for retention. Notes the rosbag2
  message-loss-at-split caveat and why clipper does not follow splits.
- [`systemd/`](systemd/README.md) — the recorder, clipper, and a prune timer as
  systemd units, with the ROS env and `MOMENTEDGE_*` configuration in
  `Environment=`.
