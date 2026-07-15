# Examples

Setup guides for running the recording + clipper stack, plus an example trigger
source to drive it. The `scripts/record.sh` / `scripts/run.sh` pair in the repo
root is the minimal continuous setup; the guides expand on it.

- [`continuous/`](continuous/README.md) — one growing MCAP file tailed by
  clipper (the pairing clipper is built for). Covers the `--max-cache-size` and
  `--storage-preset-profile` latency/size trade-offs.
- [`split-bags/`](split-bags/README.md) — `--max-bag-size` / `--max-bag-duration`
  split recording with pruning for retention. clipper follows the rollovers;
  notes the rosbag2 message-loss-at-split caveat and the split-boundary clip
  trade-off.
- [`launch/`](launch/README.md) — the recorder and clipper brought up together
  with `ros2 launch` (the ROS-native orchestration), via a launch file.
- [`trigger-pub/`](trigger-pub/README.md) — an example trigger source: a small
  r2r node that publishes `momentedge_msgs/Trigger` periodically, so you can
  exercise the recorder without a real trigger publisher.
- [`custom-mcap-writer/`](custom-mcap-writer/README.md) — a minimal standalone
  program that writes an MCAP file directly with the `mcap` crate (no ROS): two
  JSON data channels (a typed struct and a raw JSON string) each carrying a
  capture timestamp in `publish_time`, plus an optional synthetic
  `momentedge_msgs/Trigger` message. The producer-side half of capture-time
  windowing — the fixture the live e2e suite drives clipper against — and a
  from-scratch how-to for writing MCAP.
