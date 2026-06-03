#!/usr/bin/env bash
# Launch the edgestream recorder stack on the deployment target (the Jetson Orin)
# as Docker containers built from the nix images (see nix/images.nix). Two
# containers make up "the recorder":
#
#   edgestream-record  continuous `ros2 bag record -a` of every live topic into
#                      5 s MCAP splits under <data>/recordings. Wrapped in
#                      `rm -rf` so the bag dir is fresh on each (re)start, which
#                      rosbag2 requires (it refuses to write into an existing bag).
#   edgestream-rec     the triggered extractor: cuts clips into <data>/captured
#                      on each /events/edgestream/trigger, gated on /events/write_split.
#
# Both run on the host network with FastDDS forced to UDP-only. Each container
# gets a private /dev/shm, so FastDDS shared-memory delivery silently drops every
# sample even though discovery still matches; UDPv4 is the interoperable transport
# both across the host's older ROS distro and between the containers themselves.
# Do NOT drop FASTDDS_BUILTIN_TRANSPORTS — without it nothing is received.
#
# Docker needs root on the target, so the default runner is `sudo docker`;
# override with DOCKER=docker if your user can reach the daemon directly.
# Re-running recreates the containers.
set -euo pipefail

DOCKER="${DOCKER:-sudo docker}"
DATA_DIR="${EDGESTREAM_DIR:-$HOME/edgestream-rec}"
IMAGE="${REC_IMAGE:-edgestream-rec:jazzy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
SPLIT_SECONDS="${SPLIT_SECONDS:-5}"

mkdir -p "$DATA_DIR/captured"

# Continuous recorder → <data>/recordings (wiped + recreated on each start).
$DOCKER rm -f edgestream-record >/dev/null 2>&1 || true
$DOCKER run -d --name edgestream-record --restart unless-stopped \
  --network host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -v "$DATA_DIR:/data" \
  "$IMAGE" \
  bash -c "rm -rf /data/recordings && exec ros2 bag record -a --storage mcap --max-bag-duration $SPLIT_SECONDS --output /data/recordings"

# Triggered extractor → <data>/captured, reading the splits above.
$DOCKER rm -f edgestream-rec >/dev/null 2>&1 || true
$DOCKER run -d --name edgestream-rec --restart unless-stopped \
  --network host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -v "$DATA_DIR:/data" \
  "$IMAGE" \
  edgestream-rec --record-dir /data/recordings --out-dir /data/captured

echo "recorder stack up:"
$DOCKER ps --filter name=edgestream-re --format '  {{.Names}}  {{.Status}}'
echo "recordings → $DATA_DIR/recordings   clips → $DATA_DIR/captured"
