#!/usr/bin/env bash
# Launch the demo trigger publisher (trigger-pub) as a Docker container on the
# target, built from the nix image (see nix/images.nix). trigger-pub is a
# development stand-in: it publishes an edgestream_msgs/Trigger on
# /events/edgestream/trigger ~1/s with a random preroll/postroll window, which
# edgestream-rec turns into clips. In production a real trigger source replaces
# this — the recorder stack (scripts/start_recorder.sh) does not depend on it.
#
# Same DDS setup as the recorder: host network, FastDDS UDP-only. A separate
# container means a separate /dev/shm, so SHM delivery to edgestream-rec would be
# silently dropped — UDPv4 is what carries the trigger across. See
# scripts/start_recorder.sh for the full rationale.
#
# Extra arguments are forwarded to trigger-pub, e.g.
#   ./scripts/start_demo_trigger_pub.sh --preroll 2 --postroll 3
#
# Docker needs root on the target, so the default runner is `sudo docker`;
# override with DOCKER=docker if your user can reach the daemon directly.
set -euo pipefail

DOCKER="${DOCKER:-sudo docker}"
IMAGE="${TRIG_IMAGE:-trigger-pub:jazzy}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

$DOCKER rm -f edgestream-trigger >/dev/null 2>&1 || true
$DOCKER run -d --name edgestream-trigger --restart unless-stopped \
  --network host \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  "$IMAGE" \
  trigger-pub "$@"

echo "trigger publisher up:"
$DOCKER ps --filter name=edgestream-trigger --format '  {{.Names}}  {{.Status}}'
