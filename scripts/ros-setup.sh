#!/usr/bin/env bash
# Source the ROS2 distro and the local momentedge_msgs overlay into the current
# shell. Source this file; do not execute it — it sets environment for the
# caller. The overlay (install/) is added only once scripts/ros-build-messages.sh
# has built it, so a cargo build before that step sees only the base distro.
#
#   ROS_SETUP   ROS setup.bash to source  (default /opt/ros/${ROS_DISTRO:-humble})

# ROS2's setup.bash and the colcon overlay are not nounset-clean. Relax -u while
# sourcing, then restore the caller's setting — this file is sourced, so a bare
# `set -u` would leak into the caller.
case $- in *u*) _ros_u=1 ;; *) _ros_u=0 ;; esac
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
_ros_overlay="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
# shellcheck disable=SC1091
[[ -f "$_ros_overlay" ]] && source "$_ros_overlay"
if [[ $_ros_u == 1 ]]; then set -u; fi
unset _ros_u _ros_overlay
