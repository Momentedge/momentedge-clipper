#!/usr/bin/env bash
# Build the local momentedge_msgs interface package into a colcon overlay under
# install/. Its Trigger/Recorded typesupport is not in apt, so it is generated
# here; the overlay feeds both r2r's codegen (scripts/ros-cargo.sh) and the
# binaries at runtime. Needs the base ROS2 distro sourced — scripts/ros-setup.sh
# does that and honours ROS_SETUP / ROS_DISTRO.
set -euo pipefail
_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/ros-setup.sh
source "$_here/ros-setup.sh"
: "${ROS_DISTRO:?ROS not sourced — set ROS_DISTRO or ROS_SETUP}"
cd "$_here/.."

echo "building momentedge_msgs against ROS2 $ROS_DISTRO"
colcon build --packages-select momentedge_msgs
