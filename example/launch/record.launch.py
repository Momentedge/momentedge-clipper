"""Continuous `ros2 bag record` for clipper to tail, as a launch file.

The recorder half of the stack (pairs with clipper.launch.py). Records every
live topic into one growing MCAP file. `ros2 launch` inherits the environment
you already sourced to call it, so there is no per-process sourcing.

Run with a sourced ROS 2 environment:

    ros2 launch example/launch/record.launch.py
    ros2 launch example/launch/record.launch.py record_dir:=/data/record storage_preset:=fastwrite max_cache_size:=0

The recorder is not respawned: rosbag2 refuses to reuse the bag directory this
launch wipes once at startup.
"""

import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _setup(context, *args, **kwargs):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    record_dir = cfg("record_dir")

    # rosbag2 refuses to record into an existing bag directory.
    shutil.rmtree(record_dir, ignore_errors=True)

    return [
        ExecuteProcess(
            cmd=[
                "ros2", "bag", "record", "--all",
                "--storage", "mcap",
                "--storage-preset-profile", cfg("storage_preset"),
                "--max-cache-size", cfg("max_cache_size"),
                "--output", record_dir,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("record_dir", default_value="./record"),
        DeclareLaunchArgument("storage_preset", default_value="zstd_fast"),
        DeclareLaunchArgument("max_cache_size", default_value="104857600"),
        OpaqueFunction(function=_setup),
    ])
