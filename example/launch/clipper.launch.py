"""Bring up the continuous recorder and clipper together with `ros2 launch`.

The ROS-native way to start nodes is a launch file run with `ros2 launch`; it
inherits the environment you already sourced to call it, so there is no env
wrangling. This starts the continuous `ros2 bag record` clipper tails and the
clipper binary, both as plain processes — clipper installs as `clipper` on PATH
(the momentedge-clipper package), not as an ament executable, and the recorder
is the ros2 CLI, so neither is a `launch_ros` Node.

Run with a sourced ROS 2 environment:

    ros2 launch example/launch/clipper.launch.py
    ros2 launch example/launch/clipper.launch.py record_dir:=/data/record clipped_dir:=/data/clipped
    ros2 launch example/launch/clipper.launch.py clipper_bin:=./target/release/clipper   # dev build

Ctrl-C stops both. clipper respawns on exit (it re-discovers the recording);
the recorder does not, because rosbag2 refuses to reuse the bag directory the
launch wiped once at startup.
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

    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record", "--all",
            "--storage", "mcap",
            "--storage-preset-profile", cfg("storage_preset"),
            "--max-cache-size", cfg("max_cache_size"),
            "--output", record_dir,
        ],
        output="screen",
    )
    clipper = ExecuteProcess(
        cmd=[
            cfg("clipper_bin"),
            "--record-dir", record_dir,
            "--out-dir", cfg("clipped_dir"),
            "--grace-secs", cfg("grace_secs"),
        ],
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    return [recorder, clipper]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("record_dir", default_value="./record"),
        DeclareLaunchArgument("clipped_dir", default_value="./clipped"),
        DeclareLaunchArgument("clipper_bin", default_value="clipper"),
        DeclareLaunchArgument("storage_preset", default_value="zstd_fast"),
        DeclareLaunchArgument("max_cache_size", default_value="104857600"),
        DeclareLaunchArgument("grace_secs", default_value="30"),
        OpaqueFunction(function=_setup),
    ])
