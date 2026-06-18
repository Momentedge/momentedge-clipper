"""clipper, as a launch file (pairs with record.launch.py).

The clipper half of the stack: tails the recording record.launch.py writes and
cuts a clip per trigger. `ros2 launch` inherits the environment you already
sourced to call it, so clipper resolves its momentedge_msgs typesupport for free.

clipper installs as `clipper` on PATH (the momentedge-clipper package), not as an
ament executable, so it is an ExecuteProcess rather than a launch_ros Node.

Run with a sourced ROS 2 environment:

    ros2 launch example/launch/clipper.launch.py
    ros2 launch example/launch/clipper.launch.py record_dir:=/data/record clipped_dir:=/data/clipped grace_secs:=2
    ros2 launch example/launch/clipper.launch.py clipper_bin:=./target/release/clipper   # dev build

clipper is launched with respawn=True so it restarts on exit (it re-discovers
the recording).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("record_dir", default_value="./record"),
        DeclareLaunchArgument("clipped_dir", default_value="./clipped"),
        DeclareLaunchArgument("clipper_bin", default_value="clipper"),
        DeclareLaunchArgument("grace_secs", default_value="30"),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("clipper_bin"),
                "--record-dir", LaunchConfiguration("record_dir"),
                "--out-dir", LaunchConfiguration("clipped_dir"),
                "--grace-secs", LaunchConfiguration("grace_secs"),
            ],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
