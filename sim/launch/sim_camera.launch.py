"""Synthetic camera source: a GStreamer fake source published as ROS2 topics,
both raw and H.265-encoded.

A single ``videotestsrc`` test pattern is fed through gscam, which publishes the
raw ``sensor_msgs/Image`` and — via its image_transport ffmpeg plugin — an H.265
``FFMPEGPacket`` on a sibling topic. No real camera or capture device is touched;
everything is generated locally.

Topics published (with the default ``ns:=''``):
  /camera/image_raw             sensor_msgs/Image                 (rgb8, raw)
  /camera/camera_info           sensor_msgs/CameraInfo
  /camera/image_raw/compressed  sensor_msgs/CompressedImage       (JPEG, debug)
  /camera/image_raw/ffmpeg      ffmpeg_image_transport_msgs/FFMPEGPacket  (H.265)

gscam advertises ``image_raw`` through an image_transport CameraPublisher, so
every installed transport plugin gets a sibling sub-topic. The nix closure
installs only the ``compressed`` (JPEG) and ``ffmpeg`` (H.265) plugins, so those
are the only siblings — theora/zstd/compressedDepth are deliberately left out
(see nix/ros-env.nix). image_transport plugins encode lazily — a sub-topic only
spends CPU while it has a subscriber — so the H.265 frames flow as soon as
something subscribes to ``…/image_raw/ffmpeg``. A matching ffmpeg_image_transport
subscriber decodes the packets back to an Image.

The ffmpeg plugin reads its encoder parameters from the gscam node under a prefix
derived from the resolved image topic (namespace stripped, ``/`` → ``.``), which
is ``camera.image_raw.ffmpeg.`` for any ``ns:=``. ``bit_rate`` must be set: the
plugin's default of -1 makes libav refuse to open the codec.

Override defaults at launch, e.g.:
  ros2 launch sim_camera.launch.py width:=1280 height:=720 pattern:=ball
  ros2 launch sim_camera.launch.py encoder:=hevc_nvenc av_options:=preset:ll,profile:main
  ros2 launch sim_camera.launch.py ns:=/sim          # publish under /sim/camera/...

Encoding defaults to fast real-time libx265 (``preset:ultrafast,tune:zerolatency``,
``gop_size=1``) so the stream stays low-latency at the cost of bitrate. Switch
``encoder:=hevc_nvenc`` for NVIDIA hardware HEVC.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _make_nodes(context, *args, **kwargs):
    width = LaunchConfiguration("width").perform(context)
    height = LaunchConfiguration("height").perform(context)
    fps = LaunchConfiguration("fps").perform(context)
    pattern = LaunchConfiguration("pattern").perform(context)
    frame_id = LaunchConfiguration("frame_id").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)
    sensor_qos = _bool(LaunchConfiguration("sensor_data_qos").perform(context))
    ns = LaunchConfiguration("ns").perform(context)
    encoder = LaunchConfiguration("encoder").perform(context)
    av_options = LaunchConfiguration("av_options").perform(context)
    gop_size = int(LaunchConfiguration("gop_size").perform(context))
    bit_rate = int(LaunchConfiguration("bit_rate").perform(context))

    # videotestsrc produces the raw test pattern; videoconvert lets gscam's
    # appsink negotiate RGB (the format gscam expects for image_encoding=rgb8).
    # Both elements ship in gst-plugins-base, already in gscam's nix closure.
    # videorate caps the rate at the source: is-live pacing alone does not gate
    # gscam's appsink, which otherwise free-runs at thousands of fps.
    gscam_config = (
        f"videotestsrc is-live=true pattern={pattern} "
        f"! videorate max-rate={fps} "
        f"! video/x-raw,width={width},height={height},framerate={fps}/1 "
        f"! videoconvert"
    )

    # The ffmpeg image_transport plugin reads encoder params from this gscam node
    # under a prefix built from the resolved image topic (namespace stripped,
    # '/' -> '.'), which is "camera.image_raw.ffmpeg." for any ns:=. bit_rate
    # MUST be set — the plugin default of -1 makes libav reject the codec.
    pfx = "camera.image_raw.ffmpeg."

    return [Node(
        package="gscam",
        executable="gscam_node",
        name=camera_name,
        namespace=ns or None,
        output="screen",
        parameters=[{
            "gscam_config": gscam_config,
            "image_encoding": "rgb8",          # raw sensor_msgs/Image
            "frame_id": frame_id,
            "camera_name": camera_name,
            "use_sensor_data_qos": sensor_qos,
            "sync_sink": True,                 # sync appsink to the pipeline clock
            # H.265 leg: configure gscam's own image_transport ffmpeg plugin.
            pfx + "encoder": encoder,
            pfx + "encoder_av_options": av_options,
            pfx + "gop_size": gop_size,
            pfx + "bit_rate": bit_rate,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="1920"),
        DeclareLaunchArgument("height", default_value="1080"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument(
            "pattern", default_value="smpte",
            description="videotestsrc pattern: smpte, ball, snow, gradient, ..."),
        DeclareLaunchArgument("frame_id", default_value="camera_optical_frame"),
        DeclareLaunchArgument("camera_name", default_value="sim_cam"),
        DeclareLaunchArgument(
            "sensor_data_qos", default_value="false",
            description="false: reliable QoS (echo works without flags); "
                        "true: best-effort SensorData QoS"),
        DeclareLaunchArgument(
            "ns", default_value="",
            description="ROS namespace, e.g. /sim, to run alongside other sources"),
        DeclareLaunchArgument(
            "encoder", default_value="libx265",
            description="ffmpeg/libav encoder: libx265 (CPU) or hevc_nvenc (GPU)"),
        DeclareLaunchArgument(
            "av_options", default_value="preset:ultrafast,tune:zerolatency",
            description="comma-separated libav key:value options for the encoder"),
        DeclareLaunchArgument(
            "gop_size", default_value="1",
            description="keyframe interval; 1 = lowest latency, higher = lower bitrate"),
        DeclareLaunchArgument("bit_rate", default_value="4000000"),
        OpaqueFunction(function=_make_nodes),
    ])
