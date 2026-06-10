"""The sim camera plus an in-process rosbag2 recorder: gscam and
``rosbag2_transport::Recorder`` composed into one ``component_container_mt``.

Same synthetic source and topics as ``sim_camera.launch.py`` (see its docstring
for the gscam/ffmpeg details), with recording added in the same process:

  /camera/image_raw             sensor_msgs/Image                 (rgb8, raw)
  /camera/camera_info           sensor_msgs/CameraInfo
  /camera/image_raw/compressed  sensor_msgs/CompressedImage       (JPEG)
  /camera/image_raw/ffmpeg      ffmpeg_image_transport_msgs/FFMPEGPacket (H.265)
        │
        ▼  FastDDS intra-process delivery (one process, one DDS participant)
  rosbag2_transport::Recorder ──▶ <out>/  (MCAP)

Why composition: publisher and recorder share one process and therefore one
FastDDS participant, so samples reach the recorder through FastDDS's
intra-process delivery instead of the SHM/UDP transports — the raw 1080p
frames never leave the process. Note this is the *DDS-level* fast path, not
rclcpp intra-process: the recorder subscribes via rclcpp::GenericSubscription,
which does not support rclcpp IPC, so ``use_intra_process_comms`` is enabled
only on gscam (where it benefits any future typed in-process subscriber). The
serialization step itself remains — a bag stores serialized CDR regardless.

The recorder is configured by sim/config/recorder_params.yaml (standard rosbag2
recorder node-parameters schema). Composable nodes take parameter dicts, not
file paths, so this launch dumps the YAML and injects ``storage.uri`` from the
``out:=`` argument. rosbag2 refuses an existing output directory — use
``sim/cam_sim.sh record``, which wipes it first.

Launch arguments: everything sim_camera.launch.py takes (width, height, fps,
pattern, ns, encoder, ...) plus ``out:=`` (bag output directory, default
./record). With ``ns:=`` the topic names move — keep the recorder topic list in
recorder_params.yaml in step.
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

_SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _bool(value):
    return value.lower() in ("1", "true", "yes", "on")


def _recorder_parameters(out_dir):
    with open(os.path.join(_SIM_ROOT, "config", "recorder_params.yaml")) as f:
        params = yaml.safe_load(f)["recorder"]["ros__parameters"]
    params.setdefault("storage", {})["uri"] = out_dir
    return [params]


def _make_container(context, *args, **kwargs):
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
    out_dir = LaunchConfiguration("out").perform(context)

    # Identical gscam configuration to sim_camera.launch.py — see the comments
    # there for the pipeline and the ffmpeg parameter-prefix gotcha.
    gscam_config = (
        f"videotestsrc is-live=true pattern={pattern} "
        f"! videorate max-rate={fps} "
        f"! video/x-raw,width={width},height={height},framerate={fps}/1 "
        f"! videoconvert"
    )
    pfx = "camera.image_raw.ffmpeg."

    gscam = ComposableNode(
        package="gscam",
        plugin="gscam::GSCam",
        name=camera_name,
        namespace=ns or None,
        parameters=[{
            "gscam_config": gscam_config,
            "image_encoding": "rgb8",
            "frame_id": frame_id,
            "camera_name": camera_name,
            "use_sensor_data_qos": sensor_qos,
            "sync_sink": True,
            pfx + "encoder": encoder,
            pfx + "encoder_av_options": av_options,
            pfx + "gop_size": gop_size,
            pfx + "bit_rate": bit_rate,
        }],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # rclcpp IPC stays off here: the recorder's GenericSubscriptions ignore it,
    # and IPC-enabled publishers reject non-volatile QoS, which would bite the
    # recorder's own event publishers.
    recorder = ComposableNode(
        package="rosbag2_transport",
        plugin="rosbag2_transport::Recorder",
        name="recorder",
        parameters=_recorder_parameters(out_dir),
    )

    return [ComposableNodeContainer(
        name="cam_sim_record_container",
        namespace=ns or "/",
        package="rclcpp_components",
        executable="component_container_mt",   # encoder + writer in parallel
        output="screen",
        composable_node_descriptions=[gscam, recorder],
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
            description="ROS namespace, e.g. /sim — the recorder topic list in "
                        "config/recorder_params.yaml must match"),
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
        DeclareLaunchArgument(
            "out", default_value="./record",
            description="bag output directory (must not pre-exist; "
                        "sim/cam_sim.sh record wipes it)"),
        OpaqueFunction(function=_make_container),
    ])
