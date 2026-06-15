# The ROS2 closure for the whole repo, in two halves:
#
#   corePaths  what r2r builds and links against (dev shell + nix-built
#              binaries) and what the live e2e suite drives — the recorder, the
#              ros2 CLI, rosbag2 with the mcap backend, and the message packages
#              the bag carries. Builds under every distro this repo targets.
#   simPaths   the synthetic gscam camera stack (sim/) — gscam, the
#              image_transport plugins, the ffmpeg H.265 leg, and the component
#              container. Pulled in only where `withSim` is set, because the
#              overlay packages these for Jazzy but not for Humble, Lyrical, or
#              Rolling (gscam and ffmpeg_image_transport fail to configure
#              there). The recorder, the e2e suite, and deployment use none of
#              it, so gating it off elsewhere costs the bench nothing.
#
# r2r generates Rust bindings (at build time) for every message package on
# AMENT_PREFIX_PATH; the IDL_PACKAGE_FILTER in flake.nix restricts codegen to
# exactly the recorder's set, since r2r does no dependency resolution and must
# be handed every used package explicitly. Everything in simPaths is env-only —
# present for the sim pipeline, invisible to the Rust builds.
{ ros, edgestream-msgs, lib, withSim ? true }:

let
  corePaths = with ros; [
    # core client library + Fast DDS RMW that r2r builds against
    rcl
    rcl-action
    rmw-fastrtps-cpp
    # rclcpp/rclpy, launch + launch_ros and the `ros2 launch`/`ros2 run` verbs.
    ros-core
    # CLI, handy for `ros2 topic list` from the same shell
    ros2cli
    ros2cli-common-extensions
    # `ros2 bag record` with the mcap storage backend — the continuous
    # circular recorder the triggered extractor reads from (see README).
    ros2bag
    rosbag2-transport
    rosbag2-storage-mcap
    # WriteSplitEvent on /events/write_split, published by rosbag2 each
    # time it finalises a split; the extractor waits on it.
    rosbag2-interfaces
    # message packages carried by the UGV bag (see ../ros2_sources/REPLAY.md)
    builtin-interfaces
    std-msgs
    sensor-msgs
    geometry-msgs
    nav-msgs
    tf2-msgs
    velodyne-msgs
    rosgraph-msgs       # /clock when replaying with --clock

    # pulled in by rcl/actions; r2r's codegen needs them present
    action-msgs
    unique-identifier-msgs
    std-srvs
    # rclrs's vendored interfaces (rclrs/src/vendor) unconditionally link
    # their typesupport C libs, so these must be on the link path even
    # though no bag topic uses them — see ros2-rust/ros2_rust#557. r2r
    # ignores them.
    example-interfaces
    test-msgs
    # local edgestream Trigger/Recorded interfaces
    edgestream-msgs
  ];

  # ---- the sim camera (sim/) ----
  simPaths = with ros; [
    # GStreamer -> ROS2 bridge: feeds videotestsrc through an appsink and
    # publishes sensor_msgs/Image (raw) + sensor_msgs/CameraInfo.
    gscam
    # image_transport core + only the transport plugins gscam's CameraPublisher
    # should host: compressed (JPEG, handy for rqt_image_view debugging) and
    # ffmpeg (the H.265 leg). The image-transport-plugins metapackage is
    # deliberately NOT pulled in — it also drags compressedDepth (errors on
    # rgb8), theora, and zstd, which advertise unwanted sibling topics and add
    # noise to record-all bags.
    image-transport
    compressed-image-transport
    # The H.265 leg: ffmpeg_image_transport publishes FFMPEGPacket and does the
    # libav (libx265 / hevc_nvenc) encode. *-msgs is the wire type — it also
    # gives `ros2 bag record` the type support to capture the topic
    # (config/cam_sim.yaml). Env-only: not in IDL_PACKAGE_FILTER, no Rust
    # crate decodes it.
    ffmpeg-image-transport
    ffmpeg-image-transport-msgs
    # rosbag2_transport::Recorder composed next to gscam inside one
    # component_container_mt (sim/launch/sim_camera_record.launch.py).
    rclcpp-components
  ];
in
ros.buildEnv {
  paths = corePaths ++ lib.optionals withSim simPaths;
}
