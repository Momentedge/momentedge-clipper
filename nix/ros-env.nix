# The ROS2 closure r2r builds and links against, shared by the dev shell and the
# nix-built binaries/images. r2r generates Rust bindings (at build time) for
# every message package on AMENT_PREFIX_PATH; the IDL_PACKAGE_FILTER in flake.nix
# restricts codegen to exactly this set, since r2r does no dependency resolution
# and must be handed every used package explicitly.
{ ros, edgestream-msgs }:

ros.buildEnv {
  paths = with ros; [
    # core client library + Fast DDS RMW that r2r builds against
    rcl
    rcl-action
    rmw-fastrtps-cpp
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
}
