# sim/ — contributor notes

The synthetic gscam camera. [README.md](README.md) covers the topics,
subcommands, launch arguments, and recording; the repo-root
[CLAUDE.md](../CLAUDE.md) covers the shared flake/env mechanics. This file is
only the gotchas that bite when changing the sim.

- **gscam's GStreamer pipeline carries raw video only.** The appsink negotiates
  `video/x-raw`; an `x265enc` in `gscam_config` goes nowhere ROS can see. The
  H.265 leg is the `ffmpeg_image_transport` plugin hosted on gscam's own
  `image_transport::CameraPublisher` — no separate `republish` node.
- **The ffmpeg plugin's parameter prefix is the resolved image topic**,
  namespace stripped, `/`→`.`, suffixed `.ffmpeg.` — i.e.
  `camera.image_raw.ffmpeg.`, not `out.ffmpeg.`. A wrong prefix is silently
  ignored (encoder stays the libx264 default). If the topic names change,
  recheck with `ros2 param list` on the gscam node (`sim_cam`).
- **The in-process recorder's fast path is FastDDS-level, not rclcpp IPC.**
  The Recorder subscribes via `rclcpp::GenericSubscription`, which rclcpp's
  `IntraProcessManager` does not support
  ([ros2/rosbag2#2267](https://github.com/ros2/rosbag2/issues/2267); fix in
  flight: [ros2/rclcpp#3083](https://github.com/ros2/rclcpp/pull/3083)). So
  `use_intra_process_comms` is enabled on gscam only — IPC-enabled publishers
  also reject the recorder's non-volatile event QoS.
- **Composable nodes take parameter dicts, not file paths.** The record launch
  loads `config/recorder_params.yaml` (standard rosbag2 recorder
  node-parameters schema) and injects `storage.uri` from `out:=`. The topic
  list names the default-`ns` topics — `ns:=` moves them, so the YAML must
  follow.
- Stop a run with Ctrl-C (clean). For orphans, `cam_sim.sh stop` matches
  `gscam_node` and `component_container_mt` by process name (`-x`, never
  `pkill -f`); the latter matches ANY mt container on the machine.
