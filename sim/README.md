# sim — synthetic camera

Publish a synthetic GStreamer source as ROS 2 topics — **raw and H.265-encoded**
at the same time — with no camera or capture hardware. It is the in-repo data
source for this repository's recorders, and the camera-free local sibling of
[`../../ros2_cam_orin_nx`](../../ros2_cam_orin_nx) (which forwards a real
Jetson USB camera): same gscam-based design, but the source is a `videotestsrc`
test pattern and everything runs on the local workstation.

The ROS 2 stack (gscam, image_transport, ffmpeg_image_transport) and GStreamer
come from the repository's flake dev shell — see [Environment](#environment);
nothing is installed system-wide.

## Quick start

```bash
sim/cam_sim.sh          # publish raw + H.265 at 1080p30 (from the repo root)
```

`cam_sim.sh` expects the ROS2 env on `PATH` — the dev shell provides it
(direnv loads it on cd). From a bare shell, wrap the call:
`nix develop --command sim/cam_sim.sh`. Subcommands:

| Command | Purpose |
|---|---|
| `sim/cam_sim.sh run [args]` | publish raw + H.265 (the default; Ctrl-C stops cleanly) |
| `sim/cam_sim.sh record [args]` | publish **and record in-process** (gscam + rosbag2 recorder composed) |
| `sim/cam_sim.sh stop` | force-stop an orphaned `gscam_node` / `component_container_mt` |

## Topics

With the default namespace (`ns:=''`), `cam_sim.sh` publishes:

| Topic | Type | Notes |
|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `rgb8`, raw test pattern |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG, handy for `rqt_image_view` |
| `/camera/image_raw/ffmpeg` | `ffmpeg_image_transport_msgs/FFMPEGPacket` | H.265 (HEVC) |

The `compressed` and `ffmpeg` siblings come from the two image_transport plugins
in the nix closure; theora/zstd/compressedDepth are deliberately left out (see
[`../nix/ros-env.nix`](../nix/ros-env.nix)). image_transport encodes lazily, so a
sibling only spends CPU while something is subscribed to it.

Inspect them from inside the dev shell:

```bash
ros2 topic hz   /camera/image_raw
ros2 topic echo /camera/image_raw --no-arr
ros2 topic hz   /camera/image_raw/ffmpeg
```

A subscriber that wants the decoded H.265 stream uses the
`ffmpeg_image_transport` subscriber plugin (it decodes `FFMPEGPacket` back to a
`sensor_msgs/Image`), e.g.:

```bash
ros2 run image_transport republish ffmpeg raw \
  --ros-args -r in:=/camera/image_raw -r out:=/camera/image_decoded
```

## How the two streams are produced

```
videotestsrc ──► gscam_node ──► image_transport CameraPublisher
                                      ├──► /camera/image_raw           (sensor_msgs/Image, rgb8)
                                      ├──► /camera/image_raw/compressed (CompressedImage, JPEG)
                                      └──► /camera/image_raw/ffmpeg     (FFMPEGPacket, H.265)
```

A single `gscam_node` owns both legs — there is no separate `republish` node.

- **Raw leg** — gscam runs the GStreamer pipeline
  `videotestsrc ! video/x-raw ! videoconvert` and publishes the frames as a raw
  `sensor_msgs/Image`. The GStreamer pipeline only ever carries raw video; it has
  no path to publish a pipeline-side H.265 encode as a topic.
- **H.265 leg** — gscam advertises `image_raw` through an
  `image_transport::CameraPublisher`, which hosts every installed transport
  plugin. The `ffmpeg_image_transport` plugin libav-encodes each raw frame
  (libx265 by default) and publishes `ffmpeg_image_transport_msgs/FFMPEGPacket`
  on `…/image_raw/ffmpeg`. The encode runs through libav, not a GStreamer
  `x265enc`, so it needs no extra GStreamer plugins. The encoder parameters are
  set on the gscam node under the `camera.image_raw.ffmpeg.` prefix.

The encoder defaults to **fast real-time** libx265:
`preset:ultrafast, tune:zerolatency, gop_size=1` — minimum latency at the cost of
bitrate. See the launch arguments below to retune or switch to NVIDIA hardware
HEVC (`hevc_nvenc`).

## Launch arguments

`cam_sim.sh` forwards every `name:=value` argument to the launch file
(`run` and `record` alike):

| Argument | Default | Meaning |
|---|---|---|
| `width` / `height` | `1920` / `1080` | frame size |
| `fps` | `30` | frame rate |
| `pattern` | `smpte` | `videotestsrc` pattern (`ball`, `snow`, `gradient`, …) |
| `frame_id` | `camera_optical_frame` | TF frame stamped on each image |
| `camera_name` | `sim_cam` | gscam node + camera name |
| `sensor_data_qos` | `false` | `true` = best-effort SensorData QoS (subscribers must match) |
| `ns` | `''` | ROS namespace, e.g. `/sim`, to run alongside other sources |
| `encoder` | `libx265` | libav encoder: `libx265` (CPU) or `hevc_nvenc` (GPU) |
| `av_options` | `preset:ultrafast,tune:zerolatency` | comma-separated libav `key:value` options |
| `gop_size` | `1` | keyframe interval; `1` = lowest latency, higher = lower bitrate |
| `bit_rate` | `4000000` | target bitrate (bits/s) |
| `out` | `./record` | (`record` only) bag output directory |

Examples:

```bash
sim/cam_sim.sh width:=1280 height:=720 fps:=60 pattern:=ball
sim/cam_sim.sh encoder:=hevc_nvenc av_options:=preset:ll,profile:main
sim/cam_sim.sh ns:=/sim                 # /sim/camera/image_raw, /sim/camera/image_raw/ffmpeg
```

The H.265 leg encodes lazily, so "raw only" needs no flag — nothing subscribes to
`…/image_raw/ffmpeg`, nothing encodes.

## Recording in-process

`sim/cam_sim.sh record` runs the sim camera and a rosbag2 recorder **in one
process**: [`launch/sim_camera_record.launch.py`](launch/sim_camera_record.launch.py)
composes `gscam::GSCam` and
[`rosbag2_transport::Recorder`](https://github.com/ros2/rosbag2/blob/jazzy/README.md#composition)
into a single `component_container_mt`. Publisher and recorder then share one
FastDDS participant, so samples reach the recorder through FastDDS
intra-process delivery — the raw 1080p frames never cross the SHM/UDP
transports on their way into the bag.

This is DDS-level delivery, not rclcpp intra-process communication: the
recorder subscribes through `rclcpp::GenericSubscription`, which rclcpp's
`IntraProcessManager` does not support
([ros2/rosbag2#2267](https://github.com/ros2/rosbag2/issues/2267)). Upstream
support is in progress in
[ros2/rclcpp#3083](https://github.com/ros2/rclcpp/pull/3083); once it lands in
the distro used here, enabling `use_intra_process_comms` on the recorder node
picks it up without structural changes to this pipeline.

```bash
sim/cam_sim.sh record                      # 1080p30 smpte → ./record (MCAP)
sim/cam_sim.sh record out:=/tmp/simbag     # bag output directory
sim/cam_sim.sh record width:=1280 fps:=60  # any launch argument above
ros2 bag info record                       # inspect the result
```

The recorder is configured by
[`config/recorder_params.yaml`](config/recorder_params.yaml) (the standard
rosbag2 recorder node-parameters schema): it records the four `/camera/...`
topics above into MCAP. The output directory is wiped on each start (rosbag2
refuses an existing bag dir) and `./record` is gitignored. Mind the size: the
raw `rgb8` stream alone is ~190 MB/s at 1080p30.

To record the same topics **out of process** instead — e.g. as the continuous
recording the triggered-recording workflow cuts clips from — use the repo's
standalone recorder from the root: `./scripts/record.sh config/cam_sim.yaml`
(that is the root `config/`, not this directory's). See the root
[README](../README.md#triggered-recording).

## Environment

There is no flake here — the repository root flake provides everything. Its dev
shell carries the gscam / image_transport / rosbag2 stack (declared env-only in
[`../nix/ros-env.nix`](../nix/ros-env.nix)) plus the GStreamer plugin set, and
exports `GST_PLUGIN_SYSTEM_PATH_1_0` so the pipeline gscam spawns finds those
plugins. `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and `ROS_DOMAIN_ID=0` are the
same exports the recorders use, so everything meets on one ROS graph. See the
root [README](../README.md) for the dev-shell quickstart.
