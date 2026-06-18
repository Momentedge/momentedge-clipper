# Launch-file setup (`ros2 launch`)

The ROS-native way to bring up nodes is a **launch file** run with `ros2 launch`
— the official orchestration layer (params, namespaces, composition, restart).
[`clipper.launch.py`](clipper.launch.py) starts the continuous `ros2 bag record`
clipper tails and the `clipper` binary together.

Because `ros2 launch` inherits the environment you already sourced to call it,
the launched processes get the ROS runtime for free — no per-process sourcing.

## Run

```bash
# with a sourced ROS 2 environment (the dev shell, or /opt/ros/<distro>/setup.bash)
ros2 launch example/launch/clipper.launch.py
```

Override any default with `name:=value`:

```bash
ros2 launch example/launch/clipper.launch.py \
  record_dir:=/data/record clipped_dir:=/data/clipped \
  storage_preset:=fastwrite max_cache_size:=0 grace_secs:=2
```

| argument | default | meaning |
|----------|---------|---------|
| `record_dir` | `./record` | continuous recording directory (wiped at startup) |
| `clipped_dir` | `./clipped` | where clips are written |
| `clipper_bin` | `clipper` | clipper executable; set to `./target/release/clipper` for a dev build |
| `storage_preset` | `zstd_fast` | mcap profile (`none`/`fastwrite`/`zstd_fast`/`zstd_small`) |
| `max_cache_size` | `104857600` | rosbag2 cache bytes (`0` = write-through) |
| `grace_secs` | `30` | clipper's coverage grace (size to the storage flush latency) |

Ctrl-C stops both processes. `clipper` is launched with `respawn=True` so it
restarts on exit (it re-discovers the recording); the recorder is not respawned,
because rosbag2 refuses to reuse the bag directory the launch wiped once at
startup.

## Configuration notes

`ros2 bag record`'s settings are CLI flags only (no env vars) — they are the
launch arguments above. `clipper`'s flags additionally read `MOMENTEDGE_*`
environment variables (CLI > env > default), so launched under `ros2 launch`
they inherit anything you exported in the calling shell.

## Start on boot

`ros2 launch` is the orchestration layer, not a boot supervisor. To start it at
boot, run `ros2 launch …` under your init system; on Ubuntu targets,
[`robot_upstart`](https://github.com/clearpathrobotics/robot_upstart) generates
the systemd unit that sources the overlay and runs the launch file for you,
rather than hand-writing one.
