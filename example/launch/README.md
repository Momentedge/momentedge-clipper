# Launch-file setup (`ros2 launch`)

The ROS-native way to bring up nodes is a **launch file** run with `ros2 launch`
— the official orchestration layer (params, namespaces, composition, restart).
This mirrors the `scripts/record.sh` + `scripts/run.sh` pair as two launch files:

- [`record.launch.py`](record.launch.py) — the continuous `ros2 bag record`
  clipper tails.
- [`clipper.launch.py`](clipper.launch.py) — the `clipper` binary that cuts
  clips from it.

`ros2 launch` inherits the environment you already sourced to call it, so the
launched processes get the ROS runtime for free — no per-process sourcing.

## Run

Each in its own shell, both with a sourced ROS 2 environment (the dev shell, or
`/opt/ros/<distro>/setup.bash`):

```bash
# shell 1 — recorder
ros2 launch example/launch/record.launch.py

# shell 2 — clipper
ros2 launch example/launch/clipper.launch.py
```

Override any default with `name:=value`, e.g. a low-latency recording:

```bash
ros2 launch example/launch/record.launch.py  storage_preset:=fastwrite max_cache_size:=0
ros2 launch example/launch/clipper.launch.py grace_secs:=2
```

To bring both up from one terminal, include them from a parent launch file:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    here = "example/launch"
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(f"{here}/record.launch.py")),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(f"{here}/clipper.launch.py")),
    ])
```

## Arguments

`record.launch.py`:

| argument | default | meaning |
|----------|---------|---------|
| `record_dir` | `./record` | continuous recording directory (wiped at startup) |
| `storage_preset` | `zstd_fast` | mcap profile (`none`/`fastwrite`/`zstd_fast`/`zstd_small`) |
| `max_cache_size` | `104857600` | rosbag2 cache bytes (`0` = write-through) |

`clipper.launch.py`:

| argument | default | meaning |
|----------|---------|---------|
| `record_dir` | `./record` | recording to tail (match `record.launch.py`) |
| `clipped_dir` | `./clipped` | where clips are written |
| `clipper_bin` | `clipper` | clipper executable; set to `./target/release/clipper` for a dev build |
| `grace_secs` | `30` | clipper's coverage grace (size to the storage flush latency) |

Ctrl-C stops a launch. `clipper` runs with `respawn=True` so it restarts on exit
(it re-discovers the recording); the recorder is not respawned, because rosbag2
refuses to reuse the bag directory the launch wiped once at startup.

## Configuration notes

`ros2 bag record`'s settings are CLI flags only (no env vars) — they are the
`record.launch.py` arguments above. `clipper`'s flags additionally read
`MOMENTEDGE_*` environment variables (CLI > env > default), so under
`ros2 launch` they inherit anything you exported in the calling shell.

## Start on boot

`ros2 launch` is the orchestration layer, not a boot supervisor. To start it at
boot, run `ros2 launch …` under your init system; on Ubuntu targets,
[`robot_upstart`](https://github.com/clearpathrobotics/robot_upstart) generates
the systemd unit that sources the overlay and runs the launch file for you,
rather than hand-writing one.
