# systemd service stack

The recorder, clipper, and a pruner as three units on a target: a continuous
`ros2 bag record` writing one growing file, clipper tailing it and cutting
clips, and a timer pruning old clips.

```
rosbag.service ──▶ /var/lib/clipper/record/<bag>_0.mcap ──tailed──▶ clipper.service ──▶ /var/lib/clipper/clipped/
                                                                    prune-recordings.timer ──▶ deletes old clips
```

Deployment runs natively (no container), so all ROS2 processes share the host
`/dev/shm` and FastDDS shared-memory transport works between them and the host's
other nodes — see the repo README's deployment section.

## Does rosbag2 accept configuration from the environment?

**No — `ros2 bag record`'s options are CLI flags only.** There is no env var for
`--max-cache-size`, `--storage-preset-profile`, `--max-bag-duration`, topic
selection, and so on; pass them on the `ExecStart` command line. rosbag2 *does*
honour the standard ROS environment — `ROS_DISTRO`, `RMW_IMPLEMENTATION`,
`ROS_DOMAIN_ID`, `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`, … — but those are set by
sourcing the ROS setup, not by hand. Setting `Environment=ROS_DISTRO=…` alone
does **not** put `ros2` on `PATH` or its libraries on the loader path, so each
`ExecStart` sources `setup.bash` first.

**clipper is the opposite:** every flag has a `MOMENTEDGE_*` env fallback (CLI >
env > default), so clipper is configured entirely through `Environment=` lines
and needs no arguments. It still needs the ROS runtime sourced to resolve its
`momentedge_msgs` typesupport (the `ros-<distro>-momentedge-msgs` package
installs into `/opt/ros/<distro>`).

## Units

Install under `/etc/systemd/system/`. They assume a `clipper` user owning
`/var/lib/clipper`, the `momentedge-clipper` and `ros-<distro>-momentedge-msgs`
packages installed, and `<distro>` substituted (e.g. `humble`).

### `rosbag.service`

```ini
[Unit]
Description=Continuous ros2 bag recording (clipper source)
After=network-online.target
Wants=network-online.target

[Service]
Type=exec
User=clipper
# ros2 record options are CLI flags, not env vars. ROS_DISTRO is set here (and
# re-set by setup.bash); the source brings in PATH / AMENT_PREFIX_PATH / the
# loader path that ROS_DISTRO alone does not.
Environment=ROS_DISTRO=humble
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_DOMAIN_ID=0
# rosbag2 refuses to record into an existing bag directory.
ExecStartPre=/bin/rm -rf /var/lib/clipper/record
ExecStart=/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && \
  exec ros2 bag record --all \
    --storage mcap \
    --storage-preset-profile zstd_fast \
    --max-cache-size 104857600 \
    --output /var/lib/clipper/record'
# ros2 bag record finalises the bag cleanly on SIGINT.
KillSignal=SIGINT
TimeoutStopSec=30
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

### `momentedge-clipper.service`

```ini
[Unit]
Description=clipper triggered MCAP clip recorder
After=rosbag.service
Wants=rosbag.service

[Service]
Type=exec
User=clipper
Environment=ROS_DISTRO=humble
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_DOMAIN_ID=0
# clipper's whole configuration — flag > env > default.
Environment=MOMENTEDGE_RECORD_DIR=/var/lib/clipper/record
Environment=MOMENTEDGE_OUT_DIR=/var/lib/clipper/clipped
Environment=MOMENTEDGE_GRACE_SECS=30
# clipper resolves momentedge_msgs typesupport from the sourced ROS env.
ExecStart=/bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash && exec clipper'
# clipper exits 0 on SIGINT/SIGTERM (systemd's default stop signal is fine).
TimeoutStopSec=30
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

### `prune-recordings.service` + `.timer`

clipper writes one clip per trigger into `clipped/`; those accumulate. A timer
deletes clips older than 24 h. (The continuous recording is a single growing
file — bound it by switching to [split bags](../split-bags/README.md), or track
in-place retention in beads `clipper-wkg`.)

```ini
# prune-recordings.service
[Unit]
Description=Delete clipper clips older than 24h

[Service]
Type=oneshot
User=clipper
ExecStart=/usr/bin/find /var/lib/clipper/clipped -mindepth 1 -type f -mmin +1440 -delete
```

```ini
# prune-recordings.timer
[Unit]
Description=Hourly prune of clipper clips

[Timer]
OnBootSec=10min
OnUnitActiveSec=1h
Persistent=true

[Install]
WantedBy=timers.target
```

## Enable

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now rosbag.service momentedge-clipper.service prune-recordings.timer
journalctl -u momentedge-clipper.service -f
```
