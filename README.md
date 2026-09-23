# Momentedge Clipper

> Event-triggered clips from a continuous ROS 2 recording — including the
> seconds *before* the event.

[![CI](https://github.com/Momentedge/momentedge-clipper/actions/workflows/ci.yml/badge.svg)](https://github.com/Momentedge/momentedge-clipper/actions/workflows/ci.yml)
[![Release](https://img.shields.io/github/v/release/Momentedge/momentedge-clipper?sort=semver)](https://github.com/Momentedge/momentedge-clipper/releases)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
![ROS 2: Humble · Jazzy · Lyrical](https://img.shields.io/badge/ROS%202-humble%20%7C%20jazzy%20%7C%20lyrical-22314E)

On a robot the data worth keeping is sparse: most of the time nothing
interesting happens. But you can't know an event mattered until after it has
already started — and a useful clip needs the lead-up, not just the aftermath.
That **preroll** only exists if the data was already on disk when the event
fired.

**Momentedge Clipper** turns an ordinary `ros2 bag record` into an on-demand
event recorder. It runs alongside the recorder, tails the growing MCAP file, and
on each trigger cuts a standalone clip covering a window around the event.
Recording stays rosbag2's job; clipping is clipper's. The two never talk except
through the file on disk.

**clipper clips; it does not ship.** It is a clip engine and nothing else: it
turns "this moment mattered" into a standalone clip on disk, and stops. What
counts as a moment is your robots, your models, your thresholds — clipper takes
a trigger on a topic and asks nothing about where it came from. Getting clips
off the vehicle is your sync tool, your pipeline, your cloud; what clipper hands
it is an output directory holding nothing but finished clips, each complete the
moment one file appears in it. So clipper drops into the stack you already run
instead of replacing part of it — it opens no sensor subscription and holds the
recording open read-only, so taking it away leaves that recording byte-for-byte
as it was.

- **Nearly free to leave running.** 0.45 % of one core and 22 MiB on a Jetson
  Orin Nano, no disk reads while tailing, and the recorder it sits beside does
  not notice it is there. [The numbers →](docs/performance.md)
- **MCAP in, MCAP out.** Clips are standard, complete MCAP files —
  [Foxglove](https://foxglove.dev/), the `mcap` CLI and `ros2 bag` replay read
  them. No vendor format on either side.
- **Decode-free.** clipper copies message bytes straight through and never
  deserializes a message body, so it is agnostic to your message types.
- **Triggers are just a topic, and ROS is optional.** Anything publishing
  `momentedge_msgs/Trigger` — a fault detector, a watchdog, an operator button —
  drives it; the default build links no ROS, reads triggers out of the recording
  instead, and cuts identical clips.
- **Every clip says what it is.** A clip is a directory whose
  `clip_metadata.yaml` names the trigger, the window and the source bytes, so
  even an empty clip can say why.

## How it works

clipper is a standalone application that sits beside a continuous
`ros2 bag record`:

```
  trigger source                              ros2 bag record  (continuous, --all)
  (fault / button / perception)                       │
        │  momentedge_msgs/Trigger                     ▼
        │  on /events/momentedge/trigger        ./record/<bag>.mcap   (one growing file)
        ▼                                              │
     clipper ◀──────────────── tails (keeps the file open) ──────────┘
        │
        ├── copies [anchor − preroll, anchor + postroll] ──▶ ./clipped/<anchor_ns>_<hash>/
        │                                                       <id>_0.mcap …
        │                                                       clip_metadata.yaml
        │
        └── announces ──▶ /events/momentedge/recorded   (momentedge_msgs/Recorded, naming the directory)
```

1. **Tail.** It keeps the growing MCAP file open and incrementally scans the new
   bytes, decoding nothing but each message's timestamp — so a clip can be cut
   the moment its data is physically on disk.
2. **Listen.** It waits for a `momentedge_msgs/Trigger` carrying a name and a
   pre/post window.
3. **Copy.** It copies every message inside the window into a clip directory,
   writes the document that marks it complete, and announces it. The preroll is
   already on disk, so it is simply there to copy.

## Quickstart

**No ROS 2 to hand?** [Cut a clip on your laptop in two
minutes](docs/try-it.md) — a Rust toolchain, and nothing else.

Three things, each in its own shell sharing one ROS 2 environment
(`RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` must match):

```bash
# 1. Continuous recording → ./record (one growing MCAP file)
ros2 bag record --all --storage mcap --output ./record

# 2. clipper, tailing ./record, writing clips to ./clipped
clipper tail --record-dir ./record --out-dir ./clipped --clip-compression zstd

# 3. Fire a trigger: 5 s before and 5 s after the instant clipper receives it
ros2 topic pub --once /events/momentedge/trigger momentedge_msgs/msg/Trigger \
  "{name: clip1, trigger_time: {sec: 0, nanosec: 0}, preroll: 5000000000, postroll: 5000000000}"
```

A clip directory lands in `./clipped`, named by its **clip id**: the window's
anchor plus a digest of the trigger, so two detectors firing on one instant never
collide and no trigger text reaches a path.

```console
$ ls ./clipped/1738000000000000000_35a7-60a7-01fc-8561/
1738000000000000000_35a7-60a7-01fc-8561_0.mcap  clip_metadata.yaml
```

One ordinary, standalone MCAP per source recording the window crossed — open it
in Foxglove, replay it with `ros2 bag play` — plus the document that says what
they are. **`clip_metadata.yaml` is written last, and its presence is what
"complete" means**: a directory without it is a cut still running, or the residue
of one that was killed. Every field of the document is
[What a clip carries](docs/clip-manifest.md).

`trigger_time: 0` anchors the window on the instant clipper receives the trigger;
to choose the instant yourself, see [Triggers and
time](docs/triggers-and-time.md). To keep triggers firing while you develop, run
the bundled [`trigger-pub`](examples/trigger-pub/README.md) instead of step 3.

## Install

Each [GitHub release](https://github.com/Momentedge/momentedge-clipper/releases)
attaches two arm64 Debian packages, for **Humble** (Ubuntu 22.04) and **Jazzy**
(Ubuntu 24.04) — the deployment target is a Jetson-class board:

```bash
sudo apt install ./ros-humble-momentedge-msgs_*.deb ./momentedge-clipper_*.deb
source /opt/ros/humble/setup.bash
/opt/momentedge-clipper/bin/clipper --help
```

From source, ROS is a cargo feature: `cargo build -p clipper` links none, and
`--features ros` is what every `.deb` is built with. Both builds in full, the
supported distros, the nix package, a target build and the upgrade note:
[Installing and building](docs/install.md).

## Two modes

clipper is one binary and the mode is a subcommand: **`clipper tail`** is the
recorder above, and **`clipper clip`** cuts windows out of one *finished*
recording and exits — same window plan, same copy, same clip directory. That is
how a bag pulled off a vehicle becomes clips afterwards, with no ROS installed
anywhere:

```bash
clipper clip ./record --out-dir ./clipped --trigger-source mcap
```

## What clipper does not do

- **It does not record.** clipper opens no sensor subscription. `ros2 bag record`
  — or any append-only MCAP writer — has to be running beside it, and nothing on
  disk means nothing to cut.
- **It does not back-index a backlog.** At startup clipper adopts the newest
  recording in `--record-dir` and indexes it whole; every *older* split beside it
  is skipped and stays skipped. Cut those windows with `clipper clip` instead —
  [what a running recorder does](docs/operating.md#in-normal-operation).
- **It does not manage retention.** The recording grows until you stop or split
  it, and clipper prunes neither it nor `--out-dir` — it will unlink *expired,
  already-rolled-over* recordings if you ask (`--delete-old-files`), and nothing
  else.
- **It never overwrites or repairs a clip.** A clip directory already there is
  skipped whatever it holds, so the residue a killed cut leaves stays, and that
  window cannot be re-cut, until you remove it:
  [what `--out-dir` holds](docs/operating.md#what---out-dir-holds).
- **It handles 16 triggers at once.** A trigger arriving while all 16 slots are
  busy is rejected with a logged error — no clip, no `Recorded`, so automation
  should read a missing announcement as a dropped trigger.
- **`--time-source publish` is a no-op on Humble**, whose `rosbag2_storage_mcap`
  writes `publish_time = log_time` verbatim: [which clock a window lives
  on](docs/triggers-and-time.md#time-source-log-or-publish).

## Where to go next

| Page | What it answers |
|---|---|
| [Try it without a robot](docs/try-it.md) | cutting a real clip on a laptop: a Rust toolchain, no ROS, two minutes |
| [Configuration](docs/configuration.md) | every flag, environment variable and TOML key, and which layer wins |
| [Triggers and time](docs/triggers-and-time.md) | the `Trigger` message, where triggers come from, which clock a window lives on |
| [`clipper clip`](docs/clip-command.md) | cutting from a finished recording: bag directories, refusals, re-runs |
| [What a clip carries](docs/clip-manifest.md) | the clip directory, its `clip_metadata.yaml`, and how a clip's id is derived |
| [Operating clipper](docs/operating.md) | shutdown, logs, retention, a recording that stops producing clips, overload, tuning |
| [Installing and building](docs/install.md) | the `.deb`s, the two cargo builds, the nix package, a target build, the upgrade note |
| [What it costs to run](docs/performance.md) | measured CPU, memory and IO on a Jetson, and what it does to the recorder |
| [ARCHITECTURE.md](ARCHITECTURE.md) | inside: threads, tailing, what a clip is on disk, recovery |
| [`examples/`](examples/README.md) | setup guides — continuous recording, split bags, `ros2 launch`, ROS-free writers |
| [`crates/clip`](crates/clip), [`crates/tail`](crates/tail) | the libraries to build on: the format layer, the index, the cut path, the trigger contract — no ROS toolchain anywhere |

## Support and contributing

Questions, bug reports and feature requests: [open an
issue](https://github.com/Momentedge/momentedge-clipper/issues).
Contributions are welcome — [CONTRIBUTING.md](CONTRIBUTING.md) covers the
workspace layout, the build, and the conventions a change is expected to follow.

clipper is built by [Momentedge](https://momentedge.xyz/).

## License

Licensed under the [Apache License 2.0](LICENSE).
