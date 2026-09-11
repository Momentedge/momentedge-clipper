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

- **Nearly free to leave running.** 0.45 % of one core and 22 MiB on a Jetson
  Orin Nano, no disk reads while tailing, and the recorder it sits beside does
  not notice it is there. Plain OS threads, sequential IO, no async runtime,
  fail-fast supervision — built to be pinned at a version and forgotten.
  [The numbers →](#what-it-costs-to-run)
- **MCAP in, MCAP out.** Clips are standard, complete MCAP files — readable by
  [Foxglove](https://foxglove.dev/), the `mcap` CLI, and `ros2 bag` replay. No
  vendor format on either side.
- **Decode-free.** clipper copies message bytes straight through; it never
  deserializes message bodies, so it is agnostic to your message types.
- **Triggers are just a topic.** Anything that can publish
  `momentedge_msgs/Trigger` — a fault detector, a watchdog, an operator button,
  your perception stack — can drive it.
- **ROS is optional.** It is a cargo feature. The default build links no ROS at
  all, reads its triggers out of the recording, and cuts identical clips — on
  the vehicle, in a container, or in the cloud.
- **Every clip says what it is.** A manifest record inside each file names the
  trigger, the window, the source bytes and the per-channel counts — so a clip
  that came back empty can still tell you why.

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
        ├── copies [anchor − preroll, anchor + postroll] ──▶ ./clipped/<anchor_ns>_<name>.mcap
        │
        └── announces ──▶ /events/momentedge/recorded   (momentedge_msgs/Recorded, lists every file written)
```

1. **Tail.** clipper keeps the growing MCAP file open and incrementally scans
   the new bytes, decoding nothing but each message's timestamp. A clip can be
   cut the moment its data is physically on disk.
2. **Listen.** It waits for a `momentedge_msgs/Trigger` carrying a name and a
   pre/post window.
3. **Copy.** It copies every message inside the window into a standalone clip,
   then announces the result on `/events/momentedge/recorded`.

Because the recording is already on disk, the preroll — the data from *before*
the trigger — is there to copy.

## Quickstart

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

A standalone MCAP lands in `./clipped`, and it can tell you what it is:

```console
$ mcap get metadata --name momentedge.clip ./clipped/1738000000000000000_clip1.mcap
{
  "trigger.name":     "clip1",
  "trigger.anchor_ns": "1738000000000000000",
  "window.start_ns":  "1737999995000000000",
  "window.end_ns":    "1738000005000000000",
  "clip.messages":    "4211",
  "clip.short":       "false",
  ...
}
```

Or open it in Foxglove, replay it with `ros2 bag play`, inspect it with
`ros2 bag info` — it is an ordinary MCAP file.

`trigger_time: 0` means "anchor on the instant clipper receives this" — the
default. To anchor on an instant of your own choosing instead, see
[Triggers and time](docs/triggers-and-time.md). For a trigger source that keeps
firing while you develop, run the bundled
[`trigger-pub`](examples/trigger-pub/README.md) example instead of step 3.

## What it costs to run

Keeping the preroll on disk instead of in memory is what makes clipper cheap.
Measured on Jetson Orin Nano and Orin NX against a `ros2 bag record` writing
about 20 MB/s:

| | Orin Nano | Orin NX |
|---|---|---|
| **clipper, tailing** | **0.45 % of one core**, 22.0 MiB | **0.39 % of one core**, 21.4 MiB |
| the recorder alone | 5.60 % | 5.98 % |
| the recorder, with clipper attached | 5.68 % | 6.13 % |

Attaching clipper moves the recorder by less than the spread between
repetitions — so the measurement says the recorder does not notice, rather than
saying by how much.

- **No disk reads while tailing.** The scan of the growing file is served
  entirely from page cache: 0.0 MB of read traffic to the device over a
  two-minute measurement.
- **Memory does not grow with pending windows.** 22.0 MiB tailing, 22.6 MiB with
  ten windows queued — because a window's preroll is on disk, not in RAM.
- **Copying is the part that costs.** Ten overlapping seventy-second windows at
  20 MB/s cost about one core and finish in under three minutes.

Figures are per board and do not travel between them. Full methodology, the
per-configuration numbers and the conditions each one depends on:
[Momentedge/clipper-benchmarks](https://github.com/Momentedge/clipper-benchmarks).

## Install

**From a release** — each [GitHub release](../../releases) attaches two arm64
Debian packages for **Humble** (Ubuntu 22.04) and **Jazzy** (Ubuntu 24.04) —
the deployment target is a Jetson-class board. Install both on a host running
the matching distro:

```bash
sudo apt install ./ros-humble-momentedge-msgs_*.deb ./momentedge-clipper_*.deb
source /opt/ros/humble/setup.bash
/opt/momentedge-clipper/bin/clipper --help
```

**Upgrading a device already running clipper.** Where a run's triggers come from
is `--trigger-source`, or `MOMENTEDGE_TRIGGER_SOURCE` in the environment — no
configuration file may set it. A configuration file or unit
file carrying an earlier release's `interface` spelling stops the run at
startup — see
[the upgrade note](docs/configuration.md#upgrading-a-deployment-configured-for-an-earlier-release)
for what to edit, before the new package lands.

**From source** — ROS is a cargo feature, and it decides which of two builds you
get:

```bash
cargo build -p clipper                                       # ROS-free
nix build .#clipper-ros-free                                 # …or as a nix package
nix develop --command cargo build -p clipper --features ros  # the device build
```

The ROS-free build needs no ROS installation, no dev shell and no environment;
it reads triggers out of the recording it tails. `--features ros` adds the live
trigger subscription and the `Recorded` publish, and is what every Debian
artefact is built with. Everything else — the tail, the window, the cut — is the
same code in both. To build the device half on a target,
`./scripts/build-on-target.sh` compiles it natively against the host's apt ROS 2;
see [ARCHITECTURE.md](ARCHITECTURE.md#deployment) for why.

**Which ROS 2 distros.** CI builds, lints and runs the unit and live end-to-end
suites on **Humble, Jazzy and Lyrical** on every push; Jazzy is the default dev
shell. Humble and Jazzy are the two that ship `.deb`s. Rolling is not supported:
r2r references an rmw QoS variant Rolling has removed, so the device build does
not compile there — the ROS-free build, which links no r2r, is unaffected.

## Two modes

clipper is one binary and the mode is a subcommand:

| Mode | What it does |
|---|---|
| **`clipper tail`** | the recorder — follow a recording still being written and cut a clip per trigger, until shutdown |
| **`clipper clip`** | cut windows out of one *finished* recording and exit — same window plan, same copy, same manifest, without the waits a growing file costs |

`clipper clip` is how a bag pulled off a vehicle becomes clips afterwards, on a
workstation or in the cloud, with no ROS installed anywhere:

```bash
clipper clip ./record --out-dir ./clipped --trigger-source mcap
```

## What clipper does not do

- **It does not record.** clipper opens no sensor subscription and writes no
  continuous recording. `ros2 bag record` — or any append-only MCAP writer — has
  to be running beside it. Nothing on disk means nothing to cut.
- **It does not back-index.** clipper recovers only rollovers it observed during
  its own run, so a recording already on disk when it started contributes
  nothing to a trigger fired afterwards. Cut those with `clipper clip` instead.
- **It does not manage retention.** The continuous recording grows until you
  stop or split it, and clipper never prunes the file it is tailing. It will
  unlink *expired, already-rolled-over* recordings if you ask
  (`--delete-old-files`), and nothing else.
- **It handles 16 triggers at once.** A trigger arriving while all 16 slots are
  busy is rejected with a logged error and produces no clip and no `Recorded`.
  Automation waiting on that announcement should treat its absence as a dropped
  trigger.
- **`--time-source publish` is a no-op on Humble**, whose `rosbag2_storage_mcap`
  writes `publish_time = log_time` verbatim. It differs on Jazzy and newer, and
  for a writer that owns its own capture stamps.

## Where to go next

| Page | What it answers |
|---|---|
| [Configuration](docs/configuration.md) | every flag, environment variable and TOML key, and which layer wins |
| [Triggers and time](docs/triggers-and-time.md) | the `Trigger` message, the two places a trigger comes from, and which clock a window lives on |
| [`clipper clip`](docs/clip-command.md) | cutting from a finished recording: bag directories, refusals, re-runs |
| [What a clip carries](docs/clip-manifest.md) | the `momentedge.clip` manifest inside every clip |
| [Operating clipper](docs/operating.md) | shutdown, logs, retention, overload, and tuning under load |
| [ARCHITECTURE.md](ARCHITECTURE.md) | how it works inside: threads, tailing, atomic publication, recovery |
| [`examples/`](examples/README.md) | setup guides — continuous recording, split bags, `ros2 launch`, and ROS-free MCAP writers |

Building clipper into something of your own: [`crates/clip`](crates/clip) is the
MCAP format layer, the recording index, the cut path and the trigger contract;
[`crates/tail`](crates/tail) adds what a recording still being written needs.
Both build with no ROS toolchain anywhere.

## Support and contributing

Questions, bug reports and feature requests: [open an issue](../../issues).
Contributions are welcome — [CLAUDE.md](CLAUDE.md) and the per-crate notes under
[`crates/`](crates/CLAUDE.md) cover the workspace layout, the build, and the
conventions a change is expected to follow.

## License

Licensed under the [Apache License 2.0](LICENSE).
