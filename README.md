# Momentedge Clipper

> Event-triggered clips from a continuous ROS 2 recording — including the
> seconds *before* the event.

On a robot the data worth keeping is sparse: most of the time nothing
interesting happens. But you can't know an event mattered until after it has
already started — and a useful clip needs the lead-up, not just the aftermath.
That **preroll** only exists if the data was already on disk when the event
fired.

**Momentedge Clipper** turns an ordinary `ros2 bag record` into an on-demand
event recorder. It runs alongside the recorder, tails the growing MCAP file, and
on each trigger cuts a standalone clip covering a window around the event —
`[trigger_time − preroll, trigger_time + postroll]`. Recording stays rosbag2's
job; clipping is clipper's. The two never talk except through the file on disk.

- **MCAP in, MCAP out.** Clips are standard, complete MCAP files — readable by
  [Foxglove](https://foxglove.dev/), the `mcap` CLI, and `ros2 bag` replay. No
  vendor format on either side.
- **Decode-free.** clipper copies message bytes straight through; it never
  deserializes message bodies, so it is agnostic to your message types.
- **Triggers are just a topic.** Anything that can publish
  `momentedge_msgs/Trigger` — a fault detector, a watchdog, an operator button,
  your perception stack — can drive it.
- **Small and frozen-friendly.** Plain OS threads, sequential IO, no async
  runtime, fail-fast supervision. Built to be pinned at a version and left
  running on a robot.

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
        ├── copies [trigger_time − preroll, trigger_time + postroll] ──▶ ./clipped/<trigger_ns>_<name>.mcap
        │
        └── announces ──▶ /events/momentedge/recorded   (momentedge_msgs/Recorded, lists every file written)
```

1. **Tail.** clipper keeps the growing MCAP file open and incrementally scans
   the new bytes, decoding nothing but each message's timestamp. A clip can be
   cut the moment its data is physically on disk.
2. **Listen.** It waits for a `momentedge_msgs/Trigger` on
   `/events/momentedge/trigger`, carrying a reference time and a pre/post window.
3. **Copy.** It copies every message whose timestamp falls in
   `[trigger_time − preroll, trigger_time + postroll]` into a standalone clip,
   then announces the result on `/events/momentedge/recorded`.

Because the recording is already on disk, the preroll — the data from *before*
the trigger — is there to copy.

## Quickstart

You need three things, each in its own shell sharing one ROS 2 environment
(`RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` must match): a continuous recording,
clipper, and a trigger.

```bash
# 1. Continuous recording → ./record (one growing MCAP file)
ros2 bag record --all --storage mcap --output ./record
#    or ./scripts/record.sh for storage-tuned defaults

# 2. clipper, tailing ./record, writing clips to ./clipped
clipper --record-dir ./record --out-dir ./clipped --clip-compression zstd
#    from a source checkout: cargo run -p clipper -- --record-dir ./record --out-dir ./clipped

# 3. Fire a trigger: 5 s before and 5 s after "now"
ros2 topic pub --once /events/momentedge/trigger momentedge_msgs/msg/Trigger \
  "{name: clip1, trigger_time: {sec: $(date +%s), nanosec: 0}, preroll: 5000000000, postroll: 5000000000}"
```

A clip lands at `./clipped/<trigger_ns>_clip1.mcap` and a
`momentedge_msgs/Recorded` is published on `/events/momentedge/recorded`.
Inspect it with `ros2 bag info ./clipped/<file>.mcap` or open it in Foxglove.

For a continuous test trigger source during development, run the bundled
[`trigger-pub`](examples/trigger-pub/README.md) example instead of step 3.

## Installation

### From a release (recommended for deployment)

Each [GitHub release](../../releases) attaches two Debian packages per ROS 2
distro. Install both on a host running the matching distro (Humble packages on a
Humble host, Jazzy on Jazzy, …):

```bash
sudo apt install ./ros-humble-momentedge-msgs_*.deb ./momentedge-clipper_*.deb
source /opt/ros/humble/setup.bash
/opt/momentedge-clipper/bin/clipper --help
```

`momentedge-clipper` resolves its message typesupport from the
`ros-<distro>-momentedge-msgs` package through the distro's own `setup.bash`,
like every ROS executable — no bundled overlay, no baked rpath.

### From source

clipper is a standard Rust workspace, but the build needs a ROS 2 environment
(for `rcl`/`rmw` and the message typesupport):

- **Development:** a [Nix](https://nixos.org/) dev shell provides ROS 2 —
  `nix develop --command cargo build`. See [CLAUDE.md](CLAUDE.md) for the
  dev-shell and per-distro build details.
- **On a deployment target:** `./scripts/build-on-target.sh` compiles `clipper`
  and `momentedge_msgs` natively against the host's apt ROS 2 install (the
  binaries are ABI-compatible with the rest of the host's ROS graph by
  construction). See [ARCHITECTURE.md](ARCHITECTURE.md#deployment) for the
  rationale.

## Configuration

`clipper` is configured by CLI flags, each with a `MOMENTEDGE_*` environment
fallback and a built-in default: a flag overrides the env var, which overrides
the default. `clipper --help` lists them; `clipper --version` prints the
version. Everything is optional — `clipper` runs with no arguments.

| Flag | Env var | Default | Meaning |
|---|---|---|---|
| `--record-dir` | `MOMENTEDGE_RECORD_DIR` | `./record` | bag directory of the continuous recording to tail |
| `--out-dir` | `MOMENTEDGE_OUT_DIR` | `./clipped` | where finished clips are written |
| `--interface` | `MOMENTEDGE_INTERFACE` | `ros` | how triggers arrive and completions are signalled: `ros` or `mcap` (see [below](#two-ways-in-ros-and-mcap)) |
| `--grace-secs` | `MOMENTEDGE_GRACE_SECS` | `30` | how long past the window end to wait for the recording to cover it before cutting from what is on disk |
| `--clip-compression` | `MOMENTEDGE_CLIP_COMPRESSION` | `zstd` | codec for written clips: `none`, `lz4`, or `zstd` (smallest) |
| `--extract-parallelism` | `MOMENTEDGE_EXTRACT_PARALLELISM` | `1` | concurrent clip copies (1 = one at a time, FIFO) |
| `--watch-old-files-duration` | `MOMENTEDGE_WATCH_OLD_FILES_DURATION` | `600` | seconds to keep a finished (split/restart) recording indexed so a trigger's preroll can still reach into it; set comfortably above the largest preroll any trigger will request |
| `--delete-old-files` | `MOMENTEDGE_DELETE_OLD_FILES` | `false` | also unlink an expired `.mcap` from disk when it is pruned (off by default — clipper does not own the recordings) |

`--grace-secs` must exceed the recorder's flush latency: near zero for an
unchunked `fastwrite` recording, roughly one chunk-fill for a chunked profile.
The [`examples/continuous`](examples/continuous/README.md) guide explains the
recorder's latency-vs-size knobs and how to size `--grace-secs` against them.

## The trigger interface

A trigger is a `momentedge_msgs/Trigger` message:

| Field | Type | Meaning |
|---|---|---|
| `name` | `string` | trigger identifier; becomes part of the clip filename |
| `description` | `string` | optional free-form context |
| `trigger_time` | `builtin_interfaces/Time` | reference stamp the window is centred on |
| `preroll` | `uint64` | nanoseconds before `trigger_time` to keep |
| `postroll` | `uint64` | nanoseconds after `trigger_time` to keep |

For each finished clip, clipper publishes a `momentedge_msgs/Recorded` on
`/events/momentedge/recorded`, echoing the trigger's `name`, `description`, and
`trigger_time` and listing every file written in its `string[] filenames`. Every
path it names is already complete and crash-durable on disk.

**Clip naming.** A window that falls inside a single recording produces one
file, `<trigger_ns>_<name>.mcap`. A window that straddles a rollover (a rosbag2
bag split or a recorder restart clipper observed while running) produces one
segment per source file — `<trigger_ns>_<name>_00.mcap`, `_01.mcap`, … — tiling
the window in time order, all listed in `filenames`.

### Two ways in: `ros` and `mcap`

How a trigger reaches clipper and how completion is signalled is one choice, set
by `--interface` (default `ros`). clipper runs exactly one interface per launch.

- **`ros`** (the default, deployed path) subscribes to
  `/events/momentedge/trigger` and publishes `momentedge_msgs/Recorded` on
  `/events/momentedge/recorded`.
- **`mcap`** reads triggers straight out of the recording clipper already tails
  (run `ros2 bag record --all` so the trigger topic is captured) and runs
  **ROS-free** — no node, subscription, or publisher. A finished clip is
  signalled only by the file appearing in `--out-dir`.

Both cut identical clips; only the trigger and completion edges differ.

## Operational notes

- **Lifecycle.** Ctrl-C (SIGINT/SIGTERM) stops clipper cleanly with exit 0. Any
  internal fault — a dead tail thread, an unrecoverable scan fault — exits
  non-zero so a process supervisor (systemd, …) restarts it.
- **No startup back-indexing.** clipper recovers only rollovers it observed
  during its own run. A recording already on disk before clipper started
  contributes nothing to a trigger fired afterwards.
- **Retention is the recorder's job.** The continuous recording grows until you
  stop or split it; clipper never prunes the file it is tailing. See
  [`examples/split-bags`](examples/split-bags/README.md) for bounding the
  recording, and prune `./clipped` on your own schedule.
- **Concurrency cap.** Up to 16 triggers are handled at once; a trigger arriving
  while all 16 slots are busy is rejected with a logged error and produces no
  clip and no `Recorded` announcement. Automation waiting on the announcement
  should treat its absence — and the logged error — as a dropped trigger.

## Examples

Setup guides for the recording + clipper stack live under
[`examples/`](examples/README.md):

| Guide | What it covers |
|---|---|
| [`continuous/`](examples/continuous/README.md) | one growing MCAP file (the pairing clipper is built for) + the latency/size knobs |
| [`split-bags/`](examples/split-bags/README.md) | split recording with pruning for retention |
| [`launch/`](examples/launch/README.md) | recorder + clipper brought up together with `ros2 launch` |
| [`trigger-pub/`](examples/trigger-pub/README.md) | an example trigger source for development |

## Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** — the technical overview: thread model,
  tailing mechanics, atomic clip publication, restart/rollover recovery, damage
  tolerance, and the deployment build model.
- **[CLAUDE.md](CLAUDE.md)** and **[crates/clipper/CLAUDE.md](crates/clipper/CLAUDE.md)**
  — contributor and agent notes: workspace layout, build mechanics, and the
  recorder's internal design.

## License

Licensed under the [Apache License 2.0](LICENSE).
