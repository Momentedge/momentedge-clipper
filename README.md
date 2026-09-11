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
`[anchor − preroll, anchor + postroll]`, where the anchor is the event instant
the trigger resolves to. Recording stays rosbag2's job; clipping is clipper's.
The two never talk except through the file on disk.

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
        ├── copies [anchor − preroll, anchor + postroll] ──▶ ./clipped/<anchor_ns>_<name>.mcap
        │
        └── announces ──▶ /events/momentedge/recorded   (momentedge_msgs/Recorded, lists every file written)
```

1. **Tail.** clipper keeps the growing MCAP file open and incrementally scans
   the new bytes, decoding nothing but each message's timestamp. A clip can be
   cut the moment its data is physically on disk.
2. **Listen.** It waits for a `momentedge_msgs/Trigger` on
   `/events/momentedge/trigger`, carrying a name and a pre/post window; the
   window's anchor is resolved per the [time source](#time-source-log-or-publish).
3. **Copy.** It copies every message whose timestamp falls in
   `[anchor − preroll, anchor + postroll]` into a standalone clip, then announces
   the result on `/events/momentedge/recorded`.

Because the recording is already on disk, the preroll — the data from *before*
the trigger — is there to copy.

## Resource overhead

Keeping the preroll on disk instead of in memory is what makes clipper cheap.
Measured on Jetson Orin Nano and Orin NX against a `ros2 bag record` writing
about 20 MB/s, clipper's standing cost is a rounding error next to the recorder
it tails — and the recorder does not notice it is there:

| | Orin Nano | Orin NX |
|---|---|---|
| **clipper, tailing** | **0.45 % of one core**, 22.0 MiB | **0.39 % of one core**, 21.4 MiB |
| the recorder alone | 5.60 % | 5.98 % |
| the recorder, with clipper attached | 5.68 % | 6.13 % |

Attaching clipper moves the recorder by 0.08 and 0.15 percentage points, which
is the size of the spread between repetitions — so the measurement says the
recorder does not notice, rather than by exactly how much.

- **No disk reads while tailing.** clipper's scan of the growing file is served
  entirely from page cache: its own read traffic to the device is 0.0 MB over a
  two-minute measurement. Copying a clip does read the disk — about 450 MB to
  cut ten seventy-second windows. (Measured on the Nano; the NX kernel carries
  no per-process IO accounting, so it cannot answer either way.)
- **Pending clips are close to free.** Ten overlapping windows waiting for their
  postroll to elapse cost two hundredths of a percentage point of one core, and
  about 60 KB and one and a half threads each.
- **Copying ten windows costs about one core.** Ten overlapping windows all
  copying at once, at 20 MB/s, cost 103 % of one core on a six-core Nano and
  finish in under three minutes. `--extract-parallelism` trades that against
  wall clock rather than reducing it: at one worker per core the same work takes
  432 % of one core and a third of the time.
- **Memory does not grow with pending windows.** clipper holds 22.0 MiB tailing
  and 22.6 MiB with ten windows queued, because a window's preroll is on disk
  rather than in memory. It grows only while actually copying — 55 MiB at the
  default parallelism — and returns afterwards.

Clip compression is opt-in and is the one thing that costs real CPU. On a 410 MB
window it turns roughly 0.7 s of copying into just over 5 s, for a clip about
14 % smaller; the tailing figures above are unaffected by it, because a run that
cuts nothing never compresses anything.

On a six-core board, leaving `--extract-parallelism` at its default is what
keeps the recorder lossless: copying ten windows across every core costs the
recorder a handful of dropped messages on the Nano, and dozens when the board is
also busy, while one worker at a time drops none. The eight-core NX drops none
in any configuration.

Figures are per board and are not comparable across the two: the Nano and the NX
differ in SoC, core count, RAM, kernel and ROS distro, so the ratios above hold
within a column and the absolutes do not travel between them.

Full methodology, per-configuration figures, and the conditions each number
depends on live in
[Momentedge/clipper-benchmarks](https://github.com/Momentedge/clipper-benchmarks).

## Quickstart

You need three things, each in its own shell sharing one ROS 2 environment
(`RMW_IMPLEMENTATION` and `ROS_DOMAIN_ID` must match): a continuous recording,
clipper, and a trigger.

```bash
# 1. Continuous recording → ./record (one growing MCAP file)
ros2 bag record --all --storage mcap --output ./record
#    or ./scripts/record.sh for storage-tuned defaults

# 2. clipper, tailing ./record, writing clips to ./clipped
clipper tail --record-dir ./record --out-dir ./clipped --clip-compression zstd
#    from a source checkout (--features ros for the live trigger topic below):
#    cargo run -p clipper --features ros -- tail --record-dir ./record --out-dir ./clipped

# 3. Fire a trigger: 5 s before and 5 s after the instant clipper receives it.
#    Under the default --time-source log the window anchors on clipper's own
#    receipt instant, so trigger_time is 0 (a non-zero value is rejected here;
#    only --interface ros --time-source publish reads it — see Time source).
ros2 topic pub --once /events/momentedge/trigger momentedge_msgs/msg/Trigger \
  "{name: clip1, trigger_time: {sec: 0, nanosec: 0}, preroll: 5000000000, postroll: 5000000000}"
```

A clip lands at `./clipped/<anchor_ns>_clip1.mcap` and a
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

The package installs one executable, `/opt/momentedge-clipper/bin/clipper`.
Its modes are subcommands, and the recorder is `clipper tail` — that is the
command a unit file names.

`momentedge-clipper` resolves its message typesupport from the
`ros-<distro>-momentedge-msgs` package through the distro's own `setup.bash`,
like every ROS executable — no bundled overlay, no baked rpath.

### From source

clipper is a standard Rust workspace, and **ROS is a cargo feature** of it. The
feature decides which of two builds you get:

```bash
cargo build -p clipper                                       # ROS-free
nix develop --command cargo build -p clipper --features ros  # the device build
```

- **Default (no feature):** the binary links no ROS at all and builds and runs on
  a host with no ROS installation. It offers `--interface mcap` — triggers read
  out of the recording it tails — and nothing else. Plain `cargo build`, no ROS 2
  environment, no dev shell. It also ships as a nix package — the flake's one
  deployable output, which runs anywhere the store path goes:

  ```bash
  nix build .#clipper-ros-free      # -> ./result/bin/clipper
  ```
- **`--features ros`:** the device build, and what every Debian release artefact
  is. It links `rcl`/`rmw` and the `momentedge_msgs` typesupport, so it needs a ROS 2
  environment to compile against, and it adds `--interface ros` (its default
  there): the live trigger subscription and the `Recorded` publish.

Everything else — the tail, the window, the cut, every other flag — is the same
code in both. To build the device half:

- **Development:** a [Nix](https://nixos.org/) dev shell provides ROS 2 —
  `nix develop --command cargo build --features clipper/ros`. See
  [CLAUDE.md](CLAUDE.md) for the dev-shell and per-distro build details.
- **On a deployment target:** `./scripts/build-on-target.sh` compiles
  `clipper` (with the feature) and `momentedge_msgs` natively against the host's
  apt ROS 2 install (the binaries are ABI-compatible with the rest of the host's
  ROS graph by construction). See
  [ARCHITECTURE.md](ARCHITECTURE.md#deployment) for the rationale.

## Configuration

clipper is one binary and the mode is a subcommand, and `clipper --help` lists
them:

- **`clipper tail`** — the recorder: follow a continuous recording and cut a clip
  per trigger, until a shutdown signal.
- **`clipper clip`** — cut clips out of one finished recording and exit
  ([below](#cutting-clips-from-a-finished-recording-clipper-clip)).

A flag handed to the bare `clipper` is refused, with a message naming the mode
that owns it. `clipper <mode> --help` lists that mode's flags;
`clipper --version` prints the version.

**Every setting resolves through four layers.** Weakest first:

1. the **system configuration file**, `/etc/momentedge/clipper.toml`
2. the **per-run configuration file**, named by `--config`
3. the **environment**, `MOMENTEDGE_<KEY>`
4. the **command-line flag**

with the built-in default underneath all four. The strongest layer that names a
setting wins it, and a layer silent about a setting hands the question down: a
value present in all four layers is the flag's, dropping the flag leaves the
environment's, dropping that leaves the per-run file's, dropping that leaves the
system file's, and dropping that leaves the built-in default. Layers merge per
setting, not per file — a per-run file naming one key leaves every other key to
the layers below it.

A setting's `[settings]` key is its flag without the `--` and with `-` written
`_` (`--grace-secs` → `grace_secs`), and its environment variable is
`MOMENTEDGE_` + that key upper-cased (`MOMENTEDGE_GRACE_SECS`). The **Per-run**
column below says whether a per-run file may set the key at all — see
[The configuration file](#the-configuration-file).

Three flags belong to every mode and to none of the tables below, because they
are about the configuration rather than in it: `--config PATH` names the per-run
file, `--system-config PATH` moves the system file, and `--print-config` prints
[the effective configuration](#the-effective-configuration) and exits.

### `clipper tail`

Every flag is optional — `clipper tail` runs with no further arguments.

| Flag | Env var | Type | Default | Per-run | Meaning |
|---|---|---|---|---|---|
| `--record-dir` | `MOMENTEDGE_RECORD_DIR` | path | `./record` | no | bag directory of the continuous recording to tail |
| `--out-dir` | `MOMENTEDGE_OUT_DIR` | path | `./clipped` | yes | where finished clips are written |
| `--interface` | `MOMENTEDGE_INTERFACE` | `ros` \| `mcap` | `ros` (`mcap` in a ROS-free build) | no | how triggers arrive and completions are signalled (see [below](#two-ways-in-ros-and-mcap)) |
| `--time-source` | `MOMENTEDGE_TIME_SOURCE` | `log` \| `publish` | `log` | no | clock domain the clip window lives in (see [below](#time-source-log-or-publish)) |
| `--grace-secs` | `MOMENTEDGE_GRACE_SECS` | integer, seconds | `30` | yes | how long past the window end to wait for the recording to cover it before cutting from what is on disk |
| `--clip-compression` | `MOMENTEDGE_CLIP_COMPRESSION` | `none` \| `lz4` \| `zstd` | `zstd` | yes | codec for written clips (`zstd` writes the smallest) |
| `--extract-parallelism` | `MOMENTEDGE_EXTRACT_PARALLELISM` | integer | `1` | no | concurrent clip copies (1 = one at a time, FIFO) |
| `--watch-old-files-duration` | `MOMENTEDGE_WATCH_OLD_FILES_DURATION` | integer, seconds | `600` | no | seconds to keep a finished (split/restart) recording indexed so a trigger's preroll can still reach into it; set comfortably above the largest preroll any trigger will request |
| `--delete-old-files` | `MOMENTEDGE_DELETE_OLD_FILES` | boolean | `false` | no | also unlink an expired `.mcap` from disk when it is pruned (off by default — clipper does not own the recordings) |

`--grace-secs` must exceed the recorder's flush latency: near zero for an
unchunked `fastwrite` recording, roughly one chunk-fill for a chunked profile.
The [`examples/continuous`](examples/continuous/README.md) guide explains the
recorder's latency-vs-size knobs and how to size `--grace-secs` against them.

### Cutting clips from a finished recording: `clipper clip`

`clipper tail` exists because the recording has no end yet: it follows the file
and waits for each window to land on disk before cutting. A recording that is
already finished needs no such wait — `clipper clip` takes one, cuts the window
each trigger names out of it, and exits.

```bash
# one clip, from a trigger named on the command line
clipper clip ./record/rosbag2_0.mcap \
  --out-dir ./clipped \
  --trigger-time 1738000000000000000 \
  --preroll 5000000000 --postroll 5000000000 \
  --trigger-name brake-event --trigger-description "hard brake over 0.8 g"

# the whole bag directory, read as one time-ordered collection
clipper clip ./record --out-dir ./clipped --trigger-source mcap
```

| Argument | Env var | Type | Default | Per-run | Meaning |
|---|---|---|---|---|---|
| `<recording>` | `MOMENTEDGE_RECORDING` | path | — | yes | the finished recording to cut from (positional): one `.mcap`, or a bag directory of splits |
| `--out-dir` | `MOMENTEDGE_OUT_DIR` | path | — | yes | where the clips are written |
| `--trigger-source` | `MOMENTEDGE_TRIGGER_SOURCE` | `param` \| `mcap` | `param` | yes | where this run's triggers come from (see [below](#where-a-clip-runs-triggers-come-from-param-and-mcap)) |
| `--trigger-time` | `MOMENTEDGE_TRIGGER_TIME` | integer, ns | — | yes | the instant the window centres on, in nanoseconds since the epoch (`param` only, required) |
| `--preroll` | `MOMENTEDGE_PREROLL` | integer, ns | — | yes | nanoseconds before that instant to include (`param` only, required) |
| `--postroll` | `MOMENTEDGE_POSTROLL` | integer, ns | — | yes | nanoseconds after it to include (`param` only, required) |
| `--trigger-name` | `MOMENTEDGE_TRIGGER_NAME` | string | `clip` | yes | the trigger's name, which also names the clip file (`param` only) |
| `--trigger-description` | `MOMENTEDGE_TRIGGER_DESCRIPTION` | string | *(empty)* | yes | the trigger's description, carried into the manifest (`param` only) |

Each clip lands at `<out-dir>/<anchor-ns>_<trigger-name>.mcap` and is the same
file the recorder would have written from the same recording and window — the
window plan, the byte copy, the [manifest](#what-a-clip-carries) and the atomic
publication are all the shared path. The five `--trigger-*` arguments are the
fields of a `momentedge_msgs/Trigger`, so the clip states the same trigger a clip
cut from a live topic does; only `producer.mode` differs, reading `clip` rather
than `tail`.

Five things follow from the input being finished:

- **Nothing waits.** No postroll sleep, no wait for coverage. A window reaching
  past the end of the recording is simply short, and the clip's `clip.short` key
  says so.
- **There is no clock-domain flag.** A recording's summary states its message
  times on `log_time` alone, so that is the clock the window lives on. Passing
  `--time-source` is a parse error.
- **Reading it is cheap.** The recording is indexed from its own summary — a
  footer seek and one read, whatever the file's size — rather than by walking it,
  so no chunk is decompressed until the copy asks for one. A bag directory costs
  that per split. Reading the recording's own triggers costs the same summary
  plus the chunks that summary names as holding the trigger channel, and nothing
  else.
- **A recording it cannot index is refused by name**, from that same footer and
  summary, before anything is written — see below.
- **A clip that is already there is refused too.** The same recording and the
  same trigger describe the same window, so a re-run would write the clip that is
  already in `--out-dir`. It names that clip and exits non-zero instead — see
  [below](#when-a-clip-is-already-there).

#### A bag directory is one collection

A recorder that ran for hours left a directory of splits, and `<recording>` takes
that directory as readily as it takes one file. The splits are read as one
time-ordered collection, so a window straddling a split is cut whole: it yields
one segment per *contributing* recording, named `<anchor-ns>_<name>_00.mcap`,
`_01.mcap` and so on — the same set the recorder writes when a window straddles a
rollover. A segment's number is its position among the segments that hold data,
so a window over three splits whose middle recording contributes nothing yields
`_00` and `_01`, where `_01` holds the third recording's data.

The order is the recorder's own where it stated one:

- **With `metadata.yaml`** — the sidecar `ros2 bag record` writes when it stops —
  the ordered `relative_file_paths` it states is the split order, and its
  collection-wide per-topic message counts are cross-checked against what the
  recordings present add up to. Every topic the two disagree about is reported;
  it is not fatal, since a collection short a split still cuts every window its
  splits do cover.
- **Without it** the recordings are ordered oldest-first by modification time.
  That is the directory copied off a device while it was still recording: the
  file is written at shutdown, so its absence is the signal.

Every split is indexed on its own and has to satisfy the same contract on its
own, so a directory holding one that does not is
[refused naming that recording](#when-a-recording-is-refused) — the last split of
a directory copied mid-recording is the usual offender.

#### When a recording is refused

Not every `.mcap` carries a summary worth planning a window from. `clipper clip`
decides that from the footer and the summary alone, names the fault, exits
non-zero, and writes nothing at all — no output directory, no staged file. Over a
bag directory the message names the split that failed, not the directory.

| The message says | The recording is |
|---|---|
| *is N bytes … not an MCAP recording* | too small to hold a footer |
| *does not end with the MCAP magic* | truncated, or copied off a device while it was still being written |
| *footer points at no summary section* | written by a writer configured without a summary |
| *holds no message* | empty — its statistics report a message count of zero |
| *summary indexes no chunk* | written with an unchunked profile |
| *no … chunk index carries a message index* | written by a writer with message indexing disabled |

A directory is refused too when it holds no `*.mcap` at all — usually the
directory *above* the one the splits are in — or when its `metadata.yaml` is
there and does not parse. The metadata file is the split order, so a file that
cannot be read is not quietly replaced by the modification times: repair it, or
delete it to fall back to those.

Every one of them but the empty recording is repaired by rewriting the file,
which clipper never does itself — it opens an input read-only and leaves it
exactly as it found it:

```bash
mcap recover  ./record/rosbag2_0.mcap -o ./record/fixed.mcap   # rebuild the index
mcap compress ./record/rosbag2_0.mcap -o ./record/fixed.mcap   # rebuild it smaller
mcap list chunks ./record/fixed.mcap                           # check the result
```

`mcap list chunks` prints a `message index length` per chunk; that column is the
field the last refusal above reads, and a repaired recording has it non-zero.

#### When a clip is already there

A finished recording and a trigger describe one window and one copy of its bytes,
so running the same cut twice would write the clip that is already in
`--out-dir`. The second run names that clip, exits non-zero, and writes nothing —
no clip, no suffixed sibling, nothing left in the staging directory:

```console
$ clipper clip ./record/rosbag2_0.mcap --out-dir ./clipped \
    --trigger-time 1738000000000000000 --preroll 5000000000 --postroll 5000000000
Error: ./clipped/1738000000000000000_clip.mcap already exists: this window has been
cut into this output directory before, and cutting it again writes a second copy of
the same clip rather than new data. Move or delete it, or cut into a different
output directory, to cut this window again
```

The check is one read of the output directory, made before the window is planned,
so a window that would have been written as several segments (`_00`, `_01`, …) is
refused whole rather than half-written. It asks whether the clip's own name **or any
`_NN` segment beside it** is taken, because how many segments a window becomes is
settled only while it is being cut — so a clip already there under either shape
refuses the run. There is no flag to override it: to cut the window again, move
or delete the clip, or point `--out-dir` somewhere else.

This is where `clipper clip` and `clipper tail` differ on purpose. On the vehicle
a clip name that is already taken means a *second* trigger asked for the same
instant and name, and that clip is data no re-run can produce again, so the
recorder publishes it beside the first as `<name>_1.mcap`. A cut from a finished
recording is replayable, so the same name means the same bytes, and a second file
would be a duplicate.

Nothing machine-readable is printed. The result is the output directory's
contents when the process exits, each clip carrying its own manifest, and the
exit status is the verdict. No ROS is involved, so the ROS-free build cuts these
clips as well as the device build does.

### The configuration file

Both configuration files are TOML and share one schema: a `[settings]` table
whose keys are the modes' settings, and a `[topics]` table deciding
[which topics a clip contains](#which-topics-a-clip-contains). One loader reads
them for every mode, so `clipper tail` and `clipper clip` take the same file.

```toml
# /etc/momentedge/clipper.toml — the system file: this machine's recorder
[settings]
record_dir = "/data/record"
out_dir = "/data/clipped"
interface = "ros"
extract_parallelism = 1
grace_secs = 45

[topics]
exclude_regex = "^/diagnostics"
```

```toml
# ./night-run.toml — one job, passed as --config ./night-run.toml
[settings]
out_dir = "./clips/night-run"

[topics]
include = ["/camera/front/image_raw", "/imu/data"]
```

| | Where it is read from | How to move it |
|---|---|---|
| **system file** | `/etc/momentedge/clipper.toml` | `--system-config PATH`, `MOMENTEDGE_SYSTEM_CONFIG` |
| **per-run file** | *(nowhere by default — named per run)* | `--config PATH`, `MOMENTEDGE_CONFIG` |

Neither file may name a file: `--config`, `--system-config` and
`--print-config` are flags and environment variables only, so a configuration
file can never point at another one.

**A file that is not there is not an error.** A missing system file, a missing
per-run file, and both missing are all legal — every setting simply falls
through to the layer below, and `clipper tail` with no files and no flags runs
on the built-in defaults. A path named explicitly but absent is reported as a
warning naming it, so a typo is visible without stopping the run. A file that
*is* there and does not parse, names a key that does not exist, or carries a
value of the wrong type is a startup error naming the file and the key — a
misspelled key never silently does nothing.

**What a per-run file may set.** The system file may set every key. A per-run
file may set only the keys marked *yes* in the **Per-run** column of the tables
above and of [`[topics]`](#which-topics-a-clip-contains) below. The line is
what the key describes: a key naming the machine the recorder runs on or the
resources it may spend there — where the recording lives, how triggers arrive,
which clock windows live on, how much IO and memory the tail may take, and
whether clipper may unlink a recording — is the system's alone. A key
describing one job — where its clips go, how long it waits, how they are
compressed, which window, and which topics — is the per-run file's to set.

A per-run file that sets a system-only key is **refused**: the key is named in a
warning and in `--print-config`, the system file's value (or the built-in
default) stands, and the run continues. The environment and the flags are not
scoped this way — whoever launches the process already commands both.

### Which topics a clip contains

`[topics]` decides which of the recording's channels a clip is cut from. It is
applied by the shared cut path, so the same configuration produces the same
channel set whether the clip is cut by `clipper tail` on the device or by
`clipper clip` from the finished recording.

| Key | Type | Default | Per-run | Meaning |
|---|---|---|---|---|
| `all` | boolean | `true` with no include key set, `false` with one | yes | take every topic the recording carries |
| `include` | list of strings | `[]` | yes | exact topic names to keep |
| `include_regex` | string, a regular expression | *(unset)* | yes | keep every topic it matches |
| `exclude` | list of strings | `[]` | yes | exact topic names to drop |
| `exclude_regex` | string, a regular expression | *(unset)* | yes | drop every topic it matches |
| `exclude_trigger_topic` | boolean | `false` | yes | drop `/events/momentedge/trigger` |

A topic is copied when it is **selected and not dropped**. It is selected when
`all` is true, or its name is in `include`, or `include_regex` matches it; it is
dropped when its name is in `exclude`, or `exclude_regex` matches it. Dropping
wins over selecting, as it does for `ros2 bag record`. `all` needs no setting
in the two ordinary cases — it is `true` when no include key is set (every
topic) and `false` when one is (only what the include keys name) — and setting
it to `true` beside an include list widens the clip back to everything.
Patterns are [Rust `regex`](https://docs.rs/regex) syntax and are unanchored,
so `^/camera/` matches at the start of a topic name and `camera` matches
anywhere in one.

Two rules sit outside the table:

- **clipper's own announcement topic, `/events/momentedge/recorded`, is never
  copied into a clip**, whatever any file says. A clip is about the robot, not
  about clipper announcing clips about the robot.
- **The trigger topic, `/events/momentedge/trigger`, is kept by default**, so a
  clip carries the trigger that asked for it. `exclude_trigger_topic = true`
  drops it, and that is the only key that governs it.

An excluded topic leaves nothing behind. Its channel and its schema are never
registered in the clip, so a reader listing the clip's topics or its schemas
sees neither, and it has no `channel.<id>.*` keys in the
[manifest](#what-a-clip-carries). Selection only ever narrows a clip: a topic
the recording does not carry cannot be added by naming it.

**The keys mirror `ros2 bag record`'s own topic selection**, whose spelling
moves between ROS 2 distributions. clipper's keys do not move with it:

| `[topics]` key | `ros2 bag record` on Humble | `ros2 bag record` on Jazzy and newer |
|---|---|---|
| `all` | `-a`, `--all` | `-a`, `--all`, `--all-topics` |
| `include` | `--topics A B` | `--topics A B` |
| `include_regex` | `-e`, `--regex RE` | `-e`, `--regex RE` |
| `exclude` | *(no equivalent — Humble excludes by regex only)* | `--exclude-topics A B` |
| `exclude_regex` | `-x`, `--exclude RE` | `-x`, `--exclude-regex RE` |
| `exclude_trigger_topic` | *(clipper's own)* | *(clipper's own)* |

The recorder decides what reaches the disk and these keys decide what reaches a
clip, so the two compose: a topic the recording never captured is not in a clip
however this table is filled in.

### The effective configuration

`clipper <mode> --print-config` prints every setting the run would use, the
value it resolved to, and the layer that decided it, then exits. The same text
is logged at startup, so a run's log states the configuration it ran with:

```console
$ clipper tail --config ./night-run.toml --grace-secs 12 --print-config
clipper tail effective configuration
  [settings]
    clip_compression         = zstd                   <- built-in default
    delete_old_files         = false                  <- built-in default
    extract_parallelism      = 1                      <- system file /etc/momentedge/clipper.toml
    grace_secs               = 12                     <- flag
    interface                = ros                    <- system file /etc/momentedge/clipper.toml
    out_dir                  = ./clips/night-run      <- per-run file ./night-run.toml
    record_dir               = /data/record           <- system file /etc/momentedge/clipper.toml
    time_source              = log                    <- built-in default
    watch_old_files_duration = 600                    <- built-in default
  [topics]
    all                      = false                  <- built-in default
    include                  = ["/camera/front/image_raw", "/imu/data"] <- per-run file ./night-run.toml
    include_regex            = (unset)                <- built-in default
    exclude                  = []                     <- built-in default
    exclude_regex            = ^/diagnostics          <- system file /etc/momentedge/clipper.toml
    exclude_trigger_topic    = false                  <- built-in default
```

The values are read back out of the parsed command line rather than re-derived,
so what the report prints is what the run uses. A per-run file's refused key is
listed under the report, naming the key and the file it came from.

## Time source: `log` or `publish`

`--time-source` picks the clock domain the **whole clip window** lives in — the
anchor it centres on, which messages fall inside it, which bytes are read, and
the coverage a cut waits for. Every MCAP message carries two stamps, and clipper
windows on whichever the flag selects:

- **`log`** (the default) — the message's `log_time`: when the producer received
  it. One writer stamps every recording in receive order, so log times run
  (approximately) non-decreasing on disk. Coverage on `log` is a *completeness*
  proof: once it passes a window end, every in-window message is on disk.
- **`publish`** — the message's `publish_time`: whatever the producer wrote
  there. `ros2 bag record` fills it with the DDS source timestamp; a momentedge
  writer fills it with the capture time (see
  [`examples/custom-mcap-writer`](examples/custom-mcap-writer/README.md)). clipper
  never interprets it — it windows on the raw value.

### The anchor: which instant the window centres on

The window centres on an **anchor** the active interface resolves from what it
has. A live ROS trigger carries no recording stamp, so the ROS interface anchors
on `now` or the publisher's `trigger_time`; an in-recording trigger carries its
own stamps, so the MCAP interface anchors on those. The four
interface × `--time-source` cells resolve it thus:

| | `--time-source log` | `--time-source publish` |
|---|---|---|
| **`--interface ros`** | `now` at the subscription instant | the trigger's `trigger_time` |
| **`--interface mcap`** | the trigger record's `log_time` | the trigger record's `publish_time` |

**`trigger_time` is read in exactly one cell — `ros` + `publish`.** There it is
the anchor: a publisher declaring its own publish-domain instant, standing in for
the `publish_time` it cannot set on the wire, so a request like "clip around ten
minutes ago" lands where it means to. Every other cell anchors on a transport
stamp and **rejects** a trigger that sets a non-zero `trigger_time` — logging it
at `error!` and cutting no clip — rather than silently dropping the field and
mis-anchoring the window. A producer for those cells must send `trigger_time = 0`
(the [`trigger-pub`](examples/trigger-pub/README.md) example does by default).

Retention is unaffected by the flag — a recording is always aged out on its
`log_time`, so a producer cannot drive file deletion through `publish_time`.

**Publish coverage is a liveness signal, not a completeness proof.** Publish
times carry no ordering guarantee; out-of-order arrival is normal. Under
`--time-source publish` a message can land on disk *after* a cut with a
`publish_time` that falls inside the window, and is then missing from that clip.
`--grace-secs` bounds how long a cut waits, exactly as on `log`.

**On Humble, `--time-source publish` is a no-op.** Humble's
`rosbag2_storage_mcap` writes `publish_time = log_time` verbatim, so the two
domains are identical there. It differs on Jazzy and newer (where `publish_time`
is the DDS source timestamp) and for a momentedge writer (capture time).

## The trigger interface

A trigger is a `momentedge_msgs/Trigger` message:

| Field | Type | Meaning |
|---|---|---|
| `name` | `string` | trigger identifier; becomes part of the clip filename |
| `description` | `string` | optional free-form context |
| `trigger_time` | `builtin_interfaces/Time` | publish-domain anchor; read only under `--interface ros --time-source publish` (see [the anchor matrix](#the-anchor-which-instant-the-window-centres-on)), must be `0` in every other cell |
| `preroll` | `uint64` | nanoseconds before the anchor to keep |
| `postroll` | `uint64` | nanoseconds after the anchor to keep |

**Validation.** Every field is checked before any work; a trigger failing any
check is logged at `error!` and ignored — no clip, no `Recorded`. The limits
(each value exactly at its bound is accepted):

- `preroll` and `postroll` — at most **30 minutes** (`1_800_000_000_000` ns) each.
- The resolved **anchor** — at most **30 minutes** past the current clock. The
  anchor drives the window's wait, so a far-future one (a producer clock fault or
  a hostile record stamp) is refused rather than parking a handler for that long.
- `name` — non-empty, at most **128 bytes**, and safe to embed in the clip
  pathname: no path separator, NUL, leading `.`, or `..`.
- `trigger_time` — `0` except in the one cell that reads it (`--interface ros
  --time-source publish`); non-zero elsewhere is rejected (see
  [the anchor matrix](#the-anchor-which-instant-the-window-centres-on)).

For each finished clip, clipper publishes a `momentedge_msgs/Recorded` on
`/events/momentedge/recorded`, echoing the trigger's `name`, `description`, and
`trigger_time` and listing every file written in its `string[] filenames`. Every
path it names is already complete and crash-durable on disk.

**Clip naming.** A window that falls inside a single recording produces one
file, `<anchor_ns>_<name>.mcap`, where `<anchor_ns>` is the resolved window
anchor. A window that straddles a rollover (a rosbag2 bag split or a recorder
restart clipper observed while running) produces one segment per source file —
`<anchor_ns>_<name>_00.mcap`, `_01.mcap`, … — tiling the window in time order,
all listed in `filenames`.

### Two ways in: `ros` and `mcap`

How a trigger reaches clipper and how completion is signalled is one choice, set
by `--interface`. clipper runs exactly one interface per launch.

- **`ros`** (the deployed path, and the default where it exists) subscribes to
  `/events/momentedge/trigger` and publishes `momentedge_msgs/Recorded` on
  `/events/momentedge/recorded`.
- **`mcap`** reads triggers straight out of the recording clipper already tails
  (run `ros2 bag record --all` so the trigger topic is captured) and runs
  **ROS-free** — no node, subscription, or publisher. A finished clip is
  signalled only by the file appearing in `--out-dir`.

Both cut identical clips; only the trigger and completion edges differ.

`ros` is the interface the `ros` cargo feature adds, so a
[ROS-free build](#from-source) offers `mcap` alone and takes it by default, and
`clipper tail --help` lists the values the binary in front of you accepts. The
`mcap` interface decodes each trigger by its MCAP `message_encoding`: `json`
decodes in every build, while `cdr` — what `ros2 bag record` writes — needs the
rmw typesupport the same feature links, and a ROS-free build skips such a trigger
with an error naming the feature. So a ROS-free deployment wants a producer that
writes its triggers as `json` (see
[`examples/custom-mcap-writer`](examples/custom-mcap-writer/README.md)).

### Where a `clip` run's triggers come from: `param` and `mcap`

`clipper clip` faces the same question from the other side: a finished recording
holds no live topic to subscribe to, so the triggers come either from the command
line or from the recording itself. `--trigger-source` picks one, and exactly one
is active per run.

- **`param`** (the default) cuts the single trigger the `--trigger-*` flags name.
  It needs `--trigger-time`, `--preroll` and `--postroll`; `--trigger-name` and
  `--trigger-description` fill in. One run, one clip.
- **`mcap`** cuts every trigger the recording carries on
  `/events/momentedge/trigger` — one clip per trigger, each anchored on the
  `log_time` the recording stamped that trigger message with, with the name,
  description, preroll and postroll the message itself states. It takes no
  `--trigger-*` flag at all. Reading the trigger list costs the summary plus the
  chunks that summary names as holding the trigger channel; no other chunk is
  decompressed.

This is the same recorded trigger the `mcap` *interface* reads while tailing, and
the same decoder: `json` decodes in every build, `cdr` needs the `ros` cargo
feature. A trigger this run cannot use — an undecodable payload, or a name that
cannot be embedded in a clip pathname — costs that trigger its clip and no more;
the run logs it and cuts the rest.

Both ways of stating the trigger wrongly are refused while the command line is
being read, with the flag at fault named and nothing written: `param` without one
of the three flags it needs, and `mcap` alongside any `--trigger-*` flag. A
recording holding no trigger at all, read under `mcap`, cuts nothing and says so
— a normal, zero-status run that leaves the output directory untouched.

## What a clip carries

Every clip is a standalone MCAP holding the copied messages **and one metadata
record naming what it is**, written under `momentedge.clip`. It is indexed in
the clip's summary and counted in its statistics, so any MCAP tool finds it
without scanning the file:

```console
$ mcap get metadata --name momentedge.clip ./clipped/1738000000000000000_brake-event.mcap
{
  "manifest.version": "1",
  "producer.name": "clipper",
  "producer.mode": "tail",
  "producer.version": "0.1.3",
  "producer.url": "https://github.com/Momentedge/momentedge-clipper",
  "trigger.name": "brake-event",
  "trigger.description": "hard brake over 0.8 g",
  "trigger.anchor_ns": "1738000000000000000",
  "trigger.preroll_ns": "5000000000",
  "trigger.postroll_ns": "5000000000",
  "window.time_source": "log",
  "window.start_ns": "1737999995000000000",
  "window.end_ns": "1738000005000000000",
  "source.path": "/data/record/rosbag2_0.mcap",
  "source.files_planned": "1",
  "source.extents_read": "3",
  "source.bytes_read": "12058624",
  "clip.messages": "4211",
  "clip.short": "false",
  "channel.1.messages": "251",
  "channel.1.first_ns": "1737999995012000000",
  "channel.1.last_ns": "1738000004988000000"
}
```

The keys are flat and dotted, the values all strings:

| Group | What it says |
|---|---|
| `manifest.version` | the record's own schema version; a reader checks it before trusting the rest |
| `producer.*` | the binary, the subcommand that cut the clip (`tail` or `clip`), the cut path's crate version, the project URL |
| `trigger.*` | the trigger that asked: name, description, the anchor it resolved to, preroll and postroll |
| `window.*` | the clock the window lives on (`log`/`publish`) and its inclusive bounds |
| `source.*` | the recording the bytes came from, how many recordings the window was planned over, and how much was read |
| `clip.*` | how many messages were copied, and whether the cut ended short of the window |
| `channel.<id>.*` | per channel of *this clip* — message count and the earliest and latest stamp; a channel the clip holds nothing of has no keys |

Which channels the clip holds at all is the
[`[topics]` selection](#which-topics-a-clip-contains): an excluded topic is
absent from the clip's channels, its schemas, and these keys alike.

A window straddling a bag split publishes one segment per source recording, and
each segment carries its own record naming its own `source.path`.

**An empty clip still explains itself.** Every trigger produces a clip even when
there was nothing to copy, and three keys say which kind of empty it is:

| `source.files_planned` | `clip.short` | What happened |
|---|---|---|
| `0` | `true` | nothing covered the window — the recording never reached it (`--grace-secs` ran out) |
| `0` | `false` | the window fell in a gap between splits — the recording ran past it but held no bytes inside it |
| `>= 1` | `false` | a recording was read, and no message fell inside the window |

`clip.short` is the one fact a clip cannot show from its own contents: a clip
whose last message sits well before the window end looks the same whether the
recorded topics went quiet or the recorder never got there.

The record survives the MCAP CLI's rewrite commands (`compress`, `decompress`,
`sort`, `filter`, `recover`). `mcap merge` refuses two clips by default, since
both carry a record under the same name — pass `--allow-duplicate-metadata`.

## Operational notes

- **Lifecycle.** Ctrl-C (SIGINT/SIGTERM) stops clipper cleanly with exit 0. Any
  internal fault — a dead tail thread, an unrecoverable scan fault — exits
  non-zero so a process supervisor (systemd, …) restarts it.
- **Logs go to stdout.** clipper logs at `info` on stdout; `RUST_LOG`
  raises or lowers that. A run publishes nothing machine-readable on stdout —
  its result is the clips in `--out-dir`, each carrying its own metadata — so
  the stream is free for the output an operator reads first, and discarding
  stderr keeps the logs. Under `--interface ros` the ROS layer's own
  diagnostics are a separate stream: rcutils writes them to stderr unless
  `RCUTILS_LOGGING_USE_STDOUT=1`. Under systemd both streams land in the
  journal; under `ros2 launch` see [`examples/launch`](examples/launch/README.md).
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
| [`custom-mcap-writer/`](examples/custom-mcap-writer/README.md) | a ROS-free MCAP writer that owns `publish_time` as a capture timestamp |
| [`chunked-mcap-writer/`](examples/chunked-mcap-writer/README.md) | a ROS-free MCAP writer producing tailable zstd-compressed chunks |
| [`cu-mcap-record/`](examples/cu-mcap-record/README.md) | a copper (cu29) `CuSinkTask` appending routed outputs to a tailable Recording — ROS-free |

## Documentation

- **[ARCHITECTURE.md](ARCHITECTURE.md)** — the technical overview: thread model,
  tailing mechanics, atomic clip publication, restart/rollover recovery, damage
  tolerance, and the deployment build model.
- **[`crates/clip`](crates/clip)** and **[`crates/tail`](crates/tail)** — the
  library halves of clipper, both buildable with no ROS toolchain anywhere.
  `clip` is the MCAP format layer, the recording index, the cut path and the
  trigger contract — what a program of your own links to cut clips out of a
  recording without running the recorder. `tail` adds what a recording still
  being written needs: discovery, the recording collection, coverage, retention,
  and the waiting a cut does when its window reaches past the last byte on disk.
- **[CLAUDE.md](CLAUDE.md)** and the per-crate notes under
  **[crates/](crates/CLAUDE.md)** — contributor and agent notes: workspace
  layout, build mechanics, and the internal design of each of the three
  crates.
- **[Momentedge/clipper-benchmarks](https://github.com/Momentedge/clipper-benchmarks)**
  — the overhead benchmarks behind [Resource overhead](#resource-overhead): the
  harness, the full report, and the methodology each figure depends on.

## License

Licensed under the [Apache License 2.0](LICENSE).
