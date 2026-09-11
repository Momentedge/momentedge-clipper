# Configuration

Every flag, environment variable and configuration-file key clipper accepts,
and the rule that decides which one wins. For what clipper is and how to run
it, start at the [README](../README.md).

`clipper <mode> --help` lists the flags of the mode in front of you, and
`clipper <mode> --print-config` prints
[what a run actually resolved](#the-effective-configuration). A flag handed to
the bare `clipper` is refused, with a message naming the mode that owns it;
`clipper --version` prints the version.

## The four layers

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
column below reads three ways: **yes**, either configuration file may set the
key; **no**, the system file alone may (a per-run file naming it is refused);
and **—**, no configuration file may name it at all — that setting has no
`[settings]` key, and a file naming one fails the run. The column is read per
table: the line is drawn by the mode that is running, so one key name may answer
differently under each. See
[The configuration file](#the-configuration-file).

Three flags belong to every mode and to none of the tables below, because they
are about the configuration rather than in it: `--config PATH` names the per-run
file, `--system-config PATH` moves the system file, and `--print-config` prints
[the effective configuration](#the-effective-configuration) and exits.

`--trigger-source` is the one setting in the tables below that carries a **—**.
It says how *this process was launched* rather than what the machine is
configured with, which puts it in the same category as those three flags: it is
the flag and `MOMENTEDGE_TRIGGER_SOURCE`, and nothing else. A device sets it
once, in the unit file that already carries the rest of the invocation. A
configuration file naming `trigger_source` fails the run at startup, naming the
file and the key, exactly as a misspelling does.

## `clipper tail`

Every flag is optional — `clipper tail` runs with no further arguments.

| Flag | Env var | Type | Default | Per-run | Meaning |
|---|---|---|---|---|---|
| `--record-dir` | `MOMENTEDGE_RECORD_DIR` | path | `./record` | no | bag directory of the continuous recording to tail |
| `--out-dir` | `MOMENTEDGE_OUT_DIR` | path | `./clipped` | yes | where finished clips are written |
| `--trigger-source` | `MOMENTEDGE_TRIGGER_SOURCE` | `ros` \| `mcap` | `ros` (`mcap` in a ROS-free build) | — | where triggers come from, and with them how completion is signalled (see [Two ways in](triggers-and-time.md#two-ways-in-ros-and-mcap)) |
| `--time-source` | `MOMENTEDGE_TIME_SOURCE` | `log` \| `publish` | `log` | no | clock domain the clip window lives in (see [Time source](triggers-and-time.md#time-source-log-or-publish)) |
| `--grace-secs` | `MOMENTEDGE_GRACE_SECS` | integer, seconds | `30` | yes | how long past the window end to wait for the recording to cover it before cutting from what is on disk |
| `--clip-compression` | `MOMENTEDGE_CLIP_COMPRESSION` | `none` \| `lz4` \| `zstd` | `zstd` | yes | codec for written clips (`zstd` writes the smallest) |
| `--extract-parallelism` | `MOMENTEDGE_EXTRACT_PARALLELISM` | integer | `1` | no | concurrent clip copies (1 = one at a time, FIFO) |
| `--watch-old-files-duration` | `MOMENTEDGE_WATCH_OLD_FILES_DURATION` | integer, seconds | `600` | no | seconds to keep a finished (split/restart) recording indexed so a trigger's preroll can still reach into it; set comfortably above the largest preroll any trigger will request |
| `--delete-old-files` | `MOMENTEDGE_DELETE_OLD_FILES` | boolean | `false` | no | also unlink an expired `.mcap` from disk when it is pruned (off by default — clipper does not own the recordings) |

`--grace-secs` must exceed the recorder's flush latency: near zero for an
unchunked `fastwrite` recording, roughly one chunk-fill for a chunked profile.
The [`examples/continuous`](../examples/continuous/README.md) guide explains the
recorder's latency-vs-size knobs and how to size `--grace-secs` against them.

## `clipper clip`

The positional `<recording>` is required; `--out-dir` is required unless a
configuration file or the environment supplies it. Which of the `--trigger-*`
arguments apply depends on `--trigger-source` — see
[Cutting clips from a finished recording](clip-command.md).

| Argument | Env var | Type | Default | Per-run | Meaning |
|---|---|---|---|---|---|
| `<recording>` | `MOMENTEDGE_RECORDING` | path | — | yes | the finished recording to cut from (positional): one `.mcap`, or a bag directory of splits |
| `--out-dir` | `MOMENTEDGE_OUT_DIR` | path | — | yes | where the clips are written |
| `--trigger-source` | `MOMENTEDGE_TRIGGER_SOURCE` | `param` \| `mcap` | `param` | — | where this run's triggers come from (see [Where a run's triggers come from](clip-command.md#where-a-clip-runs-triggers-come-from-param-and-mcap)) |
| `--trigger-time` | `MOMENTEDGE_TRIGGER_TIME` | integer, ns | — | yes | the instant the window centres on, in nanoseconds since the epoch (`param` only, required) |
| `--preroll` | `MOMENTEDGE_PREROLL` | integer, ns | — | yes | nanoseconds before that instant to include (`param` only, required) |
| `--postroll` | `MOMENTEDGE_POSTROLL` | integer, ns | — | yes | nanoseconds after it to include (`param` only, required) |
| `--trigger-name` | `MOMENTEDGE_TRIGGER_NAME` | string | `clip` | yes | the trigger's name, which also names the clip file (`param` only) |
| `--trigger-description` | `MOMENTEDGE_TRIGGER_DESCRIPTION` | string | *(empty)* | yes | the trigger's description, carried into the manifest (`param` only) |

## The configuration file

Both configuration files are TOML and share one schema: a `[settings]` table
whose keys are the modes' settings, and a `[topics]` table deciding
[which topics a clip contains](#which-topics-a-clip-contains). One loader reads
them for every mode, so `clipper tail` and `clipper clip` take the same file — a
device's `/etc/momentedge/clipper.toml` can describe the recorder and still be
the file a `clipper clip` run on that machine reads. A key belonging to the
*other* mode is inert: it matches no argument of this run and does nothing. Only
a key **no** mode has is a misspelling, and that fails the run.

```toml
# /etc/momentedge/clipper.toml — the system file: this machine's recorder
[settings]
record_dir = "/data/record"
out_dir = "/data/clipped"
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
file may set only the keys marked *yes* in the **Per-run** column of the running
mode's table above and of [`[topics]`](#which-topics-a-clip-contains) below. The
line is what the key describes: a key naming the machine the recorder runs on or
the resources it may spend there — where the recording lives, how triggers arrive,
which clock windows live on, how much IO and memory the tail may take, and
whether clipper may unlink a recording — is the system's alone. A key
describing one job — where its clips go, how long it waits, how they are
compressed, which window, and which topics — is the per-run file's to set.

A per-run file that sets a system-only key is **refused**: the key is named in a
warning and in `--print-config`, the system file's value (or the built-in
default) stands, and the run continues. The line is drawn per mode, not per
process — a key can be one mode's machine setting and another mode's per-job
one — so it is the table of the mode about to run that decides, and a key that
mode does not have is not refused at all, only inert. The environment and the
flags are not scoped this way — whoever launches the process already commands
both.

The **—** rows are outside that line rather than at one end of it: no file may
name `trigger_source`, so there is nothing for either file to be refused. A file
that names it fails the run at startup the way an unknown key does, whichever
mode is running.

## Upgrading a deployment configured for an earlier release

Releases before this one spelled the recorder's trigger source `interface`, with
the same two values, and let a configuration file set it. **There is no
compatibility alias and no migration path**: every configuration file, unit file
and environment naming the old spelling has to be edited *before* the new
package lands. Two of the three ways of naming it stop the run; the third is
read by nothing at all.

The trigger source is a flag and an environment variable alone, and no
configuration file may set it, so a file is the one place the replacement cannot
go: put it in the unit file that launches clipper, as `--trigger-source` or as
`MOMENTEDGE_TRIGGER_SOURCE`.

**Set it for that unit, not for the machine.** `MOMENTEDGE_TRIGGER_SOURCE`
reaches every subcommand, and the two take different values, so `ros` exported
from a login profile or a shared `EnvironmentFile=` is handed to `clipper clip`
as well — which takes `mcap` and `param`, and refuses to start. It belongs in
the recorder unit's own `Environment=`, or on its `ExecStart` line as
`--trigger-source`, where it reaches the one process it is about.

| Where it was named | What happens | Replace it with |
|---|---|---|
| `interface` in a configuration file | the loader fails the run, naming the file and the key and listing the keys each mode does have; exit status 2 | `--trigger-source ros` on the recorder's `ExecStart`, or `MOMENTEDGE_TRIGGER_SOURCE=ros` in that unit's own environment — **not** `trigger_source` in the file, which fails the run the same way |
| `--interface ros` on the command line | clap refuses it as an unexpected argument; exit status 2 | `--trigger-source ros` |
| `MOMENTEDGE_INTERFACE` in the environment | nothing reads it, so the run starts on whatever the remaining layers decide — silently, and possibly on the wrong source | `MOMENTEDGE_TRIGGER_SOURCE=ros`, in the recorder unit's environment rather than the machine's |

A device whose `/etc/momentedge/clipper.toml` still carries `interface = "ros"`
does not come up, and neither does one that carries `trigger_source = "ros"`; a
systemd unit still passing `--interface ros` restart-loops. That is deliberate:
an operator meets the fault at the first start after the upgrade, with the key
named, rather than discovering weeks later that a run has been taking its
triggers from somewhere else.

## Which topics a clip contains

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
[manifest](clip-manifest.md). Selection only ever narrows a clip: a topic
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

## The effective configuration

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
    out_dir                  = ./clips/night-run      <- per-run file ./night-run.toml
    record_dir               = /data/record           <- system file /etc/momentedge/clipper.toml
    time_source              = log                    <- built-in default
    trigger_source           = ros                    <- built-in default
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
so what the report prints is what the run uses. Every setting the run uses is
listed, `trigger_source` among them — its origin reads `flag`, `environment` or
`built-in default` and never a file, since no file layer can decide it. A
per-run file's refused key is listed under the report, naming the key and the
file it came from.

