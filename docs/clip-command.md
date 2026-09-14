# Cutting clips from a finished recording: `clipper clip`

Cutting one or many windows out of a recording nobody is writing any more —
a bag pulled off a vehicle, an archive, a directory of splits. Every argument
this page mentions is tabulated in
[Configuration](configuration.md#clipper-clip).

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

Each clip lands at `<out-dir>/<clip-id>/` and is the same clip the recorder would
have written from the same recording and window — the window plan, the byte copy,
the [layout and its metadata file](clip-manifest.md) are all the shared path. The
five `--trigger-*` arguments are the fields of a `momentedge_msgs/Trigger`, so
the clip states the same trigger a clip cut from a live topic does; only
`producer.mode` differs, reading `clip` rather than `tail`.

`--out-dir` is created with parents when missing, never required to be empty, and
never cleared: the only thing a run ever adds to its root is a clip directory. A
run clipper accepted creates it whether or not it has a window to put in it, so
the directory is there afterwards even when the recording carried no trigger.

Six things follow from the input being finished:

- **Nothing waits.** No postroll sleep, no wait for coverage. A window reaching
  past the end of the recording is simply short, and the clip's `clip.short`
  field says so.
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
- **A clip that is already there is skipped.** The same recording and the same
  trigger describe the same window, so a window whose clip directory is already
  in `--out-dir` has already been cut. It is left alone with a warning and the
  run goes on — see [below](#when-a-clip-is-already-there).
- **A window that fails stops the run**, with exit 1 and the clips published
  before it left where they are. The recorder cannot stop — it has to be there
  for the next trigger — but a run over a recording with an end can, and a fault
  the disk or the input will raise again is worth reporting before it repeats
  once per remaining trigger — see [below](#when-a-window-fails).

## A bag directory is one collection

A recorder that ran for hours left a directory of splits, and `<recording>` takes
that directory as readily as it takes one file. The splits are read as one
time-ordered collection, so a window straddling a split is cut whole: its
directory holds one file per *contributing* recording, named `<id>_0.mcap`,
`<id>_1.mcap` and so on, where `<id>` is the clip's
[id](clip-manifest.md#the-clip-id) — the same shape the recorder writes when a
window straddles a rollover. A file's number is its position among the files that
hold data, so a window over three splits whose middle recording contributes
nothing yields `_0` and `_1`, where `_1` holds the third recording's data and the
clip's `clip_metadata.yaml` records `files_planned: 3`.

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

## When a recording is refused

Not every `.mcap` carries a summary worth planning a window from. `clipper clip`
decides that from the footer and the summary alone, names the fault, exits 1,
and writes nothing at all — not even the output directory. Over a bag directory
the message names the split that failed, not the directory.

| The message says | The recording is |
|---|---|
| *is N bytes … not an MCAP recording* | too small to hold a footer |
| *does not end with the MCAP magic* | truncated, or copied off a device while it was still being written |
| *footer points at no summary section* | written by a writer configured without a summary |
| *holds no message* | empty — its statistics report a message count of zero |
| *summary indexes no chunk* | written with an unchunked profile |
| *no … chunk index carries a message index* | written by a writer with message indexing disabled |
| *summary indexes a chunk at offset … which the … recording cannot hold* | describing bytes it does not have — a summary from a longer recording, or a file truncated after its summary was written |

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

## When a clip is already there

A finished recording and a trigger describe one window and one copy of its bytes,
so a window whose clip directory is already in `--out-dir` has already been cut.
The run says so and moves on:

```console
$ clipper clip ./record/rosbag2_0.mcap --out-dir ./clipped \
    --trigger-time 1738000000000000000 --preroll 5000000000 --postroll 5000000000
WARN  clip::segment > clip ./clipped/1738000000000000000_5761-7fa4-ab83-75dc is
already there; skipping this window. A clip is written once: an id that is taken means
this window has been cut, or a cut of it died leaving the directory behind. Remove it
to cut the window again
$ echo $?
0
```

The clip on disk is not touched — not a byte of it, and not its
`clip_metadata.yaml`. The check is the directory's own existence, tested before
the window is planned, so a skipped window reads nothing and copies nothing.

**That is what makes a re-run a resume.** A run over a recording with many
embedded triggers that was interrupted half way is finished by running it again:
the clips already there are skipped, the rest are cut, and the run exits 0. A run
that finds every clip already there exits 0 having written nothing, so a pipeline
that re-runs one for safety pays nothing and breaks nothing. Two `clipper clip`
jobs pointed at one output directory are harmless to each other for the same
reason — whichever creates a clip's directory first cuts it, and the other skips
it.

**A directory left by a cut that died is skipped too**, and deliberately. Such a
directory has no `clip_metadata.yaml` — it is incomplete, and no consumer reads
it as a clip — but clipper does not repair or overwrite it, because it is the
only evidence that something went wrong. To cut that window again, remove the
directory.

## When a window fails

A run cuts its windows in order and stops at the first one that fails, with exit
1. Three things are true of what it leaves behind:

- **The clips published before it stay.** A clip is complete the moment its
  `clip_metadata.yaml` is there, and nothing later in the run can unmake one.
- **The window that failed leaves nothing.** Its directory goes with the error,
  so a directory without a `clip_metadata.yaml` in `--out-dir` is the residue of
  a process that *died* and never the leavings of a cut that merely erred. Where
  even the removal fails, the message names the directory to remove by hand.
- **The windows after it are not attempted.** A window fails because of the disk
  or the input, and both outlive the window that met them, so carrying on would
  raise one fault once per remaining trigger and bury the first report under the
  rest.

Fix the cause and run it again: the clips already there are skipped, so the
re-run pays only for the windows that are still missing.

A run that got through every window it had closes with the count it did them in,
which is also how a re-run reports that it found everything already cut:

```console
$ clipper clip ./record --out-dir ./clipped --trigger-source mcap
...
INFO  clipper > 0 clip(s) cut, 7 skipped as already there in ./clipped
$ echo $?
0
```

The statuses a run ends with are [clipper's own](operating.md): **0** for a run
that cut every window it had, skips included; **1** for a window that failed or a
recording [refused](#when-a-recording-is-refused); **2** for a command line or a
configuration file clipper cannot use; and **101** for a clipper bug.

Nothing machine-readable is printed. The result is the output directory's
contents when the process exits, each clip a directory carrying its own
`clip_metadata.yaml`, and the exit status is the verdict. No ROS is involved, so
the ROS-free build cuts these clips as well as the device build does.

## Where a `clip` run's triggers come from: `param` and `mcap`

Both subcommands answer this with the same key, `--trigger-source`, and the
values differ only where the recording's end makes them. A finished recording
holds no live topic to subscribe to, so a `clip` run's triggers come either from
the command line or from the recording itself. Exactly one source is active per
run.

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

`mcap` reads the same recorded trigger
[`clipper tail --trigger-source mcap`](triggers-and-time.md#two-ways-in-ros-and-mcap)
reads out of the recording it is still following, with the same decoder: `json`
decodes in every build, `cdr` needs the `ros` cargo feature. That is what one key
across both subcommands buys — the trigger stream that cut clips on the vehicle
cuts the same clips from the bag afterwards. A trigger this run cannot use — an
undecodable payload, or a name past the 128-byte bound — costs that trigger its
clip and no more; the run logs it and cuts the rest.

Both ways of stating the trigger wrongly are refused while the command line is
being read, with the flag at fault named and nothing written: `param` without one
of the three flags it needs, and `mcap` alongside any `--trigger-*` flag. A
recording holding no trigger at all, read under `mcap`, cuts nothing and says so
— a normal, zero-status run, after which `--out-dir` is there and empty like any
other run's.

**`ros` is not offered here.** It is a live subscription on a ROS node, and a
recording nobody is writing has no live topic to carry a trigger and nothing
waiting on a `Recorded` publish. Asking for it is refused the same way, naming
the value and listing what this subcommand does take:

```console
$ clipper clip ./record --out-dir ./clipped --trigger-source ros
error: invalid value 'ros' for '--trigger-source <TRIGGER_SOURCE>'
  [possible values: mcap, param]
```

