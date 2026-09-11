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

Each clip lands at `<out-dir>/<anchor-ns>_<trigger-name>.mcap` and is the same
file the recorder would have written from the same recording and window — the
window plan, the byte copy, the [manifest](clip-manifest.md) and the atomic
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

## A bag directory is one collection

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

## When a recording is refused

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

## When a clip is already there

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
undecodable payload, or a name that cannot be embedded in a clip pathname — costs
that trigger its clip and no more; the run logs it and cuts the rest.

Both ways of stating the trigger wrongly are refused while the command line is
being read, with the flag at fault named and nothing written: `param` without one
of the three flags it needs, and `mcap` alongside any `--trigger-*` flag. A
recording holding no trigger at all, read under `mcap`, cuts nothing and says so
— a normal, zero-status run that leaves the output directory untouched.

**`ros` is not offered here.** It is a live subscription on a ROS node, and a
recording nobody is writing has no live topic to carry a trigger and nothing
waiting on a `Recorded` publish. Asking for it is refused the same way, naming
the value and listing what this subcommand does take:

```console
$ clipper clip ./record --out-dir ./clipped --trigger-source ros
error: invalid value 'ros' for '--trigger-source <TRIGGER_SOURCE>'
  [possible values: mcap, param]
```

