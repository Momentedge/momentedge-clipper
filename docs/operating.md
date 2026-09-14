# Operating clipper

What a deployed `clipper tail` does about shutdown, logging, retention and
overload. For the flags behind any of it, see
[Configuration](configuration.md).

## In normal operation

- **Lifecycle.** Ctrl-C (SIGINT/SIGTERM) stops clipper cleanly with exit **0**.
  Any internal fault — a dead tail thread, an unrecoverable scan fault — exits
  **1** so a process supervisor (systemd, …) restarts it; a command line or
  configuration file clipper cannot use exits **2** before the recorder starts,
  which under `Restart=on-failure` is a restart loop until the file or the unit
  is fixed; and a panic — a clipper bug, its message in the log — exits **101**.
  Those four are the statuses a run ends with: clipper ends its own process on
  the status it chose, so a signal death in the journal means something outside
  clipper killed it.
- **Logs go to stdout.** clipper logs at `info` on stdout; `RUST_LOG`
  raises or lowers that. A run publishes nothing machine-readable on stdout —
  its result is the clips in `--out-dir`, each carrying its own metadata — so
  the stream is free for the output an operator reads first, and discarding
  stderr keeps the logs. Under `--trigger-source ros` the ROS layer's own
  diagnostics are a separate stream: rcutils writes them to stderr unless
  `RCUTILS_LOGGING_USE_STDOUT=1`. Under systemd both streams land in the
  journal; under `ros2 launch` see [`examples/launch`](../examples/launch/README.md).
- **No startup back-indexing.** clipper recovers only rollovers it observed
  during its own run. A recording already on disk before clipper started
  contributes nothing to a trigger fired afterwards.
- **Retention is the recorder's job.** The continuous recording grows until you
  stop or split it; clipper never prunes the file it is tailing. See
  [`examples/split-bags`](../examples/split-bags/README.md) for bounding the
  recording; the output side is [`--out-dir`'s](#what---out-dir-holds).
- **Concurrency cap.** Up to 16 triggers are handled at once; a trigger arriving
  while all 16 slots are busy is rejected with a logged error and produces no
  clip and no `Recorded` announcement. Automation waiting on the announcement
  should treat its absence — and the logged error — as a dropped trigger.
- **A damaged recording need not stop the process.** clipper never re-reads
  bytes it has already tailed, so damage appearing behind its scan leaves it
  running and surfaces when a clip is cut. Damage inside a message's payload is
  copied into the clip as recorded. Damage that breaks a record's length prefix
  costs the clips that read that region, and clipper says so — see
  [A recording that stops producing clips](#a-recording-that-stops-producing-clips).
  Damage *ahead* of the scan is the other case, and there clipper exits 1 for
  the supervisor rather than limping on.

## What `--out-dir` holds

A run's whole result is this directory, and its contents are one rule: **every
entry in it is a clip directory, and a clip directory is complete when it holds
`clip_metadata.yaml`.** The fields of that document, and the id the directory is
named by, are [What a clip carries](clip-manifest.md).

```
/data/clipped/
  1726300000000000000_fc43-6475-ade8-4730/    a complete clip
    1726300000000000000_fc43-6475-ade8-4730_0.mcap
    clip_metadata.yaml
  1726300180000000000_08b1-9d2a-44ff-c017/    incomplete: a cut running, or residue
    1726300180000000000_08b1-9d2a-44ff-c017_0.mcap
```

- **Point a sync tool at it with no exclude list.** clipper writes nothing into
  the root but clip directories — no staging area, no lock, no sidecar — so
  rsync, syncthing or an upload agent needs no rule about what to ignore.
- **Filter on the metadata file, never on an MCAP file appearing.** It is
  written after every `<id>_N.mcap` beside it is durable, and the directories
  are fsynced after it, so a clip that answers the rule survives power loss and
  is whole. A directory without it is a cut in progress or crash residue, and
  the two look alike from outside.
- **clipper creates the directory and never clears it.** `clipper tail` creates
  `--out-dir` with parents at startup and refuses to start if it cannot, so a
  run that cuts nothing still leaves the directory it was pointed at. It is
  never required to be empty, and foreign files already in it are left exactly
  as they were found. Pruning old clips is yours to schedule;
  `--delete-old-files` is about *recordings* and touches nothing here.

### What a crashed cut leaves, and what happens next

A cut that fails for an ordinary reason — a full disk, an IO error — removes its
own directory before reporting. So a directory with no `clip_metadata.yaml` is
either a cut still in flight or, once the process is gone, evidence that it
**died** mid-cut: SIGKILL, an OOM kill, power loss. It is never the leavings of
a cut that merely erred. (Where even that removal fails, the error names the
directory to remove by hand.)

That residue stays, and it is meant to:

- **Nothing repairs or overwrites it.** A later trigger resolving to the same id
  is **skipped** with a warning naming the directory, exactly as it would be for
  a complete clip — the claim reads the directory's existence and never its
  contents. The residue is the evidence that something died, and clipper does
  not destroy evidence to tidy up.
- **Re-cutting that window means removing the directory first.** Until then the
  id is taken and every trigger for it is skipped. Everything else keeps
  working: the recorder goes on cutting every other window, and a restart cuts
  clips normally.
- **No consumer mistakes it for a clip.** It has no metadata file, so a pipeline
  filtering on that file passes it over without knowing anything about crashes.

```
WARN  clip::segment > clip /data/clipped/1726300180000000000_08b1-9d2a-44ff-c017 is
already there; skipping this window. A clip is written once: an id that is taken means
this window has been cut, or a cut of it died leaving the directory behind. Remove it
to cut the window again
```

A skipped window publishes no `Recorded`, since nothing was recorded — the
warning is its whole trace. Automation watching the announcement should read a
skip the way it reads any missing clip.

### Two processes on one output directory

They cannot corrupt each other's clips, and that is a property of the design
rather than a feature to rely on. A clip's directory is claimed with a single
`mkdir`, which the kernel makes atomic: whichever process creates it cuts the
clip, and the other is told the id is taken and skips the window. There is no
lock file, no staging area either can wipe, and no startup step that clears
anything.

What that buys is that a scheduling accident — two `clipper clip` jobs over one
recording, or a supervisor briefly running two recorders — loses no clip and
damages none. What it does not buy is a supported deployment: nothing
coordinates the two beyond that claim, so they duplicate every scan and every
window plan, and their logs interleave. Run one process per output directory.

## A recording that stops producing clips

A run of stray bytes across a record's length prefix — a bad block, a filesystem
hiccup, anything that rewrites a byte `ros2 bag record` already wrote — leaves
the recording unreadable from that record onward. clipper refuses to build a clip
out of bytes whose framing disagrees with what it indexed, and says so the first
time it costs a clip:

```
ERROR tail::handler > recording /data/bags/rec_0.mcap changed under the tail after it
was indexed: record at extent offset 19773 declares 18446744073709551615 B; extent
framing inconsistent with the tail's scan. Every clip whose window plans the extent
at 16777216 is refused with it — up to 4 MiB of recording, data written after the
damage included — for as long as this recording is tailed, and this recorder goes on
cutting every window that reads elsewhere. Rolling the recording over (a bag split,
or restarting `ros2 bag record`) is what clears it; restarting clipper does not — a
fresh scan meets these bytes ahead of it and exits on the scan-fault budget instead.
```

That announcement is made **once per recording**. Every trigger refused after it
carries a running count instead, so the tally is what to watch:

```
ERROR clipper > trigger handling failed: clip 7 refused against /data/bags/rec_0.mcap
since its framing desynced: record at extent offset 19773 declares …
```

What to do about it:

- **Roll the recording over.** A bag split or a fresh `ros2 bag record` puts an
  undamaged file under the tail, and clipper cuts from it normally. Running the
  recorder with `--max-bag-size`/`--max-bag-duration` (see
  [`examples/split-bags`](../examples/split-bags/README.md)) bounds how long any
  such damage can cost clips in the first place.
- **Do not restart clipper.** A fresh process indexes that recording from the
  start, so the damaged bytes are then *ahead* of its scan — the fatal case — and
  it exits within seconds. Where the recording is not being split there is no
  later file to adopt instead, so a supervisor restarts it into a loop that
  publishes nothing at all. A running clipper still cuts every window that reads
  outside the damaged region; a looping one cuts nothing.
- **The clips already published are unaffected**, and so is the recording outside
  the refused region. clipper runs no repair; `mcap recover` salvages the file
  offline once the recorder has moved on from it.

Only file damage is counted this way. A cut that fails for another reason — a
full disk, an IO error, an output failure — reports itself as that and counts
toward nothing.

## Tuning under load

Two settings trade CPU against something else, and both matter on a busy board.

**Clip compression is the one thing that costs real CPU.** On a 410 MB window
`--clip-compression zstd` turns roughly 0.7 s of copying into just over 5 s, for
a clip about 14 % smaller. It does not affect the tailing cost at all: a run
that cuts nothing never compresses anything.

**On a six-core board, leaving `--extract-parallelism` at its default is what
keeps the recorder lossless.** Copying ten windows across every core costs the
recorder a handful of dropped messages on a Jetson Orin Nano, and dozens when
the board is also busy, while one worker at a time drops none. The eight-core
Orin NX drops none in any configuration. Raising the setting trades that against
wall clock rather than reducing the work: at one worker per core the same ten
windows take 432 % of one core and a third of the time.

Both figures come from
[Momentedge/clipper-benchmarks](https://github.com/Momentedge/clipper-benchmarks),
which carries the methodology and the per-configuration numbers.
