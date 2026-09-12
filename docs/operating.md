# Operating clipper

What a deployed `clipper tail` does about shutdown, logging, retention and
overload. For the flags behind any of it, see
[Configuration](configuration.md).

## In normal operation

- **Lifecycle.** Ctrl-C (SIGINT/SIGTERM) stops clipper cleanly with exit 0. Any
  internal fault — a dead tail thread, an unrecoverable scan fault — exits
  non-zero so a process supervisor (systemd, …) restarts it.
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
  recording, and prune `./clipped` on your own schedule.
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
  Damage *ahead* of the scan is the other case, and there clipper exits non-zero
  for the supervisor rather than limping on.

## A recording that stops producing clips

A run of stray bytes across a record's length prefix — a bad block, a filesystem
hiccup, anything that rewrites a byte `ros2 bag record` already wrote — leaves
the recording unreadable from that record onward. clipper refuses to build a clip
out of bytes whose framing disagrees with what it indexed, and says so the first
time it costs a clip:

```
ERROR clipper > recording /data/bags/rec_0.mcap changed under the tail after it was
indexed: record at extent offset 19773 declares 18446744073709551615 B; extent
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
