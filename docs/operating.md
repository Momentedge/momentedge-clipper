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
  is refused: clipper logs `trigger handling failed: … extent framing
  inconsistent with the tail's scan`, publishes no clip for that trigger, and
  goes on refusing every window that reads the same region — up to 4 MiB of
  recording, data written after the damage included. So a healthy process that
  has quietly stopped producing clips is worth grepping the log for that line;
  restarting the recording (a rollover or a fresh `ros2 bag record`) clears it.
  Damage *ahead* of the scan is the other case, and there clipper exits non-zero
  for the supervisor rather than limping on.

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
