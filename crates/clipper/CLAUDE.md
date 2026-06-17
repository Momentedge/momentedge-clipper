# clipper

A *triggered* clip recorder over **one continuous MCAP file**. It keeps the
single growing recording open and **tails it**, so a clip can be cut as soon as
the data is physically on disk: clip latency is bounded by the recorder's
write-through latency. Built on [r2r](https://github.com/sequenceplanner/r2r)
over plain OS threads — there is no async runtime.

## The pipeline it sits in

```
ros2 bag record (scripts/record-continuous.sh) ──▶ ./record-cont/<bag>_0.mcap   (one growing file)
        ▲ kept open + tailed (incremental scan, persistent offsets)
clipper ◀── /events/momentedge/trigger ── trigger-pub (or any publisher)
        │ cuts [trigger_time-preroll, trigger_time+postroll]
        ├──▶ ./triggered-cont/<trigger_ns>_<name>.mcap
        └──▶ /events/momentedge/recorded
```

`record-continuous.sh` is a standalone `ros2 bag record` — this binary never
spawns it. The two communicate only through the file: there is no split event
(rosbag2 publishes none for an unsplit bag) and none is needed.

## Why tailing a live MCAP is sound (`tail.rs`)

Two properties carry the whole design:

1. **The MCAP writer is append-only while recording.** Bytes below the current
   end of file never change; the summary/footer is appended only at close. So
   everything behind the last complete record is immutable.
2. **Every record is length-prefixed** (1-byte opcode + u64le length). A record
   whose declared extent runs past the current file length is still being
   appended — the scan stops there and resumes on a later pass. An in-progress
   file is indistinguishable from a crash-truncated one, which MCAP readers are
   designed to tolerate.

The tail keeps the recording open and never re-reads consumed bytes. Each pass
yields three artefacts, shared with the per-trigger handlers:

- **Extent index** — contiguous byte ranges (closed at 4 MiB) with the min/max
  message `effective_time` (capture time) they hold. Extraction reads only the
  extents overlapping its window; the overlap test is exact (real min/max, not a
  heuristic), so no in-window message can be missed.
- **Schema/channel registry** — owned copies of every `Schema`/`Channel`
  record, keyed by channel ID (unique within one continuous file). The MCAP
  spec puts a Schema before any Channel referencing it, so resolution always
  succeeds on conformant files; an inverted (invalid) file degrades the
  channel to schemaless rather than erroring. Chunked recordings carry these
  records *inside* chunks, so chunks are decompressed during the tail
  (zstd, lz4 and uncompressed chunks all work — mcap's default features);
  the default fastwrite profile is unchunked and skips that cost entirely.
- **Coverage watch** — a `Watch<Coverage>` (`Mutex` + `Condvar`, `src/watch.rs`)
  holding the highest `effective_time` (capture time) on disk plus an `ended`
  flag (DataEnd/Footer scanned). Sound because messages land in the file in
  (approximately) non-decreasing `effective_time` order — the monotonicity
  precondition coverage relies on holds for capture time under the assumption
  that all channels share roughly one transport delay, so a single window cuts
  them all at ~one instant.

Per top-level `Message` record the tail reads the 22-byte fixed header (channel
id, sequence, `log_time`, `publish_time`) plus — on a stamped channel — up to 12
leading payload bytes to decode the capture stamp; the rest of each body is
first touched at extraction. Chunked recordings decode the stamp from the
chunk-interior message payload. The same "decode only the timestamp" discipline
as the rest of the workspace, applied to file tailing.

**Damage in the recording is survivable up to the point of framing desync.**
The scan tolerates localized damage the same way extraction does (see "The copy
is direct (`clip.rs`)"): a chunk that fails to decompress, fails its CRC, or
carries an unsupported compression algorithm contributes nothing — its interior
is absorbed into a throwaway sub-delta merged into the live state only once the
chunk iterates cleanly, so a chunk whose CRC fails mid-iteration leaves no
registry entry and no time folded into coverage or extent bounds, and coverage
never claims data the cut would silently drop. An unparseable top-level
`Schema`/`Channel` record (spec-legal bytes the parser rejects, e.g. an
invalid-UTF-8 name) is warned and skipped. Both keep the framing intact — the
length prefix is self-consistent — so the record is consumed and the scan keeps
indexing the records behind it.

A **framing** fault has no resync point and so cannot be skipped: a record whose
declared length exceeds `MAX_RECORD_LEN`, or an IO error reading a record's
header or body. The scan stops at the faulted record, having already applied the
delta it accumulated before it, and reports the fault with that record's offset.
Everything before the fault therefore stays plannable — clips cut from the
pre-fault index still extract and announce. The tail then retries from exactly
that offset, never re-attaching and never rescanning from scratch (the index is
attached once per recording, so a retry that resumed earlier would re-extend an
extent the open delta already spans and underflow). Retries are bounded by
`MAX_SCAN_FAULTS` consecutive faults with backoff escalating from `DISCOVER_POLL`
toward `SCAN_BACKOFF_CAP`, slept in `DISCOVER_POLL` increments so a recorder
restart mid-backoff is noticed within one increment and taken as recovery; a
single fault-free pass resets the count, so transient trouble that clears never
accumulates. Exhausting the budget is fatal: every retry in a row ended in a
fault — usually the same stuck byte — so `run()` returns the fault (named with
the path, offset, and attempt count), `supervise()` carries it out, and the
process exits non-zero for a supervisor to restart. Limping on would degrade every clip to a
grace-timeout cut with no other signal, which is exactly what the fail-fast
budget exists to prevent.

**Recorder restarts:** the recording is discovered as the newest `*.mcap`
under `record_dir`; when that path stops resolving to the tailed inode (the
record script wipes the bag dir on restart), the index resets and the new file
is tailed from scratch. This is the one path that re-attaches and resets
everything — distinct from a scan fault, which keeps the index. Clips are cut
from the most recent recording only; a previous file's data is never recovered
into a clip, even when that file still exists on disk. Recovery across a
recording replacement is an explicit non-feature, tracked as out of scope in
beads `clipper-gl2`. A `NotFound` when opening the discovered path is
the same race resolved the same way: the file vanished between discovery and
open, so it is treated as a replacement and re-discovery loops. A magic
mismatch stays fatal — an append-only file whose first eight bytes are wrong
can never become a valid MCAP. In-flight extractions hold their own `Arc<File>`
and finish safely against the deleted inode.

## Per-trigger flow (`handle_trigger`)

Each admitted `momentedge_msgs/Trigger` is handled on its own thread, so
overlapping windows are cut concurrently against the one shared tail. Admission
is bounded: at most `MAX_ACTIVE_TRIGGERS` (16) handlers may be active at once,
and a trigger that arrives while all of them are is rejected — logged with
`error!` and otherwise ignored: no handler runs, no clip is extracted, and no
`momentedge_msgs/Recorded` is published.

1. **Wait out the postroll.** Sleep until the system clock passes
   `trigger_time + postroll`.
2. **Wait for coverage.** Block on the coverage watch until the recording
   covers the window end — the `effective_time` (capture time) high-water has
   reached the window end, so a message captured at/after the window end is on
   disk (or the recording ended). A grace timeout
   (`grace_secs`, default 30 s) bounds the wait; on timeout the clip is cut
   from what exists, with a warning. The grace must exceed the recorder's
   flush latency: near zero for the fastwrite profile, roughly one chunk fill
   (chunk size / aggregate data rate) for chunked profiles.
3. **Extract** via the extraction worker pool (`extract_parallelism` threads,
   default 1): the handler queues an `ExtractJob` (window bounds, output path,
   bounded reply channel) on the shared FIFO channel and blocks on the reply.
   A worker dequeues the job, snapshots `plan_window` at copy start (so a job
   that waited in the queue still cuts from the freshest index), runs
   `clip::extract_clip`, and sends the result back. With the default single
   worker the bulk copies serialize FIFO; the waits in steps 1–2 are always
   concurrent.
4. **Announce** by publishing `momentedge_msgs/Recorded` — only after the
   extraction has moved the clip into `out_dir` and fsynced that directory, so
   the announce names an already-moved, crash-durable file.

## The copy is direct (`clip.rs`)

Extraction reads each planned extent with `read_at` (no seek state shared with
the tail) and walks its records with **its own opcode + length framing** — the
same walk the tail performed to build the extent, so the boundaries are known
to tile. Owning the framing makes extraction damage-tolerant the way the MCAP
format is designed to be (length prefixes delimit every record; chunk CRCs
exist to detect and discard a damaged chunk — the format the official
`mcap recover` tool salvages by): a record whose body fails to parse, or a
message on a channel the recording never declared, is skipped with an error
log; a chunk that fails decompression, CRC, or interior parsing is dropped
whole — its messages are buffered and written only once the chunk iterates
cleanly (`mcap::read::ChunkReader` verifies the CRC at the end of iteration),
since a bad CRC cannot say which of the chunk's bytes are lying. The mcap
library readers are unsuitable for this walk: they halt at the first error,
and the `LinearReader::sans_magic` constructor additionally caps every
record — including chunk-interior records after decompression — at the slice
length, failing any conformant chunk whose contents out-compress it. Messages
whose `effective_time` (capture time) falls in the inclusive window are written
through with their **raw serialized bytes** (`write_to_known_channel`), their
original `log_time` and `publish_time` preserved; CDR bodies are never decoded.

Output channels are registered from the registry per source channel ID and
cached; `mcap::Writer` deduplicates schemas/channels by content. The clip ends
with `Writer::finish()`, which writes the summary section, footer and closing
magic — every clip is a complete, standalone MCAP file
(`mcap::MessageStream` over a clip is the validity check the
unit tests use).

**Two-staged atomic publication.** `extract_clip` composes two stages so the
output directory only ever holds finished clips. `stage_clip` assembles the
clip in a `.capturing` subdirectory of `out_dir`, `Writer::finish()`es it, and
`sync_all`s the file; `publish_clip` then moves it into `out_dir` under the
desired name. The capturing area is a *subdirectory* of the output directory
rather than a sibling so the two always share a filesystem — the move is a true
atomic link, never a cross-device copy. The move is `hard_link` + unlink of the
staged path, not `rename`: a duplicate trigger (same stamp and name) must not
clobber the earlier clip, and `rename` replaces an existing destination
silently, whereas `hard_link` is equally atomic but fails with `AlreadyExists`,
which the `_<n>`-suffix retry (`with_suffix_retry`, cap 1000) resolves against
the *desired* final name. The link is the commit point: once it succeeds the
output directory holds a complete clip (the staged file was already fsynced), so
the staged name is unlinked and `out_dir` itself is fsynced to make the new
directory entry crash-durable. A `StagedClip` is `#[must_use]` and its `Drop`
unlinks an unpublished staged file, so an early return or panic between the
stages — or a failed publish — strands nothing in `.capturing` and never
reaches `out_dir`. The capturing-dir name may carry its own `_<n>` suffix to
avoid colliding with a concurrent stage, independent of the final name a
duplicate trigger resolves to at publish. The one leftover `Drop` cannot
reclaim is a crash *between* the publish link and the staged-file unlink, which
strands a stale link in `.capturing` (harmless — only `out_dir` is observed);
`reset_capturing_dir`, called once at startup, deletes and recreates
`.capturing` (and ensures `out_dir` exists) so that clutter never outlives a
single run. Failing that reset is fatal: a recorder that cannot prepare its
output directory must not start.

Extraction degrades over localized damage and aborts on anything else.
Skipped records and dropped chunks are counted in `ClipStats`
(`records_skipped` / `chunks_dropped`) and surfaced as a warning by the
trigger handler, so a degraded clip is announced but never silent. What stays
fatal — the recording truncated under the plan, extent framing that no longer
matches the tail's scan (the bytes changed since the scan, so there is no
boundary to resync at), and output IO errors — confines its cleanup to the
capturing directory, so the output directory never holds a footer-less file
that could be mistaken for a clip. A *deleted* recording is not an error — the
plan's `Arc<File>` keeps the inode readable, so extractions in flight across a
recorder restart still complete.

**Detection limit:** the leniency applies to damage loud enough to break
parsing or a CRC. The default fastwrite profile is unchunked and carries no
CRCs, so corruption inside a message *body* that leaves the framing and the
22-byte message header intact is invisible to every MCAP reader and is copied
into clips as-is — only a CDR decode downstream would notice.

## Time base

The clip window is cut on **capture time** — the ROS2 message header stamp —
rather than the MCAP `log_time`. The window spans `[trigger_time - preroll,
trigger_time + postroll]`, with `trigger_time` treated as being in the same
clock domain as the capture stamp. One merged timeline, `effective_time`, drives message
selection (`clip.rs`), the coverage high-water mark, and the extent time-bounds
(`tail.rs`): `effective_time` is the decoded header capture stamp for a stamped
channel, and the MCAP `log_time` otherwise.

A channel is stamped when a schema gate (`schema_is_stamped`) finds its registry
schema's top-level first field to be `std_msgs/Header` or
`builtin_interfaces/Time`. For those channels the stamp is read straight from
the CDR payload — `sec` (int32) at offset 4, `nanosec` (uint32) at offset 8,
after the 4-byte encapsulation header. A decoded value that is implausible
(`sec < 0`, `nanosec >= 1e9`) or the unset `(0, 0)` sentinel is rejected, and
that message falls back to `log_time`. Channels with no leading stamp
(`tf2_msgs/TFMessage`, `std_msgs` primitives, …) are recorded and clipped fine —
they simply window on `log_time`. If no channel in the recording is stamped, the
whole thing windows on `log_time` end to end.

`log_time` is copied through to the output MCAP untouched: only message
*selection* keys on capture time; the written records keep their original
`log_time` and `publish_time`. `log_time`, the trigger stamp, and the wait clock
are all nanoseconds on the system clock — this assumes the default (no
`use_sim_time`). `time_to_ns` flattens a `builtin_interfaces/Time` stamp to that
scale (`sec * 1e9 + nanosec`).

## Topics and types

| Direction | Topic | Type |
|---|---|---|
| in | `/events/momentedge/trigger` | `momentedge_msgs/Trigger` |
| out | `/events/momentedge/recorded` | `momentedge_msgs/Recorded` |

No `rosbag2_interfaces` subscription — coverage comes from the file itself.

## Concurrency

**Thread inventory.** Four singleton threads and the extraction worker pool
run for the process's lifetime, plus one short-lived thread per admitted
trigger:

- **`tail`** — runs `Tailer::run`; performs all blocking file IO for the
  incremental scan.
- **`node-spin`** — the node's single owner; loops `node.spin_once(10 ms)` to
  pump the DDS executor and feed the typed streams. Because the node is owned
  here, no other thread touches it.
- **`trigger-consumer`** — drains the typed `Trigger` subscription with
  `futures::executor::block_on` on the stream, so the stream is consumed on
  this thread without an async runtime. For each admitted trigger it spawns a
  named `trigger-<ns>` handler thread.
- **`extract-N`** (N = 0 .. `extract_parallelism − 1`) — the extraction worker
  pool; each worker loops on the shared FIFO job channel.
- **`signals`** — blocks on signal-hook's iterator and forwards the first
  SIGINT or SIGTERM into a channel for `supervise`.
- **`trigger-<ns>`** (one per admitted trigger) — runs the wait/extract/announce
  flow for one trigger; exits when the clip is published or an error is logged.

The `Recorded` publisher is `Clone` and is shared into each handler thread.

**Admission.** `Admission` is an `AtomicUsize` counter with a fixed `limit`
(`MAX_ACTIVE_TRIGGERS` = 16). The consumer calls `try_acquire` before spawning
each handler: if the counter is below the limit it is incremented and an
`AdmissionPermit` is returned; otherwise `None` is returned and the trigger is
rejected with `error!` — no handler, no clip, no announcement. The permit
holds an `Arc<Admission>` and decrements the counter on `Drop`, so a panicking
handler returns its slot through unwinding. The cap is a flood-sanity bound
rather than a resource necessity: an active handler is a parked thread sleeping
through its postroll and waiting on the coverage watch — the heavy copy stage
is already serialized by the extraction worker pool. 16 comfortably exceeds
any legitimate concurrent trigger burst. Per-trigger failures stay isolated
inside each handler thread — logged and counted, never propagated to the
consumer.

**Extraction worker pool.** `spawn_extract_workers` starts `extract_parallelism`
threads (at least one) sharing one unbounded FIFO channel. A handler enqueues
an `ExtractJob` (window bounds, output path, bounded(1) reply channel) and
blocks on the reply. The worker dequeues FIFO, snapshots `plan_window` at copy
start (after dequeue, so a queued job still cuts from the freshest index), runs
`clip::extract_clip`, and replies. `std::panic::catch_unwind` isolates a
panicking extraction per job; the pool thread survives and continues processing.
With the default `extract_parallelism = 1` bulk copies serialize in submission
order; postroll and coverage waiting are always concurrent.

**`supervise()`.** Each long-lived companion thread is started with
`spawn_supervised`: the closure sends its return value over a `bounded(1)`
channel before returning; a panic unwinds without sending, dropping the sender.
`supervise` uses `crossbeam_channel::select!` on four arms:

1. **tail channel** — receives `anyhow::Result<()>`. `Ok(())` is an unexpected
   exit (the loop never returns on its own); `Err(e)` wraps the scan-fault root
   cause under "tail thread failed". A disconnect (panic) harvests the join
   handle for the payload under "tail thread exited unexpectedly".
2. **spin channel** — receives `()`. Any result (clean exit or panic/disconnect)
   is an error naming the node spin thread.
3. **consumer channel** — receives `()`. Same shape, names the trigger consumer.
4. **signal channel** — receives `i32` (SIGINT or SIGTERM). A delivered signal
   is the requested, orderly stop: `supervise` returns `Ok(())` and `main`
   exits zero. A disconnect (signal forwarder thread died) is an error naming
   the signal handler — losing it silently would mean SIGINT could never trigger
   a clean shutdown.

A dead tailer silently degrades every clip to a grace-timeout cut; a dead spin
thread silently stops delivering triggers; a dead consumer silently stops acting
on them — all three must run for the process's lifetime, so any of them ending
is non-zero exit for a supervisor to restart.

**Process-exit teardown.** `main` returning ends the process, which kills all
remaining threads — the immortal spin/tail loops, parked handler threads, and
any in-flight extraction. That is safe by construction: the capturing-dir reset
at startup reclaims any stranded staged file, and `out_dir` only ever holds
complete clips. There is no explicit runtime teardown step.

## Integration tests (`tests/e2e.rs`)

The inline `#[cfg(test)]` suites cover the tail/clip/supervise logic against
synthetic MCAP files; `tests/e2e.rs` covers the contract against the real
stack — a live `ros2 bag record` started through the production
`scripts/record-continuous.sh`, triggers published with the ros2 CLI, and
`Recorded` asserted via `ros2 topic echo`. How to run it (gating, the
cargo-nextest prerequisite, the exact command) is in the
[README](../../README.md#integration-tests-live-ros2-e2e); this section is the
rationale.

- **Everything is a child process; the test owns no ROS node.** The ros2 CLI
  resolves `momentedge_msgs` types from `AMENT_PREFIX_PATH`, so the test
  binary needs no r2r dependency and carries no process-global DDS state.
  The binary under test is located via `CARGO_BIN_EXE_clipper`.
- **nextest is the required runner, not launch_testing**: process-per-test
  isolation, per-test slow-timeouts, leak detection for orphaned children,
  and the `ros-e2e` test group (`.config/nextest.toml`) serializing the suite
  — concurrent bag records would contend for disk and skew the flush-latency
  assumptions. The unit under test is a Cargo binary and the assertions are
  typed in-repo MCAP reads, which ament's Python harness has no access to.
- **Isolation is per test**: a unique `ROS_DOMAIN_ID` (defense in depth on
  top of the serialization) and a `tempfile` tree for `record/`,
  `triggered/`, and child logs. `harness::Proc` spawns every child in its own
  process group and SIGTERM/SIGKILLs the group on drop, so a panicking test
  strands nothing.
- **rstest is the structuring layer**: the storage-profile matrix is one
  parameterized test (`#[case]` per profile), which nextest still expands
  into isolated per-case processes. Bring-up composes through `TestEnv`
  methods rather than fixture-on-fixture injection — rstest resolves a
  fixture fresh at each injection site, so fixtures sharing a `domain`
  dependency would each get a different domain.
- **Determinism over realism, except where realism is the point.** The
  chunked-profile case stops the recorder cleanly so the footer (`ended`)
  releases the coverage wait instead of racing a chunk flush; the offline
  corruption test plants a framing fault at a known record boundary of a
  closed file. Only `corrupt_tail_health_live` races the scan by design (the
  closest test to real corruption) and is the one case with extra nextest
  retries.
- **Restart and deletion scenarios are exercised live** against a real
  `ros2 bag record`: restarts inside an open trigger window (clean restart,
  deletion-then-restart, deletion before the trigger), deletion without a
  restart (mid-window and pre-trigger), a restart that lands after the window
  ends (producing a valid empty clip), and the no-recovery guarantee — a file
  still on disk after replacement contributes nothing to any subsequent clip.

## Retention

None. The single file grows until the recording stops. Punching holes below a
retention horizon with `fallocate(FALLOC_FL_PUNCH_HOLE)` — keeping offsets
stable under the live writer — is the designed follow-up, tracked in beads as
`clipper-wkg` (constraints: schema/channel custody, standard readers
losing access, st_size growth, punch/extraction coordination).

## Run

```bash
nix develop --command cargo run -p clipper
```

Needs `scripts/record-continuous.sh` running (for `./record-cont`) and a
trigger publisher (`trigger-pub`). `RUST_LOG=debug` raises verbosity.

## Configuration

There are no CLI args. `load_config` in `main.rs` layers config-rs sources —
defaults → optional TOML file → `CLIPPER_*` environment variables
(later wins) — and deserializes the merged result into `Config` via serde.
The TOML file is `clipper.toml` in the working directory unless
`$CLIPPER_CONFIG` names another path; a missing file is fine, so
the binary runs with no setup. The keys (`record_dir`, `out_dir`,
`grace_secs`, `extract_parallelism`) and their defaults are listed in the
[README](../../README.md#configuration).
