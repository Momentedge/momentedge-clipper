# clipper

A *triggered* clip recorder over a **continuous `ros2 bag record`** output. It keeps the growing recording(s) open and **tails them**, so a clip can be cut as soon as the data is physically on disk: clip latency is bounded by the recorder's write-through latency. Built on [r2r](https://github.com/sequenceplanner/r2r) over plain OS threads — there is no async runtime.

## The pipeline it sits in

```
ros2 bag record (scripts/record.sh) ──▶ ./record/<bag>_0.mcap   (one growing file)
                                    ──▶ ./record/<bag>_1.mcap   (next split, on rollover)
        ▲ each file discovered, kept open, and tailed (incremental scan)
clipper ◀── trigger ── EITHER /events/momentedge/trigger (ros interface)
        │              OR read out of the tailed ./record/*.mcap (mcap interface)
        │ cuts [anchor-preroll, anchor+postroll]  (anchor resolved per cell)
        │   one recording  → ./clipped/<anchor_ns>_<name>.mcap
        │   rollover split → ./clipped/<anchor_ns>_<name>_00.mcap + _01.mcap …
        └──▶ completion: ros → /events/momentedge/recorded (filenames[] lists
             every segment); mcap → the clip's atomic move into ./clipped is
             the only signal (no Recorded published)
```

The trigger and the completion are paired into one **interface**, selected by
`--interface {ros|mcap}` (default `ros`); the two interfaces are mutually
exclusive and clipper drives exactly one per run. The `ros` interface
subscribes on a ROS node and publishes `Recorded`; the `mcap` interface reads
triggers out of the recording clipper already tails and runs ROS-free, with the
clip's move into `out_dir` as the only completion signal. See "The interface
abstraction" below.

`record.sh` is a standalone `ros2 bag record` — this binary never
spawns it. The two communicate only through the files. Under the `mcap`
interface that file path is also the *trigger* path: the continuous recording
must capture the trigger topic (`ros2 bag record --all`) so clipper can lift the
triggers back out of it.

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

Both properties are a producer requirement in disguise — append complete
records, never seek back to rewrite one (the producer-facing statement lives in
[ARCHITECTURE.md](../../ARCHITECTURE.md#tailing-a-live-mcap)). The Rust `mcap`
crate's chunked writer (`use_chunks(true)`, its default) is exactly the
violation: it leaves a `Chunk` header's length as the placeholder `u64::MAX`
until the chunk closes and the true length is back-patched in, so a `len >
MAX_RECORD_LEN` framing fault (below) is guaranteed the moment the scan meets
one mid-write. Because `u64::MAX` is a value only a seek-back writer's
placeholder could ever produce — no valid record reaches it under
`MAX_RECORD_LEN` — that one length is special-cased to name the cause instead
of reading as bare corruption: "record at offset {offset} declares u64::MAX
bytes — an unpatched length from a seek-back (chunked) writer? such a recording
cannot be tailed until it is finalised". (The compliant chunked configuration —
buffered chunks via `disable_seeking(true)`, which appends each chunk as one
complete record — is demonstrated by
[`examples/chunked-mcap-writer`](../../examples/chunked-mcap-writer/README.md).)
This is also why the scan never waits
out an over-`MAX_RECORD_LEN` length instead of faulting on it: `u64::MAX` (or
anything else past the ceiling) has no path to becoming a valid record, so
retrying it as though it were a transient stall would only delay the identical
fault. Treating it as fatal once the retry budget below is exhausted is the
deliberate choice: the alternative — waiting it out — would silently degrade
every clip to a grace-timeout cut instead of failing fast.

The tail keeps the recording open and never re-reads consumed bytes. Each pass
yields three artefacts, shared with the per-trigger handlers:

- **Extent index** — contiguous byte ranges (closed at 4 MiB) carrying the
  min/max `log_time` and `publish_time` of the messages they hold. Extraction
  reads only the extents whose span on the active [time source](#time-source)
  overlaps its window; the overlap test is exact on both spans (real min/max,
  not a heuristic), so no in-window message can be missed on either clock
  domain. Retention ages on the `log` span alone.
- **Schema/channel registry** — owned copies of every `Schema`/`Channel`
  record, keyed by channel ID (unique within one continuous file). The MCAP
  spec puts a Schema before any Channel referencing it, so resolution always
  succeeds on conformant files; an inverted (invalid) file degrades the
  channel to schemaless rather than erroring. Chunked recordings carry these
  records *inside* chunks, so chunks are decompressed during the tail
  (zstd, lz4 and uncompressed chunks all work — mcap's default features);
  the default fastwrite profile is unchunked and skips that cost entirely.
- **Coverage watch** — a `Watch<Coverage>` (`Mutex` + `Condvar`, `src/watch.rs`)
  holding a collection-wide high-water per source: the highest `log_time`
  (`high_water_ns`) and the highest `publish_time` (`publish_high_water_ns`). A
  handler waits on the high-water of its window's [time source](#time-source).
  The `log` high-water is a completeness proof — messages land in the file in
  (approximately) non-decreasing `log_time` order (rosbag2's single writer stamps
  `log_time` at receive) and the tail scans recordings strictly oldest-first, one
  at a time, so a later recording's coverage cannot advance before an earlier one
  is complete. The `publish` high-water is a liveness signal only: `publish_time`
  has no ordering guarantee, so a message can arrive after a cut with an in-window
  `publish_time` and be missing from that clip. Both rise independently and never
  regress (the watch only raises each).

Per top-level `Message` record only the 22-byte fixed header is read, in one
read (channel id, sequence, `log_time`, `publish_time`); bodies are first
touched at extraction. Both stamps are indexed so a window can live on either
[time source](#time-source); the gap between the two — recorder queue backlog
plus producer clock skew — is also observable (logged per scan pass at debug). A
record too short
to hold the full 22-byte header is skipped like other localized damage: warned
and consumed via its intact framing, contributing to neither bounds nor
coverage. The same "decode only the timestamps" discipline as the rest of the
workspace, applied to file tailing.

The one exception is an **opt-in trigger tap**, enabled only under the `mcap`
interface (`Tailer::with_trigger_tap`). With the tap on, the scan additionally
reads the *full body* of every message on the configured trigger topic, lifting
it as a raw `(message_encoding, body, log_time, publish_time)` quadruple the
MCAP interface decodes by `message_encoding` — the two record stamps are what
the mcap interface resolves its anchor from; the tap learns the trigger topic's channel IDs
from the same registry pass, so a body is read only for a message it has already
matched to that topic. With the tap disabled — the `ros` interface, the default
— the scan is byte-for-byte the timestamp-only walk above: no message body is
ever read.

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

**Recorder restarts and bag splits:** recordings are discovered by
`discover::NewFileWatchIterator`, a lazy iterator that yields new `*.mcap` files
one per `next()`. Each poll drains it; each yielded path is opened and inserted
as a `New` recording into the collection (`TailState`). Files are tracked by
`(dev, ino)` **identity**, not a timestamp cursor: a file under tail grows and
its mtime (and ctime) advances, so a cursor would re-yield it every poll and
index the same recording as a phantom duplicate. The iterator records the inode
of every file it yields and never yields it again, forgetting inodes no longer on
disk (so the set stays bounded and a reused inode yields its new file). mtime
orders the unseen files oldest-first, so several appearing between polls drain in
creation order. No file observed during a run is skipped.

At startup the newest existing file (by mtime) is adopted directly and the
iterator seeded past every file present then. Pre-existing bags older than that
newest file are not indexed: clipper recovers only rollovers it observes during
its own run, never reconstructing offsets or footers it did not scan
incrementally. A trigger fired shortly after startup whose preroll reaches into a
prior split that existed before launch gets no segment from that file.

A bag split (rosbag2 `--max-bag-size`/`--max-bag-duration`) is detected when a
footer appears on disk, when a successor is yielded by the iterator while
`current` is length-stable, or when the tailed inode vanishes or is replaced. In
each case the recording transitions to `Ended` and the tail advances to the next
indexed recording — no index reset, no data lost from recordings already in the
collection. A recorder restart (record script wipes the bag directory) is detected
via `inode_changed`: the tailed path no longer resolves to the open fd's inode.
The `Arc<File>` keeps the old inode readable, so the final scan drains every
complete record before the recording is retired, and in-flight extractions finish
safely against the deleted inode. A magic mismatch stays fatal — an append-only
file whose first eight bytes are wrong can never become a valid MCAP. A `NotFound`
when opening a discovered path (the file vanished between discovery and open) is
silently skipped; the iterator has already advanced past it.

## Per-trigger flow (`handle_trigger` / `record_clip`)

Each admitted trigger is handled on its own thread, so overlapping windows are
cut concurrently against the shared tail. The handler (`handler.rs`) is generic
over the [`Announce`](#the-interface-abstraction) the active interface supplies
and knows only the neutral `Trigger`/`Completion` contract plus the window's
`anchor_ns` and [time source](#time-source) — nothing of ROS or any wire
encoding. The interface resolves the `anchor_ns` (the window centre) and hands it
in; the handler never derives an anchor itself. Admission is bounded: at most
`MAX_ACTIVE_TRIGGERS` (16) handlers may be active at once, and a trigger that
arrives while all of them are is rejected — logged with `error!` and otherwise
ignored: no handler runs, no clip is extracted, and no completion is announced.

1. **Wait out the postroll.** Sleep until the system clock passes
   `anchor + postroll`. The wall floor is always the system clock, whatever the
   time source. `checked_sub` reads the clock once per iteration, so a clock that
   crosses `end_ns` between the check and the sleep cannot underflow.
2. **Wait for coverage.** Block on the collection-wide coverage watch until the
   window's source high-water reaches `end_ns` (`c.for_source(time_source)`). A
   grace timeout (`grace_secs`, default 30 s) bounds the wait; on timeout the clip
   is cut from what exists, with a warning. On `log` a window whose end falls
   inside an already-scanned recording is satisfied as soon as the scan reaches
   `end_ns` (or at the footer), so only a window whose end is past the last
   recorded message with no successor waits out the full grace; on `publish` the
   high-water is a liveness signal, so a later out-of-order message can still be
   missed. The grace must exceed the recorder's flush latency: near zero for the
   fastwrite profile, roughly one chunk fill (chunk size / aggregate data rate)
   for chunked profiles.
3. **Multi-file snapshot.** Call `tailer.plan_window(start_ns, end_ns,
   time_source)` once, producing a `Vec<WindowPlan>` — one plan per recording
   whose extents overlap the window on the active time source, oldest first. Each plan carries its own `Arc<File>` clone, so a
   retention prune or rollover after this snapshot cannot pull the bytes out.
4. **Stage** via the staging worker pool (`extract_parallelism` `stage-N`
   threads, default 1): one `StageJob` per plan is enqueued on the shared FIFO
   channel and the handler blocks on each reply. A worker runs
   `clip::stage_clip` into `.capturing/` and replies a `StagedClip`. When no
   recording covers the window, one empty plan is staged so every trigger
   produces a valid (possibly empty) clip.
5. **Publish.** Empty segments are dropped when the window produced real data
   elsewhere (one is kept if all are empty). The count determines naming: a
   single segment keeps the bare `<anchor_ns>_<name>.mcap`; multiple segments
   get `<base>_00.mcap`, `<base>_01.mcap`, … Each is atomically published into
   `out_dir` via `hard_link` + unlink.
6. **Announce** a single `Completion` (the trigger echo plus all segment paths)
   through the active interface's announcer — only after every segment is in
   `out_dir` and fsynced, so every announced path is already crash-durable. The
   `ros` interface turns the `Completion` into one `momentedge_msgs/Recorded`
   published on `/events/momentedge/recorded`; the `mcap` interface's announcer
   is a no-op — the segments' atomic move into `out_dir` (step 5) is the only
   completion signal, with the per-clip `info!` lines as the log.

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
length, failing any conformant chunk whose contents out-compress it. Messages whose
`log_time` falls in the inclusive window are written through with their **raw
serialized bytes** (`write_to_known_channel`); CDR bodies are never decoded.

The clip writer is built from explicit `mcap::WriteOptions` with `.compression(..)`
set — the codec is a deliberate choice (`--clip-compression`, default zstd; see
the [README](../../README.md#configuration)), not inherited from the mcap crate
default, so a future default change cannot silently alter clip output. `copy_window`
takes the codec as `Option<mcap::Compression>` (`None` = uncompressed), threaded
down from `Config` through `extract_clip`/`stage_clip`; chunk size and chunking
stay at the `WriteOptions` default. Output channels are registered from the
registry per source channel ID and
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

`log_time`, `publish_time`, the trigger stamp, and the wait clock are all
nanoseconds on the system clock — this assumes the default (no `use_sim_time`).
The trigger stamp is the neutral `Stamp` (`trigger.rs`), a
`builtin_interfaces/Time` flattened to its `sec`/`nanosec` fields free of `r2r`;
`Stamp::ns()` flattens it to that scale (`sec.max(0) * 1e9 + nanosec`, negative
seconds clamped to 0). The same arithmetic anchors the window identically whether
the trigger arrived live over ROS or was decoded out of the tailed MCAP.

## Time source

`--time-source` (`log` or `publish`, default `log`; `TimeSource` in `main.rs`,
threaded through the handler, tail, and clip) picks the clock domain the whole
window lives in: the anchor, which messages fall inside, which extents are read
(`Extent::overlaps` / `plan_window` on the source's `Span`), and which coverage
high-water the wait blocks on (`Coverage::for_source`). It governs nothing else —
retention ages files on `log_time` (`TimeBounds.log.max`) whatever the window's
source, and the postroll floor is the wall clock. clipper never interprets
`publish_time`; it windows on the raw value, so `publish` coverage is a liveness
signal, not a completeness proof (out-of-order publish times are normal). On ROS 2
Humble `publish_time = log_time` verbatim, so `publish` is a no-op there.

**The anchor seam.** The interface resolves each trigger's [`Anchor`] (in
`interface.rs`) — the instant the window centres on, plus whether it came from
`trigger_time` — and passes it to the driver's `fire` callback, which hands
`anchor.ns` to `handle_trigger` for both the window bounds and the output name
`<anchor_ns>_<name>.mcap`. The four interface × `--time-source` cells resolve it:

|                    | `--time-source log`             | `--time-source publish`      |
| ------------------ | ------------------------------- | ---------------------------- |
| **`--interface ros`**  | `now_ns()` at the subscription | the trigger's `trigger_time` |
| **`--interface mcap`** | the record's `log_time`        | the record's `publish_time`  |

`resolve_ros_anchor` and `resolve_mcap_anchor` (in `interface.rs`) do the
resolution; a live ROS trigger has no record stamp and r2r surfaces no wire
timestamp, so ROS anchors on `now` or the publisher's `trigger_time`. The
`Completion` echoes the trigger's `trigger_time` unchanged.

**The admission gate.** `validate_trigger(trig, anchor, now_ns)` in `main.rs` is
the single gate every resolved trigger passes in the `fire` callback before any
handler work — a pure function (the clock is passed in) so its cases are
exhaustively unit-tested. Any failing check logs at `error!` and drops the
trigger: no handler spawned, no clip, no `Recorded`. All limits are named consts
in `main.rs`; each value exactly at its bound is accepted:

- **`trigger_time` in a cell that ignores it.** `trigger_time` is read in exactly
  one cell — `ros` + `publish` (`Anchor::from_trigger_time`); every other cell
  anchors on a transport stamp. A non-zero `trigger_time` where it is ignored is
  rejected: sending it there would silently anchor the window on the trigger's
  arrival rather than the requested instant — a deferred-window request lost
  without a trace. `trigger_time == 0` is always accepted.
- **`preroll`/`postroll` past `MAX_ROLL_NS`** (30 min) — bounds how far a cut
  reaches into the retained recordings and how long a handler parks.
- **A resolved anchor more than `MAX_ANCHOR_FUTURE_SKEW_NS` (30 min) past `now`.**
  The guarded value is the *resolved anchor*, not `trigger_time` — the anchor is
  what parks a handler through its postroll wall-floor sleep (`anchor + postroll`)
  whatever cell resolved it, so a far-future anchor (a producer clock fault or a
  hostile record stamp) would wedge a handler. `ros`+`log` resolves the anchor to
  `now` and always passes; the guard bites on a `ros`+`publish` `trigger_time` and
  on a tail record's own stamp.
- **A `name` that is empty, past `MAX_TRIGGER_NAME_LEN` (128 B), or unsafe in the
  clip pathname** (`validate_name`: no path separator, NUL, leading dot, or `..`).
  `handler::sanitize` still maps stray characters to `_` at clip creation; the
  structural hazards are refused whole here rather than silently rewritten.

[`Anchor`]: src/interface.rs

## The two interfaces

The trigger input and the completion output are one unit — an **interface** —
chosen by `--interface {ros|mcap}` (default `ros`). The two are mutually
exclusive; clipper drives exactly one per run. No `rosbag2_interfaces`
subscription either way — coverage always comes from the file itself.

**`ros`** (the default, the deployed path) talks to the ROS graph:

| Direction | Topic | Type |
|---|---|---|
| in | `/events/momentedge/trigger` | `momentedge_msgs/Trigger` |
| out | `/events/momentedge/recorded` | `momentedge_msgs/Recorded` |

It subscribes to the trigger topic on a ROS node and publishes one `Recorded`
per finished clip, with every segment path in `filenames[]`.

**`mcap`** has no ROS surface at all. Its trigger *input* is the tailed
recording itself: the continuous `ros2 bag record` (run `--all`) captures the
trigger topic, and the tail's opt-in trigger tap lifts each trigger message
back out by `message_encoding` (see "The interface abstraction"). Its
completion *output* is the clip's atomic move into `out_dir` — there is no
`Recorded` topic and nothing is published; the per-clip `info!` log lines are
the only completion record. It runs fully ROS-free at runtime: no ROS
`Context`/`Node`, no spin thread, no subscription.

## The interface abstraction

The recorder is decoupled into four layers around one neutral boundary, so the
clip-cutting half never learns of ROS or any wire encoding and the
outside-facing half is the only place either appears:

- **`trigger.rs`** — the neutral contract: `Trigger`, `Stamp`, `TriggerRecord`
  (the MCAP message record carrying an undecoded trigger), `Completion`, and the
  `Announce` trait. It pulls in neither `r2r` nor `mcap` — only `serde`, so
  `Trigger`/`Stamp` derive `Deserialize` for the `json` decode shape — so either
  side can change without dragging the other along.
- **`decode.rs`** — `decode_trigger`, which decodes one trigger payload
  according to its MCAP channel's `message_encoding`, dispatching over two
  encodings, both decodable with dependencies already in the closure (no new
  ones):
  - **`cdr`** (the ROS2 default) via `r2r`'s rmw deserialization — the linked
    rmw library only, never a ROS `Context`/`Node`/executor, so it works in the
    fully ROS-free MCAP interface. r2r's generated `Trigger` maps onto the
    domain `Trigger` through a `From` impl shared with the live ROS path.
  - **`json`** via `serde_json` — a first-class peer of `cdr`, so a writer
    interleaving a trigger into the bag is never forced to serialize as CDR;
    the domain `Trigger` derives `Deserialize`, so serde reads straight into it.

  `cbor`, schema-bound `protobuf`/`flatbuffer`, `ros1`, and any unknown encoding
  return an error the interface logs and skips — one undecodable trigger never
  stops the recorder.
- **`interface.rs`** — the `trait Interface` (generic, dispatched statically,
  no `Box<dyn>`), with `RosInterface` (owns the node and its own internal spin
  thread) and `McapInterface` (drains the tail's trigger tap and decodes each
  raw trigger), plus the two announcers: `RosAnnouncer` (publishes `Recorded`)
  and `NullAnnouncer` (no-op). An interface produces decoded `Trigger`s and owns
  the completion half through its `Announce`r.
- **`handler.rs`** — `handle_trigger` (generic over `Announce`), `record_clip`,
  and the staging worker pool: the ROS- and encoding-agnostic clip-cutting half,
  speaking only the `Trigger`/`Completion` contract.

`main.rs` wires it together: it builds the selected interface, and a generic
`drive<I: Interface>` runs the recorder and supervises the tail, the one
interface thread, and the signal forwarder.

## Concurrency

**Thread inventory.** Three singleton threads and the staging worker pool
run for the process's lifetime, plus one short-lived thread per admitted
trigger:

- **`tail`** — runs `Tailer::run`; discovers recordings via
  `NewFileWatchIterator`, performs all blocking file IO for the incremental
  scan, and prunes the collection every poll. Under the `mcap` interface it
  also forwards each tapped trigger to the interface thread (the trigger tap).
- **`interface`** — runs `iface.run`, the single owner of the active
  interface's trigger source. For the **ROS** interface this thread internally
  owns *both* the node spin (a `node-spin` worker looping `node.spin_once(10
  ms)` to pump the DDS executor) *and* the subscription drain (a `trigger-drain`
  worker draining the typed `Trigger` stream with `futures::executor::block_on`),
  running them concurrently and returning when either resolves — so the driver
  supervises one uniform interface thread regardless of mode. For the **MCAP**
  interface this thread drains the tail's trigger tap, decoding each raw trigger
  by its `message_encoding`. For each decoded trigger it fires the per-trigger
  callback, which admits and spawns a named `trigger-<ns>` handler thread.
- **`stage-N`** (N = 0 .. `extract_parallelism − 1`) — the staging worker
  pool; each worker loops on the shared FIFO `StageJob` channel, runs
  `clip::stage_clip`, and replies a `StagedClip`. The handler publishes the
  staged segments itself once the window's segment count is known.
- **`signals`** — blocks on signal-hook's iterator and forwards the first
  SIGINT or SIGTERM into a channel for `supervise`.
- **`trigger-<ns>`** (one per admitted trigger) — runs the
  wait/snapshot/stage/publish/announce flow for one trigger; exits when the
  clip is published or an error is logged.

The announcer the handler uses is `Clone + Send` and is moved into each handler
thread (`RosAnnouncer` for the ROS interface, the no-op `NullAnnouncer` for the
MCAP interface).

**Admission.** `Admission` is an `AtomicUsize` counter with a fixed `limit`
(`MAX_ACTIVE_TRIGGERS` = 16). The consumer calls `try_acquire` before spawning
each handler: if the counter is below the limit it is incremented and an
`AdmissionPermit` is returned; otherwise `None` is returned and the trigger is
rejected with `error!` — no handler, no clip, no announcement. The permit
holds an `Arc<Admission>` and decrements the counter on `Drop`, so a panicking
handler returns its slot through unwinding. The cap is a flood-sanity bound
rather than a resource necessity: an active handler is a parked thread sleeping
through its postroll and waiting on the coverage watch — the heavy copy stage
is already serialized by the staging worker pool. 16 comfortably exceeds
any legitimate concurrent trigger burst. Per-trigger failures stay isolated
inside each handler thread — logged and counted, never propagated to the
consumer.

**Staging worker pool.** `spawn_stage_workers` starts `extract_parallelism`
threads (at least one) sharing one unbounded FIFO channel. A handler enqueues
one `StageJob` per `WindowPlan` (the plan snapshot, window bounds, base output
path, bounded(1) reply channel) and blocks on each reply. The worker dequeues
FIFO, runs `clip::stage_clip` into `.capturing/`, and replies a `StagedClip`.
The handler — not the worker — publishes the staged segments once the window's
segment count is known, so naming (`_00`/`_01`) and atomic publication happen
together. `std::panic::catch_unwind` isolates a panicking stage per job; the
pool thread survives and continues processing. With the default
`extract_parallelism = 1` bulk copies serialize in submission order; postroll
and coverage waiting are always concurrent.

**`supervise()`.** Each long-lived companion thread is started with
`spawn_supervised`: the closure sends its return value over a `bounded(1)`
channel before returning; a panic unwinds without sending, dropping the sender.
`supervise` uses `crossbeam_channel::select!` on three arms:

1. **tail channel** — receives `anyhow::Result<()>`. `Ok(())` is an unexpected
   exit (the loop never returns on its own); `Err(e)` wraps the scan-fault root
   cause under "tail thread failed". A disconnect (panic) harvests the join
   handle for the payload under "tail thread exited unexpectedly".
2. **interface channel** — receives `anyhow::Result<()>`. `Ok(())` is an
   unexpected exit (the interface drains its trigger source for the process's
   lifetime); `Err(e)` wraps the fault under "interface thread failed"; a
   disconnect (panic) harvests the handle under "interface thread exited
   unexpectedly". This one arm covers both interfaces — the ROS interface's
   internal node spin and subscription drain are supervised *inside* `iface.run`
   and surface here as a single interface fault, so a dead spin thread (which
   would otherwise silently stall trigger delivery) is never lost.
3. **signal channel** — receives `i32` (SIGINT or SIGTERM). A delivered signal
   is the requested, orderly stop: `supervise` returns `Ok(())` and `main`
   exits zero. A disconnect (signal forwarder thread died) is an error naming
   the signal handler — losing it silently would mean SIGINT could never trigger
   a clean shutdown.

A dead tailer silently degrades every clip to a grace-timeout cut; a dead
interface thread silently stops delivering and acting on triggers — both must
run for the process's lifetime, so either ending is non-zero exit for a
supervisor to restart.

**Process-exit teardown.** `main` returning ends the process, which kills all
remaining threads — the immortal tail and interface loops, parked handler
threads, and any in-flight extraction. That is safe by construction: the capturing-dir reset
at startup reclaims any stranded staged file, and `out_dir` only ever holds
complete clips. There is no explicit runtime teardown step.

## Integration tests (`tests/e2e.rs`)

The inline `#[cfg(test)]` suites cover the tail/clip/supervise logic against
synthetic MCAP files; `tests/e2e.rs` covers the contract against the real
stack — a live `ros2 bag record` matching the production `scripts/record.sh`
invocation (the harness builds the command directly), triggers published with
the ros2 CLI, and
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
- **The MCAP interface is exercised end to end** (`mcap_interface_*`): clipper
  runs `--interface mcap`, fully ROS-free, against a `ros2 bag record --all`
  that captures a ROS-published trigger into the bag. clipper lifts that trigger
  back out of the recording it tails, cuts the clip, and signals completion by
  the file's appearance in `out_dir` — there is no `Recorded` topic to echo, so
  the assertion is on the clipped file rather than a published message.
- **Capture-time windowing is proved against a live momentedge writer**
  (`live_writer_capture_time_windowing`, ROS-free at runtime): clipper tails a
  recording while `examples/custom-mcap-writer` appends it, with every
  `publish_time` deliberately offset from `log_time`. Per `--time-source` case
  the clip's every message is in-window on the *selected* stamp while at least
  one message is out-of-window on the *contrasting* stamp — jointly impossible
  unless the two clock domains genuinely select different message sets. The
  writer binary is resolved beside `CARGO_BIN_EXE_clipper` (built on demand if
  absent), so the case needs no extra build step.

## Retention

The tail prunes `Ended` recordings every poll (not only at rollover). A
recording is pruned when its max `log_time` (`bounds.log.max`) is older than
`now - watch_old_files_duration` (default 600 s, env
`MOMENTEDGE_WATCH_OLD_FILES_DURATION`). Retention always ages on `log_time`,
whatever the window's [time source](#time-source): a producer must not be able to
keep a file alive — or force its deletion — through what it writes into
`publish_time`. Pruning is file-granular — never the `current` recording, never
mid-file. Dropping a `RecordingIndex` releases its `Arc<File>`, closing the
descriptor once no in-flight plan still holds a clone. Running the prune every
poll (rather than only at rollover) is what bounds open fds and index memory when
the recorder stops splitting or goes idle.

`high_water_ns` (the `log` coverage) is monotonic across prunes: a pruned file is
below the watch floor and never held the collection maximum, so dropping it never
lowers the high-water. Handlers never see `log` coverage regress. The `publish`
high-water is not tied to the retention floor, but the watch only ever raises it,
so a handler waiting on it never sees it regress either.

By default pruning forgets a recording in-memory only; the `.mcap` file remains
on disk for `ros2 bag record` and other consumers. When `--delete-old-files` is
set (env `MOMENTEDGE_DELETE_OLD_FILES`, default false), a prune also unlinks the
expired file from disk. An in-flight extraction's own `Arc<File>` clone keeps
the unlinked inode readable to completion (POSIX unlink-while-open), so deletion
never breaks a clip already in progress.

**Prune vs in-flight trigger:** `watch_old_files_duration` must be set
comfortably above the largest preroll any trigger will request. A trigger whose
preroll reaches past the retention floor may lose its oldest segment — that
recording was intentionally forgotten. See the [README](../../README.md#configuration)
for the flag reference.

## Run

```bash
nix develop --command cargo run -p clipper
```

Needs `scripts/record.sh` running (for `./record`) and a
trigger publisher (`trigger-pub`). `RUST_LOG=debug` raises verbosity.

## Configuration

`Config` is a clap `derive(Parser)`: every field is a CLI flag with a
`MOMENTEDGE_*` environment fallback and a per-field default, so precedence is
CLI flag > env var > default. `load_config` in `main.rs` parses it — clap prints
`--help`/`--version` and any parse error and exits before it returns, so the
binary still runs with no setup. The `MOMENTEDGE_*` env names are not wired
per field: `with_env_prefix` walks every argument with `Command::mut_args` and
binds `<field>` to `MOMENTEDGE_<FIELD>` (`grace_secs` → `MOMENTEDGE_GRACE_SECS`),
leaving the auto-generated `--help`/`--version` untouched. Changing the prefix is
the one `ENV_PREFIX` constant. clap's `env` feature provides the per-arg env
fallback and `string` lets the runtime-built env names be set on the args. The
flags, env vars, and defaults are tabulated in the
[README](../../README.md#configuration).

The interface seam is one such flag: `--interface {ros|mcap}` (env
`MOMENTEDGE_INTERFACE`, default `ros`), a clap `ValueEnum` over `InterfaceKind`
that picks the active [interface](#the-two-interfaces) at startup.
