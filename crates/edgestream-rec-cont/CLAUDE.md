# edgestream-rec-cont

A *triggered* clip recorder over **one continuous MCAP file** — the
no-splits sibling of [`edgestream-rec`](../edgestream-rec/CLAUDE.md). Where
that crate waits for rosbag2 split boundaries (`/events/write_split`) and reads
closed split files, this one keeps the single growing recording open and
**tails it**, so a clip can be cut as soon as the data is physically on disk:
clip latency is bounded by the recorder's write-through latency, not a split
duration. Built on [r2r](https://github.com/sequenceplanner/r2r) with tokio.

## The pipeline it sits in

```
ros2 bag record (scripts/record-continuous.sh) ──▶ ./record-cont/<bag>_0.mcap   (one growing file)
        ▲ kept open + tailed (incremental scan, persistent offsets)
edgestream-rec-cont ◀── /events/edgestream/trigger ── trigger-pub (or any publisher)
        │ cuts [trigger_time-preroll, trigger_time+postroll]
        ├──▶ ./triggered-cont/<trigger_ns>_<name>.mcap
        └──▶ /events/edgestream/recorded
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
  message `log_time` they hold. Extraction reads only the extents overlapping
  its window; the overlap test is exact (real min/max, not a heuristic), so no
  in-window message can be missed.
- **Schema/channel registry** — owned copies of every `Schema`/`Channel`
  record, keyed by channel ID (unique within one continuous file). The MCAP
  spec puts a Schema before any Channel referencing it, so resolution always
  succeeds on conformant files; an inverted (invalid) file degrades the
  channel to schemaless rather than erroring. Chunked recordings carry these
  records *inside* chunks, so chunks are decompressed during the tail
  (zstd, lz4 and uncompressed chunks all work — mcap's default features);
  the default fastwrite profile is unchunked and skips that cost entirely.
- **Coverage watch** — a `tokio::sync::watch` of the highest `log_time` on
  disk plus an `ended` flag (DataEnd/Footer scanned). Sound because messages
  land in the file in (approximately) non-decreasing `log_time` order —
  rosbag2's single writer stamps `log_time` at receive.

Per top-level `Message` record only the 14-byte prefix is read (channel id,
sequence, `log_time`); bodies are first touched at extraction. The same
"decode only the timestamp" discipline as the rest of the workspace, applied
to file tailing.

**Recorder restarts:** the recording is discovered as the newest `*.mcap`
under `record_dir`; when that path stops resolving to the tailed inode (the
record script wipes the bag dir on restart), the index resets and the new file
is tailed from scratch. In-flight extractions hold their own `Arc<File>` and
finish safely against the deleted inode.

## Per-trigger flow (`handle_trigger`)

Each `edgestream_msgs/Trigger` is handled in its own tokio task, so overlapping
windows are cut concurrently against the one shared tail:

1. **Wait out the postroll.** Sleep until the system clock passes
   `trigger_time + postroll`.
2. **Wait for coverage.** Block on the coverage watch until a message with
   `log_time` at/after the window end is on disk (or the recording ended).
   This replaces `edgestream-rec`'s split rendezvous. A grace timeout
   (`grace_secs`, default 30 s) bounds the wait; on timeout the clip is cut
   from what exists, with a warning. The grace must exceed the recorder's
   flush latency: near zero for the fastwrite profile, roughly one chunk fill
   (chunk size / aggregate data rate) for chunked profiles.
3. **Extract** under an extraction permit (`extract_parallelism`, default 1:
   copies queue FIFO so concurrent windows don't compete with the recorder
   for disk bandwidth; the waits in steps 1–2 stay concurrent): a
   `plan_window` snapshot (file handle, overlapping extents, channel
   registry) handed to `clip::extract_clip` (blocking IO, run on
   `spawn_blocking`).
4. **Announce** by publishing `edgestream_msgs/Recorded` — only after the
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
length, failing any conformant chunk whose contents out-compress it. Messages whose
`log_time` falls in the inclusive window are written through with their **raw
serialized bytes** (`write_to_known_channel`); CDR bodies are never decoded.

Output channels are registered from the registry per source channel ID and
cached; `mcap::Writer` deduplicates schemas/channels by content. The clip ends
with `Writer::finish()`, which writes the summary section, footer and closing
magic — every clip is a complete, standalone MCAP of the same form as
`edgestream-rec`'s (`mcap::MessageStream` over a clip is the validity check the
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

`log_time`, the trigger stamp, and the wait clock are all nanoseconds on the
system clock — this assumes the default (no `use_sim_time`), matching
`edgestream-rec`. The `time_to_ns` / `sanitize` helpers are kept in step with
that crate's identical copies.

## Topics and types

| Direction | Topic | Type |
|---|---|---|
| in | `/events/edgestream/trigger` | `edgestream_msgs/Trigger` |
| out | `/events/edgestream/recorded` | `edgestream_msgs/Recorded` |

No `rosbag2_interfaces` subscription — coverage comes from the file itself.

## Concurrency

Same single-owner node model as `edgestream-rec`: one `spawn_blocking` thread
spins the node, the typed trigger stream is consumed on a tokio task, the
`Recorded` publisher is `Clone` and shared into each handler. A second
`spawn_blocking` thread runs the tail loop for the process's lifetime; it
talks to handlers only through the mutex-guarded index and the coverage watch.
Clip copies are gated by a FIFO semaphore (`extract_parallelism`, default 1).
`main` `select!`s on the tail and spin thread handles and exits non-zero if
either dies, for a supervisor to restart — with a dead tailer the process
would otherwise limp on, cutting every clip at the grace timeout.

## Retention

None. The single file grows until the recording stops. Punching holes below a
retention horizon with `fallocate(FALLOC_FL_PUNCH_HOLE)` — keeping offsets
stable under the live writer — is the designed follow-up, tracked in beads as
`ros2_subscribe-wkg` (constraints: schema/channel custody, standard readers
losing access, st_size growth, punch/extraction coordination).

## Run

```bash
nix develop --command cargo run -p edgestream-rec-cont
```

Needs `scripts/record-continuous.sh` running (for `./record-cont`) and a
trigger publisher (`trigger-pub`). `RUST_LOG=debug` raises verbosity.

## Configuration

There are no CLI args. `load_config` in `main.rs` layers config-rs sources —
defaults → optional TOML file → `EDGESTREAM_*` environment variables
(later wins) — and deserializes the merged result into `Config` via serde.
The TOML file is `edgestream-rec-cont.toml` in the working directory unless
`$EDGESTREAM_CONFIG` names another path; a missing file is fine, so
the binary runs with no setup. The keys (`record_dir`, `out_dir`,
`grace_secs`, `extract_parallelism`) and their defaults are listed in the
[README](../../README.md#continuous-single-file-variant).
