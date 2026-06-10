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
  record, keyed by channel ID (unique within one continuous file). Chunked
  recordings carry these records *inside* chunks, so chunks are decompressed
  during the tail; the default fastwrite profile is unchunked and skips that
  cost entirely.
- **Coverage watch** — a `tokio::sync::watch` of the highest `log_time` on
  disk plus an `ended` flag (DataEnd/Footer scanned).

Per top-level `Message` record only the 14-byte prefix is read (channel id,
sequence, `log_time`); bodies are first touched at extraction. The same
"decode only the timestamp" discipline as the rest of the workspace, applied
to file tailing.

**Recorder restarts:** the recording is discovered as the newest `*.mcap`
under `--record-dir`; when that path stops resolving to the tailed inode (the
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
   This replaces `edgestream-rec`'s split rendezvous. A 30 s grace timeout
   (`COVERAGE_GRACE`) bounds the wait when the recorded topics go quiet; on
   timeout the clip is cut from what exists, with a warning.
3. **Extract** via a `plan_window` snapshot (file handle, overlapping extents,
   channel registry) handed to `clip::extract_clip` (blocking IO, run on
   `spawn_blocking`).
4. **Announce** by publishing `edgestream_msgs/Recorded`.

## The copy is direct (`clip.rs`)

Extraction reads each planned extent with `read_at` (no seek state shared with
the tail), iterates its records with `mcap::read::LinearReader::sans_magic`
(which accepts a mid-file slice), and descends into chunk records — so chunked
and unchunked recordings extract through the same path. Messages whose
`log_time` falls in the inclusive window are written through with their **raw
serialized bytes** (`write_to_known_channel`); CDR bodies are never decoded.

Output channels are registered from the registry per source channel ID and
cached; `mcap::Writer` deduplicates schemas/channels by content. The clip ends
with `Writer::finish()`, which writes the summary section, footer and closing
magic — every clip is a complete, standalone MCAP of the same form as
`edgestream-rec`'s (`mcap::MessageStream` over a clip is the validity check the
unit tests use).

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

## Retention

None. The single file grows until the recording stops. Punching holes below a
retention horizon with `fallocate(FALLOC_FL_PUNCH_HOLE)` — keeping offsets
stable under the live writer — is the designed follow-up, tracked in beads as
`ros2_subscribe-wkg` (constraints: schema/channel custody, standard readers
losing access, st_size growth, punch/extraction coordination).

## Run

```bash
nix develop --command cargo run -p edgestream-rec-cont   # --record-dir ./record-cont --out-dir ./triggered-cont
```

Needs `scripts/record-continuous.sh` running (for `./record-cont`) and a
trigger publisher (`trigger-pub`). `RUST_LOG=debug` raises verbosity.
