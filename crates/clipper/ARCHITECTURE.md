# clipper — Architecture

For the business rationale and value proposition see [OVERVIEW.md](OVERVIEW.md).
For the user-facing quickstart, configuration reference, and pipeline setup see
[README § Triggered recording](../../README.md#triggered-recording).
For deep implementation rationale and concurrency invariants see [CLAUDE.md](CLAUDE.md).

## Module map

| Source file | Role |
|---|---|
| `src/main.rs` | Entry point, configuration, supervision, admission, per-trigger flow, staging worker pool |
| `src/tail.rs` | Recording collection: extent index per recording, schema/channel registry per recording, collection-wide coverage watch, retention pruning |
| `src/discover.rs` | `NewFileWatchIterator`: lazy directory iterator yielding each new `*.mcap` file once (by `(dev,ino)` identity, mtime-ordered), one per `next()` |
| `src/clip.rs` | Window extraction: reads planned extents, assembles and publishes a standalone MCAP clip |
| `src/watch.rs` | `Watch<T>` primitive: `Mutex` + `Condvar`; coverage notification between the tail and trigger handlers |

## Data flow

```
ros2 bag record ──▶ <record_dir>/<bag>_0.mcap  (one growing file, append-only)
                    <record_dir>/<bag>_1.mcap  (next split, after rollover)
                              │
                         [tail thread]
                    NewFileWatchIterator
                    discovers each *.mcap
                    incrementally scans current
                              │
               ┌──────────────┼──────────────┐
               ▼              ▼              ▼
        per-recording     per-recording   coverage watch
        extent index      schema/channel  (collection-wide
        (byte ranges      registry        high_water_ns)
        + timestamps)     (per channel)
               │              │              │
               └──────────────┴──────────────┘
                     TailState (VecDeque<RecordingIndex>)
                                    │
                       [trigger-<ns> thread]  ◀── /events/momentedge/trigger
                         1. sleep postroll
                         2. wait coverage (high_water_ns ≥ end_ns, or grace)
                         3. plan_window snapshot → Vec<WindowPlan> (one per recording)
                            ──▶ StageJob ──▶ [stage-N thread] (one per plan)
                                               read extents (read_at)
                                               assemble segment in .capturing/
                         4. publish staged segments atomically ──▶ out_dir/
                         5. publish /events/momentedge/recorded (all filenames)
```

## Thread model

Six thread roles run for the process's lifetime; one short-lived thread is added per admitted trigger:

| Thread | Count | Role |
|---|---|---|
| `tail` | 1 | Discovery + incremental MCAP scan; feeds the recording collection, per-recording indexes, and coverage watch |
| `node-spin` | 1 | Pumps the r2r DDS executor; feeds the typed trigger stream |
| `trigger-consumer` | 1 | Drains the trigger stream; spawns one `trigger-<ns>` per admitted trigger |
| `stage-N` | `extract_parallelism` (≥ 1) | FIFO staging worker pool; runs one `stage_clip` per `StageJob` |
| `signals` | 1 | Forwards SIGINT/SIGTERM to `supervise` for orderly shutdown |
| `trigger-<ns>` | ≤ 16 concurrent | Per-trigger wait → multi-file snapshot → stage → announce flow |

`supervise()` selects on four channels (tail, spin, consumer, signal). Any thread ending unexpectedly — clean return or panic — exits the process non-zero for a supervisor to restart. A SIGINT/SIGTERM exits zero. There is no async runtime; all coordination uses plain OS threads, `crossbeam_channel`, and `Mutex`/`Condvar`.

## Tailing

The tail keeps each discovered recording open and reads only the bytes added since the previous pass. Two MCAP properties make this sound:

1. **Append-only while recording.** Bytes below the current end of file never change; the summary and footer are appended only at close.
2. **Length-prefixed records.** A record whose declared length runs past the current file end is still being written; the scan stops there and resumes on the next pass.

Each pass maintains three artefacts per `RecordingIndex`, plus one collection-wide watch:

- **Extent index** — contiguous byte ranges (capped at 4 MiB) with their min/max `log_time`. Extraction reads only the extents overlapping the requested window.
- **Schema/channel registry** — one `ChannelDef` per channel ID, used by extraction to register schemas and channels in the output MCAP writer.
- **Coverage watch** (`Watch<Coverage>`) — the collection-wide maximum `log_time` (`high_water_ns`). Trigger handlers block on this until it reaches their window end.

Only the 14-byte `Message` record prefix (channel id, sequence, `log_time`) is read during the tail; message bodies are first touched at extraction.

**Coverage contract.** The tail trusts rosbag2's approximately non-decreasing `log_time` order (the single writer stamps `log_time` at receive). `grace_secs` absorbs flush latency; it is not a reordering budget — a message the recorder appends out of order after a window has already been cut is not guaranteed into that clip.

## Recording collection

`TailState` owns a `VecDeque<RecordingIndex>` in time order (oldest first). Each `RecordingIndex` carries an explicit lifecycle state:

| State | Meaning |
|---|---|
| `New` | Indexed (fd open), waiting behind the current scan |
| `Tailing` | The single recording being incrementally scanned |
| `Ended` | Fully scanned to EOF; eligible for retention pruning |

Exactly one recording is `Tailing` at a time (`current`). The tail scans it to `Ended` before advancing to the next. This strict ordering is what makes the collection-wide `high_water_ns` sound: a later recording's coverage cannot advance before an earlier one is complete, so a handler waking on `high_water_ns >= end_ns` is guaranteed that all recordings up to that point have been fully scanned.

**Discovery** is `NewFileWatchIterator` — a lazy, non-fused iterator that yields each new `*.mcap` file once, reading the directory only when called (no background thread). It tracks files by `(dev, ino)` identity rather than a timestamp cursor: a file under tail grows and its mtime advances, so a cursor would re-yield it and index a phantom duplicate; recording the yielded inode (and forgetting inodes no longer on disk) yields each recording exactly once. mtime orders the unseen files oldest-first. Each poll drains the iterator; each yielded path is opened and inserted as a `New` recording. At startup the newest existing file (by mtime) is adopted directly and the iterator seeded past every file present then, so older pre-existing bags are not indexed. Clipper recovers only rollovers it observes during its own run — a split that existed before startup contributes nothing to any subsequent trigger.

**Rollover / end-of-recording detection.** `current` transitions to `Ended` on any of three signals; a final scan to EOF runs first in each case so no complete trailing record is lost:

1. **Footer/DataEnd scanned** — rosbag2 closed the file cleanly (split or stop).
2. **Inode vanished or replaced** — the tailed path no longer resolves to the open fd's inode (the record script wiped the bag directory). The open `Arc<File>` keeps the unlinked inode readable through the final scan.
3. **Successor present, length stable** — `NewFileWatchIterator` yielded a newer file while `current` produced no new bytes; rosbag2 already closed the old file (covers abrupt splits whose footer never flushed).

`inode_changed` owns the "is my file still live" check (own path vs own fd). The iterator owns the "is there a newer file" check. The two are separate concerns.

**Retention.** Every tail poll prunes `Ended` recordings whose newest data (`max_log_time`) is older than `now - watch_old_files_duration` (default 600 s). Pruning is file-granular — never the `current` recording, never mid-file. Dropping a `RecordingIndex` releases its `Arc<File>`, closing the descriptor once no in-flight plan still holds a clone. When `--delete-old-files` is set, a prune also unlinks the expired file from disk; an in-flight extraction's own `Arc<File>` clone keeps the unlinked inode readable to completion. See the [README](../../README.md#configuration) for the flag reference.

**`high_water_ns` is monotonic across prunes.** A pruned file's `max_log_time` is below the watch floor and therefore never held the collection maximum, so dropping it never lowers the high-water. Handlers never see coverage regress.

## Per-trigger flow

Each admitted trigger spawns a `trigger-<ns>` thread (`record_clip` in `main.rs`) that runs these steps:

1. **Postroll wall floor.** Sleep until the system clock passes `trigger_time + postroll`. `checked_sub` reads the clock once per iteration, preventing underflow if the clock crosses `end_ns` between the check and the sleep.
2. **Coverage wait.** Block on the collection-wide coverage watch until `high_water_ns ≥ end_ns`, bounded by `grace_secs`. The timeout fires when recorded topics go quiet; the clip is then cut from whatever is on disk with a warning. There is no `ended` short-circuit: a window inside an already-closed recording is satisfied as soon as `high_water_ns` reaches `end_ns` (which happens at or before the footer scan), so the cost of the simplified predicate is narrow — only a window whose end is past the last message with no successor waits out the full grace.
3. **Multi-file snapshot.** Call `tailer.plan_window(start_ns, end_ns)` once under the mutex, producing a `Vec<WindowPlan>` — one plan per recording whose extents overlap the window, oldest first. Each plan pins its recording's `Arc<File>`, so a retention prune or rollover after this point cannot pull the bytes out.
4. **Stage.** Enqueue one `StageJob` per plan on the FIFO staging worker channel and block on each reply. Workers run `clip::stage_clip` — the bulk copy into `.capturing/`. When no recording covers the window an empty plan is staged instead, so every trigger produces a valid (possibly empty) clip.
5. **Publish.** Drop empty segments when the window produced real data elsewhere (keep one if all are empty). Name the segments: one segment keeps the bare `<trigger_ns>_<name>.mcap`; two or more get `<base>_00.mcap`, `<base>_01.mcap`, … Atomically publish each staged segment into `out_dir`.
6. **Announce.** Publish one `/events/momentedge/recorded` with all segment filenames in `filenames[]` — only after every segment is in `out_dir` and fsynced, so every announced path is crash-durable.

## Clip assembly and atomic publication

Extraction is two-staged so `out_dir` only ever holds complete clips:

1. **Stage in `.capturing/`.** Assemble the clip in `out_dir/.capturing/`, call `Writer::finish()` (summary + footer + closing magic), and `sync_all` the file.
2. **Atomic publish.** `hard_link` into `out_dir` under the desired name, unlink the staged path, fsync `out_dir`. `hard_link` fails with `AlreadyExists` when a duplicate trigger races for the same name — resolved by appending `_<n>` to the desired name. A `StagedClip` unlinks the staged file on drop, so an early return or panic between the two stages strands nothing in `.capturing`.

`reset_capturing_dir()`, called once at startup, deletes and recreates `.capturing`, bounding crash litter to a single run.

## Recorder restart handling

A recorder restart (the record script wipes the bag directory and starts fresh) is detected via `inode_changed`: the tailed path no longer resolves to the open fd's inode. The `current` recording transitions to `Ended`, and the inode stays readable through any in-flight extractions holding an `Arc<File>` clone.

Bag splits are detected the same way — either a footer on disk, or a successor appearing with `current` length-stable — and handled as a normal `Ended` transition. The successor is already indexed as a `New` recording (discovered by `NewFileWatchIterator`), so the tail advances to it without resetting any other index.

Recordings that were observed and indexed by clipper during its run are retained within the watch window (`watch_old_files_duration`) and remain plannable for triggers whose preroll reaches into them. Recordings that existed on disk before clipper started are not indexed — the startup seed adopts the newest file and seeds the iterator past it, so the pre-existing backlog is never scanned. A trigger fired shortly after startup whose preroll would reach into a prior split that existed before launch gets no segment from that file.

## Admission control

`Admission` is an `AtomicUsize` counter capped at `MAX_ACTIVE_TRIGGERS` (16). The trigger consumer calls `try_acquire()` before spawning each handler; a trigger arriving when all 16 slots are held is rejected with `error!` — no handler, no clip, no announcement. The permit is held by the handler thread and released on drop, including on panic.

## Damage tolerance

The tail and extraction tolerate localized file damage:

- **Chunk-level:** a chunk that fails decompression, CRC, or interior parsing is dropped whole. Its messages are buffered and written only if the chunk iterates cleanly, so a bad CRC cannot quietly pass corrupt messages into a clip.
- **Record-level:** an unparseable `Schema`/`Channel` or a message on an unknown channel is warned and skipped. The length-prefix framing remains intact, so the scan continues from the next record.
- **Framing faults:** a record length exceeding `MAX_RECORD_LEN`, or an IO error reading a record header/body, has no resync point. The scan stops there and retries from exactly that offset under a bounded, backing-off `MAX_SCAN_FAULTS` budget. Exhausting the budget exits the process non-zero.

Degraded clips (records skipped or chunks dropped) are announced with a warning rather than silently; a failed extraction leaves nothing in `out_dir`.
