# edgestream-rec-cont — Architecture

For the business rationale and value proposition see [OVERVIEW.md](OVERVIEW.md).
For the user-facing quickstart, configuration reference, and pipeline setup see
[README § Triggered recording](../../README.md#triggered-recording).
For deep implementation rationale and concurrency invariants see [CLAUDE.md](CLAUDE.md).

## Module map

| Source file | Role |
|---|---|
| `src/main.rs` | Entry point, configuration, supervision, admission, per-trigger flow, extraction worker pool |
| `src/tail.rs` | Incremental file scan: extent index, schema/channel registry, coverage watch |
| `src/clip.rs` | Window extraction: reads planned extents, assembles and publishes a standalone MCAP clip |
| `src/watch.rs` | `Watch<T>` primitive: `Mutex` + `Condvar`; coverage notification between the tail and trigger handlers |

## Data flow

```
ros2 bag record ──▶ <record_dir>/<bag>_0.mcap   (one growing file, append-only)
                              │
                         [tail thread]
                         incremental scan
                              │
               ┌──────────────┼──────────────┐
               ▼              ▼              ▼
          extent index   schema/channel  coverage watch
          (byte ranges   registry        (high-water log_time
           + timestamps)  (per channel)   + ended flag)
               │              │              │
               └──────────────┴──────────────┘
                                    │
                       [trigger-<ns> thread]  ◀── /events/edgestream/trigger
                         1. sleep postroll
                         2. wait coverage
                         3. ──▶ ExtractJob ──▶ [extract-N thread]
                                                 plan_window snapshot
                                                 read extents (read_at)
                                                 assemble clip in .capturing/
                                                 move atomically ──▶ out_dir/
                         4. publish /events/edgestream/recorded
```

## Thread model

Six thread roles run for the process's lifetime; one short-lived thread is added per admitted trigger:

| Thread | Count | Role |
|---|---|---|
| `tail` | 1 | Incremental MCAP scan; feeds extent index, registry, and coverage watch |
| `node-spin` | 1 | Pumps the r2r DDS executor; feeds the typed trigger stream |
| `trigger-consumer` | 1 | Drains the trigger stream; spawns one `trigger-<ns>` per admitted trigger |
| `extract-N` | `extract_parallelism` (≥ 1) | FIFO clip extraction worker pool |
| `signals` | 1 | Forwards SIGINT/SIGTERM to `supervise` for orderly shutdown |
| `trigger-<ns>` | ≤ 16 concurrent | Per-trigger wait → extract → announce flow |

`supervise()` selects on four channels (tail, spin, consumer, signal). Any thread ending unexpectedly — clean return or panic — exits the process non-zero for a supervisor to restart. A SIGINT/SIGTERM exits zero. There is no async runtime; all coordination uses plain OS threads, `crossbeam_channel`, and `Mutex`/`Condvar`.

## Tailing

The tail keeps the recording open and reads only the bytes added since the previous pass. Two MCAP properties make this sound:

1. **Append-only while recording.** Bytes below the current end of file never change; the summary and footer are appended only at close.
2. **Length-prefixed records.** A record whose declared length runs past the current file end is still being written; the scan stops there and resumes on the next pass.

Each pass maintains three shared artefacts:

- **Extent index** — contiguous byte ranges (capped at 4 MiB) with their min/max `log_time`. Extraction reads only the extents overlapping the requested window.
- **Schema/channel registry** — one `ChannelDef` per channel ID, used by extraction to register schemas and channels in the output MCAP writer.
- **Coverage watch** (`Watch<Coverage>`) — the highest `log_time` on disk and an `ended` flag (DataEnd/Footer seen). Trigger handlers block on this until the recording provably covers their window end.

Only the 14-byte `Message` record prefix (channel id, sequence, `log_time`) is read during the tail; message bodies are first touched at extraction.

**Coverage contract.** The tail trusts rosbag2's approximately non-decreasing `log_time` order (the single writer stamps `log_time` at receive). `grace_secs` absorbs flush latency; it is not a reordering budget — a message the recorder appends out of order after a window has already been cut is not guaranteed into that clip.

## Per-trigger flow

Each admitted trigger spawns a `trigger-<ns>` thread that runs four steps:

1. **Postroll sleep.** Suspend until the system clock passes `trigger_time + postroll`.
2. **Coverage wait.** Block on the coverage watch until `high_water_ns ≥ end_ns` (or `ended`). The `grace_secs` timeout fires when recorded topics go quiet, cutting the clip from what exists.
3. **Extraction.** Enqueue an `ExtractJob` on the FIFO worker channel and block on the reply. The worker snapshots `plan_window` at dequeue time (ensuring the freshest index), reads the planned extents with `read_at` (no shared seek state with the tail), and assembles the clip.
4. **Announce.** Publish `/events/edgestream/recorded` — only after the clip is in `out_dir` and `out_dir` is fsynced, so the announced path is always crash-durable.

## Clip assembly and atomic publication

Extraction is two-staged so `out_dir` only ever holds complete clips:

1. **Stage in `.capturing/`.** Assemble the clip in `out_dir/.capturing/`, call `Writer::finish()` (summary + footer + closing magic), and `sync_all` the file.
2. **Atomic publish.** `hard_link` into `out_dir` under the desired name, unlink the staged path, fsync `out_dir`. `hard_link` fails with `AlreadyExists` when a duplicate trigger races for the same name — resolved by appending `_<n>` to the desired name. A `StagedClip` unlinks the staged file on drop, so an early return or panic between the two stages strands nothing in `.capturing`.

`reset_capturing_dir()`, called once at startup, deletes and recreates `.capturing`, bounding crash litter to a single run.

## Recorder restart handling

The recording is discovered as the newest `*.mcap` under `record_dir`. When the tailed inode no longer matches the discovered path (rosbag2 recreates the bag directory on restart), the index resets and the new file is tailed from scratch. In-flight extractions hold their own `Arc<File>` and complete safely against the deleted inode. Data from a replaced recording is never recovered into any subsequent clip.

## Admission control

`Admission` is an `AtomicUsize` counter capped at `MAX_ACTIVE_TRIGGERS` (16). The trigger consumer calls `try_acquire()` before spawning each handler; a trigger arriving when all 16 slots are held is rejected with `error!` — no handler, no clip, no announcement. The permit is held by the handler thread and released on drop, including on panic.

## Damage tolerance

The tail and extraction tolerate localized file damage:

- **Chunk-level:** a chunk that fails decompression, CRC, or interior parsing is dropped whole. Its messages are buffered and written only if the chunk iterates cleanly, so a bad CRC cannot quietly pass corrupt messages into a clip.
- **Record-level:** an unparseable `Schema`/`Channel` or a message on an unknown channel is warned and skipped. The length-prefix framing remains intact, so the scan continues from the next record.
- **Framing faults:** a record length exceeding `MAX_RECORD_LEN`, or an IO error reading a record header/body, has no resync point. The scan stops there and retries from exactly that offset under a bounded, backing-off `MAX_SCAN_FAULTS` budget. Exhausting the budget exits the process non-zero.

Degraded clips (records skipped or chunks dropped) are announced with a warning rather than silently; a failed extraction leaves nothing in `out_dir`.
