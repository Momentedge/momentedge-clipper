# Architecture

A technical overview of how Momentedge Clipper is built. For what it does and
how to run it, start with the [README](README.md). For deep implementation
rationale and concurrency invariants, see
[`crates/clipper/CLAUDE.md`](crates/clipper/CLAUDE.md).

## System overview

The system is **two cooperating processes that share only a file**:

- A continuous `ros2 bag record` (rosbag2 with MCAP storage) writes a growing
  recording. clipper never starts, stops, or configures it.
- `clipper` keeps that recording open, tails it, and on each trigger cuts a
  standalone clip out of the bytes already on disk.

The split is deliberate. Recording and deciding-what-matters are different jobs
with different change rates: the recorder keeps its own config, lifecycle, and
support story, and clipper can be added, upgraded, or removed without touching
the capture path. The two communicate only through the recording file — there is
no shared memory, no IPC, and (under the `ros` interface) no `rosbag2_interfaces`
service call. Coverage is read from the file itself, never from a split-event
topic.

clipper **decodes nothing but timestamps**. While tailing it reads only each
message's header; while extracting it copies raw serialized message bytes
straight through. It is therefore agnostic to the message types in the
recording, and a clip is a complete, standard MCAP file on both sides.

## Module map

clipper is one crate, [`crates/clipper`](crates/clipper); `momentedge_msgs` is
the local ROS 2 interface package defining `Trigger`/`Recorded`.

| Source file | Role |
|---|---|
| `src/main.rs` | Entry point, configuration (clap), admission gate, thread supervision |
| `src/tail.rs` | Recording collection: per-recording extent index + schema/channel registry, collection-wide coverage watch, retention pruning, the trigger tap |
| `src/discover.rs` | `NewFileWatchIterator`: lazy directory iterator yielding each new `*.mcap` once, by `(dev,ino)` identity, mtime-ordered |
| `src/handler.rs` | The ROS- and encoding-agnostic per-trigger flow and the staging worker pool |
| `src/clip.rs` | Window extraction: read planned extents, assemble and atomically publish a standalone MCAP clip |
| `src/interface.rs` | `trait Interface` + the `ros` and `mcap` implementations and their announcers |
| `src/decode.rs` | `decode_trigger`: decode a trigger payload by its MCAP `message_encoding` (`cdr`, `json`) |
| `src/trigger.rs` | The neutral contract: `Trigger`, `Stamp`, `TriggerRecord`, `Completion`, the `Announce` trait, `now_ns` |
| `src/supervision.rs` | `spawn_supervised`/`harvest_panic`: pair each long-lived thread with a channel carrying its verdict |
| `src/watch.rs` | `Watch<T>`: a `Mutex` + `Condvar` primitive for coverage notification |

## Data flow

```
ros2 bag record ──▶ <record_dir>/<bag>_0.mcap   (one growing file, append-only)
                    <record_dir>/<bag>_1.mcap   (next split, after a rollover)
                              │
                         [tail thread]
                    NewFileWatchIterator discovers each *.mcap;
                    the current file is incrementally scanned
                              │
               ┌──────────────┼──────────────┐
               ▼              ▼              ▼
        per-recording    per-recording   coverage watch
        extent index     schema/channel  (collection-wide
        (byte ranges     registry        high_water_ns)
        + min/max time)  (per channel)
               │              │              │
               └──────────────┴──────────────┘
                  TailState (VecDeque<RecordingIndex>)
                                    │
                       [trigger-<ns> thread]  ◀── a decoded Trigger
                         1. sleep out the postroll
                         2. wait for coverage (high_water_ns ≥ end_ns, or grace)
                         3. plan_window snapshot → Vec<WindowPlan> (one per recording)
                            ──▶ StageJob ──▶ [stage-N worker] assembles a segment in .capturing/
                         4. publish staged segments atomically ──▶ out_dir/
                         5. announce completion (ros: publish Recorded; mcap: the move is the signal)
```

## Thread model

There is **no async runtime**; all coordination uses plain OS threads,
`crossbeam_channel`, and `Mutex`/`Condvar`. Long-lived threads run for the
process's lifetime; one short-lived thread is added per admitted trigger.

| Thread | Count | Role |
|---|---|---|
| `tail` | 1 | Discovery + incremental MCAP scan; feeds the recording collection, per-recording indexes, and the coverage watch. Under the `mcap` interface it also taps trigger-topic messages out of the scan. |
| `interface` | 1 | Owns the active trigger source. For `ros` it internally runs the node spin (pumping the DDS executor) and the subscription drain; for `mcap` it drains the tail's trigger tap and decodes each raw trigger. It fires the per-trigger callback. |
| `stage-N` | `extract_parallelism` (≥ 1) | FIFO staging worker pool; runs one `clip::stage_clip` per `StageJob`. |
| `signals` | 1 | Forwards the first SIGINT/SIGTERM to `supervise` for an orderly shutdown. |
| `trigger-<ns>` | ≤ 16 concurrent | Per-trigger wait → snapshot → stage → publish → announce. |

`supervise()` selects on three channels — **tail**, **interface**, and
**signal**. A delivered signal is the requested, orderly stop: the process exits
0. Either critical thread ending — clean return or panic — is a fault: the
process exits non-zero for a supervisor to restart it. A dead tail would
silently degrade every clip to a grace-timeout cut; a dead interface would
silently stop delivering triggers — so neither is allowed to fail quietly. The
`ros` interface's node spin and subscription drain are supervised *inside* the
interface thread and surface as one interface fault, so supervision is uniform
regardless of interface.

## Tailing a live MCAP

The tail keeps each discovered recording open and reads only the bytes added
since the previous pass. Two MCAP properties make this sound:

1. **Append-only while recording.** Bytes below the current end of file never
   change; the summary and footer are appended only at close. Everything behind
   the last complete record is immutable.
2. **Length-prefixed records.** A record whose declared length runs past the
   current file end is still being written — the scan stops there and resumes on
   the next pass. An in-progress file is indistinguishable from a
   crash-truncated one, which MCAP readers are designed to tolerate.

Both properties are a producer requirement in disguise: a recording clipper can
tail live must append complete records only and never seek back to rewrite one
already on disk. The Rust `mcap` crate's chunked writer (`use_chunks(true)`, its
default) breaks this — it writes each `Chunk` record's header with a
placeholder length (`u64::MAX`) and back-patches the true length only once the
chunk closes, so the file is parseable only after that patch lands and cannot
be tailed live. `rosbag2` satisfies the requirement on every profile (the
unchunked `fastwrite` profile has no chunks to back-patch; its chunked profiles
write each chunk record atomically, never seeking back mid-chunk);
[`examples/custom-mcap-writer`](examples/custom-mcap-writer/README.md) writes
with `use_chunks(false)` for the same reason.

Each pass maintains three artefacts per recording, plus one collection-wide
watch:

- **Extent index** — contiguous byte ranges (capped at 4 MiB) carrying the
  min/max `log_time` and `publish_time` of the messages they hold. Extraction
  reads only the extents whose stamp on the active `--time-source` overlaps the
  requested window; the overlap test uses real min/max, so no in-window message
  is missed. The window's [time source](#time-source) picks which of the two
  spans the overlap and the message-membership test read.
- **Schema/channel registry** — one owned `Schema`/`Channel` per channel ID,
  used by extraction to register channels in the output writer. Chunked
  recordings carry these inside chunks, so chunks are decompressed during the
  tail; the default unchunked `fastwrite` profile skips that cost.
- **Coverage watch** (`Watch<Coverage>`) — a collection-wide high-water per
  source: the maximum `log_time` (`high_water_ns`) and the maximum `publish_time`
  (`publish_high_water_ns`). A trigger handler blocks on the high-water of the
  source its window lives in. Each rises independently and never regresses.

Only the 22-byte `Message` record fixed header (channel id, sequence,
`log_time`, `publish_time`) is read during the tail, in one read; message bodies
are first touched at extraction. Both stamps are carried through so a window can
live on either. The one exception is the opt-in **trigger tap** under the `mcap`
interface, which also reads the full body of messages on the trigger topic.

**Coverage contract.** On `log` the tail trusts rosbag2's approximately
non-decreasing `log_time` order (a single writer stamps `log_time` at receive),
so the high-water is a completeness proof. On `publish` it is a liveness signal
only: `publish_time` has no ordering guarantee, so a message can arrive after a
cut with an in-window `publish_time` and be missing from that clip. `grace_secs`
absorbs flush latency on either source; it is not a reordering budget.

## Recording collection

`TailState` owns a `VecDeque<RecordingIndex>` in time order, oldest first. Each
recording carries an explicit lifecycle state:

| State | Meaning |
|---|---|
| `New` | Indexed (fd open), waiting behind the current scan |
| `Tailing` | The single recording being incrementally scanned |
| `Ended` | Fully scanned to EOF; eligible for retention pruning |

Exactly one recording is `Tailing` at a time. The tail scans it to `Ended`
before advancing. This strict oldest-first ordering is what makes the
collection-wide `high_water_ns` sound: a later recording's coverage cannot
advance before an earlier one is complete, so a handler waking on
`high_water_ns ≥ end_ns` knows every recording up to that point is fully scanned.

**Discovery** is `NewFileWatchIterator` — a lazy iterator that yields each new
`*.mcap` once, reading the directory only when polled. It tracks files by
`(dev, ino)` identity rather than a timestamp cursor: a file under tail grows and
its mtime advances, so a cursor would re-yield it and index a phantom duplicate.
At startup the newest existing file is adopted directly and the iterator seeded
past every file present then — so older pre-existing bags are never indexed.
clipper recovers only rollovers it observes during its own run.

**Rollover / end-of-recording detection.** `current` transitions to `Ended` on
any of three signals, each preceded by a final scan to EOF so no trailing record
is lost:

1. **Footer/DataEnd scanned** — rosbag2 closed the file cleanly (split or stop).
2. **Inode vanished or replaced** — the tailed path no longer resolves to the
   open fd's inode (a recorder restart wiped the bag directory). The open
   `Arc<File>` keeps the unlinked inode readable through the final scan.
3. **Successor present, length stable** — a newer file appeared while `current`
   produced no new bytes (covers abrupt splits whose footer never flushed).

**Retention.** Every tail poll prunes `Ended` recordings whose newest data is
older than `now − watch_old_files_duration` (default 600 s). Pruning is
file-granular — never the `current` recording, never mid-file. Dropping a
recording releases its `Arc<File>`, closing the descriptor once no in-flight plan
still holds a clone. With `--delete-old-files`, a prune also unlinks the file;
an in-flight extraction's own clone keeps the unlinked inode readable to
completion (POSIX unlink-while-open). `high_water_ns` is monotonic across prunes:
a pruned file was below the watch floor and never held the maximum, so handlers
never see coverage regress.

## Per-trigger flow

Each admitted trigger runs on its own `trigger-<ns>` thread, so overlapping
windows are cut concurrently against the one shared tail. The window centres on
the `anchor_ns` the interface resolved (see [The two interfaces](#the-two-interfaces))
and lives on the active `--time-source`:

1. **Postroll wall floor.** Sleep until the system clock passes
   `anchor + postroll`. The wall floor is always the system clock, whatever the
   time source.
2. **Coverage wait.** Block on the coverage watch until the window's source
   high-water reaches `end_ns`, bounded by `grace_secs`. On timeout the clip is
   cut from whatever is on disk, with a warning.
3. **Multi-file snapshot.** `plan_window(start_ns, end_ns, source)` produces a
   `Vec<WindowPlan>`, one per recording whose extents overlap the window on
   `source`, oldest first. Each plan pins its recording's `Arc<File>`, so a later
   prune or rollover cannot pull the bytes out.
4. **Stage.** Enqueue one `StageJob` per plan on the FIFO staging channel and
   block on each reply. A worker copies each message whose stamp on `source` is
   in the window. A window covered by nothing still stages one empty plan, so
   every trigger produces a valid (possibly empty) clip.
5. **Publish.** Drop empty segments when the window produced real data elsewhere.
   One segment keeps the bare `<anchor_ns>_<name>.mcap`; multiple get
   `_00`/`_01`/… suffixes. Each is published into `out_dir` atomically.
6. **Announce** one completion through the active interface — only after every
   segment is in `out_dir` and fsynced, so every announced path is crash-durable.

## Clip assembly and atomic publication

Extraction reads each planned extent with `read_at` and walks its records with
its own opcode + length framing — the same walk the tail performed, so the
extent boundaries are known to tile. Messages whose `log_time` falls in the
inclusive window are written through with their raw serialized bytes; CDR bodies
are never decoded. The clip writer is built from explicit `mcap::WriteOptions`
with the codec set deliberately (`--clip-compression`), and finished with
`Writer::finish()` (summary + footer + closing magic) so every clip is a
complete, standalone MCAP file.

Publication is **two-staged** so `out_dir` only ever holds finished clips:

1. **Stage in `.capturing/`.** Assemble the clip in `out_dir/.capturing/`,
   `Writer::finish()` it, and `sync_all` the file.
2. **Atomic publish.** `hard_link` into `out_dir` under the desired name, unlink
   the staged path, fsync `out_dir`. `hard_link` (not `rename`) fails with
   `AlreadyExists` rather than silently clobbering an earlier clip when a
   duplicate trigger races for the same name — resolved by an `_<n>` suffix
   retry. A `StagedClip` is `#[must_use]` and unlinks the staged file on drop, so
   an early return or panic between the stages strands nothing.

`.capturing/` is a *subdirectory* of `out_dir` so the two always share a
filesystem and the move is a true atomic link. `reset_capturing_dir()`, called
once at startup, clears it, bounding crash litter to a single run.

## Restart and rollover recovery

A recorder restart (the record script wipes the bag directory and starts fresh)
is detected via `inode_changed`: the tailed path no longer resolves to the open
fd's inode. `current` transitions to `Ended` and the inode stays readable
through any in-flight extraction holding an `Arc<File>` clone. Bag splits are
handled the same way as normal `Ended` transitions, and the successor — already
discovered as a `New` recording — becomes the next `current` without resetting
any other index.

Recordings clipper observed during its run are retained within the watch window
and remain plannable for triggers whose preroll reaches into them. Recordings
that existed before clipper started are never indexed: a trigger fired shortly
after startup whose preroll would reach into a pre-existing prior split gets no
segment from that file.

## The two interfaces

The trigger input and the completion output are one unit — an **interface** —
chosen by `--interface` (default `ros`). The recorder is decoupled around one
neutral boundary so the clip-cutting half (`handler.rs`) never learns of ROS or
any wire encoding:

- **`trigger.rs`** is the neutral contract (`Trigger`, `Stamp`, `Completion`,
  the `Announce` trait), depending on neither `r2r` nor `mcap`.
- **`decode.rs`** maps an MCAP channel's `message_encoding` to a decoder:
  `cdr` (the ROS 2 default) via r2r's rmw deserialization — the linked rmw
  library only, no ROS `Context`/`Node`, so it works ROS-free — and `json` via
  `serde_json`. Other encodings return an error the interface logs and skips.
- **`interface.rs`** holds `trait Interface` (statically dispatched) with
  `RosInterface` (owns a node and its internal spin thread, announces by
  publishing `Recorded`) and `McapInterface` (drains the trigger tap, announces
  via a no-op — the clip's move into `out_dir` is the only signal).

**The anchor seam.** An interface resolves each trigger's anchor — the instant
its window centres on — and hands it to the handler alongside the neutral
`Trigger`, so the clip-cutting half never derives an anchor itself. The four
interface × `--time-source` cells resolve it thus:

| | `--time-source log` | `--time-source publish` |
|---|---|---|
| **`--interface ros`** | `now` at the subscription instant | the trigger's `trigger_time` |
| **`--interface mcap`** | the trigger record's `log_time` | the trigger record's `publish_time` |

A live ROS trigger carries no recording stamp and r2r surfaces no wire timestamp,
so the ROS interface anchors on `now` or the publisher's own `trigger_time`; an
in-recording trigger carries its own stamps, the faithful ones since a publisher
cannot align a wire timestamp to the recording's clock.

**The rejection gate.** `trigger_time` is read in exactly one cell — `ros` +
`publish` — where it is the anchor. Every other cell anchors on a transport stamp
and *rejects* a trigger that sets a non-zero `trigger_time`: logged at `error!`,
no clip cut, no `Recorded` — a single admission gate before any handler runs.
Sending it where it is ignored would silently anchor the window on the trigger's
arrival rather than the requested instant, so the request is refused loudly
instead of mis-served. The `Completion` still echoes the trigger's `trigger_time`
unchanged.

`main.rs` wires the selected interface to the tail and runs a generic
`drive<I: Interface>`. The `ros` interface talks to the ROS graph
(`/events/momentedge/trigger` in, `/events/momentedge/recorded` out); the `mcap`
interface has no ROS surface at all.

## Admission control

`Admission` is an `AtomicUsize` counter capped at `MAX_ACTIVE_TRIGGERS` (16). The
interface thread calls `try_acquire()` before spawning each handler; a trigger
arriving when all 16 slots are held is rejected with `error!` — no handler, no
clip, no announcement. The permit rides in the handler thread and returns on
drop, including on panic, so the bound never ratchets down. The cap is a
flood-sanity bound, not a resource budget: an active handler is mostly a parked
thread, and the heavy copy is already serialized by the staging pool.

## Damage tolerance

The tail and extraction tolerate localized file damage the way the MCAP format
is designed to be salvaged:

- **Chunk-level:** a chunk that fails decompression, CRC, or interior parsing is
  dropped whole — its messages are buffered and written only if it iterates
  cleanly, so a bad CRC cannot pass corrupt messages into a clip.
- **Record-level:** an unparseable `Schema`/`Channel`, or a message on an
  unknown channel, is warned and skipped; the length-prefix framing stays intact
  so the scan continues from the next record.
- **Framing faults:** a record length exceeding `MAX_RECORD_LEN`, or an IO error
  reading a record, has no resync point. The scan stops there and retries from
  exactly that offset under a bounded, backing-off `MAX_SCAN_FAULTS` budget;
  exhausting it exits the process non-zero.

Degraded clips (records skipped or chunks dropped) are counted and announced with
a warning rather than silently; a failed extraction leaves nothing in `out_dir`.

**Detection limit:** the leniency only catches damage loud enough to break
parsing or a CRC. The default `fastwrite` profile is unchunked and carries no
CRCs, so corruption inside a message *body* that leaves the framing intact is
invisible to every MCAP reader and is copied into clips as-is.

## Time base

MCAP `log_time`, `publish_time`, the trigger stamp, and the wait clock are all
nanoseconds on the system (ROS) clock — this assumes the default (no
`use_sim_time`). The same arithmetic anchors the window identically whether the
trigger arrived live over ROS or was decoded out of the tailed MCAP.

## Time source

`--time-source` (`log` or `publish`, default `log`) selects the clock domain the
whole window lives in: the anchor it centres on, which messages fall inside,
which extents are read, and the coverage a cut waits for. It governs nothing
else. Every MCAP message carries both stamps; the tail indexes both, and the
window compares against whichever the flag selects. `log_time` is when the
producer received the message (approximately ordered on disk, so its coverage is
a completeness proof); `publish_time` is whatever the producer wrote — a DDS
source timestamp under `ros2 bag record`, a capture time from a momentedge writer
— which clipper never interprets and which may arrive out of order, so its
coverage is a liveness signal only. Retention always ages a recording out on its
`log_time`, independent of the window's source, so a producer cannot drive file
deletion through `publish_time`. On ROS 2 Humble `publish_time = log_time`
verbatim, so `publish` is a no-op there; it differs on Jazzy and newer.

## Deployment

clipper ships to an edge target (a Jetson running ROS 2 Humble) as a **native
build**, not a container or nix closure. The target runs a full ROS 2 install of
the same distro the recorder is built against, so `rosbag2` (MCAP storage),
`rcl`, `rmw_fastrtps_cpp`, and the standard message packages all come from the
host. Only `clipper` and the `momentedge_msgs` interface package are built.

They are built **against the host's own ROS 2 libraries** for ABI compatibility:
a nix-built binary would bake `/nix/store` RPATHs and load the nix closure rather
than the host's ROS, breaking interop with the host's other nodes. So the build
runs on the target (or an ABI-identical box of the same arch + distro) via
[`scripts/build-on-target.sh`](scripts/build-on-target.sh). Running natively
(no container) means all ROS 2 processes share the host `/dev/shm`, so FastDDS
shared-memory transport and direct DDS interop work.

clipper ships as **two Debian packages**: `ros-<distro>-momentedge-msgs` (the
`ament_cmake` interface package, built with bloom into `/opt/ros/<distro>`) and
`momentedge-clipper` (the Rust binary, built with cargo-deb), the latter
declaring an apt `Depends` on the former. The binary carries no bundled overlay
and no baked rpath; it resolves its typesupport through the standard
`/opt/ros/<distro>/setup.bash`, like every ROS executable.

The dev-shell build (Nix, per-distro), the CI matrix, and the Debian packaging
pipeline are documented in the repository's contributor skills — see
[`CLAUDE.md`](CLAUDE.md).
