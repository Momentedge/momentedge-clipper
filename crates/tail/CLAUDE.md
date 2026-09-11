# tail

What following a recording *still being written* costs on top of
[`clip`](../clip/CLAUDE.md): discovery, the recording collection and its
lifecycle, coverage, the scan-fault budget, retention, and — in `tail::handler` —
the two waits a cut from a growing file must clear before the shared cut path
runs. A consumer cutting from a recording nobody is writing links `clip` alone:
no successor to find, no lifecycle to run, nothing to wait for.

**No ROS anywhere by default**, and no async runtime. The seam between the three
crates, the feature matrix, and the clock domain every window lives in are one
level up in [`crates/CLAUDE.md`](../CLAUDE.md).

**The line between this crate and `clip` falls where lifecycle begins.**
`clip::index::RecordingIndex` is the pure per-recording index and knows nothing
of any other recording; `tail` wraps it in `Recording { id, state, index }`,
where the id fixing this recording's place in time order and the
`New`/`Tailing`/`Ended` state are exactly what *tailing* adds. Coverage and its
watch, retention and discovery stay on this side for the same reason — they are
about following a growing file, not about a recording.

## The scan pass

The tail keeps the recording open and never re-reads consumed bytes. A pass is
[`clip::index::scan_available`](../clip/CLAUDE.md#the-format-layer-why-a-live-mcap-can-be-read-clipindex),
a free function over bytes that holds nothing: the tailer builds the `ScanSeed`
(the still-open extent, the trigger tap, the trigger channels already known)
under its state lock, drops the lock, scans with no lock held, then folds the
returned `ScanDelta` into that recording's `RecordingIndex` (`apply_delta`),
records the new offset, and raises its own coverage. That order is why the file
IO — the whole cost of a pass — never contends with a handler's `plan_window`,
and why coverage advances only after the index a cut reads has been published.

`tail::Tailer` is also the `WindowPlanner` the cut path asks for a window's
plans, implemented over the live collection, so a window straddling a rollover
yields one plan per source recording.

## The coverage watch

`Coverage` is what a handler's second wait blocks on. It rides in a
`tail::Watch<Coverage>` (`Mutex` + `Condvar`, `src/watch.rs`) holding a
collection-wide high-water per source:
the highest `log_time` (`high_water_ns`) and the highest `publish_time`
(`publish_high_water_ns`), recomputed from every indexed recording's bounds
each time a delta is applied. It is the tail's and not the index's precisely because it spans the whole
collection — only something holding all the recordings can say how far they
provably reach. A handler waits on the high-water of its window's
[time source](../CLAUDE.md#time-source).
The `log` high-water is a completeness proof — messages land in the file in
(approximately) non-decreasing `log_time` order (rosbag2's single writer stamps
`log_time` at receive) and the tail scans recordings strictly oldest-first, one
at a time, so a later recording's coverage cannot advance before an earlier one
is complete. The `publish` high-water is a liveness signal only: `publish_time`
has no ordering guarantee, so a message can arrive after a cut with an in-window
`publish_time` and be missing from that clip. Both rise independently and never
regress (the watch only raises each).

## The scan-fault budget

A [framing fault](../clip/CLAUDE.md#the-format-layer-why-a-live-mcap-can-be-read-clipindex)
stops a pass at the faulted record and returns the delta accumulated before it;
the tail applies that partial delta like any other pass's, then retries from
exactly the returned offset — never earlier, never re-attaching or rescanning
from scratch, which the scan's resume invariant requires. Retries are bounded by
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

**Recorder restarts and bag splits** are the tail's alone — a consumer of one
finished recording has no successor to find and no lifecycle to run. Recordings
are discovered by `tail::discover::NewFileWatchIterator`
(`crates/tail/src/discover.rs`), a lazy iterator that yields new `*.mcap` files
one per `next()`. Each poll drains it; each yielded path is opened and inserted
as a `New` recording into the collection (`TailState`), pairing a fresh
`RecordingIndex` with the id and state that place it among the others. Files are
tracked by `(dev, ino)` **identity**, not a timestamp cursor: a file under tail
grows and its mtime (and ctime) advances, so a cursor would re-yield it every
poll and index the same recording as a phantom duplicate. The iterator records the inode
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

## Per-trigger flow

Each admitted trigger is handled on its own thread, so overlapping windows are
cut concurrently against the shared tail. The handler (`tail::handler`) is
generic over the [`Announce`](../clipper/CLAUDE.md#the-interface-abstraction) the active interface
supplies and knows only the neutral `Trigger`/`Completion` contract plus the
window's `anchor_ns` and [time source](../CLAUDE.md#time-source) — nothing of ROS or any
wire encoding. The interface resolves the `anchor_ns` (the window centre) and hands it
in; the handler never derives an anchor itself.

The handler's first act is to fold those into one `clip::manifest::CutRequest`
— the producer, the trigger, the anchor and the time source, with the window
bounds derived from them — which is then the only window value the flow below
carries. Everything that names the window afterwards, from the `info!` line to
each segment's manifest to the membership test on every copied message, reads
that one request.

**The flow crosses the crate seam at the waits.** `tail::handler::record_clip`
is steps 1–2 and nothing else: they are the only part that needs a file still
being written, and they are why `tail` exists at all. Steps 3–5 are
`clip::segment::cut_window` — the same code a consumer cutting from a finished
recording runs, which waits for nothing. Step 6 is back in
`tail::handler::handle_trigger`, because announcing a clip belongs with the
trigger it answers and not with the cut.

Admission is bounded by
[the binary's gate](../clipper/CLAUDE.md#the-anchor-seam-and-the-admission-gate): at most
`MAX_ACTIVE_TRIGGERS` (16) handlers may be active at once, and a trigger that
arrives while all of them are is rejected — logged with `error!` and otherwise
ignored: no handler runs, no clip is extracted, and no completion is announced.

1. **Wait out the postroll** (`record_clip`). Sleep until the system clock passes
   `anchor + postroll`. The wall floor is always the system clock, whatever the
   time source. `checked_sub` reads the clock once per iteration, so a clock that
   crosses `end_ns` between the check and the sleep cannot underflow.
2. **Wait for coverage** (`record_clip`). Block on the collection-wide coverage
   watch until the window's source high-water reaches `end_ns`
   (`c.for_source(time_source)`). A grace timeout (`grace_secs`, default 30 s)
   bounds the wait; on timeout the clip is cut from what exists, with a warning.
   On `log` a window whose end falls inside an already-scanned recording is
   satisfied as soon as the scan reaches `end_ns` (or at the footer), so only a
   window whose end is past the last recorded message with no successor waits
   out the full grace; on `publish` the high-water is a liveness signal, so a
   later out-of-order message can still be missed. The grace must exceed the
   recorder's flush latency: near zero for the fastwrite profile, roughly one
   chunk fill (chunk size / aggregate data rate) for chunked profiles. The
   wait's outcome is not only a log line: it travels into the cut as a
   `clip::manifest::WindowCoverage`, which is what every segment's manifest
   reports under `clip.short`.
3. **Multi-file snapshot** (`cut_window`). Call `planner.plan_window(start_ns,
   end_ns, time_source)` once — `tail::Tailer` is the planner here — producing a
   `Vec<WindowPlan>`: one plan per recording whose extents overlap the window on
   the active time source, oldest first. Each plan carries its own `Arc<File>`
   clone, so a retention prune or rollover after this snapshot cannot pull the
   bytes out.
4. **Stage** (`cut_window`) via the staging worker pool (`extract_parallelism`
   `stage-N` threads, default 1): one `StageJob` per plan is enqueued on the
   shared FIFO channel and `cut_window` blocks on each reply. A worker runs
   `clip::cut::stage_clip` into `.capturing/` and replies a `StagedClip`. Each
   job carries the `CutRequest` and the window's `Planned` facts (how many
   recordings were planned over, and the coverage verdict), which is how the
   trigger reaches the writer that stamps the segment's manifest. When no
   recording covers the window, one empty plan is staged so every trigger
   produces a valid (possibly empty) clip — and its manifest says which kind of
   empty it is.
5. **Publish** (`cut_window`). Empty segments are dropped when the window
   produced real data elsewhere (one is kept if all are empty). The count
   determines naming, which is why the workers stage but never publish: a
   single segment keeps the bare `<anchor_ns>_<name>.mcap`; multiple segments
   get `<base>_00.mcap`, `<base>_01.mcap`, … Each is atomically published into
   `out_dir` via `hard_link` + unlink. The recorder cuts under
   `clip::segment::Publication::Suffix`: a name an earlier clip already holds
   means a *second* trigger asked for it, so its clip lands beside the first as
   `<name>_1.mcap` rather than being dropped. (`clipper clip` passes `Refuse`
   instead — see
   [`clipper clip`](../clipper/CLAUDE.md#clipper-clip-one-window-one-finished-recording).)
6. **Announce** (`handle_trigger`) a single `Completion` (the trigger echo plus
   all segment paths) through the active interface's announcer — only after
   every segment is in `out_dir` and fsynced, so every announced path is already
   crash-durable. The
   `ros` interface turns the `Completion` into one `momentedge_msgs/Recorded`
   published on `/events/momentedge/recorded`; the `mcap` interface's announcer
   is a no-op — the segments' atomic move into `out_dir` (step 5) is the only
   completion signal, with the per-clip `info!` lines as the log.

## Retention

The tail prunes `Ended` recordings every poll (not only at rollover). A
recording is pruned when its max `log_time` (`bounds.log.max`) is older than
`now - watch_old_files_duration` (default 600 s, env
`MOMENTEDGE_WATCH_OLD_FILES_DURATION`). Retention always ages on `log_time`,
whatever the window's [time source](../CLAUDE.md#time-source): a producer must not be able to
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
recording was intentionally forgotten. See the [Configuration](../../docs/configuration.md)
for the flag reference.

