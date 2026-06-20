# Rollover Recovery Review And Proposed Redesign

## Scope

This note reviews the current rollover recovery changes around `crates/clipper/src/main.rs`
and `crates/clipper/src/tail.rs`, records the follow-up investigation, and proposes a
design another agent can implement.

The active issue is `clipper-gl2`: recorder restart / split rollover can make a trigger
window lose data because the tailer historically kept only the newest recording index.
The current branch adds a rollover drain barrier, but the review found that this fixes
only a narrow in-flight case and leaves broader correctness holes.

## Current Design Summary

Current flow:

1. `Tailer` owns a single `IndexState`.
2. `Tailer::attach()` replaces that `IndexState` when a new recording is discovered.
3. `record_clip()` registers a `ReaderPermit`, waits for postroll / coverage /
   `rolling`, stages one segment from the current `IndexState`, drops the permit, and
   loops into the next file when it believes a rollover occurred.
4. `Coverage` carries `high_water_ns`, `ended`, and a transient `rolling` bool.

Important current code points:

- `tail.rs:197-205` - `IndexState` contains only one file's extents and registry.
- `tail.rs:209-218` - `Tailer` has one `state` plus a `readers` drain counter.
- `tail.rs:445-480` - `attach()` raises `coverage.rolling`, waits for readers, then
  resets `IndexState` and replaces `Coverage::default()`.
- `main.rs:657-659` - the postroll wait computes `end_ns - now_ns()` after a separate
  `now_ns() < end_ns` check.
- `main.rs:670-704` - `record_clip()` snapshots `Coverage`, stages the segment, drops
  the permit, then decides whether to continue based on the stale snapshot.
- `tail.rs:802-811` - discovery chooses only the newest `*.mcap` by ctime.

## Review Findings

### P1: Successor can be missed while staging

The existing P1 is valid. If the closing file has `high_water_ns < end_ns` and
`cov.ended == true`, a successor can attach while `stage_segment()` is running. That
successor sets `rolling = true`, waits for the handler to drop its permit, then
immediately clears `rolling` by publishing fresh default coverage. After staging,
`record_clip()` still follows the stale `cov.ended` branch and waits only for a future
`rolling = true`. If the rollover already completed, it can time out and publish only
the closing-file segment.

This is not a simple wakeup bug. `rolling` is a transient edge represented as level
state. A handler that does not observe the level while it is true cannot distinguish
"no successor appeared" from "successor already appeared and is now current".

### P2: Postroll wait underflows

The current wait loop calls `now_ns()` twice:

```rust
while now_ns() < end_ns {
    let wait = Duration::from_nanos(end_ns - now_ns());
    ...
}
```

If the second `now_ns()` advances past `end_ns`, debug builds can panic and release
builds can wrap into a huge sleep. This should be changed to a single timestamp with
`checked_sub`:

```rust
while let Some(wait_ns) = end_ns.checked_sub(now_ns()).filter(|ns| *ns > 0) {
    if coverage.wait_timeout_for(Duration::from_nanos(wait_ns), |c| c.rolling) {
        ...
    }
}
```

If the broader design below is adopted, the same `checked_sub` pattern still applies,
but the wait should no longer be interrupted by `rolling`.

### P2: Freezing rollover readers is not the right direction

The review comment suggested freezing rollover readers while draining because
`Condvar::wait_timeout` releases the `readers` mutex, allowing triggers that arrive
after `rolling` is raised to register against the closing index and prolong the drain.

That race is real for the current barrier design. However, freezing new trigger
registration is wrong for the desired semantics:

- A trigger that arrives while the recorder is rolling may have preroll in the closing
  file.
- If registration is frozen until after the swap, that trigger can only see the new
  file and loses preroll.
- The trigger listener should remain live. A trigger during rolling must become a
  normal sleeping window whose later snapshot can include both previous and current
  files.

This pushes the design away from "drain in-flight handlers at rollover" and toward
"retain all relevant recording indexes, then snapshot by window".

### Design hole: previous-file preroll is still unsupported

The biggest issue is architectural. A trigger window is `[trigger_time - preroll,
trigger_time + postroll]`. The trigger can arrive after a rollover while its preroll
still belongs to the previous file. The current `Tailer` cannot satisfy this because
`attach()` resets the only `IndexState`.

The current tests even encode this limitation:

- `tests/e2e.rs:487-494` says an old recording on disk is never read again.
- `tests/e2e.rs:553-560` asserts an old-file window cuts empty.

Those expectations conflict with the required triggered-recorder semantics if preroll
may cross file boundaries.

## Investigation Result: Move Extraction To Window Completion

The current implementation assumes rollover is the moment when the closing file must be
drained into an output segment. That creates tight coupling:

- tailer blocks on handler extraction work,
- handlers depend on a transient `rolling` flag,
- drain time can delay discovery of later files,
- multiple rollovers during one long drain can be skipped because discovery chooses only
  the newest file,
- reader permits become a correctness primitive.

The simpler design is to remove rollover-time extraction entirely.

Instead:

1. The tailer continuously discovers, opens, and indexes every relevant recording file.
2. It never blocks on clip staging or handler work.
3. Each trigger handler sleeps until its postroll floor and then waits for the tailer
   collection to be sufficiently indexed for the window, or for grace to expire.
4. Once the window is ready to cut, the handler takes one snapshot across all recording
   indexes whose bounds overlap the window and stages one output split per source file.

This makes rollover a tailer state transition, not an extraction event.

## Proposed Data Model

Replace the single `IndexState` with a collection of recording indexes.

Suggested sketch:

```rust
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
struct RecordingId {
    dev: u64,
    ino: u64,
    ctime_sec: i64,
    ctime_nsec: i64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum RecordingState {
    New,
    Tailing,
    Ended,
}

#[derive(Clone, Copy, Debug, Default)]
struct TimeBounds {
    min_log_time: u64,
    max_log_time: u64,
    min_publish_time: u64,
    max_publish_time: u64,
    has_messages: bool,
}

struct RecordingIndex {
    id: RecordingId,
    path: PathBuf,
    file: Arc<File>,
    state: RecordingState,
    offset: u64,
    extents: Vec<Extent>,
    open: Option<Extent>,
    schemas: HashMap<u16, SchemaDef>,
    channels: HashMap<u16, ChannelDef>,
    bounds: TimeBounds,
}

struct TailState {
    recordings: VecDeque<RecordingIndex>,
    current: Option<RecordingId>,
}
```

Notes:

- `New` means the file is opened and recorded in the collection, but magic and/or scan
  state has not yet been established.
- `Tailing` means the file is the active scan target or still eligible for more bytes.
- `Ended` means DataEnd/Footer was scanned, or the tailer has otherwise concluded no
  more bytes will be appended.
- The current tail only reads the message prefix through `log_time`. To track
  `publish_time`, scan code must read the full MCAP Message fixed header
  (`channel_id`, `sequence`, `log_time`, `publish_time`) before the payload.
- `Extent` should also carry both log-time and publish-time min/max, not only log-time,
  so planning can cheaply filter by whichever time base the recorder supports.

The wrapper around the collection should provide high-level operations, not expose the
raw vector:

```rust
impl TailState {
    fn insert_new_recording(&mut self, path: PathBuf, file: Arc<File>, id: RecordingId);
    fn mark_tailing(&mut self, id: RecordingId);
    fn apply_scan_delta(&mut self, id: RecordingId, delta: ScanDelta, ended: bool);
    fn plan_window(&self, start_ns: u64, end_ns: u64) -> MultiFileWindowPlan;
    fn window_status(&self, start_ns: u64, end_ns: u64) -> WindowStatus;
    fn prune_before(&mut self, retention_floor_ns: u64);
}
```

## Discovery Requirement: Do Not Skip Intermediate Files

`newest_mcap()` is no longer sufficient. If multiple splits are created while the tailer
is busy, choosing only the newest can skip intermediate files.

Discovery should scan all `*.mcap` files, sort by `(ctime, ctime_nsec)`, identify files
by `(dev, ino)`, and insert every unknown file into the collection as `New`. Then the
tailer advances through `New -> Tailing -> Ended` in order.

For a split sequence:

1. Current file reaches footer and becomes `Ended`.
2. Discovery finds one or more unknown files.
3. Each unknown file is opened and pushed as `New`.
4. Tailer marks the next file `Tailing` and scans it.

For a restart / deletion:

- Keep the old `Arc<File>` and its indexed state in the collection.
- If the old path vanishes but the fd still grows, the collection can still index what is
  readable through the open fd if the tailer keeps scanning it.
- If a successor appears, insert it as `New`; do not reset or overwrite the old index.

If the three-state enum is strict, a vanished-but-still-growing fd remains `Tailing`.
If that ambiguity becomes painful, add a fourth state later, but start with the requested
`New`, `Tailing`, `Ended` and keep `Ended` reserved for "no more bytes expected".

## Proposed Trigger Flow

With a collection-backed tailer, `record_clip()` becomes:

1. Compute `start_ns` / `end_ns`.
2. Wait until wall clock reaches `end_ns` using `checked_sub`.
3. Wait on a collection-level watch until either:
   - `window_status(start_ns, end_ns) == Ready`, or
   - grace expires.
4. Snapshot `MultiFileWindowPlan = tailer.plan_window(start_ns, end_ns)`.
5. Stage one segment for each planned recording file.
6. Drop empty segments only when at least one non-empty segment exists.
7. Publish all staged segments and emit one `Recorded { filenames }`.

No `ReaderPermit`, no `rolling` wait, no tailer drain.

The collection-level watch can publish compact state such as:

```rust
struct CollectionCoverage {
    generation: u64,
    max_log_time: u64,
    newest_state: RecordingState,
}
```

Handlers should not make correctness decisions from a stale snapshot taken before
staging. They should snapshot plans from the collection once, after the postroll/grace
wait, and the plan should own `Arc<File>` handles for every source file it uses.

## Window Readiness Semantics

`window_status(start, end)` should answer whether waiting longer can still add in-window
data.

Suggested statuses:

```rust
enum WindowStatus {
    Ready,
    Waiting,
    NoRecordingYet,
}
```

Initial conservative rule:

- `Ready` if the collection has indexed data up to `end_ns` in the currently tailing
  file (`max_log_time >= end_ns`), or if every known candidate file that can overlap
  the window is `Ended` and there is no newer `New`/`Tailing` file that can still add
  in-window data.
- `Waiting` if the current/newest file is `New` or `Tailing` and collection-level
  high-water is still below `end_ns`.
- Grace timeout still cuts from whatever is indexed.

This retains the current high-water assumption while removing the rollover edge race.
The separate open issue about out-of-order `log_time` still applies.

## Retention

The collection cannot grow forever. Retention should be explicit and independent of
rollover:

- Keep all files whose `bounds.max_log_time >= now - max_supported_preroll - safety`.
- Keep any file referenced by an in-flight `MultiFileWindowPlan` through `Arc<File>`.
- Keep `New`/`Tailing` files regardless of bounds.
- Prune only `Ended` files that are older than the retention floor and not referenced by
  active plans.

The current config does not expose max preroll. Either add a config bound or derive a
conservative default. Without a bound, correctness implies unbounded retention.

## What Gets Simpler

This design removes:

- `Coverage::rolling`,
- `ReaderPermit`,
- `Tailer::register_reader`,
- `Tailer::attach()` waiting on a drain condvar,
- `ROLLOVER_DRAIN_TIMEOUT`,
- the stale `cov` decision path after staging,
- the question of whether triggers are allowed to register during drain.

The tailer becomes the only writer to indexes. Trigger handlers become readers that take
owned snapshots. The only synchronization left is the normal `Mutex + Condvar` around
the collection and a bounded retention policy.

## Required Tests

Add or rewrite tests around these scenarios:

1. A trigger arrives after a split, with preroll reaching into the previous split.
   Expected: output has segments for previous and current files.
2. A trigger arrives while a split is being discovered / indexed.
   Expected: trigger is admitted and later snapshots all relevant files.
3. Multiple split files appear before the tailer catches up.
   Expected: all intermediate files are inserted and can contribute segments.
4. A long extraction no longer blocks tail discovery.
   Expected: tailer indexes later files while staging workers are busy.
5. The existing P1: successor attaches during what used to be staging/drain.
   Expected: no missed successor because planning happens after postroll against the
   collection.
6. Postroll wait near `end_ns`.
   Expected: no underflow or huge sleep; use `checked_sub`.
7. Old-file retention boundary.
   Expected: a trigger within retention recovers previous-file preroll; a trigger older
   than retention degrades explicitly and logs why.
8. E2E replacement for `old_recording_on_disk_is_not_recovered_after_restart`.
   Expected: old recording is recovered when its bounds overlap the trigger window and
   it remains within retention.

## Migration Plan

Recommended implementation order:

1. Fix the local postroll subtraction with `checked_sub`.
2. Add `RecordingId`, `RecordingState`, `RecordingIndex`, and `TailState` without
   changing trigger behavior yet.
3. Change discovery from newest-only to ordered unknown-file insertion.
4. Make scanning update a selected `RecordingIndex` by `RecordingId`.
5. Implement `MultiFileWindowPlan` and collection `plan_window()`.
6. Change `record_clip()` to one postroll/grace wait followed by one multi-file plan.
7. Remove the rollover drain path and `rolling` machinery.
8. Replace the e2e expectations that encode old-file non-recovery.
9. Document retention and configure the max supported preroll.

## Open Questions

- What is the maximum supported preroll in production? This must become a real retention
  bound if the collection is bounded.
- Should `publish_time` become the primary planning time for any future capture-time
  mode, or is it stored only for diagnostics and future proofing?
- How should vanished-but-open files transition to `Ended` when no footer appears?
  A quiet timeout may be needed, separate from trigger grace.
- Should old indexed files restored into `record_dir` be ignored if their `(dev, ino)` is
  unknown but their ctime is older than current retention? The likely answer is yes.

## Bottom Line

The current rollover drain is a partial fix that creates fragile synchronization with
trigger handlers. The stronger and simpler design is a tail-owned collection of open
recording indexes with explicit `New -> Tailing -> Ended` transitions and min/max
`log_time` / `publish_time` bounds. Trigger handlers should wait until their postroll is
ready, then snapshot all relevant files in one pass and publish one split per source
file. This supports triggers during rollover, previous-file preroll, multiple fast
rollovers, and long extraction queues without blocking the tailer.
