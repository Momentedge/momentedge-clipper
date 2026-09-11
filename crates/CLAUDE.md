# The three crates

`clipper` is three crates cut along two lines: [`clip`](clip) is what every
consumer of a recording shares, [`tail`](tail) is what following one still being
written adds, and [`clipper`](clipper) is the ROS node and command line on top.
This file is what all three share — the seam between them, the feature matrix
that keeps ROS out of two of them, and the clock domain every window lives in.
Each crate's own internals are in its own `CLAUDE.md`:

- [`clip/CLAUDE.md`](clip/CLAUDE.md) — the format layer and the scan that makes
  a live MCAP readable, the copy, the manifest, segment assembly and
  publication, the whole-file index and bag directories.
- [`tail/CLAUDE.md`](tail/CLAUDE.md) — discovery, the recording collection and
  its lifecycle, coverage, the scan-fault budget, retention, and the per-trigger
  flow with the two waits a growing file costs.
- [`clipper/CLAUDE.md`](clipper/CLAUDE.md) — the interfaces, the anchor seam and
  admission gate, thread supervision, `clipper clip`, configuration, and the
  live e2e suite.

The producer-facing statement of the same design is
[ARCHITECTURE.md](../ARCHITECTURE.md); the user-facing one is
[README.md](../README.md).

## Three crates, two lines

**What every consumer of a recording shares** is [`clip`](clip): the MCAP
format layer and its recording index (`clip::index`, and `clip::whole` for a
recording that is already finished), the copy that cuts a window out of one
(`clip::cut`), the neutral trigger and completion contract (`clip::trigger`,
`clip::decode`), the segment assembly that turns one window into published clips
(`clip::segment`), the record each of those clips carries saying what it is
(`clip::manifest`), which of the recording's topics a clip is cut from
(`clip::select`), and the layered configuration file that decides those topics
and every other setting (`clip::config`).

**What following a recording still being written adds** is [`tail`](tail):
discovery, the recording collection and its lifecycle, coverage, retention, the
scan-fault budget, and — in `tail::handler` — the two waits a cut from a growing
file must clear before the shared cut path runs. A consumer cutting from a
recording nobody is writing links `clip` alone: no successor to find, no
lifecycle to run, nothing to wait for.

**What is left is the binary**, four files in [`clipper`](clipper): the interface
seam and its MCAP implementation (`clipper/src/interface.rs`), the ROS
implementation behind the `ros` feature (`clipper/src/interface/ros.rs`), and the
clap configuration of each mode, the admission gate and the thread supervision
(`clipper/src/main.rs`, `clipper/src/supervision.rs`). Telling ROS from MCAP,
taking configuration, and deciding what to do when a thread dies is the whole of
what a device recorder adds over the two libraries. The per-module table is in
[ARCHITECTURE.md](../ARCHITECTURE.md#module-map).

**The binary has two modes** (`Mode`, `clipper/src/main.rs`), and both flow
through the same libraries. `clipper tail` is the device recorder that
[`tail/CLAUDE.md`](tail/CLAUDE.md) and [`clipper/CLAUDE.md`](clipper/CLAUDE.md)
describe; `clipper clip` cuts a window per trigger out of one finished recording
and exits — see
[`clipper clip`](clipper/CLAUDE.md#clipper-clip-one-window-one-finished-recording).
Adding a mode to the enum is a compile error until it has a body to run *and*
says what its clips are stamped with (`Mode::producer`), so a clip names the
subcommand that cut it without the cut path learning anything about modes.

### Two builds

**Nothing here links ROS unless a feature asks for it — the binary included.**
Neither library’s default feature set pulls r2r, and neither does the binary’s,
so a consumer cutting clips out of a recording on a plain Linux host runs the
same format layer, the same copy, and — while the recording is still being
written — the same tail the device runs, and can run the whole recorder binary
too:

```bash
cargo build -p clipper                                       # links no ROS
nix develop --command cargo build -p clipper --features ros  # the device build
```

The feature buys the `ros` interface and nothing else: `dep:r2r`, the
`dep:futures` its subscription stream is drained with, and `clip/ros`
underneath. The tail, the window plan, the cut, the admission gate, the
supervision and every other flag are the same code either way. What differs on
the command line is one flag — `clipper tail --trigger-source` accepts `mcap`
alone in the default build and takes it by default, and accepts `ros` and defaults
to it under the feature — plus what a `cdr` trigger in the recording does: decoded
with the feature, skipped with an error naming it without.

The `libraries` CI job (`clip + tail + clipper (ROS-free)`) holds that down for
all three crates: it asserts `cargo tree` names no r2r in any default tree
*before* anything is compiled, then clippies and tests all three with
`-D warnings` on a stock stable toolchain with no nix and no ROS on `PATH` — so a
dependency that escapes a feature gate turns that job red in seconds instead of
surfacing as a missing rmw at link time in a build that has no ROS at all. Only
`--features ros` needs the dev shell. Every packaging path selects it, so a
shipped binary is always the device build.

`clip` carries three cargo features, all off by default:

- **`ros`** — the `cdr` arm of the trigger decoder and the two r2r message
  conversions (`momentedge_msgs/Trigger` → `Trigger`, `Completion` →
  `momentedge_msgs/Recorded`). They live in `clip` rather than in the recorder
  because the orphan rule leaves nowhere else: a downstream crate may not
  implement `From` between two types it does not own. `json` triggers decode
  either way.
- **`clap`** — `TimeSource` as a `ValueEnum`, for a binary that takes the clock
  domain on its command line. Off by default so a library consumer links no clap.
- **`test-support`** — publishes `clip::testing`, the MCAP fixture writers, so a
  consumer's tests build recordings the way clip's own do instead of keeping a
  copy that drifts from what the scan expects. A dev-only opt-in: a consumer
  enables it under `[dev-dependencies]`, where the v2+ resolver keeps it out of
  a release build.

`tail` carries one, the same shape:

- **`test-support`** — publishes `Watch`'s unconditional `get` and
  `send_replace`. Nothing in the tail itself calls either — coverage rises
  through `send_if_modified` — so they exist only to drive a waiter from a
  test: the recorder's admission-gate test parks a full 16 handlers on a
  `Watch<bool>` and frees them all with one `send_replace`.

The binary carries one:

- **`ros`** — `dep:r2r`, `dep:futures`, and `clip/ros`, which together are the
  `ros` interface (`clipper/src/interface/ros.rs`) and the `cdr` trigger decoder.
  It is the whole of the difference between the two builds above.

The recorder enables clip's `clap` normally and clip's `ros` through its own
`ros`, plus both crates' `test-support` under `[dev-dependencies]`.

**The window-plan seam** is what leaves the cut path indifferent to which side
of the line it runs on. `clip::index::WindowPlanner` is one method —
`plan_window(start_ns, end_ns, source) -> Vec<WindowPlan>` — and
`clip::segment::cut_window` takes a `&dyn WindowPlanner` and never learns which
it holds. `tail::Tailer` implements it over its live collection, so a window
straddling a rollover yields one plan per source recording; a whole-file index
implements it too, over one finished recording (yielding at most one plan) or
over a bag directory's splits read as one time-ordered collection.

**The line falls where lifecycle begins.** `clip::index::RecordingIndex` is the
pure per-recording index — path, file, scan offset, magic check, extents,
schema/channel registry, trigger channels, time bounds — and knows nothing of
any other recording. `tail` wraps it in `Recording { id, state, index }`: the id
fixing this recording's place in time order and the `New`/`Tailing`/`Ended`
state are exactly what *tailing* adds, and a consumer cutting from a finished
file needs neither. Coverage and its watch, retention and discovery stay on
`tail`'s side of the line for the same reason — they are about following a
growing file, not about a recording.

## Time base

`log_time`, `publish_time`, the trigger stamp, and the wait clock are all
nanoseconds on the system clock — this assumes the default (no `use_sim_time`).
The trigger stamp is the neutral `clip::trigger::Stamp`, a
`builtin_interfaces/Time` flattened to its `sec`/`nanosec` fields free of `r2r`;
`Stamp::ns()` flattens it to that scale (`sec.max(0) * 1e9 + nanosec`, negative
seconds clamped to 0). The same arithmetic anchors the window identically whether
the trigger arrived live over ROS or was decoded out of the tailed MCAP.

## Time source

`--time-source` (`log` or `publish`, default `log`) picks the clock domain the
whole window lives in: the anchor, which messages fall inside, which extents are
read (`Extent::overlaps` / `plan_window` in `clip::index`, on the source's
`Span`), and which coverage high-water the wait blocks on
(`Coverage::for_source`, in `tail::tailer`). It governs nothing else —
retention ages files on `log_time` (`TimeBounds.log.max`) whatever the window's
source, and the postroll floor is the wall clock. clipper never interprets
`publish_time`; it windows on the raw value, so `publish` coverage is a liveness
signal, not a completeness proof (out-of-order publish times are normal). On ROS 2
Humble `publish_time = log_time` verbatim, so `publish` is a no-op there.

`TimeSource` is `clip`'s: the clock domain is a property of a window, not of a
recorder, so it lives in the crate every consumer of a recording shares, and
clip's `clap` feature is what renders it as the `ValueEnum` behind this flag.
It travels from `Config` through the handler into `cut_window`, which hands the
one value to both the planner and every `StageJob` it queues.

## Keeping these files true

**How this layer reaches you.** Claude Code loads `CLAUDE.md` from every
directory on the path from the repository root down to the file you read or
edit, and no others. Opening `clip/src/cut.rs` loads the root file, this file,
and `clip/CLAUDE.md`; it does not load `tail/CLAUDE.md` or
`clipper/CLAUDE.md`. That is the whole reason the material is split three ways
rather than kept in one document: a fact filed under the wrong crate is a fact
nobody editing that crate is ever shown. Two consequences worth holding onto —
only the *root* `CLAUDE.md` is always in context, and the load is triggered by
the Read and Edit tools, so work done entirely through shell commands sees none
of this layer.

**What that obliges.** A change to a crate's behaviour, invariants, seams, or
thread choreography is not finished until that crate's `CLAUDE.md` says the new
truth, in the same change. Three rules make that cheap to honour:

- **File a fact where its code lives.** The test is which directory an agent
  would have open when it needs the fact. Something true of all three crates —
  the seam, the feature matrix, the clock domain — belongs here, stated once,
  and is inherited by all three. Something true of one crate belongs in that
  crate's file and nowhere else.
- **Write what the code cannot confess.** The module list, the exported symbols
  and the dependency list are already in the source and go stale here. What
  earns space is the convention a newcomer breaks first, the reason behind a
  choice that looks arbitrary, the failure signature and its cause, the
  constraint that spans two files, and what is deliberately inactive.
- **Describe the present.** These files are read by someone who never saw the
  previous version, so "no longer", "previously", "we moved" describe a state
  that to them never existed. The diff belongs in the commit message.

**Cross-references are the alternative to repetition.** A fact that would
otherwise sit in two of these files belongs in the most specific one, with the
other linking to it. Repeating it creates two things to keep in sync, and they
will not stay in sync. When a section moves between crates, rewrite the links
that pointed at it rather than leaving a second copy behind.

**Do not put procedures here.** Build, CI, packaging and release mechanics load
on every nearby edit if they live in one of these files, and are missed whenever
someone asks for them without touching a file. They live in
[`.claude/skills/`](../.claude/skills) instead, and the root
[CLAUDE.md](../CLAUDE.md) indexes them.
