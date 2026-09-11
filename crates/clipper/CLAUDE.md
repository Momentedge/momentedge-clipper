# clipper

The binary: a *triggered* clip recorder over a **continuous `ros2 bag record`**
output. It keeps the growing recording(s) open and **tails them**, so a clip can
be cut as soon as the data is physically on disk — clip latency is bounded by the
recorder's write-through latency. Plain OS threads throughout; there is no async
runtime.

Four files are all this crate is: the interface seam and its MCAP implementation
(`src/interface.rs`), the ROS implementation behind the `ros` feature
(`src/interface/ros.rs`), and the clap configuration of each mode, the admission
gate and the thread supervision (`src/main.rs`, `src/supervision.rs`). Telling
ROS from MCAP, taking configuration, and deciding what to do when a thread dies
is the whole of what a device recorder adds over the two libraries under it —
[`clip`](../clip/CLAUDE.md), the format layer and the cut, and
[`tail`](../tail/CLAUDE.md), what following a growing recording adds. ROS is this
crate's `ros` cargo feature, off by default; the feature matrix and the seam
between the three crates are one level up in
[`crates/CLAUDE.md`](../CLAUDE.md).

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
`--interface`; the interfaces are mutually exclusive and clipper drives exactly
one per run. The `ros` interface subscribes on a ROS node and publishes
`Recorded`; the `mcap` interface reads triggers out of the recording clipper
already tails and runs ROS-free, with the clip's move into `out_dir` as the only
completion signal. Which of them the binary has is the build: `mcap` is in every
one, `ros` needs the crate's `ros` feature, and the default is `ros` where it
exists and `mcap` otherwise. See "The interface abstraction" below.

`record.sh` is a standalone `ros2 bag record` — this binary never
spawns it. The two communicate only through the files. Under the `mcap`
interface that file path is also the *trigger* path: the continuous recording
must capture the trigger topic (`ros2 bag record --all`) so clipper can lift the
triggers back out of it.

## `clipper clip`: one window, one finished recording

`clipper clip <recording> --out-dir <dir> --trigger-time <ns> --preroll
<ns> --postroll <ns>` (plus optional `--trigger-name`/`--trigger-description`)
is `clip_mode` in `src/main.rs`. It builds the same `CutRequest` a
[trigger handler](../tail/CLAUDE.md#per-trigger-flow) builds — the trigger's five
values are spelled as flags, and `--trigger-time` is both the `Stamp` the trigger
carries (`Stamp::from_ns`) and the anchor the window centres on — and hands it to
the same [`clip::segment::cut_window`](../clip/CLAUDE.md#segment-assembly-and-publication-clipsegment).
The cut itself is therefore unchanged, and so are the manifest, the
`<anchor_ns>_<name>.mcap` name and the atomic publication. `--trigger-name`
passes the same `validate_name` gate a name arriving on a topic does, so a name
accepted by one mode is accepted by the other.

**Where the triggers come from is `--trigger-source`** (`TriggerSource`), and
exactly one source is active per run — the same shape `--interface` has on the
recorder, down to rendering its `--help` default through a `Display` that reads
the `ValueEnum`'s own possible-value name, so the accepted values and the help
text cannot drift. Unlike `InterfaceKind` the variant set is not the build's:
both sources exist in every build, and the `ros` feature is visible here only in
what a *recorded* trigger may be encoded as.

- **`param`** (the default) cuts the one trigger the flags name.
  `ClipConfig::param_trigger` is where the flags become that trigger, and it is
  called twice with the same answer: once by `parse_cli`, where a missing flag
  ends the process, and once by the cut, which takes the trigger it built.
- **`mcap`** cuts every trigger the recording itself carries on `TRIGGER_TOPIC`.
  `clip::embedded::read_triggers` returns them as the same undecoded
  `TriggerRecord`s the tail's trigger tap emits, `clip::decode::decode_trigger`
  turns each into a `Trigger`, and the window anchors on the record's own
  `log_time` — the stamp `resolve_mcap_anchor` picks under `--time-source log`,
  so a clip cut here and the one the device cut from that trigger centre on the
  same instant. One run, one clip per trigger, cut in trigger order. Over a bag
  directory every split is read, in the order the collection is planned in, so a
  trigger reaches its clip whichever recording of the run was being written when
  it fired.

**A trigger nobody can use costs that trigger its clip and no more**, the
isolation the recorder's interface gives an undecodable trigger: an unreadable
payload or a recorded name that cannot be embedded in a clip pathname is logged
and skipped, and the run cuts the rest. An unsafe name in the operator's own
`--trigger-name` is the opposite — a command line to fix — and ends the run. A
run left with nothing to cut writes nothing, not even the output directory, and
exits zero.

**Both ways of stating the trigger wrongly fail while the command line is being
read** (`ClipConfig::trigger_argument_fault`, raised by `parse_cli` as a
`clap::Error` against the `clip` subcommand): `param` without `--trigger-time`,
`--preroll` or `--postroll`, and `mcap` alongside any `--trigger-*` flag. The
check is hand-written because clap's derive cannot state it — `required_if_eq`
and `conflicts_with` key on an argument being *given*, and an absent
`--trigger-source` still selects `param`, so a defaulted run would slip past
both. It is also why the five `--trigger-*` arguments carry no clap default: a
default is indistinguishable from a value the caller typed, and the conflict
turns on exactly that distinction (`DEFAULT_TRIGGER_NAME` is applied when the
trigger is built instead).

**The waits are what is absent, and that is the whole difference.** The handler's
[first two steps](../tail/CLAUDE.md#per-trigger-flow) — sleeping out the postroll
and blocking on coverage — exist because a window may reach past the last byte on
disk. This input has an
end: nothing sleeps out the postroll, nothing blocks on coverage, and a window
reaching past the recording's end is simply short. The coverage verdict is still
a real decision rather than an assumption — `WholeFileIndex::log_end_ns()`
against `request.end_ns()` — and it travels into the cut as the same
`WindowCoverage`, so `clip.short` means what it means everywhere else.

**What it refuses, it refuses by name.** A recording it cannot index and a clip
name already taken in `out_dir` are both answered before anything is written, each
naming the file and the repair — the taxonomy is
[`clip`'s](../clip/CLAUDE.md#cutting-from-a-finished-recording-clipwhole-clipbag-clipembedded).
`clip_mode` opens the index *before* `reset_capturing_dir`, so a refusal creates
neither `out_dir` nor the staging directory inside it.

Nothing machine-readable is printed: the run's result is `out_dir`'s contents
when the process exits, each clip carrying its own manifest, and the exit status
is the verdict. No ROS is involved anywhere on this path, so a default (ROS-free)
`cargo build -p clipper` cuts these clips.

## The anchor seam and the admission gate

**The anchor seam.** The interface resolves each trigger's [`Anchor`] (in
`interface.rs`) — the instant the window centres on, plus whether it came from
`trigger_time` — and passes it to the driver's `fire` callback, which hands
`anchor.ns` to `handle_trigger` for both the window bounds and the output name
`<anchor_ns>_<name>.mcap`. The four interface × `--time-source` cells resolve it:

|                    | `--time-source log`             | `--time-source publish`      |
| ------------------ | ------------------------------- | ---------------------------- |
| **`--interface ros`**  | `now_ns()` at the subscription | the trigger's `trigger_time` |
| **`--interface mcap`** | the record's `log_time`        | the record's `publish_time`  |

`resolve_ros_anchor` (in `interface/ros.rs`) and `resolve_mcap_anchor` (in
`interface.rs`) do the
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
  `clip::segment::sanitize` still maps stray characters to `_` at clip creation; the
  structural hazards are refused whole here rather than silently rewritten.

[`Anchor`]: src/interface.rs

## The two interfaces

The trigger input and the completion output are one unit — an **interface** —
chosen by `--interface`. They are mutually exclusive; clipper drives exactly one
per run. No `rosbag2_interfaces` subscription either way — coverage always comes
from the file itself.

Which interfaces the binary offers is decided at compile time by the `ros`
feature. `InterfaceKind::Ros` is a `#[cfg(feature = "ros")]` variant, so clap
derives the accepted `--interface` values from what the build actually has: a
ROS-free build refuses `--interface ros` as an unknown value at parse time rather
than failing later on a node it cannot create, and `clipper tail --help` lists
`mcap` alone (with a `long_help` saying which feature the missing one needs).
`DEFAULT_INTERFACE` follows the same `#[cfg]` split, so an absent flag means
`ros` where it exists and `mcap` where it does not.

**`ros`** (the deployed path, and the default where the feature built it) talks
to the ROS graph:

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
outside-facing half is the only place either appears. The first two layers — the
contract and the decoder — are `clip`'s, so a trigger source that is neither ROS
nor this recorder still speaks them; the third is the binary's, because telling
ROS from MCAP is exactly what a device recorder is for; the fourth is `tail`'s,
because what a cut has to wait for before it can run is the tail's business and
no interface's:

- **`clip::trigger`** — the neutral contract: `Trigger`, `Stamp`, `TriggerRecord`
  (the MCAP message record carrying an undecoded trigger), `Completion`, and the
  `Announce` trait. It pulls in no `mcap` and, in a default build, no `r2r` —
  only `serde`, so `Trigger`/`Stamp` derive `Deserialize` for the `json` decode
  shape — so either side can change without dragging the other along. The one
  ROS-shaped thing here, `Completion` → `momentedge_msgs/Recorded`, is behind
  the `ros` feature and lives here only because the orphan rule allows it
  nowhere else.
- **`clip::decode`** — `decode_trigger`, which decodes one trigger payload
  according to its MCAP channel's `message_encoding`, dispatching over two
  encodings, both decodable with dependencies already in the closure (no new
  ones):
  - **`cdr`** (the ROS2 default, behind the `ros` feature) via `r2r`'s rmw
    deserialization — the linked rmw library only, never a ROS
    `Context`/`Node`/executor, so it works in the fully ROS-free MCAP
    interface. r2r's generated `Trigger` maps onto the domain `Trigger` through
    a `From` impl shared with the live ROS path. Without the feature a `cdr`
    payload is an error naming the absent feature, never a silent loss.
  - **`json`** via `serde_json` — a first-class peer of `cdr`, so a writer
    interleaving a trigger into the bag is never forced to serialize as CDR;
    the domain `Trigger` derives `Deserialize`, so serde reads straight into it.

  `cbor`, schema-bound `protobuf`/`flatbuffer`, `ros1`, and any unknown encoding
  return an error the interface logs and skips — one undecodable trigger never
  stops the recorder.
- **`clip::embedded`** — `read_triggers`, the trigger list a *finished* recording
  states about itself, found through the summary's own chunk index so that only
  the chunks holding the trigger channel are decompressed. It is the trigger tap
  answered all at once instead of a record at a time, which is what a recording
  with an end makes possible, and `clipper clip --trigger-source mcap` is its
  caller.
- **`src/interface.rs`** — the `trait Interface` (generic, dispatched statically,
  no `Box<dyn>`) and the `Anchor` it resolves, with `McapInterface` (drains the
  tail's trigger tap and decodes each raw trigger) and its no-op `NullAnnouncer`.
  Nothing in this file names ROS. **`src/interface/ros.rs`** is the other
  implementation — `RosInterface` (owns the node and its own internal spin
  thread) and `RosAnnouncer` (publishes `Recorded`) — and the whole module is
  `#[cfg(feature = "ros")]`, which is why the r2r, futures, `Pin` and
  `spawn_supervised` imports it needs live there and not in the parent. An
  interface produces decoded `Trigger`s and owns the completion half through its
  `Announce`r.
- **`tail::handler`** (`crates/tail/src/handler.rs`) — `handle_trigger`
  (generic over `Announce`) and `record_clip`: the ROS- and encoding-agnostic
  half, speaking only the `Trigger`/`Completion` contract. It waits, calls
  `clip::segment::cut_window`, and announces what comes back; the planning,
  staging and publication below it are `clip`'s and know nothing of triggers at
  all.

`main.rs` wires it together: it builds the selected interface, and a generic
`drive<I: Interface>` runs the recorder and supervises the tail, the one
interface thread, and the signal forwarder.

## Concurrency

**Thread inventory.** Three singleton threads and the staging worker pool
run for the process's lifetime, plus one short-lived thread per admitted
trigger:

- **`tail`** — runs `tail::Tailer::run`; discovers recordings via
  `NewFileWatchIterator`, performs all blocking file IO for the incremental
  scan (`clip::index::scan_available`, called with no lock held), and prunes
  the collection every poll. Under the `mcap` interface it
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
- **`stage-N`** (N = 0 .. `extract_parallelism − 1`) — the
  [staging worker pool](../clip/CLAUDE.md#segment-assembly-and-publication-clipsegment)
  (`clip::segment::spawn_stage_workers`); each worker loops on the shared
  FIFO `StageJob` channel, runs `clip::cut::stage_clip`, and replies a
  `StagedClip`. Publication is not theirs: `cut_window` does it on the handler's
  own thread once the window's segment count is known.
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
is already serialized by the
[staging worker pool](../clip/CLAUDE.md#segment-assembly-and-publication-clipsegment).
16 comfortably exceeds
any legitimate concurrent trigger burst. Per-trigger failures stay isolated
inside each handler thread — logged and counted, never propagated to the
consumer.

**`supervise()`.** Each long-lived companion thread is started with
`spawn_supervised` (`src/supervision.rs`): the closure sends its return value
over a `bounded(1)` channel before returning; a panic unwinds without sending,
dropping the sender. `supervise` uses `crossbeam_channel::select!` on three
arms:

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

The inline `#[cfg(test)]` suites span all three crates. `clip`'s cover the
index, the cut and the segment assembly; `tail`'s cover the discovery iterator,
the tail loop and its fault budget, the coverage watch and the two waits — both
against synthetic MCAP files written by `clip::testing`, which `tail` pulls in
through clip's `test-support` feature as a dev-dependency so no fixture drifts
from what the scan expects. `clipper`'s need no recording at all: the config
parser, the four anchor cells, the admission gate and `supervise`'s three arms
are pure functions and thread choreography. `tests/e2e.rs` covers the contract
against the real stack — a live `ros2 bag record` matching the production
`scripts/record.sh` invocation (the harness builds the command directly),
triggers published with the ros2 CLI, and
`Recorded` asserted via `ros2 topic echo`. How to run it (gating, the
cargo-nextest prerequisite, the exact command) is in the **`build`** skill
([`.claude/skills/build/SKILL.md`](../../.claude/skills/build/SKILL.md#live-ros-2-e2e-suite));
this section is the rationale.

- **Everything is a child process; the test owns no ROS node.** The ros2 CLI
  resolves `momentedge_msgs` types from `AMENT_PREFIX_PATH`, so the test
  binary needs no r2r dependency and carries no process-global DDS state.
  The binary under test is located via `CARGO_BIN_EXE_clipper`, spawned with
  `tail` as its one command-line argument (everything else is `MOMENTEDGE_*`
  env), and every spawn blocks until its `clipper tail up` startup line
  appears.
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
  writer binary is resolved beside `CARGO_BIN_EXE_clipper` (built on
  demand if absent), so the case needs no extra build step.
- **A copper (cu29) Producer reaches clipper end to end**
  (`copper_sink_recording_produces_clip`, ROS-free at runtime): the
  `examples/cu-mcap-record` binary — a copper `CuSinkTask` — appends an
  unchunked, epoch-stamped Recording while clipper tails it `--interface mcap`,
  writing its own in-Recording `json` `Trigger`; clipper lifts that trigger back
  out and cuts the clip, proving a copper-rs robot with no ROS surface reaches
  clipper through the Recording alone. The Producer (a workspace member whose
  cu29 deps stay crate-local) is resolved by `cu_mcap_record_bin` beside the
  clipper binary — `CU_MCAP_RECORD_BIN`, else built on demand with `-p
  cu-mcap-record`; CI's matrix `Build` step prebuilds it so the on-demand build
  stays inside the test timeout.

## Run

```bash
nix develop --command cargo run -p clipper -- tail
```

Needs `scripts/record.sh` running (for `./record`) and a
trigger publisher (`trigger-pub`). Logs go to stdout (`main` says why);
`RUST_LOG=debug` raises verbosity. Where the ROS layer's own diagnostics land
is in the [Operating clipper](../../docs/operating.md).

The other mode needs neither, and no ROS toolchain either — it takes a finished
recording and exits:

```bash
cargo run -p clipper -- clip ./record/rosbag2_0.mcap \
  --out-dir ./clipped --trigger-time 1738000000000000000 \
  --preroll 5000000000 --postroll 5000000000
```

## Configuration

**One binary, and the mode is a subcommand.** `Cli` is the clap
`derive(Parser)` and carries a single field, the `Mode` enum — `Tail(Config)`
for the recorder, `Clip(ClipConfig)` for the one-shot cutter; `main`'s `match`
over that enum is the dispatch table, so a mode added to the enum is a compile
error until it has a body to run. Every flag belongs to a mode rather than to
`clipper` itself: `clipper --help` lists the modes, `clipper tail --help` lists
the recorder's flags, and a bare `clipper --record-dir …` is a parse error. What makes naming no mode an error
rather than a run with defaults is `Cli`'s `mode` field being a plain `Mode`
and not an `Option`: clap's derive requires the subcommand and answers a bare
`clipper` with the mode listing and a non-zero exit, so
`subcommand_required`/`arg_required_else_help` would add nothing.
`mode_hint` maps the error kinds that mean the
mode went unnamed — an unknown argument, an unknown or missing subcommand — to
the line `load_cli` prints after clap's own text, so the message names
`clipper tail`; a failure *inside* a mode (a bad `--time-source` value) gets no
hint, because the caller already said which mode they wanted. `main.rs`'s
`a_bare_recorder_flag_is_rejected_and_points_at_the_tail_mode` and
`tail_help_lists_the_recorder_flags_and_no_modes` hold that shape down.

`Config` is the recorder mode's `derive(Args)` and `ClipConfig` the cutter's:
every field is a CLI flag (or, for the cutter's recording, the positional) with a
`MOMENTEDGE_*` environment fallback and a `[settings]` key in the configuration
file, so precedence is CLI flag > env var > per-run file > system file > default
where there is one ([the four layers](#the-four-configuration-layers) below). `Config`'s fields all have defaults, so
`clipper tail` runs bare; `ClipConfig`'s window arguments deliberately do not —
which moment a clip is about is the one thing only the caller knows, so omitting
`--trigger-time` is a parse error naming the flag rather than a clip about some
arbitrary instant. `load_cli` in `main.rs` parses it — clap prints
`--help`/`--version` and any parse error and the process exits before it
returns, so `clipper tail` still runs with no further setup. The `MOMENTEDGE_*`
env names are not wired per field: `with_env_prefix` walks every subcommand with
`Command::mut_subcommand` and each of its arguments with `Command::mut_args`,
binding `<field>` to `MOMENTEDGE_<FIELD>` (`grace_secs` →
`MOMENTEDGE_GRACE_SECS`) and leaving the auto-generated `--help`/`--version`
untouched — so a mode added later inherits the same env fallback with no new
wiring. Changing the prefix is the one `ENV_PREFIX` constant. clap's `env`
feature provides the per-arg env fallback and `string` lets the runtime-built
env names be set on the args. The flags, env vars, and defaults are tabulated in
the [Configuration](../../docs/configuration.md).

The interface seam is one such flag: `--interface {ros|mcap}` (env
`MOMENTEDGE_INTERFACE`, default `ros`), a clap `ValueEnum` over `InterfaceKind`
that picks the active [interface](#the-two-interfaces) at startup.

### The four configuration layers

A setting resolves through four layers over its built-in default: the **system**
configuration file (`/etc/momentedge/clipper.toml`), the **per-run** file, the
`MOMENTEDGE_*` **environment** variable, and the **flag**. `clip::config` owns
the bottom two and `clap` the top two, and the join between them is one line in
`with_file_defaults`: what the files resolved for a key becomes that argument's
`default_value`. clap already resolves a flag over an environment variable over a
default, so handing it the files' value *as* the default puts all four in order
with no per-key wiring — and makes a file's value pass exactly the value parser a
flag's value passes, so a file cannot smuggle in a value the command line would
have refused. `required` is cleared with the same stroke, since clap's required
check does not count a default as an answer: a configuration file that names
`recording`, `--out-dir` and the window is a complete `clipper clip` invocation.

The files have to be read *before* the parser exists, so `config_paths` scans
argv for `--config` and `--system-config` (either spelling, on bytes, stopping at
`--`) and falls back to the environment variable clap would have read for the
same flag. Those two flags and `--print-config` are also declared as real
arguments, injected onto every subcommand by `with_config_args` rather than
declared as fields of `Config`/`ClipConfig` — so `--help` lists them, an unknown
spelling is refused the ordinary way, and neither mode's struct grows a field it
never reads. None of the three is a `[settings]` key, so no file can name another
file.

`clip::config::SETTINGS` is the key table, and it carries the one thing no
argument definition can: whether a **per-run** file may set the key at all. A key
naming the machine or the resources it may spend there is the system file's
alone, and a per-run file setting one is reported by name with the system value
left standing (`Layered::refusals`). `main.rs`'s
`every_mode_argument_is_a_settings_key_and_back` is what keeps that table and the
modes' arguments from drifting apart in either direction.

`--print-config` prints `effective_config` and exits; the same text goes to the
log at startup, so a run's log states the configuration it ran with. It is built
from the **parsed** `ArgMatches` — the same matches the mode's struct is built
from — so it cannot report a value the run does not use; clap's `ValueSource`
separates the flag and environment layers, and everything it calls a default is a
file's value where `Layered` says a file named the key. The keys come off the
`Command` rather than out of `ArgMatches::ids`, which also yields the argument
group clap's derive names after the config struct.

The `[topics]` half of the same files becomes the `clip::select::ChannelSelection`
both modes hand to their staging pool — see
[the copy](../clip/CLAUDE.md#the-copy-is-direct-clipcut). The schema, the layering rule and the
per-key scope are documented in the
[Configuration](../../docs/configuration.md#the-configuration-file).
