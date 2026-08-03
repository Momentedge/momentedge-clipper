# Environment facts and non-obvious code

Things that cost real time to discover, written down so a re-run does not
rediscover them. [README.md](README.md) is how to run the suite;
[CONTRACT.md](CONTRACT.md) is the data contract between components. This file
is why parts of the harness are shaped the way they are.

## The two boards are not interchangeable

Both are Jetson Orin, and almost everything else about them differs.

| | Orin Nano | Orin NX 16 GB |
|---|---|---|
| Kernel | 6.8.12-tegra (L4T 39) | 5.15.148-tegra (L4T 36) |
| ROS 2 / Python | Jazzy / 3.12 | Humble / 3.10 |
| `/proc/<pid>/io` | present | **absent** |
| nvpmodel modes | 15W, 25W, MAXN_SUPER | MAXN_SUPER, 10W, 15W, 25W, 40W |
| Timezone | `Etc/UTC` | `America/New_York` |
| `tmux` | not installed | installed |

**The NX kernel is built without `CONFIG_TASKSTATS`**, so `/proc/<pid>/io` does
not exist for any process. Per-process byte counts are unobtainable there. The
sampler probes `/proc/self/io` once at startup, records the answer in
`io_accounting.json`, and writes the four IO columns empty — never `0`, because
`0` would assert a measurement that never happened. A pid is dropped only when
`stat` or `status` is unreadable; a missing `io` never drops a pid, so CPU, RSS,
threads and fds are still collected on that host.

**The Nano has no 10W power mode.** A `lowpower` arm requesting one cannot run
there. `scenarios.py` discovers the available modes from `/etc/nvpmodel.conf`
at expansion time and emits only modes the host really has. Discovery sits
behind an explicit `--nvpmodel-conf PATH` flag rather than autodetecting the
local file: expanding a plan for one host while sitting on the other is routine,
and autodetection would silently produce a plan for the wrong board. When the
file is unreadable the declared set is used and `power_modes_source` records
that, so a plan built off-target says so.

**The boards sit in different timezones and `tegrastats` emits local wall-clock
with no offset**, while every other artefact is epoch nanoseconds. Each run
records `host_tz`, `host_utc_offset_s` and a `clock_anchor` pairing one instant
in both representations. Analysis recovers the offset per run and **asserts the
tegrastats window overlaps the `measure` phase**, failing loudly rather than
reporting energy computed from a window four hours away from the data.

**`tegrastats` rail names are identical on both boards; the tuple arity is not.**
The Nano emits temps as two slash-separated values and power rails as three; the
NX emits one and two. Element `[0]` is the instantaneous reading on both — the
Nano's third element is a running peak (monotonically non-decreasing with
plateaus) and its second is a short average. Format detection keys on arity, not
rail names.

**`tegrastats` emits no `EMC` token at all** as invoked here, on either board.
Memory-bandwidth utilisation is not available and none is reported.

## ROS 2 and DDS

**`ros2 bag record --all` subscribes to the trigger topic too.** With clipper
running, the steady-state subscriber count on `/events/momentedge/trigger` is
therefore **2**, not 1 — the recorder and clipper. This matters because the
trigger publisher uses `VOLATILE` durability (matching r2r's
`QosProfile::default()`: `KEEP_LAST` depth 10, `RELIABLE`, `VOLATILE`), so a
message published before a subscription is matched is discarded with no error
anywhere. Waiting for "at least one subscriber" is satisfied by the recorder
alone and races clipper's subscription.

`trigger/driver.py` therefore waits for the subscriber count to **stabilise** —
at least one, unchanged for `--settle-s` — rather than merely be non-zero. It
does not wait for a hardcoded 2, because the `baseline` scenario runs no clipper
and would hang. Every trigger records `subscribers_at_publish`, which is what
makes "clipper never saw it" distinguishable from "clipper saw it and produced
nothing"; without that field a lost trigger looks like a clipper fault.

A recording made with `--all` also contains the trigger and `Recorded` traffic
that the replayed source bag does not, which inflates any message-count
comparison against the source. That comparison is recorded as
`message_count_crosscheck` and labelled an order-of-magnitude check; it sources
no published number.

## Shell and process behaviour

**`set -m` is required at the top of `run_suite.sh`.** POSIX requires a shell
that starts a background job with job control disabled — the default in a
non-interactive script — to set `SIGINT` and `SIGQUIT` to `SIG_IGN` in the
child. `SIG_IGN` survives `exec` and cannot be restored by a handler in the
child. Without `set -m`, every `kill -INT` to the recorder is silently
discarded, the recorder never stops cleanly, and it has to be `SIGKILL`ed —
which loses the mcap summary section and any drop count with it. With job
control enabled the recorder stops on `SIGINT` in about a second.

**Backgrounding a bash *function* whose body is a heredoc-fed external command
forks an extra subshell**, so `$!` captures a wrapper rather than the real
process. Signalling that pid kills the wrapper and orphans the real one, which
then reparents to pid 1 and runs forever. `clip_pruner()` therefore `exec`s its
`python3`, and its call site sets `BENCH_SUITE` so the environ-marker straggler
sweep can find it as a second line of defence.

**Closing a multi-GB unchunked mcap takes real time** — the summary and index
sections are written at close. The recorder gets a stop grace scaled by the
recorded file size, and escalates `SIGINT` → `SIGTERM` → `SIGKILL` rather than
waiting out the full grace on a signal that may be ignored.

**`rosbag2` reports dropped messages only when drops occur**, via a
`Number of messages lost on the transport layer: N` line. Absence of that line
after a clean shutdown is a measured zero; absence after a hard kill proves
nothing. `run.json` records which case applies in `rosbag2_dropped_source`, and
the value is usable only when that field is present.

## Measurement discipline

These are the rules that keep absent data from being read as a favourable
result — the failure mode this harness is most exposed to.

- **Empty is not zero.** Any unmeasurable quantity is written empty and
  aggregated as missing. An aggregate over zero present values reports "not
  measurable", never a number.
- **A value is usable only with its provenance field.** `rosbag2_dropped`
  requires `rosbag2_dropped_source`; `bitrate_achieved_mbs` on a snapshot arm
  requires `bitrate_achieved_source`. Without it, discard rather than trust.
- **`bitrate_achieved_mbs` is a write rate.** On a snapshot arm the recorder
  buffers in RAM and writes once when the service fires, so bytes ÷ elapsed
  measures the dump, not the load. Snapshot arms are identified by
  `snapshot_arm`/`variant` and labelled by target.
- **The `waiting` phase ends when the first window becomes copyable**
  (`first anchor + postroll`), not at the first `Recorded`. With a stagger
  shorter than the postroll, the first clip starts copying before the last
  trigger's postroll elapses, so the naive boundary includes active copying and
  overstates the parked-handler cost by roughly fifty times.
- **Compare capabilities, not processes.** The clipper arm's memory is clipper
  *plus* the recorder it depends on, peaked at each shared sample tick. Quoting
  clipper alone against snapshot mode's single process flatters clipper by
  omitting the recorder.
- **Exclusions are not one bucket.** Skipped (the disk preflight declined),
  failed (ran and broke), not-applicable (the host lacks the power mode),
  throttled, and unparsable are five different statements about a run. A reader
  must be able to tell "we chose not to run this" from "this ran and broke".
- **The first row of every run reads zero** for delta-derived columns, having no
  prior sample. Discard it rather than averaging it in.
- **The sampler measures itself**, under role `sampler`. Its cost is the same
  order as clipper's idle-tail cost, so the board is never idle during a
  measurement and the report says so.

## Provenance

The harness is checksummed and frozen for the duration of a suite. Each results
tree carries `harness-manifest.txt`, a sha256 manifest of every harness file
that produced it; both hosts must run a byte-identical harness for cross-host
figures to mean anything. The repo tree drifts from a deployed snapshot within
minutes of launch, so **any `run.json` key the analysis consumes must be
verified against a real collected file, not against the current
`run_suite.sh`** — those are different things.

When the harness must change mid-suite, the manifest is amended in place with
the timestamp, the changed files, and a statement of scope. A change that alters
only whether a trigger is *delivered* leaves earlier runs comparable; a change
that alters *what is recorded* does not, and those runs are re-run rather than
mixed.

`scenarios.py` shuffles run order per rep, keyed on a stable arm identity rather
than list position, so editing the matrix does not silently reorder unrelated
runs. Order is reproducible from the recorded `order_seed` and `order_index`
regardless.

## What the numbers do and do not cover

See [README.md](README.md) for the caveats that belong with the results —
recorder profile, cross-host comparability, per-process IO availability,
apparatus cost, and known noise sources on each host.
