# Benchmark harness — internal interface contract

This file is the integration spec every harness component codes against. It is
written before the components exist so they can be built in parallel. If a
component needs to deviate, the deviation lands here first.

## Settled conventions

Resolved during construction; every component follows these.

- **Throughput is decimal MB — 10⁶ bytes/s**, never MiB. The replay bag's native
  rate is 63.0 MB/s (1008040040 bytes / 15.997 s from its own Statistics
  record); the three targets 3 / 20 / 58 MB/s therefore all sit below
  `--rate 1.0`.
- **Target vs achieved rate.** The target only selects the replay rate. The bag's
  chunks are lz4-compressed and the recorder writes `fastwrite` (uncompressed,
  unchunked), so bytes on disk exceed the bag's stored rate. `run.json` records
  `bitrate_achieved_mbs`, measured from the recording directory over the
  measurement window, and that is the figure reported.
- **Memory columns.** `mem_used_kb` = `MemTotal − MemAvailable` (falling back to
  `MemFree` when `MemAvailable` is absent); `mem_cached_kb` =
  `Cached + SReclaimable`. Analysis must use the same definitions.
- **First row per run reads zero** for every delta-derived column (`cpu_pct`,
  `cpu_total_pct`, `disk_read_kb`, `disk_write_kb`) because no prior sample
  exists. Analysis discards the first row rather than averaging the zero in.
- **`role` is not validated** against the enum by the sampler; it records
  whatever the orchestrator passes. The orchestrator is responsible for using
  exactly the listed role names.
- Numeric CSV fields are rounded to 3 decimals.
- **Per-process IO exists only on the Nano.** The NX kernel
  (`5.15.148-tegra`) is built without `CONFIG_TASKSTATS`, so `/proc/<pid>/io`
  is absent for every process on that host; the Nano (`6.8.12-1021-tegra`) has
  `CONFIG_TASKSTATS=y`, `CONFIG_TASK_XACCT=y` and `CONFIG_TASK_IO_ACCOUNTING=y`
  and reports it normally. The four IO columns (`rchar`, `wchar`, `read_bytes`,
  `write_bytes`) are therefore written **empty — never `0`** — when the kernel
  cannot report them. Empty means "not measurable here"; `0` means "measured
  zero bytes". Analysis must not conflate them: reading absent IO as zero would
  manufacture the conclusion that clipper performs no disk IO on the NX.
  A pid is dropped from sampling only when `stat` or `status` is unreadable;
  a missing `io` never drops a pid, so CPU, RSS, threads and fds are still
  collected on the NX.
  System-wide disk IO (`system.csv`, from `/proc/diskstats`) is unaffected on
  both hosts, so clipper's IO contribution on the NX is recovered by
  differencing the clipper arms against `baseline` at the same bitrate.
  Every run records `io_accounting.json` —
  `{"io_accounting_available": true|false, "probed_via": "/proc/self/io"}` —
  written whether the probe succeeds or fails. Analysis reads availability from
  that file rather than inferring it from empty columns, so "this kernel cannot
  measure it" and "this run happened to record nothing" stay distinguishable.

## Hard constraints

- **On-target scripts are Python 3 stdlib + `rclpy` only.** The Jetsons have no
  pip packages, no numpy, no pandas, no virtualenv. Anything that needs a third
  party library runs on the workstation during analysis, never on the target.
- **One measurement run at a time per host.** Runs are strictly serial within a
  host; the two hosts run independently and in parallel.
- Bash is `bash`, not `zsh`, on the targets. `set -euo pipefail` everywhere.
- Every script takes `--help` and fails fast with a clear message when a
  prerequisite (sourced ROS env, missing file) is absent.

## Layout

```
benchmarks/
  CONTRACT.md          this file
  README.md            how to run it, hardware, caveats            (wave 3)
  lib/sampler.py       10 Hz /proc sampler + 1 Hz tegrastats       (A)
  lib/procinfo.py      /proc parsing helpers                       (A)
  load/replay.sh       bag replay at a calibrated rate             (B)
  load/synth_pub.py    rclpy synthetic publishers                  (B)
  load/calibrate.py    target MB/s -> bag --rate mapping           (B)
  load/hog.py          CPU contention generator                    (B)
  trigger/driver.py    Trigger publisher + Recorded latency        (C)
  scenarios.py         the scenario matrix                         (D)
  run_suite.sh         resumable orchestrator                      (D)
  analyze/*.py         summarise + plot (workstation only)         (wave 3)
```

## Run identity and output

Every run writes exactly one directory:

```
~/bench/results/<run_id>/
    run.json         config, phase boundaries, outcome  (written by D)
    samples.csv      10 Hz per-process samples          (written by A)
    system.csv       10 Hz system-wide samples          (written by A)
    tegrastats.log   1 Hz raw tegrastats                (written by A)
    io_accounting.json  whether this kernel reports     (written by A)
                        per-process IO at all
    triggers.jsonl   one line per trigger               (written by C)
    clipper.log      clipper stderr
    rosbag2.log      recorder stderr
    load.log         player / synth stderr
```

`run_id` = `<host>__<scenario>__<bitrate>__<comp>__par<N>__rep<R>`, e.g.
`nano__ten_windows__mid__zstd__par1__rep2`. A run directory containing a
`run.json` with `"complete": true` is skipped on resume.

## samples.csv

Header, fixed order:

```
ts_ns,role,pid,cpu_pct,utime_s,stime_s,rss_kb,vmhwm_kb,rchar,wchar,read_bytes,write_bytes,threads,fds
```

- `role` ∈ `clipper | rosbag2 | player | synth | hog | sampler`
  (`sampler` is the sampling process measuring itself — the cost of measuring
  belongs in the same file as the thing measured, not as an unquantified charge
  against every arm. The sampler adds its own pid; the orchestrator cannot,
  since it does not know that pid before launch.)
- `cpu_pct` is percent of ONE core, computed from the utime+stime delta over the
  sample interval — it may exceed 100 for a multithreaded process.
- `rchar`/`read_bytes` both come from `/proc/<pid>/io`. Keeping both is the
  point: `rchar` counts bytes the process read, `read_bytes` counts bytes
  actually fetched from the block device. Their difference is the page-cache
  hit rate, which is a headline result for the tail.
- A dead pid emits no row; the sampler must not crash when a process exits.

## system.csv

```
ts_ns,cpu_total_pct,mem_used_kb,mem_cached_kb,disk_read_kb,disk_write_kb,load1
```

Disk figures are deltas over the interval for the device backing `~/bench`
(read from `/proc/diskstats`).

## tegrastats

Raw lines to `tegrastats.log`; parsing happens during analysis, not on target.
Started with `--interval 1000`.

The two hosts' formats differ, but **not** in the way one might guess. The rail
names are identical on both (`VDD_IN`, `VDD_CPU_GPU_CV`, `VDD_SOC`), and both
emit `GR3D_FREQ`; what differs is **tuple arity**:

| | temps | power rails |
|---|---|---|
| Nano (L4T 39) | `cpu@56.093C/56.093C` — 2 values | `VDD_IN 7006mW/7006mW/7006mW` — 3 |
| NX (L4T 36) | `cpu@62.906C` — 1 value | `VDD_IN 7527mW/7527mW` — 2 |

Element **`[0]` is the instantaneous reading on both** — established by divergence
under load, not assumed: the Nano's third element is monotonically
non-decreasing with plateaus (a running peak) and its second gently drifts (a
short average). Energy integrates element `[0]`.

Timestamps are **local wall-clock with no epoch and no offset**, and the two
boards sit in different timezones. Analysis recovers each host's offset from the
run's own recorded `host_utc_offset_s` / `clock_anchor` and **asserts the
tegrastats window overlaps the `measure` phase**, failing loudly rather than
reporting energy computed from a non-overlapping window.

**EMC utilisation is not captured.** `tegrastats` as invoked emits no `EMC`
token at all — verified absent across both hosts, both JetPack versions, and
live suite logs. No figure for memory-bandwidth utilisation is available, and
none is reported.

## triggers.jsonl

One JSON object per line, written as events happen (never buffered to the end):

```json
{"name":"bench-3","sent_ns":1234,"anchor_hint_ns":1234,"preroll_ns":10000000000,
 "postroll_ns":60000000000,"recorded_ns":5678,"latency_ns":4444,
 "files":["/home/.../clipped/..._bench-3.mcap"],"bytes":123456}
```

`recorded_ns`, `latency_ns`, `files` and `bytes` are filled when the matching
`Recorded` arrives; a trigger that never completes keeps them `null` and the run
is flagged in `run.json`.

## run.json

Written by the orchestrator at run start with `"complete": false`, rewritten at
the end. Required keys:

```json
{"run_id": "...", "host": "nano|nx", "distro": "jazzy|humble",
 "clipper_version": "0.1.3", "scenario": "ten_windows",
 "bitrate_target_mbs": 20, "bitrate_achieved_mbs": 19.4,
 "recorder_profile": "fastwrite", "max_cache_size": 0,
 "clip_compression": "zstd|none", "extract_parallelism": 1,
 "nvpmodel": "MAXN_SUPER", "jetson_clocks": true,
 "warmup_s": 30, "measure_s": 120,
 "phases": {"warmup":[t0,t1], "measure":[t1,t2],
            "waiting":[a,b], "clipping":[b,c]},
 "rosbag2_dropped": 0, "throttled": false, "complete": true}
```

`phases` values are `ts_ns` bounds so the analyzer can slice `samples.csv`
without re-deriving anything. `waiting`/`clipping` are present only for the
ten-window scenario.

**`run_suite.sh` is the sole authority on `run.json`'s shape.** Where analysis
and the producer disagree, the producer wins and analysis adapts. In particular
there is **no `failure_reason` key**. A run that did not deliver what it claims
carries:

- `"complete": false` — never treat a run as data unless this is `true`.
- `"incomplete_reasons": [...]` — a list of specific strings
  (`"5 of 10 triggers matched a Recorded"`, `"achieved 12 MB/s against a
  20 MB/s target"`). Report these verbatim; do not collapse them to a generic
  "incomplete".
- `"oom_killed": true|false` — a boolean, and the *only* signal that the
  snapshot arm hit its memory cap. The snapshot sweep's OOM annotation reads
  this field.
- `"skipped_reason"` when the disk preflight declined to run an arm. "We chose
  not to run this" and "this ran and broke" are different outcomes and are
  reported separately.

`bitrate_achieved_mbs` is the bitrate the report quotes. The target only
selected the replay rate; runs are grouped by target and **labelled by
achieved**. It is a *write*-rate measurement, so it is meaningless for a
snapshot arm — that arm buffers in RAM and writes once when the service fires,
making the figure a dump rate rather than a load. Identify those arms by
`snapshot_arm` / `variant` and never label them by achieved bitrate.

**`rosbag2_dropped` is usable only when `rosbag2_dropped_source` is present.**
With the source field, `0` is a real measured zero: the recorder shut down
cleanly and emitted no `lost on the transport layer` line, which it emits only
when drops occur. Without the source field, the value is a residual of an
estimate — `max(0, expected − recorded)` against an `--all` recording that also
holds trigger/rosout traffic the source bag lacks — whose error dominates the
result and which cannot separate "no drops" from a few hundred. **Discard such a
value; never read it as `0`.** This is a standing rule, not a one-time cleanup:
archived runs outlive the memory of why their numbers differ.

## Component CLI contracts

```
lib/sampler.py --out DIR --pid role=PID [--pid role=PID ...] --interval-ms 100
               [--tegrastats-interval-ms 1000]
    Samples until SIGTERM, then flushes and exits 0. Adding a pid after start
    is not supported: the orchestrator starts the sampler once all pids exist.

load/replay.sh --bag PATH --rate FLOAT [--loop] [--topics "a b c"]
    Copies the bag to /dev/shm on first use, replays from there. Execs
    `ros2 bag play` so signals reach it directly.

load/calibrate.py --bag PATH --target-mbs FLOAT
    Prints the --rate that achieves the target write rate, derived from the
    bag's own duration and byte size. Prints JSON: {"rate":0.34,"est_mbs":20.1}

load/synth_pub.py --topics N --size BYTES --rate HZ [--prefix /bench]
    N publishers of std_msgs/ByteMultiArray-sized payloads at HZ each.

load/hog.py --cores N
    N busy loops pinned to distinct cores. SIGTERM stops them.

trigger/driver.py --pattern single|ten_overlap|none --preroll-ns N
                  --postroll-ns N --out FILE [--stagger-ms 1000]
    Publishes on /events/momentedge/trigger, subscribes /events/momentedge/recorded,
    writes triggers.jsonl. Exits when every trigger has been matched or
    --timeout-s elapses (default: postroll + 120 s).

run_suite.sh --host nano|nx [--only GLOB] [--reps N] [--dry-run]
    Resumable. Prints one line per run to stdout and appends to
    ~/bench/results/suite.log.
```

## Scenario matrix (owned by scenarios.py)

| scenario | clipper | triggers | notes |
|---|---|---|---|
| `baseline` | not running | none | the denominator |
| `idle_tail` | running | none | tail cost only |
| `one_clip` | running | 1 × 10 s/10 s | |
| `ten_windows` | running | 10 × 10 s pre / 60 s post, 1 s stagger | two phases: waiting, then clipping |
| `snapshot_sweep` | both arms | 1 | preroll ∈ 5/30/60/300/600 s |
| `soak` | running | 1 every 60 s, 4 h | RSS/fd/latency drift |
| `rollover` | running | windows spanning 60 s splits | multi-segment clips |
| `retention` | running | prune-record.sh concurrent | |
| `contention` | running | ten_windows under hog | 4/6 cores (nano), 6/8 (nx) |
| `lowpower` | running | ten_windows at 15W/10W | |

Axes: bitrate ∈ {light ~3, mid ~20, heavy ~58} MB/s; clip_compression ∈
{none, zstd}; extract_parallelism ∈ {1, 2, ncores}. `baseline` and `idle_tail`
do not vary over compression or parallelism.

Fixed everywhere: recorder `fastwrite` + `--max-cache-size 0`, clipper
`--interface ros --time-source log --grace-secs 2`.

## Snapshot-mode arm

`rosbag2` runs with `--snapshot-mode --max-cache-size <(pre+post)*bitrate>`; the
clip is taken by calling `/rosbag2_recorder/snapshot` at `anchor + postroll`.
Run it inside `systemd-run --scope -p MemoryMax=<board RAM * 0.8>` so an
over-large cache is OOM-killed cleanly instead of taking the board down — the
kill point is a data point, not a failure.
