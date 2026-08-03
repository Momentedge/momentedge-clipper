# clipper overhead benchmarks

What clipper costs, on real hardware, next to the `ros2 bag record` it tails.

clipper's whole proposition is that event clips — including the seconds
*before* the event — can be cut from a recording that is already on disk,
rather than held in memory. This suite measures the price of that: CPU, memory,
disk IO, energy and clip latency, on two Jetson Orin boards, always expressed
against the recorder you are already paying for.

Every number here is reproducible with the scripts in this directory. The
methodology, including the parts that flatter clipper and the parts that do
not, is stated below.

Before re-running this suite, read **[GOTCHAS.md](GOTCHAS.md)**. The two boards
differ in kernel, ROS distro, available power modes, timezone and `tegrastats`
format, and several parts of the harness are shaped around DDS and shell
behaviour that is not obvious from the code. That file exists so those facts
are read once rather than rediscovered.

## Hardware

| | Orin Nano | Orin NX 16 GB |
|---|---|---|
| Board | `orin-nano-jp72` | `momentedge-desktop` |
| OS | Ubuntu 24.04 | Ubuntu 22.04 |
| ROS 2 | Jazzy | Humble |
| Kernel | 6.8.12-tegra (L4T R39) | 5.15.148-tegra (L4T R36.4.3) |
| Cores | 6 | 8 |
| RAM | 7.5 GB | 15.6 GB |
| Storage | NVMe, 431 GB free | NVMe, 87 GB free |
| Power mode | MAXN_SUPER + `jetson_clocks` | MAXN_SUPER + `jetson_clocks` |

Both boards run pinned at their maximum power mode with `jetson_clocks`
holding CPU/GPU/EMC at that mode's ceiling, so CPU-percent is a stable unit
rather than a figure that moves with DVFS. Clocks and every thermal zone are
logged per run; a run that thermally throttles is flagged and excluded from
aggregates rather than averaged in.

## What is measured

- **CPU** — per-process, percent of one core, from `utime`+`stime` deltas.
- **Memory** — RSS and peak RSS (`VmHWM`) per process.
- **Disk IO** — per-process `rchar`/`wchar` (bytes the process read/wrote) *and*
  `read_bytes`/`write_bytes` (bytes that actually reached the block device).
  Keeping both is deliberate: their difference is the page-cache hit rate.
- **Recorder integrity** — rosbag2's dropped-message count and per-topic rates,
  with and without clipper. Whether clipper damages the recording it tails
  matters more than what it costs.
- **Clip latency** — trigger published → `Recorded` announced, end to end.
- **Energy** — `tegrastats` VDD rails: idle-tail watts, millijoules per clip.

Sampling is 10 Hz from `/proc` and 1 Hz from `tegrastats`, by a stdlib-only
Python process that samples itself alongside the processes under test — the
cost of measuring is recorded in the same file as the thing measured, rather
than left as an unquantified background charge against every arm.

## The load

Two generators, with different jobs:

- **Bag replay** — a 1.0 GB / 16 s / 28-channel UGV recording (raw stereo
  images at ~25 Hz, Velodyne scans at 20 Hz, IMU at 200 Hz, tf at 190 Hz,
  odometry, GPS) replayed on loop from `/dev/shm`, so playback reads never
  contend with the disk under measurement. `--rate` scales it to the target
  bitrate; `--topics` subsets it to separate per-message cost from per-byte
  cost. Identical on both hosts, so the ratios are comparable.
- **Synthetic publishers** — rclpy publishers at a chosen topic count, payload
  size and rate, for the controlled message-count axis.

Three operating points: **~3 MB/s** (one compressed camera), **~20 MB/s** (a
plausible multi-camera + lidar robot — the headline figure), and **~58 MB/s**.
Throughput is decimal MB (10⁶ bytes) throughout.

The bag's own stored rate is 63.0 MB/s at `--rate 1.0` (1008040040 bytes over
15.997 s), so even the heavy point replays below real time. Its chunks are
lz4-compressed, and the recorder under test writes `fastwrite` — uncompressed
and unchunked — so bytes reaching the disk exceed the bag's stored rate by the
decompression ratio (a few percent on this mostly-raw-image data) plus record
framing. Every run therefore reports the **achieved** write rate measured from
the recording directory, and that is the figure quoted; the target is only how
the replay rate was chosen.

## Configuration held fixed

Recorder: `--storage mcap --storage-preset-profile fastwrite --max-cache-size 0`
— unchunked, uncompressed, write-through, which is the profile clipper is
designed to pair with and the one with the lowest tail latency.

clipper: `--interface ros --time-source log --grace-secs 2`, varying
`--clip-compression` (`none`, `zstd`) and `--extract-parallelism` (1, 2, all
cores).

Triggers are published by `trigger/driver.py` rather than the shipped
`trigger-pub`, because the benchmark needs ten distinct overlapping windows
fired on a controlled schedule and matched back to their `Recorded`
announcements for latency.

## Scenarios

| scenario | what it isolates |
|---|---|
| `baseline` | load + rosbag2, no clipper — the denominator |
| `idle_tail` | clipper tailing, no triggers: the standing cost |
| `one_clip` | a single 10 s/10 s window |
| `ten_windows` | ten overlapping windows, 10 s pre / 60 s post, 1 s apart |
| `snapshot_sweep` | clipper vs `rosbag2 --snapshot-mode` as preroll grows |
| `soak` | 4 h, a clip a minute: RSS, fd and latency drift |
| `rollover` | windows spanning 60 s recorder splits, multi-segment clips |
| `retention` | pruning running concurrently with clipping |
| `contention` | the same work with most cores occupied by a busy stack |
| `lowpower` | the same work at 15 W / 10 W |

`ten_windows` is one run with two phases. For the first ~60 s all ten handlers
are parked waiting for their postroll to elapse; then all ten become copyable
at once. The phases are measured separately, because "ten clips pending" and
"ten clips copying" are very different states — and the difference between them
is the architecture's central claim.

### clipper vs rosbag2 snapshot mode

rosbag2 has its own event-clip feature: `--snapshot-mode` holds messages in a
circular in-memory cache and writes them out when the
`/rosbag2_recorder/snapshot` service is called. It is the direct functional
alternative to clipper, and the comparison is the point of this suite.

The structural difference: snapshot mode's preroll horizon is
`max_cache_size ÷ bitrate`, and postroll requires delaying the service call, so
the *entire* window must fit in RAM. clipper's preroll is bounded by what is on
disk, so its memory does not move with the window. The sweep grows the window
from 5 s to 600 s and records peak RSS for both. The snapshot arm runs inside a
`systemd-run` scope with a memory cap so an over-large cache is OOM-killed
cleanly — that kill point is a measurement, not a failure.

## Running it

Each host runs its own suite; the two hosts are independent and run in
parallel. Runs within a host are strictly serial — a second workload on the
same board would corrupt what the first is measuring.

```bash
# on the target, with a ROS 2 environment sourced
. /opt/ros/<distro>/setup.bash
cd ~/bench/harness
BAG=$HOME/bench/bags/example-011-ugv-ds.mcap ./run_suite.sh --host nano --reps 3
```

`BAG` names the replay recording; the two hosts differ only in `--host`
(`nano` on Jazzy, `nx` on Humble), which selects core count, user and the
`ncores` value for `--extract-parallelism`.

A full suite is many hours, so run it detached. `setsid nohup` needs nothing
installed and survives the controlling terminal going away:

```bash
cd ~/bench/harness
setsid nohup bash -c '. /opt/ros/<distro>/setup.bash && \
  BAG=$HOME/bench/bags/example-011-ugv-ds.mcap \
  ./run_suite.sh --host <nano|nx> --reps 3 --resume' \
  > ~/bench/results/suite.out 2>&1 < /dev/null &

tail -f ~/bench/results/suite.log     # watch it
pgrep -af run_suite.sh                # confirm it is alive
```

A `.suite.lock` in the results directory keeps two suites from running
concurrently on one host — a second workload would corrupt exactly what the
first is measuring. Interrupting is safe: `--resume` skips every run whose
`run.json` says `"complete": true`, so a killed suite continues where it
stopped rather than restarting.

Single scenarios can be selected for spot checks, which is also how the
harness is validated against a host before committing to a full suite:

```bash
./run_suite.sh --host nano --reps 1 --only '*idle_tail*mid*'   # tail cost only
./run_suite.sh --host nano --reps 1 --only '*one_clip*mid*'    # trigger path
./run_suite.sh --host nano --reps 1 --dry-run                  # print, run nothing
```

The suite is resumable: a run directory whose `run.json` says
`"complete": true` is skipped, so an interrupted suite continues where it
stopped. Run it under `tmux` — a full suite is several hours.

Each run is 30 s of discarded warmup (caches, DDS discovery and the recorder's
steady state settle) followed by 120 s of measurement, repeated 3 times in
randomised order, reported as median with min/max.

Results land in `~/bench/results/<run_id>/` on the target and are pulled here
for analysis:

```bash
./analyze/report.py --results <pulled results dir> --out REPORT.md
```

## How to read the numbers, and what they do not cover

- **The recorder profile is `fastwrite`, which is uncompressed and unchunked.**
  clipper's tail therefore never decompresses a chunk. Under a `zstd_fast`
  recording the tail must decompress every chunk it scans, so **these figures
  are a lower bound** for such deployments.
- **The two boards differ in SoC, core count, RAM, kernel *and* ROS distro.**
  Absolute figures are not comparable across hosts. The clipper-over-rosbag2
  ratio measured on each host is.
- **Per-process disk IO is measured on the Orin Nano only.** The Orin NX's
  kernel is built without `CONFIG_TASKSTATS`, so `/proc/<pid>/io` does not
  exist on that host and no per-process byte counts can be read there at all.
  Its IO columns are recorded empty rather than zero, so the gap is visible
  instead of being mistaken for a measurement of nothing. Per-process CPU and
  memory are unaffected on both boards, and system-wide disk IO is available on
  both — so the NX's IO overhead is reported as the difference between the
  clipper arms and `baseline` at matched bitrate, rather than attributed
  per process. The page-cache result below is therefore a direct per-process
  measurement on the Nano and a system-level inference on the NX.
- **The measurement apparatus is not free.** The sampler polls `/proc` at 10 Hz
  on the board it is measuring, and costs roughly the same as clipper's own
  idle tail. It samples itself under role `sampler`, so that cost is in the
  data rather than assumed away. It does not bias the clipper-over-rosbag2
  ratio — it runs identically in both arms and CPU is attributed per pid — but
  the board is never idle during a measurement.
- **`lowpower` coverage is asymmetric.** The Orin Nano offers 15W, 25W and
  MAXN_SUPER; only the NX has a 10W mode. The 10W arm therefore exists on the
  NX alone, and its absence on the Nano is a property of the board rather than
  a failed run.
- **Memory-bandwidth utilisation is not reported.** `tegrastats` as invoked
  emits no `EMC` figure on either board.
- The shipped `trigger-pub` binary is not exercised; a Python publisher stands
  in for it.
- Each results tree carries `harness-manifest.txt`, a sha256 manifest of the
  exact harness that produced it. Cross-host figures assume both hosts ran a
  byte-identical harness, which that manifest is what establishes.
- Boards run pinned at maximum clocks. A robot with a stock power profile sees
  different absolute power and, under DVFS, different CPU-percent.
- Storage is NVMe on both. Results on eMMC or SD would differ substantially,
  most of all for the write-heavy arms.

Where an effect is smaller than the spread across repetitions, the report says
so instead of quoting it.
