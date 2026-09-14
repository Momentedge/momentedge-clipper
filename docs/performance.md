# What it costs to run

What leaving clipper running costs on the board it is deployed to — CPU, memory,
disk reads — and what it does to the recorder beside it. For what clipper is and
how to run it, start at the [README](../README.md).

Keeping the preroll on disk instead of in memory is what makes clipper cheap.
Measured on Jetson Orin Nano and Orin NX against a `ros2 bag record` writing
about 20 MB/s:

| | Orin Nano | Orin NX |
|---|---|---|
| **clipper, tailing** | **0.45 % of one core**, 22.0 MiB | **0.39 % of one core**, 21.4 MiB |
| the recorder alone | 5.60 % | 5.98 % |
| the recorder, with clipper attached | 5.68 % | 6.13 % |

Attaching clipper moves the recorder by less than the spread between
repetitions — so the measurement says the recorder does not notice, rather than
saying by how much.

- **No disk reads while tailing.** The scan of the growing file is served
  entirely from page cache: 0.0 MB of read traffic to the device over a
  two-minute measurement.
- **Memory does not grow with pending windows.** 22.0 MiB tailing, 22.6 MiB with
  ten windows queued — because a window's preroll is on disk, not in RAM.
- **Copying is the part that costs.** Ten overlapping seventy-second windows at
  20 MB/s cost about one core and finish in under three minutes.

Figures are per board and do not travel between them. Full methodology, the
per-configuration numbers and the conditions each one depends on:
[Momentedge/clipper-benchmarks](https://github.com/Momentedge/clipper-benchmarks).

What to turn when the numbers on your own board are worse than these —
`--extract-parallelism`, `--grace-secs`, the recorder's storage profile — is
[Tuning under load](operating.md#tuning-under-load).
