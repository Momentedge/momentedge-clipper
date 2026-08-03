#!/usr/bin/env python3
"""Synthesize a fake `~/bench/results/` tree so the analysis pipeline
(summarize.py / tegraparse.py / charts.py / report.py) can be exercised
end-to-end before any real run exists — CONTRACT.md is written *before* the
on-target components (A/B/C/D) run, so this is what "tested" means for wave
3 until then.

Every number below is invented but internally consistent with the physics
CONTRACT.md and README.md describe, not just plausible-looking noise:

- rosbag2/clipper CPU and RSS scale with the bitrate axis (light/mid/heavy).
- clipper's `ten_windows` cost is near-zero in the `waiting` phase and real
  in the `clipping` phase (the point of that scenario).
- clipper's page-cache ratio (`read_bytes` / `rchar`) is deliberately small
  (~1-2%), i.e. this fixture assumes the hypothesis holds — this is a
  pipeline test, not a real measurement, and REPORT.md's prose says so.
- The snapshot-mode arm's RSS is computed as an actual
  `(preroll + postroll) * bitrate` buffer size against a real `MemoryMax`
  cap (`board RAM * 0.8`, per CONTRACT.md's "Snapshot-mode arm" section):
  at 20 MB/s and an 8 s postroll, that buffer exceeds the Orin Nano's
  6144 MB cap once preroll reaches 300 s — which is *why* those runs come
  out OOM-killed, not a hardcoded flag. clipper's tail arm's RSS is
  computed as a small near-constant instead, matching the architecture's
  actual claim.
- Clip latency is `postroll wait + extraction time`, extraction time
  shrinking with `extract_parallelism` and growing with compression and
  window size.

Deliberately reduced from the real suite's full cross-product (see
CONTRACT.md's scenario matrix) to keep generation under a few seconds while
still exercising every code path in summarize.py/charts.py/report.py:
comp/parallelism are only swept for a handful of `ten_windows` cells rather
than crossed with every scenario x bitrate combination, and `soak` runs 1 h
instead of the real suite's 4 h (same 10 Hz sampling and 5-minute bucketing,
just fewer buckets). Includes, by design: one generic incomplete run, one
throttled rep among otherwise-usable ones, one whole cell (nx/idle_tail/heavy)
thrown all-throttled, a snapshot-mode arm that starts OOM-killing at 300 s
preroll, one preflight-skipped run, one unparsable (no run.json) directory,
and three non-run harness artefacts (`calibration/`, `plan-nano.json`,
`suite.log`) that must never show up as failures.

Per CONTRACT.md's settled conventions: the NX host writes **empty** (never
`0`) `rchar`/`wchar`/`read_bytes`/`write_bytes` on every row, plus a matching
`io_accounting.json` (`io_accounting_available: false`); the Nano reports
normally with `io_accounting_available: true`. Every run's first CSV row
reads zero for its delta-derived columns, exactly as a real sampler's first
tick would. `bitrate_achieved_mbs` is biased above `bitrate_target_mbs`
(lz4-compressed bag vs uncompressed `fastwrite` recorder), never symmetric
noise that could land below it.

## Matched against real gate-run data

Three real `run.json`/`tegrastats.log`/`samples.csv` files (Nano: idle_tail +
one_clip; NX: idle_tail) settled several things this fixture generator used
to guess at, and it now matches them:

- `run.json`'s real shape has no `failure_reason`/`skip_reason` — it's
  `"incomplete_reasons"` (a list of strings), `"oom_killed"` (bool) and
  `"skipped_reason"` (string). `"rep"`, `"variant"` and `"snapshot_arm"` are
  real top-level fields, not something parsed out of the run_id string.
- tegrastats' real format: identical rail names on both hosts (no
  name-based discriminator), differing in **tuple arity** — Nano emits 2
  temperature values / 3 power values, NX emits 1 / 2 — and lowercase zone
  names (`cpu@`, `tj@`, `cv0@`, ...). `write_tegrastats` emits exactly this
  shape per host, and stamps each line using that **host's real UTC
  offset** (Nano `Etc/UTC`, NX `America/New_York`/EDT, confirmed 4 hours
  apart against real logs) rather than the analysis workstation's — the
  whole point being to exercise `summarize.py`'s per-run tegrastats
  alignment the same way the real 4-hour gap does, not to hide it by
  generating timestamps that happen to need no correction.
"""
from __future__ import annotations

import argparse
import csv
import json
import random
import sys
import time
from pathlib import Path

# ---------------------------------------------------------------------------
# Fixed "hardware" (matches benchmarks/README.md's table)
# ---------------------------------------------------------------------------

HOSTS = {
    "nano": {"cores": 6, "distro": "jazzy", "ram_mb": 7680, "tegra_fmt": "l4t39"},
    "nx": {"cores": 8, "distro": "humble", "ram_mb": 15974, "tegra_fmt": "l4t36"},
}
# Real per-host UTC offset (confirmed against real tegrastats.log vs
# run.json's epoch-ns phases — Nano is Etc/UTC, NX is America/New_York/EDT,
# 4 hours apart). tegrastats itself carries no timezone; used here only to
# stamp realistic wall-clock lines so summarize.py's alignment logic has a
# real gap to correct, not a synthetic zero.
HOST_UTC_OFFSET_HOURS = {"nano": 0, "nx": -4}
BITRATES = {"light": 3.0, "mid": 20.0, "heavy": 58.0}
WARMUP_S = 30
MEASURE_S = 120
CLIPPER_VERSION = "0.1.3"

_T0_NS = (time.time_ns() // 1_000_000_000 - 3600) * 1_000_000_000  # ~1h ago, second-aligned


def make_run_id(host, scenario, bitrate, comp, par, rep) -> str:
    return f"{host}__{scenario}__{bitrate}__{comp}__par{par}__rep{rep}"


# ---------------------------------------------------------------------------
# Per-role /proc-sample series generation
# ---------------------------------------------------------------------------

def gen_series(rng, t0_ns, duration_s, hz, cpu_pct_fn, rss_kb_fn, rchar_bps_fn,
                wchar_bps_fn, read_bps_fn, write_bps_fn, threads, fds):
    """cpu_pct_fn(t_s)->pct, rss_kb_fn(t_s)->kb, *_bps_fn(t_s)->bytes/s at
    that instant. Returns a list of row dicts ready for the samples.csv
    writer."""
    n = max(int(duration_s * hz), 1)
    dt = 1.0 / hz
    rows = []
    utime = stime = rchar = wchar = read_b = write_b = 0.0
    for i in range(n):
        t_s = i * dt
        ts_ns = t0_ns + int(t_s * 1e9)
        cpu_pct = max(0.0, cpu_pct_fn(t_s) + rng.gauss(0, 0.4))
        utime += cpu_pct / 100.0 * dt * 0.7
        stime += cpu_pct / 100.0 * dt * 0.3
        rchar += max(0.0, rchar_bps_fn(t_s)) * dt
        wchar += max(0.0, wchar_bps_fn(t_s)) * dt
        read_b += max(0.0, read_bps_fn(t_s)) * dt
        write_b += max(0.0, write_bps_fn(t_s)) * dt
        rss_kb = max(1024.0, rss_kb_fn(t_s) + rng.gauss(0, 200))
        rows.append({
            "ts_ns": ts_ns, "cpu_pct": round(cpu_pct, 3),
            "utime_s": round(utime, 4), "stime_s": round(stime, 4),
            "rss_kb": int(rss_kb), "vmhwm_kb": int(rss_kb * 1.05),
            "rchar": int(rchar), "wchar": int(wchar),
            "read_bytes": int(read_b), "write_bytes": int(write_b),
            "threads": threads, "fds": fds,
        })
    return rows


def const(v):
    return lambda t: v


def ramp(v0, rate):
    return lambda t: v0 + rate * t


# ---------------------------------------------------------------------------
# tegrastats.log line generation
# ---------------------------------------------------------------------------

def tegra_line(ts_str, fmt, ram_used_mb, ram_total_mb, cpu_pcts, gr3d_pct,
               vdd_in_mw, cpu_cv_mw, soc_mw, cpu_t, tj_t):
    """Real format (confirmed against actual tegrastats.log, not guessed):
    identical rail names on both hosts, no `EMC_FREQ`/`VIC_FREQ`/`APE`/
    `NVENC`/`NVDEC` tokens, lowercase zone names — the two formats differ
    only in **tuple arity** (l4t39/Nano: 2 temp values, 3 power values;
    l4t36/NX: 1 temp value, 2 power values). Second+ tuple values are set
    equal to the first here (this fixture doesn't need them to differ —
    tegraparse.py only ever trusts element [0])."""
    cores = ",".join(f"{int(p)}%@1728" for p in cpu_pcts)
    if fmt == "l4t39":  # Nano
        t2 = f"{cpu_t:.3f}C/{cpu_t:.3f}C"
        tj2 = f"{tj_t:.3f}C/{tj_t:.3f}C"
        soc2 = f"{cpu_t - 1:.3f}C/{cpu_t - 1:.3f}C"
        soc0 = f"{cpu_t - 0.5:.3f}C/{cpu_t - 0.5:.3f}C"
        soc1 = f"{cpu_t - 0.5:.3f}C/{cpu_t - 0.5:.3f}C"
        gpu2 = f"{cpu_t - 1.5:.3f}C/{cpu_t - 1.5:.3f}C"
        return (
            f"{ts_str} RAM {int(ram_used_mb)}/{int(ram_total_mb)}MB (lfb 11x4MB) "
            f"SWAP 0/2048MB (cached 0MB) CPU [{cores}] GR3D_FREQ {int(gr3d_pct)}% "
            f"cpu@{t2} soc2@{soc2} soc0@{soc0} gpu@{gpu2} tj@{tj2} soc1@{soc1} "
            f"VDD_IN {int(vdd_in_mw)}mW/{int(vdd_in_mw)}mW/{int(vdd_in_mw)}mW "
            f"VDD_CPU_GPU_CV {int(cpu_cv_mw)}mW/{int(cpu_cv_mw)}mW/{int(cpu_cv_mw)}mW "
            f"VDD_SOC {int(soc_mw)}mW/{int(soc_mw)}mW/{int(soc_mw)}mW"
        )
    else:  # l4t36 / NX
        return (
            f"{ts_str} RAM {int(ram_used_mb)}/{int(ram_total_mb)}MB (lfb 180x4MB) "
            f"SWAP 168/7828MB (cached 0MB) CPU [{cores}] GR3D_FREQ {int(gr3d_pct)}% "
            f"cv0@{tj_t - 4:.3f}C cpu@{cpu_t:.3f}C soc2@{cpu_t - 6:.3f}C "
            f"soc0@{cpu_t - 3:.3f}C cv1@{tj_t - 4:.3f}C gpu@{cpu_t - 1:.3f}C "
            f"tj@{tj_t:.3f}C soc1@{cpu_t - 5:.3f}C cv2@{tj_t - 8:.3f}C "
            f"VDD_IN {int(vdd_in_mw)}mW/{int(vdd_in_mw)}mW "
            f"VDD_CPU_GPU_CV {int(cpu_cv_mw)}mW/{int(cpu_cv_mw)}mW "
            f"VDD_SOC {int(soc_mw)}mW/{int(soc_mw)}mW"
        )


def write_tegrastats(path, host, t0_ns, duration_s, base_cpu_pct, cores):
    """Stamps each line with the *board's own* wall clock — real UTC offset
    per `HOST_UTC_OFFSET_HOURS`, never the analysis workstation's — so this
    fixture exercises summarize.py's per-run tegrastats alignment against a
    real 4-hour Nano/NX gap, the same way real data does, rather than
    accidentally needing no correction at all."""
    import datetime
    fmt = HOSTS[host]["tegra_fmt"]
    offset_h = HOST_UTC_OFFSET_HOURS.get(host, 0)
    lines = []
    for sec in range(int(duration_s) + 1):
        ts_ns = t0_ns + sec * 1_000_000_000
        local_dt = (
            datetime.datetime.fromtimestamp(ts_ns / 1e9, tz=datetime.timezone.utc)
            + datetime.timedelta(hours=offset_h)
        )
        ts_str = local_dt.strftime("%m-%d-%Y %H:%M:%S")
        load_frac = base_cpu_pct / (100.0 * cores)
        cpu_pcts = [min(99, base_cpu_pct / cores + (5 if i == 0 else 0)) for i in range(cores)]
        vdd_in = 3200 + 3500 * load_frac
        cpu_cv = 500 + 2500 * load_frac
        soc = 1200 + 400 * load_frac
        cpu_t = 38 + 12 * load_frac
        tj_t = cpu_t + 1.0
        ram_used = 2200 + 30 * (sec % 5)
        lines.append(tegra_line(
            ts_str, fmt, ram_used, HOSTS[host]["ram_mb"], cpu_pcts,
            gr3d_pct=2 + 3 * load_frac,
            vdd_in_mw=vdd_in, cpu_cv_mw=cpu_cv, soc_mw=soc, cpu_t=cpu_t, tj_t=tj_t,
        ))
    path.write_text("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------
# CSV writers
# ---------------------------------------------------------------------------

SAMPLES_HEADER = [
    "ts_ns", "role", "pid", "cpu_pct", "utime_s", "stime_s", "rss_kb", "vmhwm_kb",
    "rchar", "wchar", "read_bytes", "write_bytes", "threads", "fds",
]
SYSTEM_HEADER = [
    "ts_ns", "cpu_total_pct", "mem_used_kb", "mem_cached_kb",
    "disk_read_kb", "disk_write_kb", "load1",
]


def write_samples_csv(path, role_pid_rows: dict, io_available: bool = True):
    """role_pid_rows: {(role, pid): [row dict, ...]}. `io_available=False`
    (the NX kernel, no CONFIG_TASKSTATS) writes the four IO columns
    **empty** for every row, never `0` — CONTRACT.md's settled convention;
    coercing them to 0 would fabricate "clipper does no disk IO" out of
    absent data. CONTRACT.md also: the first row of every run reads zero
    for every delta-derived column since there is no prior sample — enacted
    here on `cpu_pct` (the only such column in samples.csv) so
    summarize.py's row-1-discard logic has a real artefact to discard."""
    all_rows = []
    for (role, pid), rows in role_pid_rows.items():
        for r in rows:
            row = dict(r)
            row["role"] = role
            row["pid"] = pid
            if not io_available:
                row["rchar"] = row["wchar"] = row["read_bytes"] = row["write_bytes"] = ""
            all_rows.append(row)
    all_rows.sort(key=lambda r: (r["ts_ns"], r["role"], r["pid"]))
    if all_rows:
        min_ts = min(r["ts_ns"] for r in all_rows)
        for row in all_rows:
            if row["ts_ns"] == min_ts:
                row["cpu_pct"] = 0.0
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SAMPLES_HEADER)
        w.writeheader()
        for row in all_rows:
            w.writerow({k: row[k] for k in SAMPLES_HEADER})


def write_io_accounting(path, io_available: bool):
    """`io_accounting.json`: written whether the probe succeeds or fails, so
    "this kernel can't measure it" and "this run happened to record
    nothing" stay distinguishable (CONTRACT.md) — summarize.py reads this
    rather than inferring availability from empty columns."""
    path.write_text(json.dumps({
        "io_accounting_available": io_available,
        "probed_via": "/proc/self/io",
    }, indent=2) + "\n")


def write_system_csv(path, t0_ns, duration_s, hz, rng):
    """CONTRACT.md: `disk_read_kb`/`disk_write_kb` are noise-level
    background disk activity independent of scenario or clipper's presence
    in this fixture — the point is that the with-clipper and baseline arms
    are drawn from the *same* distribution, so the system-level page-cache
    inference (analysis_page_cache_system) should come back "within noise"
    on this synthetic data. The first row reads zero for every
    delta-derived column, per CONTRACT.md."""
    n = max(int(duration_s * hz), 1)
    dt = 1.0 / hz
    rows = []
    for i in range(n):
        ts_ns = t0_ns + int(i * dt * 1e9)
        rows.append({
            "ts_ns": ts_ns,
            "cpu_total_pct": round(max(0.0, 20 + rng.gauss(0, 3)), 2),
            "mem_used_kb": 2_500_000 + int(rng.gauss(0, 20000)),
            "mem_cached_kb": 1_200_000 + int(rng.gauss(0, 30000)),
            "disk_read_kb": max(0, int(rng.gauss(5, 5))),
            "disk_write_kb": max(0, int(rng.gauss(2000, 200))),
            "load1": round(max(0.0, 0.5 + rng.gauss(0, 0.2)), 2),
        })
    if rows:
        rows[0]["cpu_total_pct"] = 0.0
        rows[0]["disk_read_kb"] = 0
        rows[0]["disk_write_kb"] = 0
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=SYSTEM_HEADER)
        w.writeheader()
        for row in rows:
            w.writerow(row)


def write_triggers_jsonl(path, triggers: list):
    with open(path, "w") as f:
        for t in triggers:
            f.write(json.dumps(t) + "\n")


def write_logs(run_dir, run_id):
    (run_dir / "clipper.log").write_text(f"[info] clipper started for {run_id}\n")
    (run_dir / "rosbag2.log").write_text(f"[info] rosbag2 recording for {run_id}\n")
    (run_dir / "load.log").write_text(f"[info] load generator running for {run_id}\n")


# ---------------------------------------------------------------------------
# Trigger / latency modelling
# ---------------------------------------------------------------------------

def extraction_time_s(window_s, bitrate_mbs, comp, par):
    k = 0.01 if comp == "zstd" else 0.003
    par_eff = max(par, 1)
    return window_s * bitrate_mbs * k / par_eff


def make_trigger(rng, name, sent_ns, preroll_ns, postroll_ns, bitrate_mbs, comp, par,
                  bytes_per_s, complete=True):
    window_s = (preroll_ns + postroll_ns) / 1e9
    extract_s = extraction_time_s(window_s, bitrate_mbs, comp, par)
    latency_ns = postroll_ns + int((extract_s + rng.gauss(0, extract_s * 0.05 + 0.05)) * 1e9)
    trig = {
        "name": name, "sent_ns": sent_ns, "anchor_hint_ns": sent_ns,
        "preroll_ns": preroll_ns, "postroll_ns": postroll_ns,
        "recorded_ns": None, "latency_ns": None, "files": None, "bytes": None,
    }
    if complete:
        trig["recorded_ns"] = sent_ns + latency_ns
        trig["latency_ns"] = latency_ns
        nbytes = int(window_s * bytes_per_s)
        trig["files"] = [f"/home/bench/clipped/{name}.mcap"]
        trig["bytes"] = nbytes
    return trig


# ---------------------------------------------------------------------------
# Scenario builders
# ---------------------------------------------------------------------------

class Builder:
    def __init__(self, root: Path, rng: random.Random):
        self.root = root
        self.rng = rng
        self.t_cursor_ns = _T0_NS
        self.n_runs = 0

    def _next_t0(self, span_s):
        t0 = self.t_cursor_ns
        self.t_cursor_ns += int((span_s + 5) * 1e9)
        return t0

    def write_run(self, host, scenario, bitrate_label, comp, par, rep, *,
                  meta_overrides=None, role_defs, triggers=None,
                  measure_s=MEASURE_S, warmup_s=WARMUP_S, phases_extra=None,
                  tegra_base_cpu_pct=15.0, hz=10, run_id_token=None):
        """`run_id_token`, if given, replaces `bitrate_label` in the run_id
        string only — `bitrate_label` still drives the real
        `bitrate_target_mbs` in run.json via `BITRATES[bitrate_label]`.
        `snapshot_sweep` uses this: CONTRACT.md's run_id scheme has no
        preroll slot, and this scenario fixes bitrate throughout its sweep,
        so its otherwise-redundant bitrate slot carries `p{preroll_s}`
        instead — the run_id must stay unique per preroll or later sweep
        points silently overwrite earlier ones (this bit the first version
        of this fixture: see git history / summarize.py's docstring point 2
        for why the run_id has no dedicated slot for this at all)."""
        run_id = make_run_id(host, scenario, run_id_token or bitrate_label, comp, par, rep)
        run_dir = self.root / run_id
        run_dir.mkdir(parents=True, exist_ok=True)

        total_s = warmup_s + measure_s
        t0 = self._next_t0(total_s)
        t_warm_end = t0 + int(warmup_s * 1e9)
        t_end = t0 + int(total_s * 1e9)

        phases = {"warmup": [t0, t_warm_end], "measure": [t_warm_end, t_end]}
        if phases_extra:
            phases.update({k: [t_warm_end + int(v[0] * 1e9), t_warm_end + int(v[1] * 1e9)]
                            for k, v in phases_extra.items()})

        # Every scenario builder below writes its cpu/rss/io closures as
        # functions of "seconds into the measure phase" (t=0 at measure
        # start) — e.g. ten_windows's `t < waiting_s`, one_clip's
        # `20 <= t <= 45`. gen_series, however, walks t from the run's own
        # start (t=0 at warmup start). Shift here, once, centrally, rather
        # than needing every closure to know about warmup_s: samples taken
        # during warmup see the function's t=0 value (steady-state settling
        # to "just entered measure" is a reasonable stand-in, and nothing
        # in summarize.py ever aggregates the warmup phase anyway).
        def _shift(fn, warmup_s=warmup_s):
            return lambda t: fn(max(t - warmup_s, 0.0))

        role_pid_rows = {}
        for (role, pid), spec in role_defs.items():
            shifted_spec = dict(spec)
            for key in ("cpu_pct_fn", "rss_kb_fn", "rchar_bps_fn", "wchar_bps_fn",
                        "read_bps_fn", "write_bps_fn"):
                if key in shifted_spec:
                    shifted_spec[key] = _shift(shifted_spec[key])
            role_pid_rows[(role, pid)] = gen_series(self.rng, t0, total_s, hz, **shifted_spec)
        # CONTRACT.md (settled): role "sampler" is the sampler measuring
        # itself, present in every real run — added here once, centrally,
        # rather than in each scenario builder's own role_defs, so
        # REPORT.md's "the measurement apparatus is not free" caveat has a
        # real computed number to quote (~0.4-0.8% of one core, per the
        # team lead's on-target report) instead of falling back to an
        # estimate every time this pipeline is tested against fixtures.
        role_pid_rows[("sampler", 999)] = gen_series(
            self.rng, t0, total_s, hz,
            cpu_pct_fn=const(0.55), rss_kb_fn=const(4_800),
            rchar_bps_fn=const(0), wchar_bps_fn=const(0),
            read_bps_fn=const(0), write_bps_fn=const(0),
            threads=2, fds=12,
        )
        # CONTRACT.md: the NX kernel (5.15.148-tegra) has no CONFIG_TASKSTATS,
        # so /proc/<pid>/io doesn't exist there at all — every process on
        # that host, not just some. The Nano has it and reports normally.
        io_available = (host == "nano")
        write_samples_csv(run_dir / "samples.csv", role_pid_rows, io_available=io_available)
        write_io_accounting(run_dir / "io_accounting.json", io_available)
        write_system_csv(run_dir / "system.csv", t0, total_s, hz, self.rng)
        cores = HOSTS[host]["cores"]
        write_tegrastats(run_dir / "tegrastats.log", host, t0, total_s, tegra_base_cpu_pct, cores)
        write_triggers_jsonl(run_dir / "triggers.jsonl", triggers or [])
        write_logs(run_dir, run_id)

        bitrate_target = BITRATES[bitrate_label]
        # CONTRACT.md: the bag is lz4-compressed, the recorder writes
        # fastwrite (uncompressed), so achieved > target by roughly the
        # decompression ratio plus record framing ("a few percent" per
        # README.md) — modelled as a +5% bias plus small positive jitter,
        # never symmetric noise that could land below target.
        bitrate_achieved = bitrate_target * (1.05 + abs(self.rng.gauss(0, 0.02)))
        meta = {
            "run_id": run_id, "host": host,
            "distro": HOSTS[host]["distro"], "clipper_version": CLIPPER_VERSION,
            "scenario": scenario,
            # Real fields (confirmed against actual gate-run run.json), not
            # parsed out of the run_id string:
            "variant": "", "rep": rep, "snapshot_arm": None,
            "bitrate_target_mbs": bitrate_target,
            "bitrate_achieved_mbs": round(bitrate_achieved, 2),
            # suite-dev (settled): achieved is now the load rate uniformly,
            # discriminated by this field — "record_dir_write_rate" for
            # every non-snapshot arm; build_snapshot_sweep overrides both
            # this and bitrate_achieved_mbs for its two arms.
            "bitrate_achieved_source": "record_dir_write_rate",
            "record_dir_write_rate_mbs": round(bitrate_achieved, 2),
            "recorder_profile": "fastwrite", "max_cache_size": 0,
            "clip_compression": comp, "extract_parallelism": par,
            "nvpmodel": "MAXN_SUPER", "jetson_clocks": True,
            "warmup_s": warmup_s, "measure_s": measure_s,
            # CONTRACT.md/real data: rosbag2 only emits a drop line when
            # drops actually occur, so `null` (not measured) — not `0` — is
            # the realistic default; scenario builders below override both
            # this and rosbag2_dropped_source for the cells meant to
            # exercise the "measured" state (see build_ten_windows/
            # build_contention).
            "phases": phases, "rosbag2_dropped": None,
            "rosbag2_dropped_source": "not_measured",
            "throttled": False,
            # CONTRACT.md (settled): no "failure_reason" — the real producer
            # writes "incomplete_reasons" (list, verbatim strings) and
            # "oom_killed" (bool, the only OOM signal).
            "incomplete_reasons": [], "oom_killed": False,
            "complete": True,
            "config": {"bitrate": bitrate_label},
        }
        if meta_overrides:
            meta.update(meta_overrides)
        (run_dir / "run.json").write_text(json.dumps(meta, indent=2) + "\n")
        self.n_runs += 1
        return run_id


def clipper_rss_kb_idle(bitrate_mbs):
    return 30_000 + bitrate_mbs * 200


def rosbag2_rss_kb(bitrate_mbs):
    return 80_000 + bitrate_mbs * 1500


def rosbag2_cpu_pct(bitrate_mbs, rng_jitter=0.0):
    return bitrate_mbs * 0.8 + rng_jitter


def player_cpu_pct(bitrate_mbs):
    return bitrate_mbs * 0.5


def build_baseline_and_idle_tail(b: Builder):
    for host in HOSTS:
        for bitrate_label, bitrate_mbs in BITRATES.items():
            for rep in (1, 2, 3):
                # baseline: load + rosbag2, no clipper
                b.write_run(
                    host, "baseline", bitrate_label, "none", 1, rep,
                    role_defs={
                        ("rosbag2", 1001): dict(
                            cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
                            rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                            read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                            threads=8, fds=40,
                        ),
                        ("player", 1002): dict(
                            cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)),
                            rss_kb_fn=const(60_000),
                            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                            read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                            threads=4, fds=20,
                        ),
                    },
                    tegra_base_cpu_pct=bitrate_mbs * 1.0,
                )
                # idle_tail: clipper tailing, no triggers. One whole cell
                # (nx/heavy) is thrown all-throttled, to exercise the "all N
                # runs in this group were excluded as throttled" note rather
                # than just a single flagged rep among usable ones.
                all_throttled_cell = (host == "nx" and bitrate_label == "heavy")
                b.write_run(
                    host, "idle_tail", bitrate_label, "none", 1, rep,
                    meta_overrides={"throttled": True} if all_throttled_cell else None,
                    role_defs={
                        ("clipper", 2001): dict(
                            cpu_pct_fn=const(0.3 + bitrate_mbs * 0.05),
                            rss_kb_fn=const(clipper_rss_kb_idle(bitrate_mbs)),
                            rchar_bps_fn=const(bitrate_mbs * 1e6 * 0.02), wchar_bps_fn=const(0),
                            read_bps_fn=const(bitrate_mbs * 1e6 * 0.0002), write_bps_fn=const(0),
                            threads=6, fds=18,
                        ),
                        ("rosbag2", 1001): dict(
                            cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
                            rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                            read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                            threads=8, fds=40,
                        ),
                        ("player", 1002): dict(
                            cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)),
                            rss_kb_fn=const(60_000),
                            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                            read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                            threads=4, fds=20,
                        ),
                    },
                    tegra_base_cpu_pct=bitrate_mbs * 1.0 + 1,
                )


def build_one_clip(b: Builder):
    bitrate_label, bitrate_mbs = "mid", BITRATES["mid"]
    comp, par = "zstd", 1
    preroll_ns, postroll_ns = 10 * 10**9, 10 * 10**9
    for host in HOSTS:
        for rep in (1, 2, 3):
            incomplete = (host == "nano" and rep == 2)  # the one generic incomplete run
            role_defs = {
                ("clipper", 3001): dict(
                    cpu_pct_fn=lambda t: (2 + bitrate_mbs * 0.3) if 20 <= t <= 45 else 0.4,
                    rss_kb_fn=const(clipper_rss_kb_idle(bitrate_mbs) + 8_000),
                    rchar_bps_fn=lambda t: bitrate_mbs * 1e6 * 1.2 if 20 <= t <= 45 else bitrate_mbs * 1e6 * 0.02,
                    wchar_bps_fn=lambda t: 2e6 if 20 <= t <= 45 else 0,
                    read_bps_fn=lambda t: bitrate_mbs * 1e6 * 0.02 if 20 <= t <= 45 else bitrate_mbs * 1e6 * 0.0002,
                    write_bps_fn=const(0), threads=6 + par, fds=20,
                ),
                ("rosbag2", 1001): dict(
                    cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
                    rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                    rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                    read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                    threads=8, fds=40,
                ),
                ("player", 1002): dict(
                    cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
                    rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                    read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                    threads=4, fds=20,
                ),
            }
            run_id = b.write_run(
                host, "one_clip", bitrate_label, comp, par, rep,
                role_defs=role_defs,
                meta_overrides=(
                    {"complete": False, "incomplete_reasons": ["0 of 1 triggers matched a Recorded"]}
                    if incomplete else None
                ),
                tegra_base_cpu_pct=bitrate_mbs * 1.0 + 5,
            )
            if not incomplete:
                run_dir = b.root / run_id
                meta = json.loads((run_dir / "run.json").read_text())
                warm0, warm1 = meta["phases"]["warmup"]
                sent_ns = warm1 + int(20 * 1e9)
                trig = make_trigger(
                    b.rng, "bench-1", sent_ns, preroll_ns, postroll_ns,
                    bitrate_mbs, comp, par, bitrate_mbs * 1e6,
                )
                write_triggers_jsonl(run_dir / "triggers.jsonl", [trig])
            else:
                run_dir = b.root / run_id
                write_triggers_jsonl(run_dir / "triggers.jsonl", [])


def _ten_windows_role_defs(bitrate_mbs, comp, par, waiting_s, clipping_s):
    clip_cpu = 8 + bitrate_mbs * 1.1 + (6 if comp == "zstd" else 0) - (par - 1) * 1.2
    clip_cpu = max(clip_cpu, 3)

    def clipper_cpu(t):
        return (0.6 + bitrate_mbs * 0.02) if t < waiting_s else clip_cpu

    def clipper_rss(t):
        base = clipper_rss_kb_idle(bitrate_mbs) + 6_000
        return base if t < waiting_s else base + 10_000

    def clipper_rchar(t):
        return bitrate_mbs * 1e6 * 0.02 if t < waiting_s else bitrate_mbs * 1e6 * 1.5

    def clipper_read(t):
        return bitrate_mbs * 1e6 * 0.0002 if t < waiting_s else bitrate_mbs * 1e6 * 1.5 * 0.015

    return {
        ("clipper", 4001): dict(
            cpu_pct_fn=clipper_cpu, rss_kb_fn=clipper_rss,
            rchar_bps_fn=clipper_rchar, wchar_bps_fn=lambda t: 0,
            read_bps_fn=clipper_read, write_bps_fn=lambda t: 0,
            threads=6 + par, fds=18 + (10 if par > 1 else 0),
        ),
        ("rosbag2", 1001): dict(
            cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
            rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
            read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
            threads=8, fds=40,
        ),
        ("player", 1002): dict(
            cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
            rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
            read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
            threads=4, fds=20,
        ),
    }


def build_ten_windows(b: Builder):
    preroll_ns, postroll_ns = 10 * 10**9, 60 * 10**9
    waiting_s, clipping_s = 60.0, 60.0

    cells = []
    for bitrate_label in BITRATES:
        cells.append((bitrate_label, "zstd", 1))
    for extra in (("none", 1), ("zstd", 2), ("zstd", HOSTS["nano"]["cores"])):
        cells.append(("mid", extra[0], extra[1]))

    for host in HOSTS:
        for bitrate_label, comp, par in cells:
            # only run the comp/par sweep cells on nano, to keep the fixture small
            if (comp, par) != ("zstd", 1) and host != "nano":
                continue
            bitrate_mbs = BITRATES[bitrate_label]
            for rep in (1, 2, 3):
                throttled = (host == "nx" and bitrate_label == "heavy" and rep == 1 and comp == "zstd" and par == 1)
                # A distinct cell exercising "throttled: null" — CONTRACT.md:
                # null is not false, so this must be excluded the same as a
                # confirmed throttle, but labelled differently (see
                # RunData.throttled_state / throttled_note).
                throttle_unknown = (host == "nx" and bitrate_label == "light" and rep == 2 and comp == "zstd" and par == 1)
                dropped = 1 if (host == "nano" and bitrate_label == "mid" and comp == "zstd" and par == 1 and rep == 1) else 0
                role_defs = _ten_windows_role_defs(bitrate_mbs, comp, par, waiting_s, clipping_s)
                run_id = b.write_run(
                    host, "ten_windows", bitrate_label, comp, par, rep,
                    role_defs=role_defs,
                    phases_extra={"waiting": (0, waiting_s), "clipping": (waiting_s, waiting_s + clipping_s)},
                    meta_overrides={
                        "throttled": None if throttle_unknown else throttled,
                        **({"throttle_method": "clocks not pinned: jetson_clocks reported inactive"}
                           if throttle_unknown else {}),
                        "rosbag2_dropped": dropped,
                        "rosbag2_dropped_source": "reported" if dropped else "clean_absent",
                    },
                    tegra_base_cpu_pct=bitrate_mbs * 1.2 + 10,
                )
                run_dir = b.root / run_id
                meta = json.loads((run_dir / "run.json").read_text())
                warm0, warm1 = meta["phases"]["warmup"]
                triggers = []
                for i in range(10):
                    sent_ns = warm1 + int(i * 1.0 * 1e9)
                    triggers.append(make_trigger(
                        b.rng, f"bench-{i}", sent_ns, preroll_ns, postroll_ns,
                        bitrate_mbs, comp, par, bitrate_mbs * 1e6,
                    ))
                write_triggers_jsonl(run_dir / "triggers.jsonl", triggers)


def build_ten_windows_no_waiting_phase(b: Builder):
    """The exact trap the team lead reported from real data: with a short
    enough postroll relative to the stagger x trigger count, trigger 0
    becomes copyable *before* trigger 9's own postroll has even elapsed, so
    there is no interval where every handler is parked and none is
    copying — "waiting" is not zero-length, it does not exist. run.json
    provides no "waiting" bounds at all here (only "clipping", spanning the
    whole measure phase) plus a "phase_notes" explanation — this fixture
    exists specifically to prove analysis_ten_windows_phases reports that
    absence explicitly rather than a blended "n/a"."""
    host, bitrate_label, bitrate_mbs = "nano", "heavy", BITRATES["heavy"]
    comp, par = "none", 2
    preroll_ns, postroll_ns = 10 * 10**9, 5 * 10**9  # postroll(5s) < stagger(1s) x count(10)
    clipping_s = 120.0
    role_defs = _ten_windows_role_defs(bitrate_mbs, comp, par, waiting_s=0.0, clipping_s=clipping_s)
    run_id = b.write_run(
        host, "ten_windows", bitrate_label, comp, par, 9,
        role_defs=role_defs,
        phases_extra={"clipping": (0, clipping_s)},  # no "waiting" key at all
        meta_overrides={
            "phase_notes": {
                "waiting": "stagger(1s) x trigger_count(10) >= postroll(5s): no interval where "
                            "every handler is parked and none is copying",
            },
        },
        tegra_base_cpu_pct=bitrate_mbs * 1.2 + 10,
    )
    run_dir = b.root / run_id
    meta = json.loads((run_dir / "run.json").read_text())
    warm0, warm1 = meta["phases"]["warmup"]
    triggers = [
        make_trigger(
            b.rng, f"bench-{i}", warm1 + int(i * 1.0 * 1e9), preroll_ns, postroll_ns,
            bitrate_mbs, comp, par, bitrate_mbs * 1e6,
        )
        for i in range(10)
    ]
    write_triggers_jsonl(run_dir / "triggers.jsonl", triggers)


def build_snapshot_sweep(b: Builder):
    host = "nano"
    bitrate_label, bitrate_mbs = "mid", BITRATES["mid"]
    comp, par = "zstd", 1
    postroll_s = 8.0
    cap_mb = HOSTS[host]["ram_mb"] * 0.8
    overhead_mb = 60.0

    for preroll_s in (5, 30, 60, 300, 600):
        window_s = preroll_s + postroll_s
        needed_mb = overhead_mb + window_s * bitrate_mbs
        oom = needed_mb > cap_mb
        measure_s = 40.0  # time for the buffer to fill + snapshot call

        # --- snapshot-mode arm (rosbag2 buffers the window in RAM) ---
        if oom:
            fill_time_s = measure_s * (cap_mb - overhead_mb) / max(needed_mb - overhead_mb, 1) * 0.6
            fill_time_s = min(fill_time_s, measure_s * 0.8)

            def rosbag2_rss(t, cap_mb=cap_mb, overhead_mb=overhead_mb, needed_mb=needed_mb, fill_time_s=fill_time_s):
                mb = overhead_mb + (needed_mb - overhead_mb) * min(t / fill_time_s, 1.0) if fill_time_s > 0 else cap_mb
                return min(mb, cap_mb) * 1024
        else:
            def rosbag2_rss(t, needed_mb=needed_mb, measure_s=measure_s):
                ramp_s = min(measure_s * 0.3, 10.0)
                frac = min(t / ramp_s, 1.0) if ramp_s > 0 else 1.0
                return (60.0 + (needed_mb - 60.0) * frac) * 1024

        role_defs = {
            ("rosbag2", 1001): dict(
                cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs) + 3),
                rss_kb_fn=rosbag2_rss,
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6 * 0.1),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6 * 0.1),
                threads=8, fds=40,
            ),
            ("player", 1002): dict(
                cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                threads=4, fds=20,
            ),
        }
        # NOTE: `rep` is (host,scenario,bitrate,comp,par,rep) in the run_id
        # scheme — this fixture uses rep=1 for the snapshot-mode arm and
        # rep=2 for the clipper-tail arm purely to keep their directories
        # distinct. The real discriminator is run.json's top-level
        # "snapshot_arm" field (see summarize.py's `_snapshot_arm` —
        # exact string vocabulary still unconfirmed against a real
        # snapshot_sweep run.json, so "snapshot"/"tail" here are this
        # module's own best guess, matching `_snapshot_arm`'s loose
        # "snap"/"tail" substring match).
        run_id = b.write_run(
            host, "snapshot_sweep", bitrate_label, comp, par, 1,
            role_defs=role_defs,
            meta_overrides={
                "recorder_profile": "snapshot", "max_cache_size": int(needed_mb * 1024 * 1024),
                "snapshot_arm": "snapshot", "variant": f"snap-pre{int(preroll_s)}",
                "complete": not oom,
                "oom_killed": oom,
                "incomplete_reasons": (
                    [f"OOM-killed: cache exceeded MemoryMax at preroll={preroll_s:.0f}s"] if oom else []
                ),
                # suite-dev (settled): achieved is the LOAD rate uniformly,
                # even for the snapshot arm — computed there from the dumped
                # mcap's own Statistics-record span (cross-checked to ~0.1%
                # of the independently-measured load on real data), never
                # the naive bytes-written/elapsed dump-rate artefact. An
                # OOM-killed cache has no dump to measure at all.
                "bitrate_achieved_mbs": None if oom else round(bitrate_mbs * (1.03 + abs(b.rng.gauss(0, 0.01))), 2),
                "bitrate_achieved_source": (
                    "not measurable: no readable cache dump — expected when the cache was "
                    "OOM-killed before the snapshot fired" if oom else
                    "snapshot_cache_dump_over_buffered_span"
                ),
            },
            measure_s=measure_s, tegra_base_cpu_pct=30,
            run_id_token=f"p{preroll_s}",
        )
        run_dir = b.root / run_id
        meta = json.loads((run_dir / "run.json").read_text())
        warm0, warm1 = meta["phases"]["warmup"]
        trig = make_trigger(
            b.rng, "snap", warm1, int(preroll_s * 1e9), int(postroll_s * 1e9),
            bitrate_mbs, comp, par, bitrate_mbs * 1e6, complete=not oom,
        )
        write_triggers_jsonl(run_dir / "triggers.jsonl", [trig])

        # --- clipper-tail arm (flat RSS regardless of preroll) ---
        role_defs_tail = {
            ("clipper", 3001): dict(
                cpu_pct_fn=const(1.0 + bitrate_mbs * 0.03),
                rss_kb_fn=const(clipper_rss_kb_idle(bitrate_mbs) + 5_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6 * 0.03), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.0003), write_bps_fn=const(0),
                threads=6 + par, fds=18,
            ),
            ("rosbag2", 1001): dict(
                cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
                rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                threads=8, fds=40,
            ),
            ("player", 1002): dict(
                cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                threads=4, fds=20,
            ),
        }
        run_id2 = b.write_run(
            host, "snapshot_sweep", bitrate_label, comp, par, 2,
            role_defs=role_defs_tail,
            meta_overrides={
                "recorder_profile": "fastwrite", "max_cache_size": 0,
                "snapshot_arm": "tail", "variant": f"tail-pre{int(preroll_s)}",
            },
            measure_s=measure_s, tegra_base_cpu_pct=15,
            run_id_token=f"p{preroll_s}",
        )
        tail_dir = b.root / run_id2
        meta2 = json.loads((tail_dir / "run.json").read_text())
        warm0b, warm1b = meta2["phases"]["warmup"]
        trig2 = make_trigger(
            b.rng, "snap", warm1b, int(preroll_s * 1e9), int(postroll_s * 1e9),
            bitrate_mbs, comp, par, bitrate_mbs * 1e6, complete=True,
        )
        write_triggers_jsonl(tail_dir / "triggers.jsonl", [trig2])


def build_soak(b: Builder):
    bitrate_label, bitrate_mbs = "mid", BITRATES["mid"]
    comp, par = "zstd", 1
    duration_s = 3600.0  # 1h compressed fixture in place of the real suite's 4h (see module docstring)
    n_clips = int(duration_s / 60)

    for host in HOSTS:
        drift_rate_kb_per_s = 0.6 if host == "nano" else 0.15  # nano: a small, real-looking leak

        def clipper_rss(t, drift=drift_rate_kb_per_s):
            return clipper_rss_kb_idle(bitrate_mbs) + drift * t

        role_defs = {
            ("clipper", 5001): dict(
                cpu_pct_fn=lambda t: 3 + bitrate_mbs * 0.15 if int(t) % 60 < 15 else 0.5,
                rss_kb_fn=clipper_rss,
                rchar_bps_fn=lambda t: bitrate_mbs * 1e6 * (0.8 if int(t) % 60 < 15 else 0.02),
                wchar_bps_fn=lambda t: 0,
                read_bps_fn=lambda t: bitrate_mbs * 1e6 * 0.01 if int(t) % 60 < 15 else bitrate_mbs * 1e6 * 0.0002,
                write_bps_fn=lambda t: 0,
                threads=7, fds=18,
            ),
            ("rosbag2", 1001): dict(
                cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs)),
                rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                threads=8, fds=40,
            ),
            ("player", 1002): dict(
                cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                threads=4, fds=20,
            ),
        }
        run_id = b.write_run(
            host, "soak", bitrate_label, comp, par, 1,
            role_defs=role_defs, measure_s=duration_s, warmup_s=30,
            tegra_base_cpu_pct=bitrate_mbs * 1.0 + 3, hz=10,
        )
        run_dir = b.root / run_id
        meta = json.loads((run_dir / "run.json").read_text())
        warm0, warm1 = meta["phases"]["warmup"]
        triggers = []
        for i in range(n_clips):
            sent_ns = warm1 + int(i * 60 * 1e9)
            triggers.append(make_trigger(
                b.rng, f"soak-{i}", sent_ns, 10 * 10**9, 10 * 10**9,
                bitrate_mbs, comp, par, bitrate_mbs * 1e6,
            ))
        write_triggers_jsonl(run_dir / "triggers.jsonl", triggers)


def build_contention(b: Builder):
    bitrate_label, bitrate_mbs = "mid", BITRATES["mid"]
    comp, par = "zstd", 1
    for host in HOSTS:
        hog_cores = 4 if host == "nano" else 6
        role_defs = {
            ("clipper", 6001): dict(
                cpu_pct_fn=const(2 + bitrate_mbs * 0.25), rss_kb_fn=const(clipper_rss_kb_idle(bitrate_mbs) + 4_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6 * 0.3), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.006), write_bps_fn=const(0),
                threads=6 + par, fds=18,
            ),
            ("rosbag2", 1001): dict(
                cpu_pct_fn=const(rosbag2_cpu_pct(bitrate_mbs) * 1.1), rss_kb_fn=const(rosbag2_rss_kb(bitrate_mbs)),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(bitrate_mbs * 1e6),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.9), write_bps_fn=const(bitrate_mbs * 1e6),
                threads=8, fds=40,
            ),
            ("player", 1002): dict(
                cpu_pct_fn=const(player_cpu_pct(bitrate_mbs)), rss_kb_fn=const(60_000),
                rchar_bps_fn=const(bitrate_mbs * 1e6), wchar_bps_fn=const(0),
                read_bps_fn=const(bitrate_mbs * 1e6 * 0.1), write_bps_fn=const(0),
                threads=4, fds=20,
            ),
        }
        for i in range(hog_cores):
            role_defs[("hog", 7000 + i)] = dict(
                cpu_pct_fn=const(98.0), rss_kb_fn=const(2_000),
                rchar_bps_fn=const(0), wchar_bps_fn=const(0),
                read_bps_fn=const(0), write_bps_fn=const(0),
                threads=1, fds=4,
            )
        run_id = b.write_run(
            host, "contention", bitrate_label, comp, par, 1,
            role_defs=role_defs,
            meta_overrides={
                "rosbag2_dropped": 1 if host == "nano" else 0,
                "rosbag2_dropped_source": "reported" if host == "nano" else "clean_absent",
            },
            tegra_base_cpu_pct=90,
        )
        run_dir = b.root / run_id
        meta = json.loads((run_dir / "run.json").read_text())
        warm0, warm1 = meta["phases"]["warmup"]
        trig = make_trigger(
            b.rng, "contention-1", warm1 + 5 * 10**9, 10 * 10**9, 10 * 10**9,
            bitrate_mbs, comp, par, bitrate_mbs * 1e6,
        )
        write_triggers_jsonl(run_dir / "triggers.jsonl", [trig])


def build_broken_dir(root: Path):
    """A directory with no run.json at all — the "unparsable" failure kind,
    distinct from a well-formed run.json with complete: false."""
    d = root / "nano__one_clip__mid__zstd__par1__rep99"
    d.mkdir(parents=True, exist_ok=True)
    (d / "clipper.log").write_text("crashed before run.json could be written\n")


def build_skipped(b: Builder):
    """A run.json-bearing directory the orchestrator's disk preflight chose
    not to run at all — "skipped", distinct from "failed" (started, didn't
    finish). No samples.csv/system.csv/tegrastats.log/triggers.jsonl, since
    the sampler was never started."""
    run_id = make_run_id("nx", "ten_windows", "heavy", "zstd", 1, 9)
    run_dir = b.root / run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    meta = {
        "run_id": run_id, "host": "nx", "distro": HOSTS["nx"]["distro"],
        "clipper_version": CLIPPER_VERSION, "scenario": "ten_windows",
        "variant": "", "rep": 9, "snapshot_arm": None,
        "bitrate_target_mbs": BITRATES["heavy"], "bitrate_achieved_mbs": None,
        "recorder_profile": "fastwrite", "max_cache_size": 0,
        "clip_compression": "zstd", "extract_parallelism": 1,
        "nvpmodel": "MAXN_SUPER", "jetson_clocks": True,
        "warmup_s": WARMUP_S, "measure_s": MEASURE_S,
        "phases": {}, "rosbag2_dropped": None, "throttled": False, "complete": False,
        "incomplete_reasons": [], "oom_killed": False,
        "config": {"bitrate": "heavy"},
        # CONTRACT.md (settled): "skipped_reason" is the real key.
        "skipped_reason": "disk preflight: <5 GB free on ~/bench's filesystem",
    }
    (run_dir / "run.json").write_text(json.dumps(meta, indent=2) + "\n")
    (run_dir / "clipper.log").write_text("skipped before start\n")


def build_harness_artefacts(root: Path):
    """Non-run directories/files a real results tree turns up alongside run
    directories — must never be reported as failures (see
    `_looks_like_run_dir` in summarize.py)."""
    calib = root / "calibration"
    calib.mkdir(parents=True, exist_ok=True)
    (calib / "nano_mid.json").write_text(json.dumps({"rate": 0.34, "est_mbs": 20.1}) + "\n")
    (root / "plan-nano.json").write_text(json.dumps({"host": "nano", "runs": []}, indent=2) + "\n")
    (root / "suite.log").write_text("nano__baseline__light__none__par1__rep1 complete\n")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True, help="fixture results tree root")
    ap.add_argument("--seed", type=int, default=20260803)
    args = ap.parse_args(argv)

    args.out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)
    b = Builder(args.out, rng)

    build_baseline_and_idle_tail(b)
    build_one_clip(b)
    build_ten_windows(b)
    build_ten_windows_no_waiting_phase(b)
    build_snapshot_sweep(b)
    build_soak(b)
    build_contention(b)
    build_broken_dir(args.out)
    build_skipped(b)
    build_harness_artefacts(args.out)

    print(f"wrote {b.n_runs} well-formed runs (+2 snapshot-sweep-arm dirs/preroll, "
          f"+1 broken dir, +1 skipped run, +3 non-run harness artefacts) under {args.out}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
