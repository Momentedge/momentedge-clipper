#!/usr/bin/env python3
"""Ingest a `~/bench/results/<run_id>/` tree and compute the report's numbers.

Runs on the workstation only. This module is both a library (report.py and
charts.py import it directly — no serialization round-trip) and a CLI that
dumps its intermediate/aggregate numbers as JSON for standalone debugging:

    python summarize.py --results DIR [--out summary.json]

## Model

`load_results_tree()` walks `DIR` and returns `(runs, exclusions)`:

- `runs`: every genuine run directory (see `_looks_like_run_dir`) with a
  parseable `run.json`, *including* ones marked `"complete": false` or
  skipped by the orchestrator's preflight — they are kept so soak/snapshot
  diagnostics can still look at partial samples, but every aggregation
  function below filters them out (`.complete`, then `.throttled` — see
  "Throttled exclusion") before computing a statistic.
- `exclusions`: a `RunExclusion` per excluded run, tagged with a `kind` —
  `"unparsable"` (looked like a run directory, no parseable run.json — a
  real failure, died before writing one), `"skipped"` (the orchestrator's
  disk preflight chose not to run this config at all) or `"failed"`
  (started, didn't finish, wasn't skipped). CONTRACT.md and the harness
  brief require a reader be able to tell "we chose not to run this" from
  "this ran and broke" — these three kinds are that distinction, kept
  separate through to REPORT.md rather than lumped into one bucket.

A directory whose name does not match the run_id scheme at all
(`calibration/`, other harness artefacts observed in a real results tree)
produces no entry in either list — see `_looks_like_run_dir`. The reliable
test for "is this a run directory" is the name, not "has run.json", so a
genuinely broken run (which has a run_id-shaped name but died before writing
run.json) still surfaces as a failure.

Each `RunData` carries `role_series: dict[role, list[Sample]]` — samples.csv
rows grouped by `role` and merged across pids sharing a role (e.g. `hog`'s N
busy-loop processes) by summing at each shared `ts_ns` tick (sampler.py
stamps one `ts_ns` per tick for every pid it reads that tick, so an exact-tick
merge is correct, not approximate). The run's first CSV row (both
samples.csv and system.csv) is discarded before any of this — CONTRACT.md:
every delta-derived column reads zero on the first row because there is no
prior sample, and averaging that in is a small but systematic bias.

`run_phase_stats(run, role, phase)` slices that role's series to the
`ts_ns` bounds in `run.json["phases"][phase]` and reduces the window to one
`PhaseStats`: CPU-percent is recomputed from the **utime+stime delta across
the whole window** (not by averaging the already-delta'd per-sample
`cpu_pct` column) because that is numerically exact regardless of sample
spacing, while RSS is a mean/peak over the same window and IO fields are
end-minus-start deltas (cumulative counters) that propagate `None` — see
"Empty vs. zero IO" below — rather than ever being treated as 0.

## Empty vs. zero IO (the highest-consequence correctness rule here)

CONTRACT.md: the NX kernel (`5.15.148-tegra`) has no `CONFIG_TASKSTATS`, so
`/proc/<pid>/io` does not exist there at all; the four IO columns (`rchar`,
`wchar`, `read_bytes`, `write_bytes`) are written **empty**, never `0`, when
unmeasurable. Coercing empty to 0 and averaging it would manufacture "clipper
performs no disk IO on the NX" out of the absence of data — CONTRACT.md's own
words for the worst failure mode available to this analysis. This module
makes that structurally hard to do by accident:

- `Sample`'s four IO fields are `Optional[float]`; `_parse_optional_float`
  parses an empty CSV cell as `None`, never `0.0`.
- Every place an IO field is combined (merging pids at a tick, taking a
  phase delta) propagates `None` rather than silently treating it as zero:
  `_merge_role_rows` sums to `None` if any contributing pid's field is
  missing; `_delta_or_none` returns `None` if either endpoint is `None`.
- `agg_io_stat` (as opposed to the generic `agg_stat`) is the only aggregator
  ever used on an IO-derived number. It returns `{"not_measurable": True}`
  when the input has zero non-missing values, and the numeric branch
  (median/min/max) is only reachable past that check — the
  `assert len(present) > 0` inside it exists specifically so that a future
  edit which removes or weakens that guard fails loudly instead of quietly
  computing a number from nothing.
- Per CONTRACT.md, availability is read from each run's `io_accounting.json`
  (`{"io_accounting_available": bool, "probed_via": str}`, written whether
  the probe succeeds or fails) via `_run_has_io()` — never inferred from
  "are the columns empty", so "this kernel cannot measure it" and "this run
  happened to record nothing" stay distinguishable, per CONTRACT.md's own
  instruction. If `io_accounting.json` is missing entirely (an older or
  malformed run), `_run_has_io` falls back to scanning that run's own
  samples, and only ever returns `True` from that fallback (positive
  evidence of a real value) — it never asserts unavailability from the
  file's absence, since that would repeat the same "silence means zero"
  mistake at one remove.
- `analysis_page_cache` (direct per-process rchar-vs-read_bytes, real
  measurement) only ever runs on runs where `_run_has_io() is True`.
  `analysis_page_cache_system` is the separate, explicitly weaker fallback —
  system.csv's `disk_read_kb` (from `/proc/diskstats`, unaffected by the
  per-process gap) differenced against `baseline` at the same host and
  target bitrate — for runs where it is not. report.py must present these as
  two different strengths of evidence and never blend them into one number.

## Throttled exclusion

A `throttled: true` run is not silently averaged in: `split_throttled()`
partitions a group's runs into `usable`/`throttled`, and every aggregate
below is computed from `usable` only. Because the harness's thermal-throttle
detector has a known false-positive bug as of this writing (misreading a fan
trip point as the CPU limit — see the team-lead message this shipped
against), a whole group can come back with `usable` empty while `throttled`
is not; `throttled_note()` turns that into an explicit
`"all N run(s) ... were excluded as throttled"` string carried on the row,
so an emptied-by-throttling group is never visually indistinguishable from
"no data at all" — the same "shout, don't hide" principle as the IO rule
above.

## Units

Per CONTRACT.md's settled conventions, throughput (rchar/read_bytes/wchar
/write_bytes rates, system.csv's disk_read_kb-derived rates) is **decimal
MB — 10⁶ bytes**, computed here directly from the underlying byte counts.
RSS is a memory *size*, not a throughput, and keeps the traditional
1024-based unit every `/proc`-reading tool uses — reported here as **MiB**
(fields are suffixed `_mib`, not `_mb`, specifically so a reader can never
mistake one binary-vs-decimal convention for the other).

## Target vs. achieved bitrate

CONTRACT.md: the replay bag is lz4-compressed while the recorder writes
`fastwrite` (uncompressed), so bytes actually written exceed the bag's
stored rate; `bitrate_target_mbs` only selected the replay rate.  Grouping
is still by target (`config_key`) — it is what identifies matching arms
across scenarios — but every row also carries `bitrate_achieved_mbs`
(median/min/max over the group's usable runs), which is the number
report.py quotes.

## Settled against real data (previously flagged assumptions)

Three real gate runs (Nano: idle_tail + one_clip; NX: idle_tail) exist as of
this writing and settled several things this module previously had to
guess at:

1. **`rep` is a first-class top-level `run.json` field** (`"rep": 1`), not
   something to parse out of the `run_id` string. `parse_run_id()` still
   exists — it is the reliable test for "is this a run directory" in
   `_looks_like_run_dir`, and a display fallback — but no analysis reads
   `.get("rep")` from it or needs to.
2. **Snapshot-sweep arm/preroll are first-class fields, not a repurposed
   run_id slot.** `run.json` carries top-level `"variant"` (e.g.
   `"snap-pre300"`) and `"snapshot_arm"` (`null` outside `snapshot_sweep`)
   directly — confirmed against real `idle_tail`/`one_clip` run.json, which
   show `"variant": ""` and `"snapshot_arm": null`. `_snapshot_arm()` and
   `_snapshot_preroll_s()` read these. **The exact `snapshot_arm` string
   vocabulary for an actual snapshot_sweep run is still unconfirmed** — no
   real snapshot_sweep run.json exists yet, only the team lead's report of
   the field names and one example run_id/variant pair — flagged in
   `_snapshot_arm`'s own docstring, not guessed past that report.
3. **There is no `failure_reason` key.** `run.json`'s real shape (per
   CONTRACT.md, confirmed against real data) is `"incomplete_reasons"` (a
   list of specific strings, reported verbatim — never collapsed to a
   generic "incomplete"), `"oom_killed"` (a bool — the *only* signal
   `analysis_snapshot_sweep` uses for its OOM annotation), and
   `"skipped_reason"` (a string, present only when the disk preflight
   declined to run an arm — read by `RunData.skip_reason`).
4. **`ts_ns` epoch.** Confirmed against `lib/sampler.py`: `ts_ns =
   time.time_ns()`, wall-clock epoch nanoseconds, shared for every pid
   sampled in one tick.
5. **tegrastats has no epoch and no per-host offset recorded (yet).**
   Confirmed the two real hosts are 4 hours apart (Nano `Etc/UTC`, NX
   `America/New_York`/EDT) — see `_load_aligned_tegrastats` below and
   `tegraparse.py`'s module docstring for how this is recovered per-run
   instead of assumed from the analysis workstation's own timezone (which
   was tried, and proven wrong, in an earlier version of this module).

**Still an open, but lower-risk, assumption:** the exact `snapshot_arm`
string vocabulary (point 2 above) — everything else in this list is now
grounded in real `run.json` content, not guessed.
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import re
import sys
from collections import defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import numpy as np

import tegraparse


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

_SAMPLE_REQUIRED_FIELDS = ("cpu_pct", "utime_s", "stime_s", "rss_kb", "vmhwm_kb", "threads", "fds")
_SAMPLE_IO_FIELDS = ("rchar", "wchar", "read_bytes", "write_bytes")


@dataclass
class Sample:
    ts_ns: int
    cpu_pct: float
    utime_s: float
    stime_s: float
    rss_kb: float
    vmhwm_kb: float
    rchar: Optional[float]
    wchar: Optional[float]
    read_bytes: Optional[float]
    write_bytes: Optional[float]
    threads: float
    fds: float


@dataclass
class SystemSample:
    ts_ns: int
    cpu_total_pct: float
    mem_used_kb: float
    mem_cached_kb: float
    disk_read_kb: float
    disk_write_kb: float
    load1: float


def _parse_optional_float(raw) -> Optional[float]:
    """Empty string means "not measurable here" — never coerced to 0.0 (see
    module docstring, "Empty vs. zero IO")."""
    if raw is None:
        return None
    s = raw.strip()
    return None if s == "" else float(s)


def _merge_role_rows(rows: list) -> list:
    """Group raw samples.csv rows sharing one role by ts_ns, summing across
    pids present at that tick. IO fields sum to `None` (not a partial sum)
    if ANY contributing pid's value at that tick is missing — a sum with a
    hole in it is not a real total."""
    buckets: dict = defaultdict(list)
    for r in rows:
        buckets[int(r["ts_ns"])].append(r)
    merged = []
    for ts_ns in sorted(buckets):
        group = buckets[ts_ns]
        kwargs = {f: sum(float(g[f]) for g in group) for f in _SAMPLE_REQUIRED_FIELDS}
        for f in _SAMPLE_IO_FIELDS:
            vals = [_parse_optional_float(g[f]) for g in group]
            kwargs[f] = None if any(v is None for v in vals) else sum(vals)
        merged.append(Sample(ts_ns=ts_ns, **kwargs))
    return merged


def _drop_first_tick(rows: list) -> list:
    """CONTRACT.md: the first row of every run reads zero for every
    delta-derived column, since there is no prior sample — discarded
    wholesale (not just its delta columns) rather than averaged in; one
    sample out of ~1500 in a typical run, so the loss is negligible."""
    if not rows:
        return rows
    min_ts = min(int(row["ts_ns"]) for row in rows)
    return [row for row in rows if int(row["ts_ns"]) != min_ts]


def read_samples_csv(path: Path) -> dict:
    with open(path, newline="") as f:
        all_rows = _drop_first_tick(list(csv.DictReader(f)))
    by_role: dict = defaultdict(list)
    for row in all_rows:
        by_role[row["role"]].append(row)
    return {role: _merge_role_rows(rows) for role, rows in by_role.items()}


def read_system_csv(path: Path) -> list:
    with open(path, newline="") as f:
        all_rows = _drop_first_tick(list(csv.DictReader(f)))
    return [
        SystemSample(
            ts_ns=int(row["ts_ns"]),
            cpu_total_pct=float(row["cpu_total_pct"]),
            mem_used_kb=float(row["mem_used_kb"]),
            mem_cached_kb=float(row["mem_cached_kb"]),
            disk_read_kb=float(row["disk_read_kb"]),
            disk_write_kb=float(row["disk_write_kb"]),
            load1=float(row["load1"]),
        )
        for row in all_rows
    ]


def read_triggers_jsonl(path: Path) -> list:
    out = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def parse_run_id(run_id: str) -> dict:
    parts = run_id.split("__")
    if len(parts) != 6:
        return {
            "host": None, "scenario": None, "bitrate": None, "comp": None,
            "par": None, "rep": None, "raw": run_id,
        }
    host, scenario, bitrate, comp, par_token, rep_token = parts
    par = par_token[3:] if par_token.startswith("par") else par_token
    rep = None
    if rep_token.startswith("rep"):
        try:
            rep = int(rep_token[3:])
        except ValueError:
            rep = None
    return {
        "host": host, "scenario": scenario, "bitrate": bitrate, "comp": comp,
        "par": par, "rep": rep, "raw": run_id,
    }


def _looks_like_run_dir(name: str) -> bool:
    """True iff `name` matches the run_id naming scheme — the reliable test
    for "is this a run directory" (a real results tree contains harness
    artefacts like `calibration/` that are not runs and must never be
    reported as failures; see module docstring)."""
    parts = name.split("__")
    if len(parts) != 6:
        return False
    host, scenario, bitrate, comp, par_token, rep_token = parts
    if not all((host, scenario, bitrate, comp)):
        return False
    if not par_token.startswith("par") or not par_token[3:]:
        return False
    if not rep_token.startswith("rep") or not rep_token[3:].isdigit():
        return False
    return True


@dataclass
class RunData:
    run_id: str
    dir: Path
    meta: dict
    role_series: dict
    system: list
    triggers: list
    id_parts: dict
    io_accounting: Optional[dict] = None

    @property
    def complete(self) -> bool:
        return bool(self.meta.get("complete", False))

    @property
    def throttled_state(self) -> Optional[bool]:
        """True/False if measured, `None` if unknown. CONTRACT.md
        (settled): `throttled` can itself be JSON `null` (with
        `throttle_method` explaining why, e.g. clocks not pinned) — **null
        is not false**. `bool(None)` equals `bool(False)` in Python, so a
        naive `bool(meta.get("throttled", False))` would silently read an
        unmeasured status as "confirmed not throttled" — exactly the
        empty-vs-zero mistake this suite keeps hitting, just in a new
        field. This property preserves the third state explicitly; use
        `throttled` (below) for exclusion decisions and `throttle_reason`
        for why a status is unknown."""
        v = self.meta.get("throttled")
        return v if isinstance(v, bool) else None

    @property
    def throttle_reason(self) -> Optional[str]:
        return self.meta.get("throttle_method")

    @property
    def throttled(self) -> bool:
        """True whenever a run should be excluded from "usable" data —
        confirmed throttled OR unknown status. An unmeasured throttle
        status is not the same claim as confirmed-clean, so it must not
        silently pass as safe; it also must not be reported as
        "confirmed throttled" (a different, stronger claim), which is why
        `throttled_note()` distinguishes the two via `throttled_state`
        rather than folding them into one undifferentiated count."""
        return self.throttled_state is not False

    @property
    def skip_reason(self) -> Optional[str]:
        """CONTRACT.md (settled): "skipped_reason" is the real key, present
        only when the disk preflight declined to run an arm."""
        return self.meta.get("skipped_reason")


@dataclass
class RunExclusion:
    run_id: str
    dir: Path
    kind: str  # "unparsable" | "skipped" | "failed"
    reason: str


def _run_has_io(run: RunData) -> Optional[bool]:
    """Whether this run's kernel could report per-process IO at all. Read
    from `io_accounting.json` first (CONTRACT.md's authoritative source);
    falls back to scanning the run's own samples only when that file is
    missing, and only ever returns `True` from the fallback — see module
    docstring."""
    if run.io_accounting is not None and "io_accounting_available" in run.io_accounting:
        return bool(run.io_accounting["io_accounting_available"])
    for series in run.role_series.values():
        for s in series:
            if s.rchar is not None:
                return True
    return None


def load_run(run_dir: Path) -> RunData:
    meta = json.loads((run_dir / "run.json").read_text())
    samples_path = run_dir / "samples.csv"
    role_series = read_samples_csv(samples_path) if samples_path.exists() else {}
    system_path = run_dir / "system.csv"
    system = read_system_csv(system_path) if system_path.exists() else []
    triggers_path = run_dir / "triggers.jsonl"
    triggers = read_triggers_jsonl(triggers_path) if triggers_path.exists() else []
    io_path = run_dir / "io_accounting.json"
    io_accounting = json.loads(io_path.read_text()) if io_path.exists() else None
    run_id = meta.get("run_id", run_dir.name)
    return RunData(
        run_id=run_id, dir=run_dir, meta=meta, role_series=role_series,
        system=system, triggers=triggers, id_parts=parse_run_id(run_id),
        io_accounting=io_accounting,
    )


def load_results_tree(root: Path):
    """Returns (runs, exclusions). See module docstring for what each holds
    and the three `RunExclusion.kind` values."""
    runs = []
    exclusions = []
    for d in sorted(p for p in root.iterdir() if p.is_dir()):
        if not _looks_like_run_dir(d.name):
            continue  # not a run directory at all (calibration/, etc.)
        meta_path = d / "run.json"
        if not meta_path.exists():
            exclusions.append(RunExclusion(d.name, d, "unparsable", "missing run.json"))
            continue
        try:
            run = load_run(d)
        except (json.JSONDecodeError, OSError, KeyError, ValueError) as exc:
            exclusions.append(RunExclusion(d.name, d, "unparsable", f"failed to load: {exc}"))
            continue
        runs.append(run)
        if run.skip_reason:
            exclusions.append(RunExclusion(run.run_id, d, "skipped", run.skip_reason))
        elif not run.complete:
            # CONTRACT.md (settled): "incomplete_reasons" is a list of
            # specific strings — report verbatim, never collapse to a
            # generic "incomplete".
            reasons = run.meta.get("incomplete_reasons") or []
            reason = "; ".join(reasons) if reasons else "incomplete (complete: false, no incomplete_reasons given)"
            exclusions.append(RunExclusion(run.run_id, d, "failed", reason))
    return runs, exclusions


# ---------------------------------------------------------------------------
# Phase slicing
# ---------------------------------------------------------------------------

@dataclass
class PhaseStats:
    role: str
    phase: str
    n_samples: int
    duration_s: float
    cpu_pct: Optional[float]
    rss_kb_mean: Optional[float]
    rss_kb_peak: Optional[float]
    rchar_delta: Optional[float]
    wchar_delta: Optional[float]
    read_bytes_delta: Optional[float]
    write_bytes_delta: Optional[float]
    rchar_rate_bps: Optional[float]
    read_bytes_rate_bps: Optional[float]
    threads_peak: Optional[float]
    fds_peak: Optional[float]


def phase_bounds(meta: dict, phase: str):
    b = (meta.get("phases") or {}).get(phase)
    if not b or len(b) != 2:
        return None
    return int(b[0]), int(b[1])


def slice_window(samples: list, t0: int, t1: int) -> list:
    return [s for s in samples if t0 <= s.ts_ns <= t1]


def _delta_or_none(end, start):
    if end is None or start is None:
        return None
    return end - start


def phase_stats(samples: list, t0: int, t1: int, role: str, phase: str):
    win = slice_window(samples, t0, t1)
    if not win:
        return None
    duration_s = (win[-1].ts_ns - win[0].ts_ns) / 1e9
    if duration_s > 0:
        cpu_time_delta = (win[-1].utime_s + win[-1].stime_s) - (
            win[0].utime_s + win[0].stime_s
        )
        cpu_pct = cpu_time_delta / duration_s * 100
    else:
        cpu_pct = win[0].cpu_pct
    rss = np.array([s.rss_kb for s in win], dtype=float)
    rchar_delta = _delta_or_none(win[-1].rchar, win[0].rchar)
    wchar_delta = _delta_or_none(win[-1].wchar, win[0].wchar)
    read_bytes_delta = _delta_or_none(win[-1].read_bytes, win[0].read_bytes)
    write_bytes_delta = _delta_or_none(win[-1].write_bytes, win[0].write_bytes)
    return PhaseStats(
        role=role, phase=phase, n_samples=len(win), duration_s=duration_s,
        cpu_pct=cpu_pct,
        rss_kb_mean=float(rss.mean()), rss_kb_peak=float(rss.max()),
        rchar_delta=rchar_delta, wchar_delta=wchar_delta,
        read_bytes_delta=read_bytes_delta, write_bytes_delta=write_bytes_delta,
        rchar_rate_bps=(rchar_delta / duration_s if (duration_s > 0 and rchar_delta is not None) else None),
        read_bytes_rate_bps=(
            read_bytes_delta / duration_s if (duration_s > 0 and read_bytes_delta is not None) else None
        ),
        threads_peak=max(s.threads for s in win),
        fds_peak=max(s.fds for s in win),
    )


def run_phase_stats(run: RunData, role: str, phase: str):
    bounds = phase_bounds(run.meta, phase)
    if not bounds:
        return None
    series = run.role_series.get(role)
    if not series:
        return None
    return phase_stats(series, bounds[0], bounds[1], role, phase)


def _system_disk_read_mb_s(run: RunData, phase: str) -> Optional[float]:
    """Mean physical-disk read rate (decimal MB/s) over a phase window, from
    system.csv's `disk_read_kb` — CONTRACT.md: that column is itself a
    per-sample delta over the interval (from `/proc/diskstats`), not a
    cumulative counter, so the window's rate is total-bytes-in-window /
    window-duration, not an endpoint difference. Decimal KB assumed (1000 B)
    per the "throughput is decimal MB" convention."""
    bounds = phase_bounds(run.meta, phase)
    if not bounds:
        return None
    t0, t1 = bounds
    win = [s for s in run.system if t0 <= s.ts_ns <= t1]
    if len(win) < 2:
        return None
    duration_s = (win[-1].ts_ns - win[0].ts_ns) / 1e9
    if duration_s <= 0:
        return None
    total_kb = sum(s.disk_read_kb for s in win)
    return (total_kb * 1000.0 / 1e6) / duration_s


# ---------------------------------------------------------------------------
# Aggregation across reps
# ---------------------------------------------------------------------------

def agg_stat(values) -> Optional[dict]:
    """median/min/max/n over non-None values — never a bare mean, so a
    thermally-throttled outlier cannot silently drag the number (see
    module docstring). Use for anything that isn't IO-derived; IO-derived
    numbers use `agg_io_stat` instead so an all-missing column can never be
    confused with "computed a number"."""
    vals = [v for v in values if v is not None]
    if not vals:
        return None
    arr = np.array(vals, dtype=float)
    return {
        "median": float(np.median(arr)), "min": float(arr.min()),
        "max": float(arr.max()), "n": len(vals),
    }


def agg_io_stat(values) -> dict:
    """Aggregate an IO-derived field that may be structurally unmeasurable
    on a host. Returns `{"not_measurable": True, ...}` rather than ever
    computing a number from zero real samples — the numeric branch is only
    reachable past that check, and the assertion inside it is the self-check
    CONTRACT.md's empty-vs-zero rule asked for: it fails loudly if some
    future edit removes the guard above it."""
    present = [v for v in values if v is not None]
    n_total = len(values)
    if not present:
        return {"not_measurable": True, "n": 0, "n_total": n_total}
    assert len(present) > 0, (
        "agg_io_stat: reached the numeric branch with zero non-missing IO "
        "samples — this must be structurally impossible; see the "
        "empty-vs-zero rule in the module docstring."
    )
    arr = np.array(present, dtype=float)
    return {
        "not_measurable": False, "median": float(np.median(arr)),
        "min": float(arr.min()), "max": float(arr.max()),
        "n": len(present), "n_total": n_total,
    }


def config_key(run: RunData, include_comp_par: bool = True) -> tuple:
    m = run.meta
    key = (m.get("host"), m.get("scenario"), m.get("bitrate_target_mbs"))
    if include_comp_par:
        key += (m.get("clip_compression"), m.get("extract_parallelism"))
    return key


def group_runs(runs: list, include_comp_par: bool = True) -> dict:
    """Groups *complete* runs (throttled or not — see `split_throttled` for
    the further, per-analysis split that actually excludes throttled runs
    from a statistic)."""
    groups: dict = defaultdict(list)
    for r in runs:
        if not r.complete:
            continue
        groups[config_key(r, include_comp_par)].append(r)
    return groups


def split_throttled(group: list):
    """Split a config-group's complete runs into (usable, excluded).
    `excluded` holds both confirmed-throttled runs and runs whose throttle
    status is unknown (`throttled_state is None` — CONTRACT.md: null is not
    false) — neither represents a confirmed-clean operating point, so
    neither is averaged in. `throttled_note()` distinguishes the two kinds
    within `excluded` when labelling, rather than reporting an unmeasured
    status as if it were a confirmed one."""
    usable = [r for r in group if not r.throttled]
    excluded = [r for r in group if r.throttled]
    return usable, excluded


def throttled_note(usable: list, throttled: list) -> Optional[str]:
    """Non-None exactly when throttling (or an unknown throttle status)
    emptied the usable set for a group — see module docstring, "Throttled
    exclusion". Breaks the count down into confirmed-throttled vs
    unknown-status via `throttled_state`, since those are different claims
    (one says "this was measured and it was bad"; the other says "we don't
    know") and collapsing them into one undifferentiated number would hide
    that distinction the same way `bool(None)` would."""
    if not usable and throttled:
        confirmed = sum(1 for r in throttled if r.throttled_state is True)
        unknown = sum(1 for r in throttled if r.throttled_state is None)
        parts = []
        if confirmed:
            parts.append(f"{confirmed} confirmed throttled")
        if unknown:
            parts.append(f"{unknown} throttle status unknown")
        detail = f" ({', '.join(parts)})" if parts else ""
        return f"all {len(throttled)} run(s) in this group were excluded{detail}"
    return None


# ---------------------------------------------------------------------------
# 1. Overhead ratio
# ---------------------------------------------------------------------------

def analysis_overhead_ratio(runs: list) -> dict:
    """clipper's own cost, measured directly, framed against `baseline`
    (load + rosbag2, no clipper) at the same host + bitrate. `idle_tail` is
    the headline row (it, like baseline, does not vary over
    compression/parallelism, so it is the one row directly comparable to
    baseline without picking a canonical compression/parallelism value);
    `one_clip`/`ten_windows` are reported at full (host, scenario, bitrate,
    comp, parallelism) granularity beneath it since those axes are real
    variables for them.
    """
    complete = [r for r in runs if r.complete]
    baseline_by_hb: dict = defaultdict(list)
    for r in complete:
        if r.meta.get("scenario") == "baseline":
            baseline_by_hb[(r.meta.get("host"), r.meta.get("bitrate_target_mbs"))].append(r)

    # snapshot_sweep has its own dedicated analysis (#4, analysis_snapshot_sweep)
    # that already distinguishes the two arms sharing this scenario name; pooling
    # it in here too would silently mix a rosbag2-only arm and a clipper-tail arm
    # under one (host, scenario, bitrate, comp, parallelism) key.
    _EXCLUDED = ("baseline", "snapshot_sweep")
    rows = []
    groups = group_runs([r for r in complete if r.meta.get("scenario") not in _EXCLUDED])
    for key, group in groups.items():
        host, scenario, bitrate, comp, par = key
        baseline_group = baseline_by_hb.get((host, bitrate), [])
        usable, throttled = split_throttled(group)
        baseline_usable, baseline_throttled = split_throttled(baseline_group)

        clipper_ps = [p for p in (run_phase_stats(r, "clipper", "measure") for r in usable) if p]
        rosbag2_with_ps = [p for p in (run_phase_stats(r, "rosbag2", "measure") for r in usable) if p]
        rosbag2_base_ps = [p for p in (run_phase_stats(r, "rosbag2", "measure") for r in baseline_usable) if p]

        rows.append({
            "host": host, "scenario": scenario, "bitrate_target_mbs": bitrate,
            "bitrate_achieved_mbs": agg_stat([_bitrate_achieved_mbs(r) for r in usable]),
            "clip_compression": comp, "extract_parallelism": par,
            "run_ids": [r.run_id for r in usable],
            "throttled_run_ids": [r.run_id for r in throttled],
            "throttled_note": throttled_note(usable, throttled),
            "baseline_run_ids": [r.run_id for r in baseline_usable],
            "baseline_throttled_note": throttled_note(baseline_usable, baseline_throttled),
            "clipper_cpu_pct": agg_stat([p.cpu_pct for p in clipper_ps]),
            "clipper_rss_mib": agg_stat([p.rss_kb_mean / 1024 for p in clipper_ps]),
            "rosbag2_cpu_pct_with_clipper": agg_stat([p.cpu_pct for p in rosbag2_with_ps]),
            "rosbag2_cpu_pct_baseline": agg_stat([p.cpu_pct for p in rosbag2_base_ps]),
            "rosbag2_rss_mib_with_clipper": agg_stat([p.rss_kb_mean / 1024 for p in rosbag2_with_ps]),
            "rosbag2_rss_mib_baseline": agg_stat([p.rss_kb_mean / 1024 for p in rosbag2_base_ps]),
        })

    headline = sorted(
        (r for r in rows if r["scenario"] == "idle_tail"),
        key=lambda r: r["bitrate_target_mbs"] or 0,
    )
    return {"headline_idle_tail": headline, "all": rows}


# ---------------------------------------------------------------------------
# 2. Page-cache proof
# ---------------------------------------------------------------------------

def analysis_page_cache(runs: list) -> list:
    """Direct per-process rchar-vs-read_bytes evidence for clipper — only for
    runs where the kernel actually reports per-process IO (`_run_has_io(r)
    is True`; on this suite's two hosts that is the Nano only, per
    CONTRACT.md). The weaker, system-level fallback for hosts without it is
    `analysis_page_cache_system` — report.py must present the two as
    different strengths of evidence, never blend them."""
    complete = [
        r for r in runs
        if r.complete and r.meta.get("scenario") not in ("baseline", "snapshot_sweep")
        and _run_has_io(r) is True
    ]
    rows = []
    for key, group in group_runs(complete).items():
        host, scenario, bitrate, comp, par = key
        usable, throttled = split_throttled(group)
        note = throttled_note(usable, throttled)
        phase = "clipping" if scenario == "ten_windows" else "measure"
        stats = [p for p in (run_phase_stats(r, "clipper", phase) for r in usable) if p]
        if not stats and note is None:
            continue
        ratios = [
            (s.read_bytes_delta / s.rchar_delta)
            if (s.rchar_delta not in (None, 0) and s.read_bytes_delta is not None)
            else None
            for s in stats
        ]
        rows.append({
            "host": host, "scenario": scenario, "bitrate_target_mbs": bitrate,
            "bitrate_achieved_mbs": agg_stat([_bitrate_achieved_mbs(r) for r in usable]),
            "clip_compression": comp, "extract_parallelism": par, "phase": phase,
            "run_ids": [r.run_id for r in usable],
            "throttled_run_ids": [r.run_id for r in throttled],
            "throttled_note": note,
            "rchar_rate_bps": agg_io_stat([s.rchar_rate_bps for s in stats]),
            "read_bytes_rate_bps": agg_io_stat([s.read_bytes_rate_bps for s in stats]),
            "read_bytes_over_rchar_ratio": agg_io_stat(ratios),
        })
    return rows


def analysis_page_cache_system(runs: list) -> list:
    """System-level (weaker) evidence for the page-cache claim, for hosts
    where per-process IO could not be measured. Diffs system.csv's
    `disk_read_kb` (from `/proc/diskstats`, unaffected by the per-process
    gap) between a with-clipper arm and `baseline` at the same host and
    target bitrate: if aggregate physical reads do not rise, that supports
    "clipper adds no read IO", but it can only show that at the whole-system
    level — it cannot attribute it to clipper's own reads being page-cache
    hits specifically, the way `analysis_page_cache`'s direct per-process
    number can. report.py must label this section as weaker evidence and
    never present it as the same kind of number as the per-process one.
    `within_noise` is true when the with-clipper and baseline rate ranges
    (min-max across usable reps) overlap."""
    complete = [r for r in runs if r.complete]
    no_direct_io = [r for r in complete if _run_has_io(r) is not True]
    rows = []
    by_host: dict = defaultdict(list)
    for r in no_direct_io:
        by_host[r.meta.get("host")].append(r)

    for host, host_runs in by_host.items():
        usable, throttled = split_throttled(host_runs)
        by_bs: dict = defaultdict(list)
        for r in usable:
            by_bs[(r.meta.get("bitrate_target_mbs"), r.meta.get("scenario"))].append(r)
        bitrates = sorted({b for (b, s) in by_bs if s == "baseline"})
        for bitrate in bitrates:
            baseline_runs = by_bs.get((bitrate, "baseline"), [])
            baseline_rates = [
                x for r in baseline_runs
                for x in [_system_disk_read_mb_s(r, "measure")] if x is not None
            ]
            if not baseline_rates:
                continue
            base_agg = agg_stat(baseline_rates)
            for scenario in ("idle_tail", "one_clip", "ten_windows"):
                clip_runs = by_bs.get((bitrate, scenario), [])
                if not clip_runs:
                    continue
                phase = "clipping" if scenario == "ten_windows" else "measure"
                clip_rates = [
                    x for r in clip_runs
                    for x in [_system_disk_read_mb_s(r, phase)] if x is not None
                ]
                if not clip_rates:
                    continue
                clip_agg = agg_stat(clip_rates)
                overlap = not (clip_agg["min"] > base_agg["max"] or clip_agg["max"] < base_agg["min"])
                rows.append({
                    "host": host, "scenario": scenario, "bitrate_target_mbs": bitrate,
                    "bitrate_achieved_mbs": agg_stat(
                        [_bitrate_achieved_mbs(r) for r in clip_runs]
                    ),
                    "phase": phase,
                    "evidence": "system-level (weaker than the Nano's direct per-process measurement)",
                    "baseline_disk_read_mb_s": base_agg,
                    "with_clipper_disk_read_mb_s": clip_agg,
                    "delta_mb_s": clip_agg["median"] - base_agg["median"],
                    "within_noise": overlap,
                    "run_ids": [r.run_id for r in clip_runs],
                    "baseline_run_ids": [r.run_id for r in baseline_runs],
                })
    return rows


# ---------------------------------------------------------------------------
# 3. The two phases of ten_windows
# ---------------------------------------------------------------------------

def analysis_ten_windows_phases(runs: list) -> list:
    """Waiting vs clipping cost, read strictly from `run.json["phases"]`'s
    `waiting`/`clipping` bounds — **never synthesised here**. A real
    measurement proved why: computing "waiting" as
    [last trigger sent -> first Recorded] gave 27.01% mean CPU, but the
    correct boundary (last trigger sent -> first anchor + postroll) gave
    0.55% — identical to the same host's `idle_tail` baseline. With a 1 s
    stagger and 60 s postroll, trigger 0 becomes copyable at
    sent[0]+60s, *before* trigger 9's own postroll elapses at sent[9]+60s,
    so the naive interval already contains ~9 s of real copying. A ~50x
    error that is not implausible on its face — exactly the fabricated-
    result class this suite keeps hitting, just arrived at by inference
    instead of a missing/null field. The fix is structural, not a smarter
    heuristic: this function only ever calls `run_phase_stats(r, role,
    "waiting")`, which reads `run.json["phases"]["waiting"]` through
    `phase_bounds()` and returns `None` if absent — there is no code path
    here that derives a boundary from trigger timestamps. When `run.json`
    provides no `waiting` bounds at all (e.g. stagger x count >= postroll
    leaves no pure-waiting interval), `waiting_phase_absent_note` says so
    explicitly rather than the row silently reading "n/a" for a reason a
    reader can't tell apart from "no runs" or "all excluded"."""
    complete = [r for r in runs if r.complete and r.meta.get("scenario") == "ten_windows"]
    rows = []
    for key, group in group_runs(complete).items():
        host, scenario, bitrate, comp, par = key
        usable, throttled = split_throttled(group)
        note = throttled_note(usable, throttled)
        waiting_bounds_present = [r for r in usable if phase_bounds(r.meta, "waiting") is not None]
        waiting_phase_absent_note = None
        if usable and not waiting_bounds_present:
            # CONTRACT.md (settled): "phase_notes" carries the producer's
            # own explanation for a deliberately-absent phase — prefer that
            # verbatim reason over this module's generic guess.
            reasons = {
                r.meta.get("phase_notes", {}).get("waiting")
                for r in usable if (r.meta.get("phase_notes") or {}).get("waiting")
            }
            waiting_phase_absent_note = "; ".join(sorted(reasons)) if reasons else (
                "run.json provided no 'waiting' phase bounds for any usable rep in this group "
                "(most likely stagger x trigger_count >= postroll, leaving no pure-waiting "
                "interval) — not computed, never synthesised from trigger timestamps"
            )
        waiting = [p for p in (run_phase_stats(r, "clipper", "waiting") for r in usable) if p]
        clipping = [p for p in (run_phase_stats(r, "clipper", "clipping") for r in usable) if p]
        rows.append({
            "host": host, "bitrate_target_mbs": bitrate,
            "bitrate_achieved_mbs": agg_stat([_bitrate_achieved_mbs(r) for r in usable]),
            "clip_compression": comp, "extract_parallelism": par,
            "run_ids": [r.run_id for r in usable],
            "throttled_run_ids": [r.run_id for r in throttled],
            "throttled_note": note,
            "waiting_phase_absent_note": waiting_phase_absent_note,
            "waiting_cpu_pct": agg_stat([p.cpu_pct for p in waiting]),
            "waiting_rss_mib": agg_stat([p.rss_kb_mean / 1024 for p in waiting]),
            "clipping_cpu_pct": agg_stat([p.cpu_pct for p in clipping]),
            "clipping_rss_mib": agg_stat([p.rss_kb_mean / 1024 for p in clipping]),
        })
    return rows


# ---------------------------------------------------------------------------
# 4. clipper vs rosbag2 snapshot mode (the centrepiece)
# ---------------------------------------------------------------------------

_PREROLL_VARIANT_RE = re.compile(r"pre(\d+)")


def _snapshot_arm(run: RunData) -> str:
    """Which arm of `snapshot_sweep` this run is. CONTRACT.md/real run.json:
    top-level `"snapshot_arm"` is confirmed present (`null` outside
    `snapshot_sweep`) and `"variant"` is confirmed present too (empty
    string outside `snapshot_sweep`; team-lead report of live data names
    the two arms' variants `"snap-preN"` and `"cont-preN"` — "cont" reads
    as "continuous", i.e. the clipper-tail arm reading a continuously-
    recorded bag, though this exact vocabulary is still not independently
    verified against a directly-inspected snapshot_sweep run.json). Both
    fields are checked, in case `snapshot_arm`'s own value for the tail arm
    turns out not to contain "tail"/"clip" — `variant`'s prefix is a second,
    independent signal for the same distinction. Falls back to
    `recorder_profile` (this module's original, now-tertiary discriminator,
    kept for make_fixtures.py compatibility) only if neither field matches."""
    arm = (run.meta.get("snapshot_arm") or "").strip().lower()
    variant = (run.meta.get("variant") or "").strip().lower()
    if "snap" in arm or variant.startswith("snap"):
        return "snapshot_mode"
    if "tail" in arm or "clip" in arm or variant.startswith("cont") or variant.startswith("tail"):
        return "clipper_tail"
    profile = run.meta.get("recorder_profile")
    if profile == "fastwrite":
        return "clipper_tail"
    if profile == "snapshot":
        return "snapshot_mode"
    return "unknown"


def _snapshot_preroll_s(run: RunData) -> Optional[float]:
    """Prefer run.json's `"variant"` field (e.g. `"snap-pre300"` -> 300s —
    confirmed a real top-level field, though this exact naming pattern is
    from the team lead's report rather than a directly-inspected
    snapshot_sweep run.json); falls back to triggers.jsonl's own
    contractually-guaranteed `preroll_ns` when variant doesn't parse."""
    m = _PREROLL_VARIANT_RE.search(run.meta.get("variant") or "")
    if m:
        return float(m.group(1))
    if run.triggers:
        preroll_ns = run.triggers[0].get("preroll_ns")
        if preroll_ns is not None:
            return preroll_ns / 1e9
    return None


def _bitrate_achieved_mbs(run: RunData, *, is_snapshot_mode_arm: bool = False) -> Optional[float]:
    """The one place `bitrate_achieved_mbs` is read from — every other
    analysis calls this instead of `run.meta.get("bitrate_achieved_mbs")`
    directly.

    CONTRACT.md (settled, per suite-dev): the field is *meant* to be the
    LOAD rate uniformly on every arm, including snapshot mode (there,
    computed from the dumped mcap's own Statistics-record span rather than
    a naive bytes-written/elapsed, which would measure the dump, not the
    ingest — cross-checked to within 0.1% of the independently-measured
    load on real data), discriminated by `bitrate_achieved_source`.

    **But the deployed harness collecting this suite's actual data does
    NOT emit `bitrate_achieved_source` at all** (confirmed directly against
    a live run.json by the team lead — the repo's run_suite.sh has drifted
    ahead of what is running). For every *other* scenario this is harmless:
    dump-vs-ingest was never a problem there, so a bare
    `bitrate_achieved_mbs` with no source field has always meant "older
    run.json, trust the raw value" — and still does. For the snapshot-mode
    arm specifically it is the opposite: absence of the source field is NOT
    license to trust the raw number, because that number is a known
    dump-rate artefact absent an explicit confirming source (real data:
    2.302 MB/s "achieved" against a 20 MB/s load). `is_snapshot_mode_arm`
    is how callers say "this row is that arm" — `analysis_snapshot_sweep`
    is the only caller that ever passes it `True`, identifying the arm via
    `_snapshot_arm()` (the `snapshot_arm`/`variant` fields, both confirmed
    present in live data) rather than via this now-absent field. Until a
    live run carries `bitrate_achieved_source ==
    "snapshot_cache_dump_over_buffered_span"`, every snapshot-mode-arm row
    reads as not-measured rather than risk a repeat of a 20 MB/s experiment
    charted as 2.3 MB/s."""
    source = run.meta.get("bitrate_achieved_source")
    if isinstance(source, str) and source.startswith("not measurable"):
        return None
    if is_snapshot_mode_arm and source != "snapshot_cache_dump_over_buffered_span":
        return None
    return run.meta.get("bitrate_achieved_mbs")


def _peak_combined_rss_mib(run: RunData, roles: list) -> Optional[float]:
    """Peak of the SUM of these roles' RSS at each shared sample tick — the
    total memory footprint of a *capability*, not one process in isolation.

    Real measurement that motivated this: baseline recorder alone peaked at
    86.5 MB; clipper's tail arm was recorder 87.3 MB + clipper 24.9 MB =
    ~112 MB combined; the snapshot-mode arm (a single process, no separate
    clipper) peaked at 389.7 MB. Comparing clipper's 24.9 MB alone against
    snapshot's 389.7 MB would flatter clipper by omitting the recorder it
    depends on — the honest comparison is total memory for the capability
    (recorder+clipper) vs total memory for the alternative (recorder alone,
    in snapshot mode). Uses the intersection of `ts_ns` present in every
    listed role's series — a tick missing from one role can't be summed."""
    per_role = [run.role_series.get(role) for role in roles]
    if any(not series for series in per_role):
        return None
    by_ts = [{s.ts_ns: s.rss_kb for s in series} for series in per_role]
    common_ts = set(by_ts[0])
    for d in by_ts[1:]:
        common_ts &= set(d)
    if not common_ts:
        return None
    peak_kb = max(sum(d[ts] for d in by_ts) for ts in common_ts)
    return peak_kb / 1024


def analysis_snapshot_sweep(runs: list) -> dict:
    """See `_snapshot_arm`/`_snapshot_preroll_s` for how the arm and preroll
    are read. Grouped by **(host, arm, preroll)** — nano (7.5 GB RAM) and nx
    (15.6 GB) have materially different OOM ceilings, so pooling both hosts
    into one preroll bucket would blend a real architectural axis into the
    flagship chart; each row below carries its own `host` so report.py can
    plot the two hosts as separate series.

    `peak_rss_mib` is the **total memory for the capability**, not one
    process: the clipper-tail arm sums clipper + the rosbag2 recorder it
    depends on (see `_peak_combined_rss_mib`); the snapshot-mode arm is
    already a single process (rosbag2 alone, no separate clipper), so its
    own RSS already is the total.

    `bitrate_target_mbs` is carried alongside `bitrate_achieved_mbs`.
    Achieved briefly could NOT be trusted here: in snapshot mode the
    recorder buffers in RAM and only writes when the snapshot service
    fires, so a naive bytes-written-to-record-dir / elapsed measures the
    *dump*, not the *ingest* rate (real data: 2.302 MB/s on a 20 MB/s
    load). suite-dev has since normalised `bitrate_achieved_mbs` itself to
    always mean the load rate on every arm — for the snapshot arm it's
    computed from the dumped mcap's own Statistics-record span, cross-
    checked to within 0.1% of the independently-measured load — so it's
    safe to read uniformly again, via `_bitrate_achieved_mbs()`, which
    still checks `bitrate_achieved_source` and treats any
    `"not measurable: ..."` value (e.g. an OOM-killed cache with nothing to
    dump) as unmeasured rather than trusting a stray number."""
    snap_runs = [r for r in runs if r.meta.get("scenario") == "snapshot_sweep"]
    by_host_arm_preroll: dict = defaultdict(list)
    for r in snap_runs:
        arm = _snapshot_arm(r)
        preroll_s = _snapshot_preroll_s(r)
        if arm == "clipper_tail":
            peak_mib = _peak_combined_rss_mib(r, ["clipper", "rosbag2"])
        else:
            series = r.role_series.get("rosbag2")
            peak_mib = (max(s.rss_kb for s in series) / 1024) if series else None
        by_host_arm_preroll[(r.meta.get("host"), arm, preroll_s)].append((r, peak_mib))

    result: dict = defaultdict(list)
    for (host, arm, preroll_s), items in by_host_arm_preroll.items():
        complete_items = [(r, p) for r, p in items if r.complete and p is not None]
        usable_items = [(r, p) for r, p in complete_items if not r.throttled]
        throttled_runs = [r for r, _ in complete_items if r.throttled]
        usable_runs = [r for r, _ in usable_items]
        incomplete_items = [(r, p) for r, p in items if not r.complete]
        # CONTRACT.md (settled): "oom_killed" is a bool and the *only*
        # signal for this — no more "failure_reason" string to match.
        oom = any(r.meta.get("oom_killed") is True for r, _ in incomplete_items)
        bitrate_target = items[0][0].meta.get("bitrate_target_mbs") if items else None
        result[arm].append({
            "host": host, "preroll_s": preroll_s, "bitrate_target_mbs": bitrate_target,
            "bitrate_achieved_mbs": agg_stat([
                _bitrate_achieved_mbs(r, is_snapshot_mode_arm=(arm == "snapshot_mode"))
                for r in usable_runs
            ]),
            "peak_rss_mib": agg_stat([p for _, p in usable_items]),
            # Partial peak reached before an incomplete (e.g. OOM-killed) run
            # died — this is what a chart should plot at that x, since the
            # run's own samples still show how far RSS got before the kill.
            "incomplete_peak_rss_mib": agg_stat([p for _, p in incomplete_items if p is not None]),
            "complete_run_ids": [r.run_id for r in usable_runs],
            "throttled_run_ids": [r.run_id for r in throttled_runs],
            "throttled_note": throttled_note(usable_runs, throttled_runs),
            "incomplete_run_ids": [r.run_id for r, _ in incomplete_items],
            "oom": oom,
        })
    for arm in result:
        result[arm].sort(key=lambda row: (row["host"] or "", row["preroll_s"] is None, row["preroll_s"]))
    return dict(result)


# ---------------------------------------------------------------------------
# 5. Clip latency distribution
# ---------------------------------------------------------------------------

def analysis_clip_latency(runs: list) -> dict:
    """Trigger-latency distribution, primarily by full config — (host,
    scenario, bitrate, compression, extract_parallelism) — plus a secondary
    rollup pooled by `extract_parallelism` alone across every scenario.

    The pooled view is what the harness brief literally asked for ("per
    extract_parallelism (1/2/ncores)"), but pooling raw is misleading on its
    own: it blends a fast `one_clip`/`none` rep with a slow
    `ten_windows`/`zstd`/`heavy` rep into one par=1 bucket, so a wide
    min-max spread there could look like noise when it's actually just
    heterogeneous configs. `by_config` is the honest breakdown underneath
    it; `by_parallelism_pooled` stays for the parallelism-only headline but
    is clearly labelled as a pooled rollup, not a controlled comparison.

    Excludes `snapshot_sweep` entirely — its rosbag2 `--snapshot-mode` arm's
    "trigger" latency is a service call on rosbag2 itself, not a clipper
    extraction, sharing no mechanism with `extract_parallelism`; its
    clipper-tail arm deliberately sweeps preroll 5-600s for the section-4
    chart, 10-100x wider than any other scenario, which would swamp this
    "typical latency" distribution. See section 4 for the sweep itself.
    Throttled runs are excluded (module docstring, "Throttled exclusion")."""
    by_config: dict = defaultdict(
        lambda: {"latencies": [], "run_ids": set(), "throttled_run_ids": set()}
    )
    for r in runs:
        if not r.complete or r.meta.get("scenario") == "snapshot_sweep":
            continue
        key = (
            r.meta.get("host"), r.meta.get("scenario"), r.meta.get("bitrate_target_mbs"),
            r.meta.get("clip_compression"), r.meta.get("extract_parallelism"),
        )
        if r.throttled:
            by_config[key]["throttled_run_ids"].add(r.run_id)
            continue
        for t in r.triggers:
            lat = t.get("latency_ns")
            if lat is not None:
                by_config[key]["latencies"].append(lat / 1e6)  # ms
                by_config[key]["run_ids"].add(r.run_id)

    def _row(latencies, run_ids, throttled_run_ids, extra: dict, noun: str):
        note = None
        if not latencies and throttled_run_ids:
            note = f"all {len(throttled_run_ids)} run(s) {noun} were excluded as throttled"
        return {
            **extra,
            "latency_ms": agg_stat(latencies),
            "n_triggers": len(latencies),
            "run_ids": sorted(run_ids),
            "throttled_run_ids": sorted(throttled_run_ids),
            "throttled_note": note,
        }

    by_config_rows = []
    by_par: dict = defaultdict(lambda: {"latencies": [], "run_ids": set(), "throttled_run_ids": set()})
    for key, d in by_config.items():
        host, scenario, bitrate, comp, par = key
        by_config_rows.append(_row(
            d["latencies"], d["run_ids"], d["throttled_run_ids"],
            {"host": host, "scenario": scenario, "bitrate_target_mbs": bitrate,
             "clip_compression": comp, "extract_parallelism": par},
            "in this config",
        ))
        by_par[par]["latencies"].extend(d["latencies"])
        by_par[par]["run_ids"] |= d["run_ids"]
        by_par[par]["throttled_run_ids"] |= d["throttled_run_ids"]
    by_config_rows.sort(key=lambda r: (
        r["host"] or "", r["scenario"] or "", r["bitrate_target_mbs"] or 0,
        r["clip_compression"] or "", str(r["extract_parallelism"]),
    ))

    pooled_rows = [
        _row(d["latencies"], d["run_ids"], d["throttled_run_ids"], {"extract_parallelism": par},
             "at this parallelism")
        for par, d in sorted(by_par.items(), key=lambda kv: (kv[0] is None, str(kv[0])))
    ]

    return {"by_config": by_config_rows, "by_parallelism_pooled": pooled_rows}


# ---------------------------------------------------------------------------
# 6. Recorder integrity
# ---------------------------------------------------------------------------

def _rosbag2_dropped_value(run: RunData) -> Optional[int]:
    """Standing rule (CONTRACT.md, per suite-dev, closing a hole a bare
    retraction wouldn't): `rosbag2_dropped` is usable ONLY when
    `rosbag2_dropped_source` is PRESENT and reads `"reported"` or
    `"clean_absent"` — never when the source field is absent, regardless
    of what `rosbag2_dropped` itself contains.

    This is deliberately the opposite fallback direction from
    `_bitrate_achieved_mbs`: there, an absent source has always meant "an
    older run predating the field, trust the raw value" because dump-vs-
    ingest was never a problem for non-snapshot arms. Here, an absent
    source means something categorically different: archived pre-freeze
    run.json files carry a same-shaped bare `rosbag2_dropped` number that
    is NOT a real measurement — it's `max(0, expected - recorded)` from
    replay-rate x span arithmetic, an estimator residual dominated by
    several-percent error per term. A retracted real example: a "baseline
    54 vs clipper 66 drops" comparison from exactly such pre-freeze data
    was reported as ~0.1% transport-layer loss; the 12-message difference
    was well inside the estimator's own noise, and both figures were
    withdrawn. Globbing old run directories must not silently pull that
    number back into an aggregate long after the claim built on it was
    retracted — so absence of the source field discards the value outright
    here, rather than falling back to trusting it."""
    source = run.meta.get("rosbag2_dropped_source")
    if source in ("reported", "clean_absent"):
        return run.meta.get("rosbag2_dropped")
    return None  # absent source, or "not_measured": always discard


def analysis_recorder_integrity(runs: list) -> dict:
    """rosbag2's dropped-message count, with and without clipper. Three
    states, kept explicitly distinct — this was a real bug, not a
    hypothetical one: `rosbag2_dropped` is a key that is *present* with
    JSON `null` when nothing was captured (rosbag2 only emits a drop line
    when drops occur), so `.get("rosbag2_dropped", 0)` returns `None` (the
    default only applies when the key is *absent*), and averaging that in
    as if it were a real measurement would render the headline
    "rosbag2 dropped zero messages" from data that was never captured at
    all. `measured_state` is `"measured"` (>=1 real number seen — check
    `any_nonzero`), `"not_measured"` (every usable run's value was `null`),
    or `"no_data"` (no usable runs at all); `any_nonzero` is only ever
    `True`/`False` in the `"measured"` state, `None` otherwise — report.py
    must never render the `None` case as if it were `False`."""
    complete = [r for r in runs if r.complete]
    usable, throttled = split_throttled(complete)

    def _bucket():
        return {"dropped": [], "not_measured_run_ids": [], "run_ids": []}

    by_presence: dict = defaultdict(_bucket)
    per_scenario: dict = defaultdict(_bucket)
    for r in usable:
        raw = _rosbag2_dropped_value(r)  # None = not measured, never 0
        presence = "baseline_no_clipper" if r.meta.get("scenario") == "baseline" else "with_clipper"
        key = f'{r.meta.get("host")}/{r.meta.get("scenario")}'
        for d, k in ((by_presence, presence), (per_scenario, key)):
            d[k]["run_ids"].append(r.run_id)
            if raw is None:
                d[k]["not_measured_run_ids"].append(r.run_id)
            else:
                d[k]["dropped"].append(raw)

    throttled_by_scenario: dict = defaultdict(list)
    for r in throttled:
        throttled_by_scenario[f'{r.meta.get("host")}/{r.meta.get("scenario")}'].append(r.run_id)

    def _summarize_bucket(v, t_ids):
        stat = agg_stat(v["dropped"])
        entry = {**stat, "run_ids": v["run_ids"]} if stat else {"run_ids": v["run_ids"]}
        entry["not_measured_run_ids"] = v["not_measured_run_ids"]
        entry["throttled_run_ids"] = t_ids
        entry["throttled_note"] = (
            f"all {len(t_ids)} run(s) in this group were excluded as throttled"
            if not v["dropped"] and not v["not_measured_run_ids"] and t_ids else None
        )
        return entry

    by_presence_out = {k: _summarize_bucket(v, []) for k, v in by_presence.items()}
    per_scenario_out = {
        key: _summarize_bucket(
            per_scenario.get(key, _bucket()), throttled_by_scenario.get(key, [])
        )
        for key in set(per_scenario) | set(throttled_by_scenario)
    }

    all_measured = [d for v in by_presence.values() for d in v["dropped"]]
    all_not_measured = [rid for v in by_presence.values() for rid in v["not_measured_run_ids"]]
    if all_measured:
        measured_state, any_nonzero = "measured", any(d for d in all_measured)
    elif all_not_measured:
        measured_state, any_nonzero = "not_measured", None
    else:
        measured_state, any_nonzero = "no_data", None

    return {
        "by_presence": by_presence_out,
        "any_nonzero": any_nonzero,
        "measured_state": measured_state,
        "per_scenario": per_scenario_out,
        "n_runs": len(usable),
        "n_measured": len(all_measured),
        "n_not_measured": len(all_not_measured),
        "throttled_run_ids": [r.run_id for r in throttled],
    }


# ---------------------------------------------------------------------------
# 7. Soak drift
# ---------------------------------------------------------------------------

def _slope_report(xs, ys, unit):
    if len(xs) < 3:
        return None
    slope, intercept = np.polyfit(xs, ys, 1)
    resid = np.array(ys) - (slope * np.array(xs) + intercept)
    resid_std = float(np.std(resid))
    duration_h = xs[-1] - xs[0]
    predicted_change = float(slope * duration_h)
    within_noise = (
        abs(predicted_change) < 2 * resid_std if resid_std > 0 else abs(predicted_change) < 1e-9
    )
    return {
        "slope_per_hour": float(slope), "unit": unit,
        "predicted_change_over_run": predicted_change,
        "residual_std": resid_std, "within_noise": bool(within_noise),
        "n_buckets": len(xs),
    }


def analysis_soak_drift(runs: list, bucket_s: float = 300.0) -> dict:
    """RSS/fd/latency drift over a soak run's `measure` phase, bucketed to
    `bucket_s`-wide windows and fit with a linear slope. A run is flagged
    "within noise" if the fit's predicted change over the whole run is under
    2x the residual scatter — soak is inherently n=1 (no reps to take a
    spread across), so this is the substitute noise floor for that rule.
    Throttled soak runs are excluded outright (soak isn't aggregated across
    reps, so there's no "usable" set to fall back to — a throttled soak run
    is simply not reported, and listed in `throttled_run_ids` instead)."""
    all_soak = [r for r in runs if r.complete and r.meta.get("scenario") == "soak"]
    soak_runs, throttled_soak = split_throttled(all_soak)
    results = []
    for r in soak_runs:
        bounds = phase_bounds(r.meta, "measure")
        if not bounds:
            continue
        t0, t1 = bounds
        series = r.role_series.get("clipper", [])
        buckets: dict = defaultdict(list)
        for s in series:
            if t0 <= s.ts_ns <= t1:
                buckets[int((s.ts_ns - t0) / (bucket_s * 1e9))].append(s)

        elapsed_h, rss_mib, fds = [], [], []
        for idx in sorted(buckets):
            grp = buckets[idx]
            elapsed_h.append(((idx + 0.5) * bucket_s) / 3600.0)
            rss_mib.append(float(np.median([g.rss_kb for g in grp])) / 1024)
            fds.append(float(np.median([g.fds for g in grp])))

        lat_by_bucket: dict = defaultdict(list)
        for t in r.triggers:
            if t.get("latency_ns") is None or t.get("sent_ns") is None:
                continue
            if not (t0 <= t["sent_ns"] <= t1):
                continue
            lat_by_bucket[int((t["sent_ns"] - t0) / (bucket_s * 1e9))].append(t["latency_ns"] / 1e6)
        lat_h, lat_ms = [], []
        for idx in sorted(lat_by_bucket):
            lat_h.append(((idx + 0.5) * bucket_s) / 3600.0)
            lat_ms.append(float(np.median(lat_by_bucket[idx])))

        results.append({
            "run_id": r.run_id, "host": r.meta.get("host"),
            "rss_mib_drift": _slope_report(elapsed_h, rss_mib, "MiB"),
            "fds_drift": _slope_report(elapsed_h, fds, "fds"),
            "latency_ms_drift": _slope_report(lat_h, lat_ms, "ms"),
            "rss_mib_series": list(zip(elapsed_h, rss_mib)),
            "fds_series": list(zip(elapsed_h, fds)),
            "latency_ms_series": list(zip(lat_h, lat_ms)),
        })
    return {
        "runs": results,
        "throttled_run_ids": [r.run_id for r in throttled_soak],
        "throttled_note": (
            f"all {len(throttled_soak)} soak run(s) were excluded as throttled"
            if not soak_runs and throttled_soak else None
        ),
    }


class TegrastatsAlignmentError(Exception):
    """Raised when a run's tegrastats log, after epoch alignment, does not
    overlap its own `measure` phase — see `_load_aligned_tegrastats`."""


_FIFTEEN_MIN_NS = 900 * 1_000_000_000


def _tegrastats_offset_candidates(run: RunData, first_naive_ts_ns: int) -> list:
    """Candidate nanosecond offsets to add to tegrastats' naive "as-if-UTC"
    timestamps to get real epoch ns, in priority order.

    This returns *candidates*, plural, rather than committing to one,
    because `host_utc_offset_s`'s sign convention is not independently
    confirmed against real data — only a single number (`-14400` for NX)
    has been relayed secondhand. If it follows the common
    `datetime.utcoffset()`-style convention (local = UTC + offset, so EDT
    is `-14400`), the correction this module needs to ADD to a naive
    "wall-clock read as UTC" value to recover real UTC is the *negation*,
    `+14400` — using the field's raw value directly would apply a 4-hour
    correction in the wrong direction (compounding with the naive value's
    own 4-hour error into an 8-hour miss) rather than fixing it. Betting on
    either sign convention and being wrong would be exactly the silent
    fabricated-energy failure mode this suite keeps hitting, just moved one
    level up (an unconfirmed field name doesn't reduce risk, it hides it).

    So: try the field's value AND its negation, then `clock_anchor` (shape
    unconfirmed, matched defensively), then the offset this module derives
    and has independently verified against real data (anchoring the log's
    first sample to the run's own start, rounded to the nearest 15
    minutes). `_load_aligned_tegrastats` tries each in order and keeps the
    first one that actually makes the tegrastats window overlap the run's
    `measure` phase — a real physical constraint (the sampler starts
    tegrastats at run start) that settles the sign question empirically
    per run instead of trusting an undocumented convention."""
    candidates = []
    offset_s = run.meta.get("host_utc_offset_s")
    if isinstance(offset_s, (int, float)):
        candidates.append(int(offset_s * 1e9))
        candidates.append(int(-offset_s * 1e9))

    anchor = run.meta.get("clock_anchor")
    if isinstance(anchor, dict):
        epoch_ns = anchor.get("epoch_ns")
        naive_ns = anchor.get("tegrastats_ts_ns", anchor.get("naive_ts_ns"))
        if isinstance(epoch_ns, (int, float)) and isinstance(naive_ns, (int, float)):
            candidates.append(int(epoch_ns) - int(naive_ns))

    phases = run.meta.get("phases") or {}
    all_bounds = [b for bound in phases.values() for b in bound]
    if all_bounds:
        run_t0 = min(all_bounds)
        raw_offset = run_t0 - first_naive_ts_ns
        candidates.append(round(raw_offset / _FIFTEEN_MIN_NS) * _FIFTEEN_MIN_NS)

    return candidates


def _load_aligned_tegrastats(run: RunData) -> list:
    """Parse `run`'s tegrastats.log and align its naive "as-if-UTC"
    timestamps (see `tegraparse.py`'s module docstring) to real epoch ns —
    see `_tegrastats_offset_candidates` for the candidate offsets and why
    there's more than one. tegrastats carries no epoch and no timezone; the
    two real hosts are 4 hours apart (Nano UTC, NX EDT — confirmed against
    real data, see `tegraparse.py`).

    Tries each candidate offset in order and keeps the first that makes the
    aligned series overlap the run's `measure` phase; raises
    `TegrastatsAlignmentError` if none do — energy computed from a
    non-overlapping window is exactly the fabricated-result class this
    suite keeps hitting, so a misalignment must be loud, not silent.
    Callers catch this per-run (one bad log must not lose every other run's
    energy figures) and are expected to surface it, not swallow it."""
    samples = tegraparse.parse_log(run.dir / "tegrastats.log")
    if not samples:
        return samples
    bounds = phase_bounds(run.meta, "measure")
    candidates = _tegrastats_offset_candidates(run, samples[0].ts_ns) or [0]

    chosen = None
    for offset in candidates:
        if bounds is None:
            chosen = offset
            break
        t0, t1 = bounds
        if samples[0].ts_ns + offset <= t1 and samples[-1].ts_ns + offset >= t0:
            chosen = offset
            break

    if chosen is None:
        t0, t1 = bounds
        tried = ", ".join(f"{o / 1e9:.0f}s" for o in candidates)
        raise TegrastatsAlignmentError(
            f"{run.run_id}: no candidate tegrastats offset ({tried}) makes the log "
            f"overlap measure phase [{t0},{t1}] — refusing to compute energy from a "
            "misaligned window"
        )
    return [dataclasses.replace(s, ts_ns=s.ts_ns + chosen) for s in samples]


# ---------------------------------------------------------------------------
# 8. Energy
# ---------------------------------------------------------------------------

def analysis_energy(runs: list) -> dict:
    """mJ per clip (marginal over idle-tail draw) and idle-tail watts, from
    tegrastats' VDD_IN-aliased rail.

    The idle-tail baseline is keyed by **(host, bitrate)**, not host alone —
    idle-tail power is not bitrate-independent (more bytes/s through the
    recorder and clipper's tail is itself a real cost), so subtracting a
    baseline pooled across bitrates would misattribute part of a busier
    bitrate's own idle cost as "marginal clip energy" at a quieter one.
    Throttled runs are excluded from both the idle-tail baseline and the
    clip-energy computation (see module docstring, "Throttled exclusion");
    the two `..._all_throttled_note` fields fire only in the (unlikely, but
    per the harness's current throttle-detector bug, not impossible) case
    where every contributing run was excluded that way."""
    all_idle = [r for r in runs if r.complete and r.meta.get("scenario") == "idle_tail"]
    idle_usable, idle_throttled = split_throttled(all_idle)

    alignment_failures: list = []

    def _tegrastats_or_none(r: RunData):
        tegra_path = r.dir / "tegrastats.log"
        if not tegra_path.exists():
            return None
        try:
            return _load_aligned_tegrastats(r)
        except TegrastatsAlignmentError as exc:
            print(f"analysis_energy: {exc}", file=sys.stderr)
            alignment_failures.append({"run_id": r.run_id, "reason": str(exc)})
            return None

    idle_watts_by_hb: dict = defaultdict(list)
    for r in idle_usable:
        samples = _tegrastats_or_none(r)
        if samples is None:
            continue
        bounds = phase_bounds(r.meta, "measure")
        if not bounds:
            continue
        t0, t1 = bounds
        watts = [
            s.vdd_in_mw / 1000 for s in samples
            if s.vdd_in_mw is not None and t0 <= s.ts_ns <= t1
        ]
        if watts:
            hb_key = (r.meta.get("host"), r.meta.get("bitrate_target_mbs"))
            idle_watts_by_hb[hb_key].append((float(np.median(watts)), r.run_id))

    idle_watts_median = {
        hb: float(np.median([w for w, _ in vals])) for hb, vals in idle_watts_by_hb.items() if vals
    }
    # Reported per-host too (folding bitrates together) purely as a headline
    # stat card — clip-energy subtraction above always uses the per-bitrate
    # value, never this pooled one.
    idle_watts_by_host: dict = defaultdict(list)
    for (h, _b), vals in idle_watts_by_hb.items():
        idle_watts_by_host[h].extend(vals)

    all_clip = [
        r for r in runs
        if r.complete and r.meta.get("scenario") in ("one_clip", "ten_windows")
    ]
    clip_usable, clip_throttled = split_throttled(all_clip)

    by_key: dict = defaultdict(lambda: {"mj_per_clip": [], "run_ids": []})
    missing_baseline_keys: dict = defaultdict(list)
    for r in clip_usable:
        samples = _tegrastats_or_none(r)
        if samples is None:
            continue
        phase = "clipping" if r.meta.get("scenario") == "ten_windows" else "measure"
        bounds = phase_bounds(r.meta, phase)
        if not bounds:
            continue
        t0, t1 = bounds
        win = [s for s in samples if s.vdd_in_mw is not None and t0 <= s.ts_ns <= t1]
        if len(win) < 2:
            continue
        ts = np.array([s.ts_ns for s in win], dtype=float) / 1e9
        mw = np.array([s.vdd_in_mw for s in win], dtype=float)
        energy_mj = float(np.trapezoid(mw, ts))  # mW * s == mJ
        duration_s = float(ts[-1] - ts[0])
        n_clips = sum(1 for t in r.triggers if t.get("latency_ns") is not None)
        if n_clips == 0:
            continue
        key = f'{r.meta.get("host")}/{r.meta.get("scenario")}/{r.meta.get("bitrate_target_mbs")}'
        idle_w = idle_watts_median.get((r.meta.get("host"), r.meta.get("bitrate_target_mbs")))
        if idle_w is None:
            # No idle-tail baseline at this (host, bitrate) to subtract —
            # e.g. every idle_tail rep there was excluded as throttled.
            # Falling back to a 0 W baseline would silently report *raw*
            # energy while still calling it "marginal", the same
            # missing-vs-zero mistake as the IO rule above. Skip and say so.
            missing_baseline_keys[key].append(r.run_id)
            continue
        marginal_mj = energy_mj - idle_w * 1000 * duration_s
        by_key[key]["mj_per_clip"].append(marginal_mj / n_clips)
        by_key[key]["run_ids"].append(r.run_id)

    return {
        "idle_tail_watts": {
            h: {**agg_stat([w for w, _ in vals]), "run_ids": [rid for _, rid in vals]}
            for h, vals in idle_watts_by_host.items() if vals
        },
        "idle_tail_all_throttled_note": (
            f"all {len(idle_throttled)} idle_tail run(s) were excluded as throttled"
            if not idle_usable and idle_throttled else None
        ),
        "mj_per_clip": {
            k: {**agg_stat(v["mj_per_clip"]), "run_ids": v["run_ids"]}
            for k, v in by_key.items() if agg_stat(v["mj_per_clip"])
        },
        "missing_idle_baseline": {
            k: {"run_ids": run_ids, "note": "no idle-tail baseline at this (host, bitrate) to "
                "subtract — cannot compute a marginal figure, not shown as one"}
            for k, run_ids in missing_baseline_keys.items()
        },
        "clip_all_throttled_note": (
            f"all {len(clip_throttled)} one_clip/ten_windows run(s) were excluded as throttled"
            if not clip_usable and clip_throttled else None
        ),
        "tegrastats_alignment_failures": alignment_failures,
    }


# ---------------------------------------------------------------------------
# Caveats support: the measurement apparatus's own cost, and provenance
# ---------------------------------------------------------------------------

def analysis_sampler_overhead(runs: list) -> dict:
    """The sampler's own CPU cost, per host — quoted in REPORT.md's caveats
    section so "the measurement apparatus is not free" is backed by a real
    number from this results tree, never an estimate. `role == "sampler"`
    (CONTRACT.md's settled role enum: the sampler measures itself) is
    consumed through the exact same generic `run_phase_stats` machinery as
    every other role — no special-casing needed. Returns `{}` when no run
    in this tree carries `role=sampler` data at all (true of every real
    run seen so far — the deployed harness that produced them predates
    self-sampling); report.py falls back to stating that plainly rather
    than quoting a number this analysis didn't compute."""
    complete = [r for r in runs if r.complete]
    usable, _ = split_throttled(complete)
    by_host: dict = defaultdict(list)
    for r in usable:
        ps = run_phase_stats(r, "sampler", "measure")
        if ps:
            by_host[r.meta.get("host")].append(ps.cpu_pct)
    return {h: agg_stat(v) for h, v in by_host.items() if agg_stat(v)}


def harness_provenance(results_dir: Path) -> dict:
    """Look for a harness manifest documenting exactly which harness files
    produced this results tree, so REPORT.md can cite the numbers as
    traceable to a specific, checksummed harness rather than to
    "benchmarks/ at some point in time" (the repo tree drifts — confirmed:
    the deployed run_suite.sh differs from the repo's current one).

    Prefers `<results_dir>/harness-manifest.txt` (the name and per-run-tree
    location described for this suite); falls back to sibling
    `manifest-*.txt` files next to `results_dir` (the shape actually
    available while this module was being verified: `manifest-nano.txt`/
    `manifest-nx.txt`/`manifest-repo.txt`, one `sha256sum`-style line per
    harness file). Reports `{"found": False}` rather than fabricating a
    hash when neither shape is present. When more than one host's manifest
    is found, also reports whether their contents are byte-identical —
    independent confirmation that both hosts ran the same harness, without
    needing to reproduce whatever combined-hash algorithm produced any
    previously-quoted short hash, which this function does not attempt to
    guess at."""
    candidates = [results_dir / "harness-manifest.txt"]
    if results_dir.parent.exists():
        candidates += sorted(results_dir.parent.glob("manifest-*.txt"))
    manifests = [p for p in candidates if p.exists()]
    if not manifests:
        return {"found": False}
    entries = []
    for p in manifests:
        content = p.read_text()
        entries.append({
            "path": str(p),
            # A manifest whose name marks it as the repo's own current
            # tree (as opposed to a deployed host's) is a different kind
            # of reference — analyze/ is explicitly not frozen while a
            # suite runs, so it is EXPECTED to drift from what's deployed,
            # and that drift is not a cross-host comparability risk. Only
            # deployed-host manifests are compared against each other for
            # that risk; a repo manifest is reported separately.
            "is_repo_reference": "repo" in p.stem.lower(),
            "sha256": hashlib.sha256(content.encode()).hexdigest(),
            "n_files": len([line for line in content.splitlines() if line.strip()]),
        })
    deployed = [e for e in entries if not e["is_repo_reference"]]
    identical = len({e["sha256"] for e in deployed}) == 1 if len(deployed) > 1 else None
    return {"found": True, "manifests": entries, "identical_across_deployed_hosts": identical}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _json_default(o):
    if isinstance(o, Path):
        return str(o)
    if hasattr(o, "__dict__"):
        return asdict(o) if hasattr(o, "__dataclass_fields__") else vars(o)
    raise TypeError(f"not JSON serialisable: {o!r}")


def build_summary(results_dir: Path) -> dict:
    runs, exclusions = load_results_tree(results_dir)
    by_kind = defaultdict(list)
    for e in exclusions:
        by_kind[e.kind].append({"run_id": e.run_id, "dir": str(e.dir), "reason": e.reason})
    return {
        "n_runs_loaded": len(runs),
        "n_complete": sum(1 for r in runs if r.complete),
        "unparsable": by_kind["unparsable"],
        "skipped": by_kind["skipped"],
        "failed": by_kind["failed"],
        "throttled_run_ids": [r.run_id for r in runs if r.complete and r.throttled],
        "overhead_ratio": analysis_overhead_ratio(runs),
        "page_cache": analysis_page_cache(runs),
        "page_cache_system": analysis_page_cache_system(runs),
        "ten_windows_phases": analysis_ten_windows_phases(runs),
        "snapshot_sweep": analysis_snapshot_sweep(runs),
        "clip_latency": analysis_clip_latency(runs),
        "recorder_integrity": analysis_recorder_integrity(runs),
        "soak_drift": analysis_soak_drift(runs),
        "energy": analysis_energy(runs),
        "sampler_overhead": analysis_sampler_overhead(runs),
        "harness_provenance": harness_provenance(results_dir),
    }


def _main(argv=None):
    ap = argparse.ArgumentParser(
        description="Ingest a results tree and emit the aggregated statistics "
        "behind REPORT.md as JSON (debugging aid; report.py imports this "
        "module directly rather than round-tripping through this JSON)."
    )
    ap.add_argument("--results", required=True, type=Path, help="results tree root")
    ap.add_argument("--out", type=Path, help="write JSON here (default: stdout)")
    args = ap.parse_args(argv)

    summary = build_summary(args.results)
    text = json.dumps(summary, indent=2, default=_json_default)
    if args.out:
        args.out.write_text(text + "\n")
        print(f"wrote {args.out}", file=sys.stderr)
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(_main())
