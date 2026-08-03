#!/usr/bin/env python3
"""Parse tegrastats.log into power/thermal/utilisation series.

Runs on the workstation only (analysis side) — never on target. Stdlib only.

## Formats

Two real tegrastats.log samples (from actual gate runs, not guessed) settle
this — see the docstrings on `detect_format`/`_parse_tuple` for what they
showed and why the first version of this parser got it wrong:

- **l4t39** — the Orin Nano host (`orin-nano-jp72`, kernel
  `6.8.12-1021-tegra`).
- **l4t36** — the Orin NX host (`momentedge-desktop`, kernel
  `5.15.148-tegra`).

**Both hosts emit identical rail names (`VDD_IN`, `VDD_CPU_GPU_CV`,
`VDD_SOC`) and both emit `GR3D_FREQ`.** An earlier version of this module
assumed the two formats differed in *which* tokens appear — real logs prove
that wrong; there is no name-based discriminator at all. What actually
differs is **tuple arity** — how many `/`-separated values follow each
temperature and power reading:

    nano (l4t39): cpu@56.093C/56.093C            (2 temp values)
                  VDD_IN 7006mW/7006mW/7006mW     (3 power values)
    nx   (l4t36): cpu@62.906C                     (1 temp value)
                  VDD_IN 7527mW/7527mW            (2 power values)

`detect_format()` counts the arity of the first power (or temperature)
match on the line. Every extractor scans the whole line with `re.finditer`
regardless of arity or field order, so the two formats need no separate
code paths beyond that one count — the same design as before, just anchored
to the right signal this time.

**Only the first value in each tuple is used**, taken to be the
instantaneous reading. At idle, real samples show all tuple positions
reading identically, which proves nothing about what the *other* positions
mean (a running average? a different window? untested) — there is no way
to tell from a single sample, and guessing wrong would silently bias every
energy figure this suite produces. `_parse_tuple` and every call site that
reads it therefore always index `[0]` and nowhere else; `power_mw` and
`temps_c` are `Dict[str, float]`, not tuples, specifically so it's
impossible to accidentally read a different position downstream.

## Fields extracted

- **RAM/SWAP**: used/total MB (`ram_used_mb`, `ram_total_mb`, `swap_used_mb`,
  `swap_total_mb`).
- **CPU**: one percent-busy figure per core (`cpu_pct_per_core`; `None` for a
  core reported `off`).
- **`*_FREQ` utilisation**: `EMC_FREQ`, `GR3D_FREQ`, `VIC_FREQ`, etc. as
  `{name: percent}` in `freq_pct` — `freq_pct.get("EMC_FREQ")` for EMC
  utilisation, when present (real logs seen so far don't emit it at all;
  `emc_pct` returns `None` gracefully rather than guessing).
- **Temperatures**: every `zone@NN.NC[/NN.NC...]` token, first tuple value
  only, as `{zone: celsius}` in `temps_c` (real zone names are lowercase —
  `cpu`, `tj`, `soc0`, `cv0`, etc. — the alias lists account for this).
- **Power rails**: every `NAME NNmW/NNmW[/NNmW...]` token, first tuple value
  only, as `{rail: instantaneous_mw}` in `power_mw`.

An unrecognised rail or temp zone is still captured under its own literal
name, never dropped; whatever's left after stripping every recognised token
is kept verbatim in `TegraSample.misc_raw` (CONTRACT.md: "parse defensively
and record unknown fields rather than guessing").

## Timestamps — no timezone conversion here, ever

tegrastats prints a local `MM-DD-YYYY HH:MM:SS` wall-clock timestamp with
1-second resolution, no timezone, no epoch — while every other contract
artifact (`samples.csv`, `system.csv`, `triggers.jsonl`, `run.json`'s
`phases`) is `time.time_ns()` epoch nanoseconds (confirmed against
`lib/sampler.py`). The two real hosts are **four hours apart** — Nano is
`Etc/UTC`, NX is `America/New_York` (EDT, UTC-4) — confirmed directly: NX's
real log opens `08-03-2026 06:59:44` for a run whose `phases.warmup[0]`
converts to `10:59:15 UTC`.

This module does **not** guess a timezone. `_parse_ts_ns` treats the parsed
wall-clock string as if it were UTC — a stable, portable reference point,
*not* a claim about either host's real zone — and returns that as `ts_ns`.
Using the analysis workstation's own timezone here (an earlier version did,
via `datetime.astimezone()`) was proven wrong: it would silently shift NX's
entire power series by 4 hours, produce zero overlap with NX's own
measurement window, and yield confident-looking energy numbers computed
from the wrong samples entirely.

The real per-host offset is recovered by the caller — `summarize.py`'s
`_load_aligned_tegrastats`, which has the run's own `phases` (real epoch ns)
to align against, plus an assertion that the aligned window actually
overlaps the `measure` phase before any energy figure is computed from it.
This module only exposes the naive, offset-free timestamp; see
`summarize.py` for the alignment and its failure mode.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional


# ---------------------------------------------------------------------------
# Format detection
# ---------------------------------------------------------------------------

def detect_format(line: str) -> str:
    """Label a raw tegrastats line "l4t39", "l4t36", or "unknown" by tuple
    arity (see module docstring) — power rails first (guaranteed present,
    needed for energy anyway), temperature zones as a fallback. Advisory
    only: every extractor below scans the whole line for its own pattern
    regardless of this label."""
    pm = _POWER_RE.search(line)
    if pm:
        arity = pm.group(2).count("mW")
        if arity >= 3:
            return "l4t39"
        if arity == 2:
            return "l4t36"
    tm = _TEMP_RE.search(line)
    if tm:
        arity = tm.group(2).count("C")
        if arity >= 2:
            return "l4t39"
        if arity == 1:
            return "l4t36"
    return "unknown"


# ---------------------------------------------------------------------------
# Field extractors — each scans the whole line, order-independent.
# ---------------------------------------------------------------------------

_TS_RE = re.compile(r"^(\d{2}-\d{2}-\d{4} \d{2}:\d{2}:\d{2})")
_RAM_RE = re.compile(r"RAM (\d+)/(\d+)MB")
_SWAP_RE = re.compile(r"SWAP (\d+)/(\d+)MB \(cached (\d+)MB\)")
_CPU_BRACKET_RE = re.compile(r"CPU \[([^\]]*)\]")
_CPU_CORE_RE = re.compile(r"(off|\d+(?:\.\d+)?)%(?:@\d+)?")
_FREQ_RE = re.compile(r"\b([A-Za-z][A-Za-z0-9]*_FREQ)\s+(\d+(?:\.\d+)?)%(?:@\d+)?")
# Tuple-valued: one or more /-separated readings, each with its own unit
# suffix repeated (real logs: "cpu@56.093C/56.093C", "VDD_IN 7006mW/7006mW/7006mW").
_TEMP_RE = re.compile(r"\b([A-Za-z][A-Za-z0-9_]*)@(-?\d+(?:\.\d+)?C(?:/-?\d+(?:\.\d+)?C)*)\b")
_POWER_RE = re.compile(
    r"\b([A-Za-z][A-Za-z0-9_]*)\s+(-?\d+(?:\.\d+)?mW(?:/-?\d+(?:\.\d+)?mW)+)\b"
)

_VDD_IN_ALIASES = ("VDD_IN", "VDD_TOTAL", "VIN_SYS_5V0", "VDD_SYS")
_CPU_TEMP_ALIASES = ("cpu", "CPU")
_TJ_TEMP_ALIASES = ("tj", "Tj", "TJ", "Tboard", "Tdiode")


def _parse_tuple(blob: str, suffix: str) -> list:
    """"7006mW/7006mW/7006mW" + "mW" -> [7006.0, 7006.0, 7006.0]. Element
    [0] is the only one ever trusted as instantaneous — see module
    docstring."""
    return [float(part[: -len(suffix)]) for part in blob.split("/") if part]


@dataclass
class TegraSample:
    ts_ns: int  # naive "as-if-UTC" — see module docstring; caller aligns
    fmt: str  # "l4t39" | "l4t36" | "unknown"
    ram_used_mb: Optional[float]
    ram_total_mb: Optional[float]
    swap_used_mb: Optional[float]
    swap_total_mb: Optional[float]
    cpu_pct_per_core: list  # list[Optional[float]]
    freq_pct: dict  # {name: percent}
    temps_c: dict  # {zone: celsius} — first tuple value only
    power_mw: dict  # {rail: instantaneous_mw} — first tuple value only
    misc_raw: str  # whatever's left after stripping recognised tokens
    raw: str

    @property
    def cpu_temp_c(self) -> Optional[float]:
        for alias in _CPU_TEMP_ALIASES:
            if alias in self.temps_c:
                return self.temps_c[alias]
        return None

    @property
    def tj_temp_c(self) -> Optional[float]:
        for alias in _TJ_TEMP_ALIASES:
            if alias in self.temps_c:
                return self.temps_c[alias]
        return None

    @property
    def vdd_in_mw(self) -> Optional[float]:
        for alias in _VDD_IN_ALIASES:
            if alias in self.power_mw:
                return self.power_mw[alias]
        return None

    @property
    def emc_pct(self) -> Optional[float]:
        return self.freq_pct.get("EMC_FREQ")

    def to_dict(self) -> dict:
        d = asdict(self)
        d["cpu_temp_c"] = self.cpu_temp_c
        d["tj_temp_c"] = self.tj_temp_c
        d["vdd_in_mw"] = self.vdd_in_mw
        d["emc_pct"] = self.emc_pct
        return d


def _parse_ts_ns(line: str) -> int:
    m = _TS_RE.match(line)
    if not m:
        raise ValueError(f"no leading timestamp in tegrastats line: {line!r}")
    dt = datetime.strptime(m.group(1), "%m-%d-%Y %H:%M:%S")
    # Naive-as-UTC: see module docstring. NOT a timezone claim — the caller
    # (summarize.py) recovers and applies the real per-host offset.
    return int(dt.replace(tzinfo=timezone.utc).timestamp() * 1e9)


def parse_line(line: str) -> TegraSample:
    line = line.rstrip("\n")
    ts_ns = _parse_ts_ns(line)
    fmt = detect_format(line)

    consumed = line

    ram_used = ram_total = None
    m = _RAM_RE.search(line)
    if m:
        ram_used, ram_total = float(m.group(1)), float(m.group(2))
        consumed = consumed.replace(m.group(0), "", 1)

    swap_used = swap_total = None
    m = _SWAP_RE.search(line)
    if m:
        swap_used, swap_total = float(m.group(1)), float(m.group(2))
        consumed = consumed.replace(m.group(0), "", 1)

    cores: list = []
    m = _CPU_BRACKET_RE.search(line)
    if m:
        for cm in _CPU_CORE_RE.finditer(m.group(1)):
            tok = cm.group(1)
            cores.append(None if tok == "off" else float(tok))
        consumed = consumed.replace(m.group(0), "", 1)

    freq_pct = {}
    for fm in _FREQ_RE.finditer(line):
        freq_pct[fm.group(1)] = float(fm.group(2))
        consumed = consumed.replace(fm.group(0), "", 1)

    temps_c = {}
    for tm in _TEMP_RE.finditer(line):
        vals = _parse_tuple(tm.group(2), "C")
        temps_c[tm.group(1)] = vals[0]
        consumed = consumed.replace(tm.group(0), "", 1)

    power_mw = {}
    for pm in _POWER_RE.finditer(line):
        vals = _parse_tuple(pm.group(2), "mW")
        power_mw[pm.group(1)] = vals[0]
        consumed = consumed.replace(pm.group(0), "", 1)

    # Strip the leading timestamp too, then whatever's left (lfb note, APE,
    # NVENC/NVDEC off, etc.) is recorded verbatim rather than interpreted.
    ts_match = _TS_RE.match(consumed)
    if ts_match:
        consumed = consumed[ts_match.end():]
    misc_raw = " ".join(consumed.split())

    return TegraSample(
        ts_ns=ts_ns,
        fmt=fmt,
        ram_used_mb=ram_used,
        ram_total_mb=ram_total,
        swap_used_mb=swap_used,
        swap_total_mb=swap_total,
        cpu_pct_per_core=cores,
        freq_pct=freq_pct,
        temps_c=temps_c,
        power_mw=power_mw,
        misc_raw=misc_raw,
        raw=line,
    )


def parse_log(path) -> list:
    """Parse a tegrastats.log file into a list of TegraSample, in order,
    with naive "as-if-UTC" timestamps (see module docstring — the caller
    must align these to the run's real epoch before using them for
    anything time-sensitive).

    A line that fails to parse (e.g. a partial write at crash time) is
    skipped with a warning to stderr rather than aborting the whole run's
    analysis — one bad line must not lose the rest of the log.
    """
    samples = []
    with open(path) as f:
        for lineno, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                samples.append(parse_line(line))
            except Exception as exc:  # noqa: BLE001 - defensive by design
                print(
                    f"tegraparse: skipping {path}:{lineno}: {exc}",
                    file=sys.stderr,
                )
    return samples


def _main(argv=None):
    ap = argparse.ArgumentParser(
        description="Parse a tegrastats.log and print a JSON summary "
        "(debugging aid; summarize.py/report.py import this module directly). "
        "Timestamps printed here are naive/unaligned — see module docstring."
    )
    ap.add_argument("log", type=Path, help="path to tegrastats.log")
    args = ap.parse_args(argv)

    samples = parse_log(args.log)
    if not samples:
        print(json.dumps({"n": 0, "formats": {}}))
        return 0

    formats: dict = {}
    for s in samples:
        formats[s.fmt] = formats.get(s.fmt, 0) + 1

    vdd = [s.vdd_in_mw for s in samples if s.vdd_in_mw is not None]
    tj = [s.tj_temp_c for s in samples if s.tj_temp_c is not None]
    emc = [s.emc_pct for s in samples if s.emc_pct is not None]
    summary = {
        "n": len(samples),
        "formats": formats,
        "span_s_naive": (samples[-1].ts_ns - samples[0].ts_ns) / 1e9,
        "vdd_in_mw_range": [min(vdd), max(vdd)] if vdd else None,
        "tj_temp_c_range": [min(tj), max(tj)] if tj else None,
        "emc_pct_range": [min(emc), max(emc)] if emc else None,
        "rails_seen": sorted({r for s in samples for r in s.power_mw}),
        "temp_zones_seen": sorted({z for s in samples for z in s.temps_c}),
        "unrecognised_misc_examples": sorted(
            {s.misc_raw for s in samples if s.misc_raw}
        )[:5],
    }
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(_main())
