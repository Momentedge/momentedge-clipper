#!/usr/bin/env python3
"""Assemble REPORT.md from summarize.py's aggregates and charts.py's SVGs.

Runs on the workstation only.

    ./analyze/report.py --results <pulled results dir> --out REPORT.md

(matches the invocation documented in README.md). Charts land in a `charts/`
directory next to `--out` by default (override with `--charts-dir`) and are
linked from the Markdown with paths relative to `--out`'s directory, so the
report is portable as long as that relationship is preserved.

This module does no independent computation — every number below comes from
one of `summarize.py`'s `analysis_*` functions, and every chart from one of
`charts.py`'s three primitives. Its own job is: pick which rows are the
report's headline framing, build Markdown tables from `agg_stat`/`agg_io_stat`
dicts, and wire chart output into `<img>` tags. See `summarize.py`'s module
docstring for the flagged CONTRACT.md assumptions (snapshot-sweep arm
discriminator, OOM detection, rep-number parsing, skip-reason key) that
numbers in this report inherit, and for the empty-vs-zero IO and
throttled-exclusion rules this module renders rather than computes.

Three things this module is careful to keep visibly separate, per the
integrity rules this report exists to uphold:

- **Excluded runs come in three kinds** — `unparsable` (died before writing
  run.json), `skipped` (the orchestrator chose not to run it), `failed`
  (ran, didn't finish) — each its own section, never merged into one
  "excluded" bucket (a reader must be able to tell "we chose not to run
  this" from "this ran and broke").
- **Page-cache evidence comes in two strengths** — the Nano's direct
  per-process measurement and the NX's system-level inference — shown in
  separate subsections with the NX one explicitly labelled weaker, never
  blended into a single number.
- **A `not_measurable` IO aggregate renders as prose, not a number** —
  `fmt_stat` never lets a "the kernel can't report this" case print
  something that looks like a computed zero.
"""
from __future__ import annotations

import argparse
import datetime as dt
import sys
from pathlib import Path

import charts
import summarize


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def fmt_stat(d, unit: str = "", decimals: int = 1) -> str:
    if not d:
        return "n/a"
    if d.get("not_measurable"):
        return "not measurable on this host"
    f = f"{{:.{decimals}f}}"
    med, lo, hi, n = d["median"], d["min"], d["max"], d["n"]
    if n == 1:
        return f"{f.format(med)}{unit} (n=1)"
    return f"{f.format(med)}{unit} ({f.format(lo)}–{f.format(hi)}{unit}, n={n})"


def bitrate_label(row: dict) -> str:
    """Short categorical label (light/mid/heavy) for grouping and chart axes
    — see `bitrate_achieved_col` for the actual measured number CONTRACT.md
    asks be quoted alongside it."""
    run_ids = row.get("run_ids") or row.get("complete_run_ids") or []
    if run_ids:
        parts = summarize.parse_run_id(run_ids[0])
        if parts.get("bitrate"):
            return parts["bitrate"]
    b = row.get("bitrate_target_mbs")
    return f"{b} MB/s" if b is not None else "?"


def bitrate_achieved_col(row: dict) -> str:
    """CONTRACT.md: grouping is by target bitrate (it identifies matching
    arms), but the achieved rate — measured from the recording directory,
    exceeding target because the bag is lz4-compressed while the recorder
    writes uncompressed — is the figure quoted."""
    return fmt_stat(row.get("bitrate_achieved_mbs"), " MB/s")


def throttled_flag(row: dict) -> str:
    if row.get("throttled_note"):
        return row["throttled_note"]
    if row.get("throttled_run_ids"):
        return "throttled rep excluded"
    return ""


def md_table(headers: list, rows: list) -> str:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
    for r in rows:
        lines.append("| " + " | ".join(str(c) for c in r) + " |")
    return "\n".join(lines)


class ChartSink:
    """Writes SVGs under `charts_dir`, returns a path relative to `md_dir`
    for embedding — keeps report.py from caring about the exact layout."""

    def __init__(self, charts_dir: Path, md_dir: Path):
        self.charts_dir = charts_dir
        self.md_dir = md_dir
        self.charts_dir.mkdir(parents=True, exist_ok=True)
        self.written = []

    def save(self, filename: str, svg_text: str) -> str:
        path = self.charts_dir / filename
        path.write_text(svg_text)
        self.written.append(path)
        rel = path.relative_to(self.md_dir) if self._is_relative(path, self.md_dir) else path
        return str(rel)

    @staticmethod
    def _is_relative(path: Path, base: Path) -> bool:
        try:
            path.relative_to(base)
            return True
        except ValueError:
            return False

    def embed(self, filename: str, svg_text: str) -> str:
        rel = self.save(filename, svg_text)
        return f"![{filename}]({rel})"


# ---------------------------------------------------------------------------
# Section builders
# ---------------------------------------------------------------------------

def section_overhead(data: dict, sink: ChartSink) -> str:
    headline = data["headline_idle_tail"]
    out = ["## 1. Overhead ratio\n",
           "clipper's own cost during `idle_tail` (tailing, no triggers — the "
           "standing cost), directly measured and framed against `baseline` "
           "(load + rosbag2, no clipper) at the same host and bitrate. "
           "`idle_tail` is the headline row because, like `baseline`, it does "
           "not vary over compression/parallelism, so no canonical choice of "
           "those axes has to be picked to compare it directly. RSS is "
           "reported in MiB (1024-based, the traditional `/proc` unit); "
           "throughput figures elsewhere in this report are decimal MB "
           "(10⁶ bytes) per CONTRACT.md — the two are never mixed under the "
           "same label.\n"]

    if headline:
        # bitrate on the x-axis, host as the series — headline has one row
        # per (host, bitrate), and a flat group list keyed on bitrate alone
        # would draw two indistinguishable "light" bars (one per host) with
        # no way to tell which is which.
        hosts = sorted({r["host"] for r in headline})
        groups, seen = [], set()
        for r in headline:
            bl = bitrate_label(r)
            if bl not in seen:
                seen.add(bl)
                groups.append(bl)
        cpu_data = {(bitrate_label(r), r["host"]): r["clipper_cpu_pct"] for r in headline}
        rss_data = {(bitrate_label(r), r["host"]): r["clipper_rss_mib"] for r in headline}
        out.append(sink.embed("overhead_cpu.svg", charts.range_bar_chart(
            "clipper CPU overhead (idle_tail)", groups, hosts, cpu_data,
            "% of one core", footnote="median (min-max, n=reps) vs baseline at the same bitrate",
        )))
        out.append(sink.embed("overhead_rss.svg", charts.range_bar_chart(
            "clipper RSS overhead (idle_tail)", groups, hosts, rss_data,
            "MiB", footnote="median (min-max, n=reps)",
        )))
        rows = []
        for r in headline:
            rows.append([
                r["host"], bitrate_label(r), bitrate_achieved_col(r),
                fmt_stat(r["clipper_cpu_pct"], "%"),
                fmt_stat(r["clipper_rss_mib"], " MiB"),
                fmt_stat(r["rosbag2_cpu_pct_baseline"], "%"),
                fmt_stat(r["rosbag2_cpu_pct_with_clipper"], "%"),
                throttled_flag(r) or "-",
                ", ".join(r["run_ids"]) or "(none usable)",
            ])
        out.append(md_table(
            ["host", "bitrate", "achieved", "clipper CPU", "clipper RSS",
             "rosbag2 CPU (baseline)", "rosbag2 CPU (w/ clipper)", "throttled", "run_ids"],
            rows,
        ))
    else:
        out.append("_No complete `idle_tail` runs yet._")

    other = [r for r in data["all"] if r["scenario"] != "idle_tail"]
    if other:
        out.append("\n### By scenario (varies over compression/parallelism)\n")
        out.append("`ten_windows` rows below are averaged across its whole `measure` "
                    "phase (waiting + clipping blended, for comparability with the other "
                    "scenarios' single-phase framing) — see section 3 for its two phases "
                    "reported separately.\n")
        rows = []
        for r in sorted(other, key=lambda r: (r["scenario"], r["bitrate_target_mbs"] or 0)):
            rows.append([
                r["host"], r["scenario"], bitrate_label(r), r["clip_compression"],
                r["extract_parallelism"],
                fmt_stat(r["clipper_cpu_pct"], "%"),
                fmt_stat(r["clipper_rss_mib"], " MiB"),
                throttled_flag(r) or "-",
                ", ".join(r["run_ids"]) or "(none usable)",
            ])
        out.append(md_table(
            ["host", "scenario", "bitrate", "comp", "parallelism",
             "clipper CPU", "clipper RSS", "throttled", "run_ids"],
            rows,
        ))
    return "\n\n".join(out)


def section_page_cache(per_process: list, system_level: list, sink: ChartSink) -> str:
    out = ["## 2. Page-cache proof\n",
           "The hypothesis: clipper's tail reads bytes still in page cache, "
           "so `read_bytes` (bytes reaching the block device) stays near "
           "zero while `rchar` (bytes the process read) tracks the "
           "recording rate. Two different strengths of evidence follow — "
           "never blended into one number.\n"]

    out.append("### Direct measurement (per-process, Nano)\n")
    out.append("Per-process `rchar`/`read_bytes` for clipper, from `/proc/<pid>/io` "
                "— only available where the kernel reports it (CONTRACT.md: the NX's "
                "`5.15.148-tegra` kernel lacks `CONFIG_TASKSTATS`, so this is Nano-only). "
                "Reported either way — not softened if it comes back negative.\n")
    if not per_process:
        out.append("_No complete runs with directly-measurable per-process IO yet._")
    else:
        def _bps_to_mb_s(d):
            if not d or d.get("not_measurable"):
                return d
            return {**d, "median": d["median"] / 1e6, "min": d["min"] / 1e6, "max": d["max"] / 1e6}

        groups = [f'{r["host"]}/{r["scenario"]}/{bitrate_label(r)}' for r in per_process]
        data = {}
        for g, r in zip(groups, per_process):
            data[(g, "rchar")] = _bps_to_mb_s(r["rchar_rate_bps"])
            data[(g, "read_bytes")] = _bps_to_mb_s(r["read_bytes_rate_bps"])
        out.append(sink.embed("page_cache.svg", charts.range_bar_chart(
            "Page-cache proof: rchar vs read_bytes (clipper, direct)", groups,
            ["rchar", "read_bytes"], data, "MB/s",
            footnote="read_bytes near zero => tail reads are page-cache hits, not disk IO",
        )))

        table_rows = []
        for r in per_process:
            ratio = r["read_bytes_over_rchar_ratio"]
            if ratio and ratio.get("not_measurable"):
                verdict = "not measurable"
            elif ratio:
                verdict = "supports no-added-disk-IO" if ratio["median"] < 0.05 else "inconclusive"
            else:
                verdict = "n/a"
            table_rows.append([
                r["host"], r["scenario"], bitrate_label(r), bitrate_achieved_col(r), r["phase"],
                fmt_stat(r["rchar_rate_bps"], " B/s", 0),
                fmt_stat(r["read_bytes_rate_bps"], " B/s", 0),
                fmt_stat(ratio, "", 3),
                verdict,
                throttled_flag(r) or "-",
                ", ".join(r["run_ids"]) or "(none usable)",
            ])
        out.append(md_table(
            ["host", "scenario", "bitrate", "achieved", "phase", "rchar rate", "read_bytes rate",
             "read_bytes/rchar", "verdict", "throttled", "run_ids"],
            table_rows,
        ))

    out.append("\n### System-level inference (weaker evidence — hosts without per-process IO)\n")
    out.append("Where `/proc/<pid>/io` is unavailable, this diffs system-wide "
                "`disk_read_kb` (from `/proc/diskstats`, unaffected by the per-process "
                "gap) between a with-clipper arm and `baseline` at the same host and "
                "target bitrate. **This is not the same kind of number as the direct "
                "measurement above**: it can only show that *aggregate* physical reads "
                "did not rise, not that clipper's own reads are page-cache hits "
                "specifically.\n")
    if not system_level:
        out.append("_No hosts required the system-level fallback (or no matching "
                    "baseline/with-clipper pair yet)._")
    else:
        groups = [f'{r["host"]}/{r["scenario"]}/{bitrate_label(r)}' for r in system_level]
        data = {}
        for g, r in zip(groups, system_level):
            data[(g, "baseline")] = r["baseline_disk_read_mb_s"]
            data[(g, "with clipper")] = r["with_clipper_disk_read_mb_s"]
        out.append(sink.embed("page_cache_system.svg", charts.range_bar_chart(
            "Page-cache proof (system-level, weaker): disk_read_kb, baseline vs with-clipper",
            groups, ["baseline", "with clipper"], data, "MB/s",
        )))
        table_rows = []
        for r in system_level:
            verdict = "within noise (supports no added system-wide reads)" if r["within_noise"] else \
                "outside noise band — does not clearly support the claim"
            table_rows.append([
                r["host"], r["scenario"], bitrate_label(r), bitrate_achieved_col(r),
                fmt_stat(r["baseline_disk_read_mb_s"], " MB/s"),
                fmt_stat(r["with_clipper_disk_read_mb_s"], " MB/s"),
                f'{r["delta_mb_s"]:+.2f} MB/s',
                verdict,
                ", ".join(r["run_ids"]),
            ])
        out.append(md_table(
            ["host", "scenario", "bitrate", "achieved", "baseline disk_read", "with-clipper disk_read",
             "delta", "verdict", "run_ids"],
            table_rows,
        ))
    return "\n\n".join(out)


def section_ten_windows(rows: list, sink: ChartSink) -> str:
    out = ["## 3. The two phases of ten_windows\n",
           "Ten pending clips (`waiting`: all ten handlers parked for their "
           "postroll to elapse) vs ten clips actually copying (`clipping`), "
           "reported as separate numbers because that difference is the "
           "architecture's central claim: parked threads should cost almost "
           "nothing; the cost should appear only once copying starts.\n"]
    if not rows:
        out.append("_No complete `ten_windows` runs yet._")
        return "\n\n".join(out)

    # Rows vary over host and comp/parallelism, not just bitrate — a group
    # label of the bitrate alone would collapse several distinct configs
    # (e.g. 5 of these rows are all "mid") onto one indistinguishable x-tick
    # (the same duplicate-group bug fixed above in section_overhead), so the
    # label is the full config instead.
    rows = sorted(rows, key=lambda r: (r["bitrate_target_mbs"] or 0, r["host"], r["clip_compression"], r["extract_parallelism"]))
    groups = [f'{r["host"]}/{bitrate_label(r)}/{r["clip_compression"]}/par{r["extract_parallelism"]}' for r in rows]
    cpu_data = {}
    rss_data = {}
    for g, r in zip(groups, rows):
        cpu_data[(g, "waiting")] = r["waiting_cpu_pct"]
        cpu_data[(g, "clipping")] = r["clipping_cpu_pct"]
        rss_data[(g, "waiting")] = r["waiting_rss_mib"]
        rss_data[(g, "clipping")] = r["clipping_rss_mib"]
    out.append(sink.embed("ten_windows_cpu.svg", charts.range_bar_chart(
        "ten_windows: waiting vs clipping (CPU)", groups, ["waiting", "clipping"],
        cpu_data, "% of one core",
    )))
    out.append(sink.embed("ten_windows_rss.svg", charts.range_bar_chart(
        "ten_windows: waiting vs clipping (RSS)", groups, ["waiting", "clipping"],
        rss_data, "MiB",
    )))

    absent_notes = [(r, r["waiting_phase_absent_note"]) for r in rows if r.get("waiting_phase_absent_note")]
    if absent_notes:
        lines = [
            f'- **{r["host"]}/{bitrate_label(r)}/{r["clip_compression"]}/par{r["extract_parallelism"]}**: {note}'
            for r, note in absent_notes
        ]
        out.append("**`waiting` phase absent for some configs** (not computed, never "
                    "synthesised from trigger timestamps — see summarize.py's "
                    "`analysis_ten_windows_phases` docstring for why that matters):\n\n"
                    + "\n".join(lines))

    table_rows = []
    for r in rows:
        waiting_marker = " (phase absent)" if r.get("waiting_phase_absent_note") else ""
        table_rows.append([
            r["host"], bitrate_label(r), bitrate_achieved_col(r), r["clip_compression"], r["extract_parallelism"],
            fmt_stat(r["waiting_cpu_pct"], "%") + waiting_marker,
            fmt_stat(r["waiting_rss_mib"], " MiB") + waiting_marker,
            fmt_stat(r["clipping_cpu_pct"], "%"), fmt_stat(r["clipping_rss_mib"], " MiB"),
            throttled_flag(r) or "-",
            ", ".join(r["run_ids"]) or "(none usable)",
        ])
    out.append(md_table(
        ["host", "bitrate", "achieved", "comp", "parallelism", "waiting CPU", "waiting RSS",
         "clipping CPU", "clipping RSS", "throttled", "run_ids"],
        table_rows,
    ))
    return "\n\n".join(out)


def section_snapshot_sweep(data: dict, sink: ChartSink) -> str:
    out = ["## 4. clipper vs rosbag2 snapshot mode\n",
           "The centrepiece comparison: peak RSS as the preroll window grows "
           "from 5 s to 600 s, for clipper's approach vs rosbag2's "
           "`--snapshot-mode` in-RAM ring buffer. **Both figures are the "
           "total memory for the capability, not one process in isolation**: "
           "clipper's number is the rosbag2 recorder it depends on *plus* "
           "clipper itself (comparing clipper alone against snapshot mode's "
           "recorder would omit the recorder clipper needs and flatter the "
           "comparison); snapshot mode's number is already a single process. "
           "**The point is the slope, not any one value** — snapshot mode's "
           "requirement is expected to grow linearly with window length "
           "while clipper's stays flat, until snapshot mode is OOM-killed. "
           "**Both hosts are plotted as separate series** — the Nano (7.5 GB "
           "RAM) and NX (15.6 GB) have materially different OOM ceilings, so "
           "pooling them into one line would blend that axis into the "
           "flagship chart. **Arm/preroll are read from run.json's "
           "`snapshot_arm`/`variant` fields** — see `summarize.py`'s "
           "`_snapshot_arm` docstring for the one still-unconfirmed detail "
           "(the exact `snapshot_arm` string vocabulary; no real "
           "snapshot_sweep run.json existed when this was written). "
           "**Bitrate is fixed for the whole sweep.** `bitrate_achieved_mbs` "
           "briefly could not be trusted for the snapshot arm (a naive "
           "bytes-written/elapsed there measures the buffer *dump* when the "
           "service call fires, not the load ingest rate — real data: "
           "2.3 MB/s \"achieved\" against a 20 MB/s load) — suite-dev has "
           "since normalised it to the true load rate on every arm (via the "
           "dumped mcap's own Statistics-record span for the snapshot arm), "
           "so it is quoted below like everywhere else in this report, "
           "still checked against `bitrate_achieved_source` per run (an "
           "OOM-killed cache with no dump reads as not-measured, not a "
           "number).\n"]
    tail_by_host: dict = {}
    snap_by_host: dict = {}
    for row in data.get("clipper_tail", []):
        tail_by_host.setdefault(row["host"], {})[row["preroll_s"]] = row
    for row in data.get("snapshot_mode", []):
        snap_by_host.setdefault(row["host"], {})[row["preroll_s"]] = row
    hosts = sorted(set(tail_by_host) | set(snap_by_host))
    x_values = sorted({
        p for by_host in (tail_by_host, snap_by_host) for rows in by_host.values() for p in rows
        if p is not None
    })
    if not x_values:
        out.append("_No complete `snapshot_sweep` runs yet._")
        return "\n\n".join(out)

    targets = sorted({
        row["bitrate_target_mbs"] for by_host in (tail_by_host, snap_by_host)
        for rows in by_host.values() for row in rows.values()
        if row.get("bitrate_target_mbs") is not None
    })
    if targets:
        out.append(f"Sweep run at target bitrate {', '.join(f'{t:g} MB/s' for t in targets)}.\n")

    series = {}
    annotations = []
    for host in hosts:
        tail = tail_by_host.get(host, {})
        snap = snap_by_host.get(host, {})
        series[f"clipper (tail), {host}"] = [
            (tail[x]["peak_rss_mib"]["median"] if x in tail and tail[x]["peak_rss_mib"] else None)
            for x in x_values
        ]
        series[f"rosbag2 (snapshot-mode), {host}"] = [
            (snap[x]["peak_rss_mib"]["median"] if x in snap and snap[x]["peak_rss_mib"] else None)
            for x in x_values
        ]
        for x in x_values:
            row = snap.get(x)
            if row and row["oom"]:
                if row.get("incomplete_peak_rss_mib"):
                    y = row["incomplete_peak_rss_mib"]["max"]
                elif row["peak_rss_mib"]:
                    y = row["peak_rss_mib"]["max"]
                else:
                    y = 0.0
                annotations.append({
                    "x": x, "y": y, "label": f"{host} OOM-killed @ {x:.0f}s", "status": "critical",
                })

    out.append(sink.embed("snapshot_sweep_rss.svg", charts.line_chart(
        "Total peak RSS vs preroll: clipper's approach vs rosbag2 snapshot-mode, by host",
        x_values, series,
        "total peak RSS (MiB)", x_label="preroll (s, log scale)", log_x=True,
        annotations=annotations,
        footnote="clipper = recorder+clipper combined; snapshot-mode = recorder alone (already the "
                 "total); lines show the median across reps; OOM marked where the snapshot arm was killed",
    )))

    table_rows = []
    for host in hosts:
        tail, snap = tail_by_host.get(host, {}), snap_by_host.get(host, {})
        for x in x_values:
            t, s = tail.get(x), snap.get(x)
            if t is None and s is None:
                continue
            flags = []
            if t and (t.get("throttled_note") or t.get("throttled_run_ids")):
                flags.append("tail:" + (t.get("throttled_note") or "throttled"))
            if s and (s.get("throttled_note") or s.get("throttled_run_ids")):
                flags.append("snapshot:" + (s.get("throttled_note") or "throttled"))
            achieved_row = t if (t and t.get("bitrate_achieved_mbs")) else s
            table_rows.append([
                host, f"{x:.0f}",
                bitrate_achieved_col(achieved_row) if achieved_row else "n/a",
                fmt_stat(t["peak_rss_mib"], " MiB") if t else "n/a",
                fmt_stat(s["peak_rss_mib"], " MiB") if s else "n/a",
                "OOM" if s and s["oom"] else "ok",
                "; ".join(flags) or "-",
                ", ".join((t["complete_run_ids"] if t else []) + (s["complete_run_ids"] if s else [])),
            ])
    out.append(md_table(
        ["host", "preroll (s)", "achieved", "clipper approach (recorder+clipper) total RSS",
         "snapshot-mode (recorder) total RSS", "outcome", "throttled", "run_ids"],
        table_rows,
    ))
    return "\n\n".join(out)


def section_clip_latency(data: dict, sink: ChartSink) -> str:
    out = ["## 5. Clip latency\n",
           "Trigger-published -> `Recorded`-announced latency, from "
           "`triggers.jsonl`.\n"]
    pooled = data.get("by_parallelism_pooled", [])
    by_config = data.get("by_config", [])
    if not pooled and not by_config:
        out.append("_No completed triggers yet._")
        return "\n\n".join(out)

    out.append("### Pooled by extract_parallelism\n")
    out.append("Every scenario/bitrate/compression at this parallelism, pooled — "
                "the harness brief's headline framing, but a wide min-max spread "
                "here reflects heterogeneous configs, not just noise. See the "
                "per-config breakdown below for that detail.\n")
    if pooled:
        groups = [str(r["extract_parallelism"]) for r in pooled]
        pdata = {(g, "latency"): r["latency_ms"] for g, r in zip(groups, pooled)}
        out.append(sink.embed("clip_latency.svg", charts.range_bar_chart(
            "Clip latency by extract_parallelism (pooled across configs)", groups, ["latency"], pdata, "ms",
        )))
        rows = [
            [r["extract_parallelism"], fmt_stat(r["latency_ms"], " ms"), r["n_triggers"],
             throttled_flag(r) or "-", ", ".join(r["run_ids"]) or "(none usable)"]
            for r in pooled
        ]
        out.append(md_table(
            ["extract_parallelism", "latency", "n triggers", "throttled", "run_ids"], rows
        ))

    if by_config:
        out.append("\n### By full config\n")
        rows = [
            [r["host"], r["scenario"], r["bitrate_target_mbs"], r["clip_compression"],
             r["extract_parallelism"], fmt_stat(r["latency_ms"], " ms"), r["n_triggers"],
             throttled_flag(r) or "-", ", ".join(r["run_ids"]) or "(none usable)"]
            for r in by_config
        ]
        out.append(md_table(
            ["host", "scenario", "bitrate", "comp", "parallelism", "latency", "n triggers",
             "throttled", "run_ids"],
            rows,
        ))
    return "\n\n".join(out)


def section_recorder_integrity(data: dict, sink: ChartSink) -> str:
    out = ["## 6. Recorder integrity\n",
           "rosbag2's dropped-message count, with and without clipper "
           "present. If clipper's presence never changes it, that is the "
           "strongest possible result and is stated plainly rather than "
           "buried in a table. **Three states are kept explicitly distinct**: "
           "rosbag2 only emits a drop line when drops occur, so "
           "`rosbag2_dropped: null` means *not measured*, not *measured "
           "zero* — conflating the two would fabricate a headline from data "
           "that was never captured.\n"]
    state = data["measured_state"]
    if state == "measured":
        if data["any_nonzero"]:
            out.append(f"**rosbag2 dropped messages in at least one of the {data['n_measured']} measured "
                        "runs** — see the breakdown below; this does not by itself indict clipper "
                        "(compare the baseline-vs-with-clipper columns for the same host/bitrate).")
        else:
            out.append(f"**rosbag2 dropped zero messages across all {data['n_measured']} measured runs, "
                        "with clipper running or not.** clipper's presence never changed recorder integrity.")
        if data["n_not_measured"]:
            out.append(f"({data['n_not_measured']} usable run(s) had no drop measurement at all — "
                        "see \"not measured\" in the table below.)")
    elif state == "not_measured":
        out.append(f"**Not measured.** All {data['n_not_measured']} usable run(s) analysed carry "
                    "`rosbag2_dropped: null` — rosbag2 never emitted a drop line, which is consistent "
                    "with zero drops but is not itself a measurement of zero. Capture is being added "
                    "upstream (suite-dev); this section reports a real number only once one exists.")
    else:
        out.append("**No usable runs to report on** (none measured, none not-measured — see "
                    "Throttled/Failed/Skipped below for why).")

    rows = []
    for key, stat in sorted(data["per_scenario"].items()):
        n_not_measured = len(stat.get("not_measured_run_ids") or [])
        if "median" in stat:
            dropped_cell = fmt_stat(stat, "", 0)
        elif n_not_measured:
            dropped_cell = f"not measured (n={n_not_measured})"
        else:
            dropped_cell = "n/a"
        rows.append([
            key, dropped_cell,
            throttled_flag(stat) or "-",
            ", ".join(stat["run_ids"]) or "(none usable)",
        ])
    out.append(md_table(["host/scenario", "rosbag2_dropped", "throttled", "run_ids"], rows))
    return "\n\n".join(out)


def section_soak_drift(data: dict, sink: ChartSink) -> str:
    out = ["## 7. Soak drift\n",
           "RSS, fd count and clip latency over a 4 h soak (one clip a "
           "minute). Slopes are fit per run (soak has no reps to spread "
           "across) and flagged \"within noise\" when the fit's predicted "
           "change over the run is under 2x the residual scatter.\n"]
    rows = data.get("runs", [])
    if data.get("throttled_note"):
        out.append(f"**{data['throttled_note']}.**")
    elif data.get("throttled_run_ids"):
        out.append(f"Excluded as throttled: {', '.join(data['throttled_run_ids'])}.")
    if not rows:
        out.append("_No usable complete `soak` runs yet._")
        return "\n\n".join(out)

    for r in rows:
        for metric, key, unit in (
            ("RSS", "rss_mib_drift", "MiB"), ("fd count", "fds_drift", "fds"),
            ("latency", "latency_ms_drift", "ms"),
        ):
            d = r[key]
            series_key = key.replace("_drift", "_series")
            pts = r[series_key]
            if not pts:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            note = "no fit (too few buckets)"
            if d:
                verdict = "within noise" if d["within_noise"] else "real drift"
                note = (f"slope {d['slope_per_hour']:.3g} {unit}/h, predicted change over run "
                        f"{d['predicted_change_over_run']:.3g} {unit} — {verdict}")
            out.append(sink.embed(f"soak_{r['run_id']}_{key}.svg", charts.line_chart(
                f"{r['run_id']}: {metric} over time", xs, {metric: ys}, f"{metric} ({unit})",
                x_label="elapsed (h)", footnote=note,
            )))
    return "\n\n".join(out)


def section_energy(data: dict, sink: ChartSink) -> str:
    out = ["## 8. Energy\n",
           "mJ per clip (marginal over idle-tail draw, from tegrastats' "
           "VDD_IN-aliased rail) and idle-tail watts.\n"]
    if data.get("idle_tail_all_throttled_note"):
        out.append(f"**{data['idle_tail_all_throttled_note']}.**")
    if data.get("clip_all_throttled_note"):
        out.append(f"**{data['clip_all_throttled_note']}.**")
    missing = data.get("missing_idle_baseline") or {}
    if missing:
        lines = [f"- **{k}**: {v['note']} ({', '.join(v['run_ids'])})" for k, v in sorted(missing.items())]
        out.append("mJ/clip could not be computed for:\n\n" + "\n".join(lines))
    alignment_failures = data.get("tegrastats_alignment_failures") or []
    if alignment_failures:
        lines = [f"- `{f['run_id']}`: {f['reason']}" for f in alignment_failures]
        out.append("**tegrastats alignment failed for the following runs — excluded from "
                    "energy figures rather than computed from a misaligned window:**\n\n"
                    + "\n".join(lines))

    watts = data["idle_tail_watts"]
    if watts:
        groups = sorted(watts)
        wdata = {(h, "idle watts"): watts[h] for h in groups}
        out.append(sink.embed("idle_tail_watts.svg", charts.range_bar_chart(
            "Idle-tail power draw", groups, ["idle watts"], wdata, "W",
        )))
        rows = [[h, fmt_stat(watts[h], " W", 2), ", ".join(watts[h]["run_ids"])] for h in groups]
        out.append(md_table(["host", "idle-tail power", "run_ids"], rows))
    else:
        out.append("_No usable `idle_tail` runs with tegrastats.log yet._")

    mj = data["mj_per_clip"]
    if mj:
        groups = sorted(mj)
        mdata = {(g, "mJ/clip"): mj[g] for g in groups}
        out.append(sink.embed("energy_mj_per_clip.svg", charts.range_bar_chart(
            "Marginal energy per clip", groups, ["mJ/clip"], mdata, "mJ",
        )))
        rows = [[g, fmt_stat(mj[g], " mJ", 0), ", ".join(mj[g]["run_ids"])] for g in groups]
        out.append(md_table(["host/scenario/bitrate", "mJ per clip", "run_ids"], rows))
    else:
        out.append("_No usable `one_clip`/`ten_windows` runs with tegrastats.log yet._")
    return "\n\n".join(out)


def _section_exclusions(title: str, prose: str, rows: list) -> str:
    out = [f"## {title}\n", prose + "\n"]
    if not rows:
        out.append("_None._")
        return "\n\n".join(out)
    table_rows = [[r["run_id"], r["reason"]] for r in rows]
    out.append(md_table(["run_id", "reason"], table_rows))
    return "\n\n".join(out)


def section_skipped(rows: list) -> str:
    return _section_exclusions(
        "Skipped runs", "Configs the orchestrator's disk preflight chose not to "
        "run at all — distinct from a run that started and broke.", rows,
    )


def section_failed(rows: list) -> str:
    return _section_exclusions(
        "Failed runs", "Runs that started but did not finish (`\"complete\": false`, "
        "not skipped) — excluded from every statistic above, listed rather than "
        "silently dropped.", rows,
    )


def section_unparsable(rows: list) -> str:
    return _section_exclusions(
        "Unparsable run directories", "Directories whose name matches the run_id "
        "scheme but have no readable `run.json` — died before writing one. Harness "
        "artefacts that are not run directories at all (`calibration/`, `suite.log`, "
        "etc.) are excluded from this list entirely rather than appearing as noise.",
        rows,
    )


def section_throttled(run_ids: list) -> str:
    out = ["## Throttled runs\n",
           "Excluded from every aggregate above by default (see the "
           "\"throttled\" column/notes in each section) rather than averaged "
           "in silently — listed here for traceability.\n"]
    out.append("_None._" if not run_ids else "\n".join(f"- `{r}`" for r in run_ids))
    return "\n\n".join(out)


def build_headline(summary: dict) -> str:
    lines = ["## Headline\n"]
    idle = summary["overhead_ratio"]["headline_idle_tail"]
    if idle:
        mid = idle[len(idle) // 2]
        cpu = fmt_stat(mid["clipper_cpu_pct"], "%")
        rss = fmt_stat(mid["clipper_rss_mib"], " MiB")
        lines.append(f"- **Overhead:** at {bitrate_label(mid)} bitrate on {mid['host']} "
                      f"({bitrate_achieved_col(mid)} achieved), clipper adds "
                      f"{cpu} of one core and {rss} of RSS on top of the rosbag2 recorder you already "
                      f"pay for (`idle_tail` vs `baseline`, n={mid['clipper_cpu_pct']['n'] if mid['clipper_cpu_pct'] else 0} reps).")
    pc = summary["page_cache"]
    if pc:
        candidates = [r for r in pc if r["read_bytes_over_rchar_ratio"] and not r["read_bytes_over_rchar_ratio"].get("not_measurable")]
        if candidates:
            best = min(candidates, key=lambda r: r["read_bytes_over_rchar_ratio"]["median"])
            ratio = best["read_bytes_over_rchar_ratio"]
            verdict = "supports" if ratio["median"] < 0.05 else "does not clearly support"
            lines.append(f"- **Page cache (direct, {best['host']}):** read_bytes/rchar = "
                          f"{fmt_stat(ratio, '', 3)} for clipper ({best['host']}/{best['scenario']}) — "
                          f"{verdict} \"clipper adds no read IO to your disk.\"")
    pcs = summary["page_cache_system"]
    if pcs:
        r = pcs[0]
        verdict = "supports" if r["within_noise"] else "does not clearly support"
        lines.append(f"- **Page cache (system-level, {r['host']}, weaker evidence):** "
                      f"disk_read delta = {r['delta_mb_s']:+.2f} MB/s vs baseline — {verdict} the "
                      "same claim at the whole-system level (no direct per-process IO on this host).")
    tw = summary["ten_windows_phases"]
    if tw:
        r = tw[0]
        lines.append(f"- **ten_windows:** waiting costs {fmt_stat(r['waiting_cpu_pct'], '%')} CPU / "
                      f"{fmt_stat(r['waiting_rss_mib'], ' MiB')} RSS; clipping costs "
                      f"{fmt_stat(r['clipping_cpu_pct'], '%')} CPU / {fmt_stat(r['clipping_rss_mib'], ' MiB')} RSS.")
    snap = summary["snapshot_sweep"]
    if snap.get("snapshot_mode"):
        oom_rows = [row for row in snap["snapshot_mode"] if row["oom"]]
        if oom_rows:
            r0 = oom_rows[0]
            lines.append(f"- **snapshot mode vs clipper:** on {r0['host']}, rosbag2 `--snapshot-mode` "
                          f"was OOM-killed at preroll={r0['preroll_s']:.0f}s (see chart in section 4, "
                          "plotted per host — OOM ceilings differ by RAM size); clipper's tail stayed "
                          "flat across the same sweep.")
    ri = summary["recorder_integrity"]
    if ri:
        if ri["measured_state"] == "not_measured":
            msg = (f"not measured — all {ri['n_not_measured']} usable run(s) carry "
                   "`rosbag2_dropped: null` (rosbag2 only emits a drop line when drops occur); "
                   "see section 6.")
        elif ri["measured_state"] == "measured":
            msg = (
                f"rosbag2 dropped zero messages across all {ri['n_measured']} measured runs, clipper present or not."
                if not ri["any_nonzero"] else
                f"rosbag2 dropped messages in at least one of {ri['n_measured']} measured runs — see section 6."
            )
        else:
            msg = "no usable runs to report on."
        lines.append(f"- **Recorder integrity:** {msg}")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def section_caveats(summary: dict, results_dir: Path) -> str:
    """The limitations that could change how a number in this report is
    read, travelling WITH the numbers rather than living only in
    benchmarks/README.md. Placed near the top of the generated report, not
    at the end — a reader who stops after the headline should still have
    seen these. Written as conditions of measurement, not hedges: every
    point states plainly what was and wasn't measured and why, so a
    skeptical reader can judge the numbers on their own terms."""
    out = ["## Scope and conditions of measurement\n",
           "Every number below was produced under the specific conditions "
           "listed here. None of them are hedges — they are what would "
           "have to be true for a given number to mean what it appears to "
           "mean, stated plainly rather than left for a reader to "
           "discover only in README.md.\n"]

    out.append(
        "**1. Recorder profile.** Every figure here was measured with the "
        "recorder in `fastwrite` mode — uncompressed, unchunked. clipper's "
        "tail therefore never decompresses a chunk it scans. Under a "
        "`zstd_fast` recording the tail must decompress every chunk it "
        "scans, so every CPU and latency figure in this report is a "
        "**lower bound** for that deployment, not an estimate of it."
    )
    out.append(
        "**2. Cross-host absolutes are not comparable.** The Nano and NX "
        "differ in SoC, core count, RAM, kernel, and ROS distro. Only the "
        "clipper-over-rosbag2 *ratio* measured on each host, independently, "
        "is comparable across them — an absolute CPU/RSS/energy number on "
        "one host says nothing about the other. Wherever this report puts "
        "both hosts on one axis (the snapshot-sweep chart, the overhead "
        "tables), they are kept as separate series or rows for exactly "
        "this reason; read them side by side, never against each other's "
        "absolute scale."
    )
    out.append(
        "**3. Per-process disk IO exists on the Nano only.** The NX kernel "
        "(`5.15.148-tegra`) lacks `CONFIG_TASKSTATS`, so its per-process "
        "byte counts are structurally unobtainable — reported empty, never "
        "zero. Section 2's page-cache result is therefore two different "
        "strengths of evidence, not one: a **direct** per-process "
        "measurement on the Nano, and a **weaker** system-level inference "
        "on the NX. They are never the same kind of number, and section 2 "
        "keeps them in separate subsections, the NX one explicitly labelled "
        "weaker, for this reason."
    )

    sampler = summary.get("sampler_overhead") or {}
    if sampler:
        quoted = "; ".join(f"{h}: {fmt_stat(v, '%')}" for h, v in sorted(sampler.items()))
        sampler_sentence = f"Measured directly from this results tree ({quoted})."
    else:
        sampler_sentence = (
            "Not measurable from this results tree — no run here carries "
            "`role=sampler` samples (the harness that produced this data predates "
            "self-sampling). The harness team has reported roughly 0.4-0.8% of one "
            "core from on-target measurement; that figure is theirs, not computed "
            "by this analysis, and is not otherwise used in this report."
        )
    out.append(
        "**4. The measurement apparatus is not free.** " + sampler_sentence + " "
        "That is the same order of magnitude as clipper's own idle-tail cost "
        "(~0.55% of one core). It does **not** bias the clipper-vs-baseline "
        "ratio reported here, since the sampler runs identically, at the same "
        "cost, in every arm, and CPU is attributed per pid — but a reader who "
        "sees only the first half of this point would over-discount every "
        "number in this report, so both halves are stated together."
    )
    out.append(
        "**5. The NX host ran with a known, uncontrolled noise source.** A "
        "stale 45-day tmux session running htop (~2.2% of one core) plus two "
        "`watch -n 1` loops ran throughout the NX suite; permission to kill "
        "it was requested and denied. It burdens baseline and clipper arms "
        "equally, so it does not invalidate the ratios measured on that "
        "host — but it does raise the NX noise floor, most of all on the "
        "light (~3 MB/s) arm, where clipper's own cost is smallest relative "
        "to that floor. This is reported as a stated condition of the NX "
        "data, not as a correction applied to it."
    )
    out.append(
        "**6. EMC utilisation is not captured.** `tegrastats`, invoked as "
        "this suite invokes it, emits no `EMC_FREQ`/EMC token at all on "
        "either host or JetPack version. No memory-bandwidth utilisation "
        "figure is available, and none is reported anywhere in this "
        "document — an absent capability, not an omitted one."
    )

    prov = summary.get("harness_provenance") or {"found": False}
    if prov.get("found"):
        lines = [
            f"  - `{e['path']}` — sha256 `{e['sha256']}` ({e['n_files']} files listed)"
            for e in prov["manifests"]
        ]
        identical_note = ""
        if prov.get("identical_across_deployed_hosts") is True:
            identical_note = (" The deployed-host manifests found are byte-identical, confirming every "
                               "host ran the same harness.")
        elif prov.get("identical_across_deployed_hosts") is False:
            identical_note = (" The deployed-host manifests found are NOT all identical — at least one "
                               "host's harness differed from another's; do not assume cross-host "
                               "comparability without checking which files diverged.")
        repo_drift_note = ""
        if any(e["is_repo_reference"] for e in prov["manifests"]):
            repo_hashes = {e["sha256"] for e in prov["manifests"] if e["is_repo_reference"]}
            deployed_hashes = {e["sha256"] for e in prov["manifests"] if not e["is_repo_reference"]}
            if repo_hashes and deployed_hashes and repo_hashes != deployed_hashes:
                repo_drift_note = (" The repo tree's own manifest differs from the deployed hosts' — "
                                    "expected, since this analysis code is not frozen while a suite runs, "
                                    "and not itself a cross-host risk.")
        out.append(
            "**7. Provenance.** These numbers are traceable to a specific, checksummed harness, "
            "not to \"benchmarks/ at some point in time\" (the repo tree drifts independently of "
            "what a running suite was launched with — confirmed during this suite's own analysis "
            "development)." + identical_note + repo_drift_note + " Manifest(s) found alongside this "
            "results tree:\n" + "\n".join(lines)
        )
    else:
        out.append(
            "**7. Provenance.** No harness manifest was found alongside this results tree "
            "(`harness-manifest.txt` or a sibling `manifest-*.txt`), so the exact harness commit/"
            "checksum that produced these numbers cannot be cited here. Treat this report as "
            "traceable only to the results directory path quoted above, not to a specific, "
            "verified harness build."
        )
    out.append(
        "**8. n=3 discipline.** Every aggregate in this report is a median "
        "with min/max across the available reps, never a mean — a single "
        "thermally-throttled or otherwise anomalous rep cannot silently "
        "drag a number. Where a difference between two conditions is "
        "smaller than the observed spread across reps, this report states "
        "that plainly as inside the noise rather than quoting it as a real "
        "effect."
    )
    return "\n\n".join(out)


def build_report(results_dir: Path, out_path: Path, charts_dir: Path) -> str:
    runs, exclusions = summarize.load_results_tree(results_dir)
    summary = {
        "overhead_ratio": summarize.analysis_overhead_ratio(runs),
        "page_cache": summarize.analysis_page_cache(runs),
        "page_cache_system": summarize.analysis_page_cache_system(runs),
        "ten_windows_phases": summarize.analysis_ten_windows_phases(runs),
        "snapshot_sweep": summarize.analysis_snapshot_sweep(runs),
        "clip_latency": summarize.analysis_clip_latency(runs),
        "recorder_integrity": summarize.analysis_recorder_integrity(runs),
        "soak_drift": summarize.analysis_soak_drift(runs),
        "energy": summarize.analysis_energy(runs),
        "sampler_overhead": summarize.analysis_sampler_overhead(runs),
        "harness_provenance": summarize.harness_provenance(results_dir),
    }
    sink = ChartSink(charts_dir, out_path.parent)
    throttled_run_ids = [r.run_id for r in runs if r.complete and r.throttled]
    n_complete = sum(1 for r in runs if r.complete)

    by_kind = {"skipped": [], "failed": [], "unparsable": []}
    for e in exclusions:
        by_kind.setdefault(e.kind, []).append({"run_id": e.run_id, "reason": e.reason})
    n_excluded = sum(len(v) for v in by_kind.values())

    parts = [
        "# clipper benchmark report\n",
        f"Generated {dt.datetime.now().isoformat(timespec='seconds')} from "
        f"`{results_dir}` — {n_complete} complete runs, {n_excluded} excluded "
        f"({len(by_kind['skipped'])} skipped, {len(by_kind['failed'])} failed, "
        f"{len(by_kind['unparsable'])} unparsable).\n",
        section_caveats(summary, results_dir),
        build_headline(summary),
        section_overhead(summary["overhead_ratio"], sink),
        section_page_cache(summary["page_cache"], summary["page_cache_system"], sink),
        section_ten_windows(summary["ten_windows_phases"], sink),
        section_snapshot_sweep(summary["snapshot_sweep"], sink),
        section_clip_latency(summary["clip_latency"], sink),
        section_recorder_integrity(summary["recorder_integrity"], sink),
        section_soak_drift(summary["soak_drift"], sink),
        section_energy(summary["energy"], sink),
        section_skipped(by_kind["skipped"]),
        section_failed(by_kind["failed"]),
        section_unparsable(by_kind["unparsable"]),
        section_throttled(throttled_run_ids),
    ]
    text = "\n\n".join(parts) + "\n"
    out_path.write_text(text)
    return text


def _main(argv=None):
    ap = argparse.ArgumentParser(description="Assemble REPORT.md from a results tree.")
    ap.add_argument("--results", required=True, type=Path, help="pulled results tree root")
    ap.add_argument("--out", required=True, type=Path, help="output REPORT.md path")
    ap.add_argument("--charts-dir", type=Path, default=None,
                     help="directory for chart SVGs (default: <out dir>/charts)")
    args = ap.parse_args(argv)

    charts_dir = args.charts_dir or (args.out.parent / "charts")
    build_report(args.results, args.out, charts_dir)
    print(f"wrote {args.out} and {len(list(charts_dir.glob('*.svg')))} charts to {charts_dir}",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(_main())
