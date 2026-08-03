#!/usr/bin/env python3
"""Hand-emit standalone SVG charts for REPORT.md — no matplotlib, no pandas.

Runs on the workstation only. Every function returns a complete, standalone
`<svg>...</svg>` document (own `xmlns`, no external references) meant to be
written straight to a `.svg` file and linked from Markdown — small, diffable,
git-committable text, per the harness brief.

## Why every chart is a fixed dark card

"Legible in both light and dark backgrounds" is the hard requirement, and a
Markdown report can be viewed on GitHub, in an editor preview, or a plain
browser — contexts this module cannot detect and does not control. Rather
than lean on `@media (prefers-color-scheme)` (real, but its propagation
through `<img src="chart.svg">` embedding varies across renderers, and this
can't be visually verified from here), every chart draws its **own** fixed
dark card: an explicit `#1a1a19` background rect plus a hairline border, with
text/marks from the dataviz skill's validated dark-surface steps. A dark
card reads fine floating on a white page (like a code block) and reads fine
on a dark page (the border delineates its edge even if the host background
is a similar dark tone) — so legibility does not depend on the host theme at
all. Colors are the dataviz skill's default categorical palette
(`references/palette.md`), dark-surface column, used as-is (already
validated for this exact surface: adjacent-pair and first-three-slots
all-pairs CVD/contrast gates both pass against `#1a1a19`) — no re-validation
needed since nothing here diverges from the reference instance.

No external fonts: the `font-family` stack is generic
(`system-ui, -apple-system, "Segoe UI", sans-serif`), so rendering never
depends on a font being installed alongside the SVG.

## Chart primitives

- `range_bar_chart` — one median tick + min–max whisker per (group, series)
  cell. This is the workhorse: wherever a number in this report is an
  aggregate across n=3 reps (overhead ratio, ten_windows phases, clip
  latency, energy), it is a median/min/max, and a whisker is the honest
  rendering of that — a plain bar would silently imply false precision.
- `line_chart` — one or more (x, y) series, optional log-x (used for the
  snapshot-sweep preroll axis, which spans 5..600 s), optional point
  annotations (used to mark the OOM kill on the snapshot-sweep chart).
- `stat_card` — a single hero number with a caption, for headline call-outs
  that are not a chart (idle-tail watts, cache-hit ratio).

All three take a `flags: dict[key, str]` for cells backed by fewer runs than
expected or a throttled rep — rendered as visible text next to the mark
(icon + label), never a color-only cue, per the "never quote a difference
smaller than the observed spread" integrity rule: a whisker this wide next
to a label saying "n=1" is the honest picture, not something to smooth over.
"""
from __future__ import annotations

import math
from xml.sax.saxutils import escape


# ---------------------------------------------------------------------------
# Palette (dataviz skill, dark-surface column — see module docstring)
# ---------------------------------------------------------------------------

SURFACE = "#1a1a19"
BORDER = "rgba(255,255,255,0.16)"
TEXT_PRIMARY = "#ffffff"
TEXT_SECONDARY = "#c3c2b7"
TEXT_MUTED = "#898781"
GRID = "#2c2c2a"
BASELINE = "#383835"

CATEGORICAL = [
    "#3987e5",  # 1 blue
    "#d95926",  # 2 orange
    "#199e70",  # 3 aqua
    "#c98500",  # 4 yellow
    "#d55181",  # 5 magenta
    "#008300",  # 6 green
    "#9085e9",  # 7 violet
    "#e66767",  # 8 red
]
STATUS = {
    "good": "#0ca30c", "warning": "#fab219",
    "serious": "#ec835a", "critical": "#d03b3b",
}

FONT = 'system-ui, -apple-system, "Segoe UI", sans-serif'


def _fmt(v, unit=""):
    if v is None:
        return "n/a"
    av = abs(v)
    if av >= 1000:
        s = f"{v:,.0f}"
    elif av >= 10:
        s = f"{v:.1f}"
    else:
        s = f"{v:.2f}"
    return f"{s}{unit}"


def _e(s) -> str:
    return escape(str(s))


class _Doc:
    """Minimal SVG element accumulator — just enough to keep the chart
    functions below readable; not a general graphics library."""

    def __init__(self, width: int, height: int, title: str):
        self.width = width
        self.height = height
        self.parts = []
        self.parts.append(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'width="{width}" height="{height}" role="img" aria-label="{_e(title)}">'
        )
        self.parts.append(f"<title>{_e(title)}</title>")
        self.parts.append(
            f'<style>text {{ font-family: {FONT}; }}</style>'
        )
        self.parts.append(
            f'<rect x="0.5" y="0.5" width="{width - 1}" height="{height - 1}" '
            f'rx="8" fill="{SURFACE}" stroke="{BORDER}" stroke-width="1"/>'
        )

    def add(self, s: str):
        self.parts.append(s)

    def text(self, x, y, s, size=12, fill=TEXT_PRIMARY, anchor="start", weight="normal"):
        self.add(
            f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" '
            f'text-anchor="{anchor}" font-weight="{weight}">{_e(s)}</text>'
        )

    def line(self, x1, y1, x2, y2, stroke, width=1, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{width}" stroke-linecap="round"{d}/>'
        )

    def circle(self, cx, cy, r, fill, ring=SURFACE, ring_w=2):
        self.add(
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r}" fill="{fill}" '
            f'stroke="{ring}" stroke-width="{ring_w}"/>'
        )

    def rect(self, x, y, w, h, fill, rx=3):
        if h < 0:
            y, h = y + h, -h
        self.add(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{max(h, 0):.2f}" rx="{rx}" fill="{fill}"/>')

    def finish(self) -> str:
        self.parts.append("</svg>")
        return "\n".join(self.parts)


def _nice_max(v: float) -> float:
    if v <= 0:
        return 1.0
    mag = 10 ** math.floor(math.log10(v))
    for step in (1, 2, 2.5, 5, 10):
        if v <= step * mag:
            return step * mag
    return 10 * mag


# ---------------------------------------------------------------------------
# range_bar_chart
# ---------------------------------------------------------------------------

def range_bar_chart(
    title: str,
    groups: list,
    series: list,
    data: dict,
    y_label: str,
    unit: str = "",
    subtitle: str = "",
    flags: dict = None,
    width: int = 760,
    height: int = 440,
    footnote: str = "",
) -> str:
    """`data[(group, series)]` -> `{"median","min","max","n"}` or `None`.

    One vertical whisker (min-max) with a median tick per cell, grouped
    along x by `groups` and colored by `series` (fixed categorical order —
    a single series draws no legend, per the marks-and-anatomy rule).
    """
    flags = flags or {}
    doc = _Doc(width, height, title)
    top = 34 if subtitle else 26
    doc.text(16, 24, title, size=15, weight="600")
    if subtitle:
        doc.text(16, top + 8, subtitle, size=12, fill=TEXT_SECONDARY)
        top += 14

    legend_h = 22 if len(series) > 1 else 0
    margin_l, margin_r = 64, 20
    margin_t = top + 22 + legend_h
    margin_b = 46 if not footnote else 62
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b

    # A cell may be `agg_io_stat`'s {"not_measurable": True, ...} shape —
    # truthy but with no "median"/"min"/"max" keys — never plotted as a bar,
    # same treatment as a missing cell but labelled distinctly (see below).
    all_max = [d["max"] for d in data.values() if d and not d.get("not_measurable")]
    ymax = _nice_max(max(all_max) * 1.15) if all_max else 1.0

    # y gridlines + axis labels
    n_ticks = 4
    for i in range(n_ticks + 1):
        val = ymax * i / n_ticks
        y = margin_t + plot_h - (val / ymax) * plot_h
        doc.line(margin_l, y, margin_l + plot_w, y, GRID, 1)
        doc.text(margin_l - 8, y + 4, _fmt(val), size=11, fill=TEXT_MUTED, anchor="end")
    doc.text(
        16, margin_t + plot_h / 2, y_label, size=11, fill=TEXT_MUTED,
        anchor="middle",
    )
    doc.parts[-1] = doc.parts[-1].replace(
        "<text ", f'<text transform="rotate(-90 16 {margin_t + plot_h / 2:.1f})" ', 1
    )
    # baseline
    doc.line(margin_l, margin_t + plot_h, margin_l + plot_w, margin_t + plot_h, BASELINE, 1.5)

    n_groups = max(len(groups), 1)
    band_w = plot_w / n_groups
    n_series = max(len(series), 1)
    bar_w = min(22, band_w / (n_series + 1) * 0.8)
    gap = (band_w - bar_w * n_series) / (n_series + 1)

    for gi, g in enumerate(groups):
        band_x0 = margin_l + gi * band_w
        for si, s in enumerate(series):
            cell = data.get((g, s))
            x = band_x0 + gap * (si + 1) + bar_w * si + bar_w / 2
            color = CATEGORICAL[si % len(CATEGORICAL)]
            if cell and cell.get("not_measurable"):
                doc.text(
                    x, margin_t + plot_h - 6, "n/m", size=10, fill=TEXT_MUTED,
                    anchor="middle",
                )
            elif cell:
                y_med = margin_t + plot_h - (cell["median"] / ymax) * plot_h
                y_min = margin_t + plot_h - (cell["min"] / ymax) * plot_h
                y_max = margin_t + plot_h - (cell["max"] / ymax) * plot_h
                doc.line(x, y_min, x, y_max, color, 3)
                doc.line(x - bar_w / 2, y_med, x + bar_w / 2, y_med, color, 4)
                lbl = flags.get((g, s))
                if lbl:
                    doc.text(x, y_max - 8, lbl, size=9.5, fill=STATUS["warning"], anchor="middle")
            else:
                doc.text(
                    x, margin_t + plot_h - 6, "–", size=12, fill=TEXT_MUTED,
                    anchor="middle",
                )
        doc.text(
            band_x0 + band_w / 2, margin_t + plot_h + 18, str(g), size=11.5,
            fill=TEXT_SECONDARY, anchor="middle",
        )

    if len(series) > 1:
        lx = margin_l
        ly = top + 18
        for si, s in enumerate(series):
            color = CATEGORICAL[si % len(CATEGORICAL)]
            doc.rect(lx, ly - 8, 10, 10, color, rx=2)
            doc.text(lx + 15, ly, str(s), size=11.5, fill=TEXT_SECONDARY)
            lx += 15 + len(str(s)) * 6.5 + 18

    if footnote:
        doc.text(16, height - 12, footnote, size=10.5, fill=TEXT_MUTED)

    return doc.finish()


# ---------------------------------------------------------------------------
# line_chart
# ---------------------------------------------------------------------------

def line_chart(
    title: str,
    x_values: list,
    series: dict,
    y_label: str,
    x_label: str = "",
    subtitle: str = "",
    log_x: bool = False,
    annotations: list = None,
    width: int = 780,
    height: int = 440,
    footnote: str = "",
) -> str:
    """`series[name]` is a list, same length as `x_values`, of `float | None`
    (`None` breaks the line — used where an arm has no data at that x, e.g.
    the snapshot arm past its OOM point). `annotations` is a list of
    `{"x","y","label","status"}` dicts drawn as a status-colored marker +
    text label (icon + label, never color alone)."""
    annotations = annotations or []
    doc = _Doc(width, height, title)
    top = 34 if subtitle else 26
    doc.text(16, 24, title, size=15, weight="600")
    if subtitle:
        doc.text(16, top + 8, subtitle, size=12, fill=TEXT_SECONDARY)
        top += 14

    legend_h = 22 if len(series) > 1 else 0
    margin_l, margin_r = 64, 24
    margin_t = top + 22 + legend_h
    margin_b = 46 if not footnote else 62
    plot_w = width - margin_l - margin_r
    plot_h = height - margin_t - margin_b

    all_y = [v for vals in series.values() for v in vals if v is not None]
    all_y += [a["y"] for a in annotations if a.get("y") is not None]
    ymax = _nice_max(max(all_y) * 1.15) if all_y else 1.0

    def y_to_px(v):
        return margin_t + plot_h - (v / ymax) * plot_h

    if log_x:
        xs_pos = [x for x in x_values if x > 0]
        lo, hi = math.log10(min(xs_pos)), math.log10(max(xs_pos))
        span = (hi - lo) or 1.0

        def x_to_px(v):
            return margin_l + (math.log10(v) - lo) / span * plot_w
    else:
        lo, hi = min(x_values), max(x_values)
        span = (hi - lo) or 1.0

        def x_to_px(v):
            return margin_l + (v - lo) / span * plot_w

    n_ticks = 4
    for i in range(n_ticks + 1):
        val = ymax * i / n_ticks
        y = y_to_px(val)
        doc.line(margin_l, y, margin_l + plot_w, y, GRID, 1)
        doc.text(margin_l - 8, y + 4, _fmt(val), size=11, fill=TEXT_MUTED, anchor="end")
    doc.line(margin_l, margin_t + plot_h, margin_l + plot_w, margin_t + plot_h, BASELINE, 1.5)

    for xv in x_values:
        x = x_to_px(xv)
        doc.text(x, margin_t + plot_h + 18, _fmt(xv), size=11, fill=TEXT_SECONDARY, anchor="middle")
    if x_label:
        doc.text(margin_l + plot_w / 2, height - (26 if footnote else 8), x_label, size=11, fill=TEXT_MUTED, anchor="middle")

    for si, (name, vals) in enumerate(series.items()):
        color = CATEGORICAL[si % len(CATEGORICAL)]
        pts = [(x_to_px(xv), y_to_px(v)) for xv, v in zip(x_values, vals) if v is not None]
        if len(pts) >= 2:
            path = "M " + " L ".join(f"{x:.2f} {y:.2f}" for x, y in pts)
            doc.add(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="2.5" '
                    f'stroke-linecap="round" stroke-linejoin="round"/>')
        for x, y in pts:
            doc.circle(x, y, 4, color)

    for a in annotations:
        x, y = x_to_px(a["x"]), y_to_px(a["y"])
        color = STATUS.get(a.get("status", "critical"), STATUS["critical"])
        doc.line(x - 6, y - 6, x + 6, y + 6, color, 2.5)
        doc.line(x - 6, y + 6, x + 6, y - 6, color, 2.5)
        doc.text(x + 10, y - 8, a["label"], size=11, fill=color, weight="600")

    if len(series) > 1:
        lx = margin_l
        ly = top + 18
        for si, name in enumerate(series):
            color = CATEGORICAL[si % len(CATEGORICAL)]
            doc.line(lx, ly - 4, lx + 14, ly - 4, color, 3)
            doc.text(lx + 20, ly, str(name), size=11.5, fill=TEXT_SECONDARY)
            lx += 24 + len(str(name)) * 6.5 + 18

    doc.text(16, margin_t + plot_h / 2, y_label, size=11, fill=TEXT_MUTED, anchor="middle")
    doc.parts[-1] = doc.parts[-1].replace(
        "<text ", f'<text transform="rotate(-90 16 {margin_t + plot_h / 2:.1f})" ', 1
    )

    if footnote:
        doc.text(16, height - 12, footnote, size=10.5, fill=TEXT_MUTED)

    return doc.finish()


# ---------------------------------------------------------------------------
# stat_card
# ---------------------------------------------------------------------------

def stat_card(label: str, value: str, sub: str = "", width: int = 320, height: int = 150) -> str:
    doc = _Doc(width, height, label)
    doc.text(20, 30, label, size=13, fill=TEXT_SECONDARY)
    doc.text(20, 90, value, size=36, weight="700")
    if sub:
        doc.text(20, height - 20, sub, size=11.5, fill=TEXT_MUTED)
    return doc.finish()
