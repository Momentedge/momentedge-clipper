#!/usr/bin/env python3
"""The benchmark scenario matrix, as data.

This module is the single source of truth for *what gets run*. It expands the
matrix in `CONTRACT.md` into a flat, ordered list of concrete runs — one dict
per run, carrying every knob the orchestrator needs — and nothing else: it
starts no process, touches no disk, and needs no ROS environment.

    from scenarios import expand, HOSTS
    runs = expand("nano", reps=3)

    python3 scenarios.py --host nano --reps 3            # the expansion, JSON
    python3 scenarios.py --host nano --reps 3 --summary  # counts + wall clock
    python3 scenarios.py --plan-file plan.json --emit-shell 7   # one run, sh vars

Each run dict is self-contained: `run_suite.sh` reads one out of the written
plan file and needs no matrix knowledge of its own. The dicts are also what
lands in `run.json` under `config`, so a result directory carries the exact
configuration that produced it.

Ordering
--------
Runs come out in priority bands — the headline arms, then the rest of the
light/mid matrix, then the long ones (the preroll sweep, every heavy arm, the
soak) — so a suite that is interrupted has finished all repetitions of what
matters most rather than a fraction of everything. Within each (band,
repetition) the arm order is shuffled from a seed derived from the repetition
number. The order is reproducible from `(host, reps)` alone, and each run
records its band, the seed and its index, so a result can be placed back in the
sequence it was measured in.

Stdlib only, Python 3.10 and 3.12.
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import re
import shlex
import sys

# ---------------------------------------------------------------------------
# Constants shared with the rest of the harness
# ---------------------------------------------------------------------------

#: Seed base for the per-repetition shuffle. Bumping it reshuffles every arm
#: order; keeping it fixed keeps a suite reproducible across re-expansions.
SEED_BASE = 20260803

#: Target write rates, MB/s (10^6 bytes), matching CONTRACT.md's bitrate axis.
BITRATES = {"light": 3.0, "mid": 20.0, "heavy": 58.0}

#: Clip codecs under test. `lz4` exists in clipper but is not on the axis.
COMPRESSIONS = ["none", "zstd"]

#: Extraction concurrency. "ncores" resolves per host.
PARALLELISM = [1, 2, "ncores"]

#: Fixed everywhere, per CONTRACT.md.
RECORDER_PROFILE = "fastwrite"
RECORDER_MAX_CACHE_SIZE = 0
CLIPPER_INTERFACE = "ros"
CLIPPER_TIME_SOURCE = "log"
CLIPPER_GRACE_SECS = 2

DEFAULT_WARMUP_S = 30
DEFAULT_MEASURE_S = 120

#: Wall-clock accounting only: process start/stop and the pre-run disk wipe.
SETUP_S = 15

#: Teardown is dominated by the recorder closing its mcap on SIGINT — writing
#: the summary and index for an unchunked multi-gigabyte file. Modelled as a
#: fixed cost plus the time to write out what was recorded, which is why the
#: heavy arms pay noticeably more for it than the light ones.
TEARDOWN_BASE_S = 25
TEARDOWN_CLOSE_MBS = 200.0

#: Sustained clip-extraction throughput used to size timeouts and the wall-clock
#: estimate. Deliberately pessimistic — a timeout that is too generous costs
#: nothing, one that is too tight aborts a clip mid-flight and voids the run.
EXTRACT_MBS = 120.0

#: Fraction of board RAM the snapshot arm's transient scope may use. The kill
#: point of an over-large cache is a result, so the bound exists to keep the
#: kill contained, not to prevent it. The fraction is applied to the board's
#: real `MemTotal` at run time — the `ram_gb` below is a nameplate figure, good
#: enough to plan with and wrong to size a memory bound with.
SNAPSHOT_MEMORY_FRACTION = 0.8

#: Priority bands. Runs are ordered band-major, so every repetition of the
#: headline arms is finished before the long ones begin: an interruption then
#: costs the expensive tail rather than half of every result. Randomisation
#: happens *within* a (band, repetition), which is what the validity of
#: comparing arms depends on — order effects are what randomisation defends
#: against, and those only matter between arms that get compared to each other.
BAND_HEADLINE = 0   # the four core scenarios at the mid bitrate
BAND_CORE = 1       # the rest of the light/mid matrix
BAND_LONG = 2       # snapshot_sweep, every heavy arm, and the soak

BAND_NAMES = {BAND_HEADLINE: "headline", BAND_CORE: "core", BAND_LONG: "long"}

#: The scenarios whose mid-bitrate arms are the headline result.
HEADLINE_SCENARIOS = {"baseline", "idle_tail", "one_clip", "ten_windows"}

GIB = 1024 ** 3
MB = 10 ** 6
NS = 10 ** 9

# ---------------------------------------------------------------------------
# Hosts
# ---------------------------------------------------------------------------

#: Everything that differs between the two boards. Nothing below this table may
#: assume a core count, a distro or a user name.
HOSTS = {
    "nano": {
        "host": "nano",
        "distro": "jazzy",
        "user": "jetson",
        "ncores": 6,
        "ram_gb": 8,
        # contention: leave two cores free, then saturate the board.
        "hog_levels": [4, 6],
        # lowpower: nvpmodel mode *names*, resolved against the board's own
        # /etc/nvpmodel.conf at run time. A name the board does not define
        # fails its runs loudly rather than measuring the wrong power budget.
        "power_modes": ["15W", "10W"],
    },
    "nx": {
        "host": "nx",
        "distro": "humble",
        "user": "momentedge",
        "ncores": 8,
        "ram_gb": 16,
        "hog_levels": [6, 8],
        "power_modes": ["15W", "10W"],
    },
}

# ---------------------------------------------------------------------------
# Scenario axes
# ---------------------------------------------------------------------------

#: Preroll sweep for the snapshot arm, seconds.
SNAPSHOT_PREROLLS_S = [5, 30, 60, 300, 600]
SNAPSHOT_POSTROLL_S = 10
#: One bitrate for the sweep. `mid` is the informative choice: the largest
#: prerolls put the required cache above both boards' MemoryMax, so the sweep
#: brackets the kill point instead of sitting entirely below or above it.
SNAPSHOT_BITRATE = "mid"

#: Split length the rollover scenario records at, and the window that has to
#: span a boundary.
ROLLOVER_SPLIT_S = 60
ROLLOVER_PREROLL_S = 45
ROLLOVER_POSTROLL_S = 45

#: Retention runs clipping against a pruner deleting splits out from under it.
#: prune-record.sh's age granularity is minutes, so the splits are short and the
#: age bound is its smallest useful value.
RETENTION_SPLIT_S = 30
RETENTION_MAX_AGE_MIN = 1
RETENTION_PRUNE_INTERVAL_S = 10

#: The ten-window pattern, shared by ten_windows / contention / lowpower /
#: retention so their numbers are directly comparable.
TEN_PREROLL_S = 10
TEN_POSTROLL_S = 60
TEN_STAGGER_MS = 1000
TEN_COUNT = 10

ONE_CLIP_PREROLL_S = 10
ONE_CLIP_POSTROLL_S = 10

#: Soak: one clip a minute for four hours.
SOAK_PERIOD_S = 60
SOAK_HOURS = 4
SOAK_PREROLL_S = 10
SOAK_POSTROLL_S = 10

#: Bitrate the single-arm scenarios run at.
SINGLE_ARM_BITRATE = "mid"

#: Where a Jetson declares its nvpmodel power modes.
NVPMODEL_CONF = "/etc/nvpmodel.conf"


def available_power_modes(path):
    """The nvpmodel mode NAMEs a board defines, or None if they can't be read.

    None is not "this board has no modes" — it is "we are not looking at that
    board's configuration", which is the normal case when expanding on a
    workstation to plan or cost a suite. The caller must keep those apart:
    filtering the requested modes against None would silently drop every
    lowpower arm from a plan generated off-target.
    """
    try:
        with open(path, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
    except OSError:
        return None
    names = []
    for block in re.finditer(r"<\s*POWER_MODEL\s+([^>]*)>", text):
        attrs = dict(re.findall(r"(\w+)\s*=\s*([^\s>]+)", block.group(1)))
        if "NAME" in attrs:
            names.append(attrs["NAME"])
    return names or None


# ---------------------------------------------------------------------------
# Arm construction
# ---------------------------------------------------------------------------


def _trigger(pattern, count, preroll_s, postroll_s, stagger_ms=0, period_s=0):
    return {
        "pattern": pattern,
        "count": count,
        "preroll_ns": int(preroll_s * NS),
        "postroll_ns": int(postroll_s * NS),
        "preroll_s": preroll_s,
        "postroll_s": postroll_s,
        "stagger_ms": stagger_ms,
        "period_s": period_s,
    }


NO_TRIGGER = _trigger("none", 0, 0, 0)


def _arm(scenario, bitrate, **kw):
    """One point of the matrix, before host resolution and repetition."""
    arm = {
        "scenario": scenario,
        # Distinguishes points of a scenario's own sub-axis (preroll, hog
        # level, power mode). Empty when the scenario has none.
        "variant": "",
        "bitrate": bitrate,
        "clip_compression": None,      # None = the axis does not apply
        "extract_parallelism": None,   # None = the axis does not apply
        "run_clipper": True,
        "trigger": NO_TRIGGER,
        "snapshot_mode": False,
        "snapshot_arm": None,
        "max_bag_duration": 0,
        "hog_cores": 0,
        "power_mode": None,
        # How the lowpower variants were chosen: "declared" from the host
        # table, or "discovered" from the board's own nvpmodel.conf. A plan
        # built off-target must say so rather than look identical to one
        # built on the board.
        "power_modes_source": None,
        "prune": False,
        "synth": None,
        "warmup_s": DEFAULT_WARMUP_S,
        "measure_s": DEFAULT_MEASURE_S,
        "soak": False,
        # Repetitions this arm takes part in. None means every repetition; an
        # arm that measures drift over hours is answered once, not three times.
        "max_reps": None,
    }
    arm.update(kw)
    return arm


def _arms(host, nvpmodel_conf=None):
    """Every arm for one host, in matrix order (the pre-shuffle order)."""
    ncores = host["ncores"]
    arms = []

    # baseline — the denominator: the recorder alone. No clipper, so neither
    # the compression nor the parallelism axis has any meaning here.
    for bitrate in BITRATES:
        arms.append(_arm("baseline", bitrate, run_clipper=False))

    # idle_tail — clipper attached to the same recording, cutting nothing. The
    # difference from baseline is the cost of the tail itself.
    for bitrate in BITRATES:
        arms.append(_arm("idle_tail", bitrate))

    # one_clip — the cheapest clip there is, over both codecs. Parallelism
    # cannot show anything with a single window, so it stays at 1.
    for bitrate in BITRATES:
        for comp in COMPRESSIONS:
            arms.append(_arm(
                "one_clip", bitrate,
                clip_compression=comp, extract_parallelism=1,
                trigger=_trigger("single", 1, ONE_CLIP_PREROLL_S, ONE_CLIP_POSTROLL_S),
            ))

    # ten_windows — the headline arm, and the only one crossed over all three
    # axes: ten overlapping windows are what makes parallelism observable.
    for bitrate in BITRATES:
        for comp in COMPRESSIONS:
            for par in PARALLELISM:
                arms.append(_arm(
                    "ten_windows", bitrate,
                    clip_compression=comp,
                    extract_parallelism=ncores if par == "ncores" else par,
                    trigger=_trigger("ten_overlap", TEN_COUNT, TEN_PREROLL_S,
                                     TEN_POSTROLL_S, stagger_ms=TEN_STAGGER_MS),
                ))

    # snapshot_sweep — clipper cutting from a continuous recording against
    # rosbag2's own snapshot mode, over a preroll sweep. The snapshot arm holds
    # the whole window in the recorder's cache, so it is the arm that can be
    # OOM-killed; the continuous arm is the reference at the same preroll.
    for preroll_s in SNAPSHOT_PREROLLS_S:
        window_s = preroll_s + SNAPSHOT_POSTROLL_S
        cache = int(window_s * BITRATES[SNAPSHOT_BITRATE] * MB)
        for snap_arm in ("continuous", "snapshot"):
            # The snapshot arm has no clipper and therefore no `Recorded` to
            # wait on: the orchestrator itself calls the recorder's snapshot
            # service at `anchor + postroll` and times the call.
            pattern = "single" if snap_arm == "continuous" else "snapshot_service"
            trig = _trigger(pattern, 1, preroll_s, SNAPSHOT_POSTROLL_S)
            arms.append(_arm(
                "snapshot_sweep", SNAPSHOT_BITRATE,
                variant=f"{'snap' if snap_arm == 'snapshot' else 'cont'}-pre{preroll_s}",
                snapshot_arm=snap_arm,
                snapshot_mode=(snap_arm == "snapshot"),
                # In snapshot mode the recorder holds the window; clipper is
                # not the cutter and does not run.
                run_clipper=(snap_arm == "continuous"),
                clip_compression="zstd" if snap_arm == "continuous" else None,
                extract_parallelism=1 if snap_arm == "continuous" else None,
                trigger=trig,
                # The recorder must already hold `preroll_s` of history before
                # the trigger, in both arms, or the sweep measures nothing.
                warmup_s=preroll_s + DEFAULT_WARMUP_S,
                max_cache_size=cache if snap_arm == "snapshot" else RECORDER_MAX_CACHE_SIZE,
            ))

    # rollover — one window straddling a split boundary, so the clip is
    # assembled from two recordings.
    arms.append(_arm(
        "rollover", SINGLE_ARM_BITRATE,
        clip_compression="zstd", extract_parallelism=1,
        max_bag_duration=ROLLOVER_SPLIT_S,
        trigger=_trigger("single", 1, ROLLOVER_PREROLL_S, ROLLOVER_POSTROLL_S),
        # One full split plus the preroll must be on disk before the trigger.
        warmup_s=ROLLOVER_SPLIT_S + ROLLOVER_PREROLL_S + 15,
    ))

    # retention — the ten-window pattern while the pruner deletes splits the
    # windows still reach into.
    arms.append(_arm(
        "retention", SINGLE_ARM_BITRATE,
        clip_compression="zstd", extract_parallelism=1,
        max_bag_duration=RETENTION_SPLIT_S,
        prune=True,
        trigger=_trigger("ten_overlap", TEN_COUNT, TEN_PREROLL_S,
                         TEN_POSTROLL_S, stagger_ms=TEN_STAGGER_MS),
        # Splits must be old enough for the pruner to bite during the measure.
        warmup_s=90,
    ))

    # contention — ten_windows with the CPU taken away, at the parallelism that
    # wants the cores most.
    for cores in host["hog_levels"]:
        arms.append(_arm(
            "contention", SINGLE_ARM_BITRATE,
            variant=f"hog{cores}",
            clip_compression="zstd", extract_parallelism=ncores,
            hog_cores=cores,
            trigger=_trigger("ten_overlap", TEN_COUNT, TEN_PREROLL_S,
                             TEN_POSTROLL_S, stagger_ms=TEN_STAGGER_MS),
        ))

    # lowpower — ten_windows at a reduced nvpmodel budget.
    #
    # The requested modes are asserted per host, but not every board defines
    # them: the Orin Nano offers 15W/25W/MAXN_SUPER and has no 10W at all, so
    # scheduling a 10W arm there produced three runs that could never succeed
    # and that `--resume` would retry forever. A mode the board does not have
    # is therefore not scheduled, rather than scheduled and failed.
    modes, modes_source = host["power_modes"], "declared"
    if nvpmodel_conf:
        found = available_power_modes(nvpmodel_conf)
        if found is None:
            # Asked to discover, but the file was unreadable. Say so rather
            # than silently emitting the declared set as if it were verified.
            modes_source = f"declared (unverified: {nvpmodel_conf} unreadable)"
        else:
            modes = [m for m in host["power_modes"] if m in found]
            modes_source = f"discovered from {nvpmodel_conf}"
    for mode in modes:
        arms.append(_arm(
            "lowpower", SINGLE_ARM_BITRATE,
            variant=mode,
            clip_compression="zstd", extract_parallelism=1,
            power_mode=mode,
            power_modes_source=modes_source,
            trigger=_trigger("ten_overlap", TEN_COUNT, TEN_PREROLL_S,
                             TEN_POSTROLL_S, stagger_ms=TEN_STAGGER_MS),
        ))

    return arms


def _soak_arm(host):
    """The four-hour drift arm, opt-in because it costs a night on its own.

    It runs at the light bitrate and exactly once. Four hours of continuous
    recording at `mid` is ~290 GB before a single clip is written, which no
    board here has the disk for; drift in RSS, file descriptors and latency is
    visible at any bitrate, and repeating a four-hour run three times answers
    nothing the first one did not.
    """
    count = SOAK_HOURS * 3600 // SOAK_PERIOD_S
    return _arm(
        "soak", "light",
        clip_compression="zstd", extract_parallelism=1,
        trigger=_trigger("periodic", count, SOAK_PREROLL_S, SOAK_POSTROLL_S,
                         period_s=SOAK_PERIOD_S),
        measure_s=SOAK_HOURS * 3600,
        soak=True,
        max_reps=1,
    )


# ---------------------------------------------------------------------------
# Derived quantities
# ---------------------------------------------------------------------------


def _clip_seconds(trigger):
    """Total clip-seconds a run's trigger pattern asks for."""
    return trigger["count"] * (trigger["preroll_s"] + trigger["postroll_s"])


def _pattern_span_s(trigger):
    """Seconds from the first trigger to the last clip becoming cuttable."""
    if trigger["count"] == 0:
        return 0.0
    if trigger["period_s"]:
        span = (trigger["count"] - 1) * trigger["period_s"]
    else:
        span = (trigger["count"] - 1) * trigger["stagger_ms"] / 1000.0
    return span + trigger["postroll_s"] + CLIPPER_GRACE_SECS


def _derive(arm, host):
    """Fill in everything computed from the arm and the host."""
    mbs = BITRATES[arm["bitrate"]]
    trigger = arm["trigger"]

    clip_bytes = int(_clip_seconds(trigger) * mbs * MB)
    extract_s = clip_bytes / (EXTRACT_MBS * MB)
    span_s = _pattern_span_s(trigger)

    # The measure window is a floor, not a cap: it never ends while a clip is
    # still being cut, or the run would record a truncated extraction as data.
    measure_est_s = max(arm["measure_s"], span_s + extract_s + 10)

    # The driver's own deadline. Generous on purpose — see EXTRACT_MBS.
    trigger_timeout_s = int(span_s + 2 * extract_s + 120) if trigger["count"] else 0

    record_bytes = int((arm["warmup_s"] + measure_est_s) * mbs * MB)
    teardown_s = TEARDOWN_BASE_S + record_bytes / (TEARDOWN_CLOSE_MBS * MB)

    # `clip_compression` / `extract_parallelism` are the *axis*: None means the
    # scenario does not vary over them, which is what the run_id records. A
    # clipper that runs still needs a concrete value for each, so the effective
    # settings are derived separately and are what actually reaches the flags.
    effective_comp = (arm["clip_compression"] or "zstd") if arm["run_clipper"] else None
    effective_par = (arm["extract_parallelism"] or 1) if arm["run_clipper"] else None

    # One clip's worth of bytes. With clip pruning on, this — not the total
    # over every window — is what bounds the clipped directory: a clip is
    # unlinked once it has been announced and its size recorded, so only the
    # ones being cut concurrently are on disk at any moment.
    window_bytes = int((trigger["preroll_s"] + trigger["postroll_s"]) * mbs * MB)
    concurrent = max(2, (effective_par or 1) + 1)
    pruned_clip_bytes = min(clip_bytes, concurrent * window_bytes)

    return {
        "bitrate_target_mbs": mbs,
        "effective_clip_compression": effective_comp,
        "effective_extract_parallelism": effective_par,
        "window_bytes": window_bytes,
        "est_clip_bytes_pruned": pruned_clip_bytes,
        "est_disk_bytes_pruned": record_bytes + pruned_clip_bytes,
        # The window the snapshot arm's recorder must hold in its cache. The
        # cache is sized from a *measured* bitrate at run time, not from the
        # nominal target, so only the window length is fixed here.
        "snapshot_window_s": (trigger["preroll_s"] + trigger["postroll_s"])
        if arm["snapshot_mode"] else 0,
        "est_record_bytes": record_bytes,
        # Sized without a compression discount: the disk guard must hold for
        # the codec that shrinks nothing.
        "est_clip_bytes": clip_bytes,
        "est_disk_bytes": record_bytes + clip_bytes,
        "est_measure_s": round(measure_est_s, 1),
        "est_teardown_s": round(teardown_s, 1),
        "est_wall_s": round(arm["warmup_s"] + measure_est_s + SETUP_S + teardown_s, 1),
        "trigger_timeout_s": trigger_timeout_s,
        # clipper keeps a finished recording indexed this long, which has to
        # outlast the longest preroll reaching back into it.
        "watch_old_files_duration": max(600, int(trigger["preroll_s"] * 2) + 600),
        "expect_triggers": trigger["count"],
        "expect_clips": bool(trigger["count"]) and arm["run_clipper"],
        # A planning figure only, from the board's nameplate RAM. The bound
        # actually applied to the scope is computed from the real MemTotal at
        # run time, which is a good deal smaller than the nameplate.
        "memory_max_planned_bytes": int(host["ram_gb"] * SNAPSHOT_MEMORY_FRACTION * GIB)
        if arm["snapshot_mode"] else 0,
        "memory_max_fraction": SNAPSHOT_MEMORY_FRACTION if arm["snapshot_mode"] else 0,
    }


def _run_id(arm, host, rep):
    comp = arm["clip_compression"] or "na"
    par = arm["extract_parallelism"]
    par = f"par{par}" if par is not None else "parna"
    scenario = arm["scenario"]
    if arm["variant"]:
        scenario = f"{scenario}-{arm['variant']}"
    return f"{host['host']}__{scenario}__{arm['bitrate']}__{comp}__{par}__rep{rep}"


def _materialise(arm, host, rep, seed, order_index):
    run = dict(arm)
    run.update(_derive(arm, host))
    run.update({
        "run_id": _run_id(arm, host, rep),
        "band": band_of(arm),
        "band_name": BAND_NAMES[band_of(arm)],
        "host": host["host"],
        "distro": host["distro"],
        "user": host["user"],
        "ncores": host["ncores"],
        "ram_gb": host["ram_gb"],
        "rep": rep,
        "order_seed": seed,
        "order_index": order_index,
        "recorder_profile": RECORDER_PROFILE,
        "clipper_interface": CLIPPER_INTERFACE,
        "clipper_time_source": CLIPPER_TIME_SOURCE,
        "clipper_grace_secs": CLIPPER_GRACE_SECS,
    })
    run.setdefault("max_cache_size", RECORDER_MAX_CACHE_SIZE)
    return run


# ---------------------------------------------------------------------------
# Expansion
# ---------------------------------------------------------------------------


def band_of(arm):
    """Which priority band an arm belongs to.

    The long band is anything that costs hours or writes tens of gigabytes:
    the whole preroll sweep, every heavy-bitrate arm, and the soak.
    """
    if arm["soak"] or arm["scenario"] == "snapshot_sweep" or arm["bitrate"] == "heavy":
        return BAND_LONG
    if arm["bitrate"] == SINGLE_ARM_BITRATE and arm["scenario"] in HEADLINE_SCENARIOS:
        return BAND_HEADLINE
    return BAND_CORE


def arm_identity(arm):
    """A stable name for an arm, independent of where it sits in the matrix."""
    return "|".join(str(x) for x in (
        arm["scenario"], arm["variant"], arm["bitrate"],
        arm["clip_compression"], arm["extract_parallelism"]))


def shuffled(slots, arms, seed):
    """`slots` in a deterministic pseudo-random order derived from `seed`.

    Two properties, both load-bearing, and the second was learned the hard way:

    1. Derived from SHA-256 rather than `random.shuffle`, so the order depends
       on the seed alone and not on the CPython release that expanded the
       matrix. The two boards run different Python versions and must agree.

    2. Keyed on each arm's IDENTITY, not on its position in the list. Keyed on
       position, "the order is reproducible from (host, reps)" holds only while
       the matrix never changes: adding or removing any arm that is not at the
       tail reshuffles almost everything after it — measured at 120-122 of 135
       runs moving when a single mid-list or leading arm was removed. A resumed
       or partially re-run suite would then no longer line up with what
       preceded it, and nothing would announce that. Keyed on identity, an arm
       keeps its place no matter what else joins or leaves the matrix.
    """
    def key(slot):
        return hashlib.sha256(
            f"{seed}:{arm_identity(arms[slot])}".encode("utf-8")).digest()
    return sorted(slots, key=key)


def expand(host_name, reps=3, only=None, include_soak=False, nvpmodel_conf=None):
    """The concrete, ordered list of runs for one host.

    Runs come out band-major (see `BAND_HEADLINE`), and within each
    (band, repetition) the arms are shuffled from `SEED_BASE + rep` (see
    `shuffled`), so the order is reproducible from the arguments alone
    on any Python 3 and is recorded in every run. `only` is an fnmatch
    pattern applied to `run_id` *after* ordering, so filtering a subset does
    not change the order the remaining runs would have been measured in.
    """
    host = HOSTS[host_name]
    arms = _arms(host, nvpmodel_conf=nvpmodel_conf)
    if include_soak:
        arms.append(_soak_arm(host))

    by_band = {}
    for slot, arm in enumerate(arms):
        by_band.setdefault(band_of(arm), []).append(slot)

    runs = []
    index = 0
    for band in sorted(by_band):
        for rep in range(1, reps + 1):
            seed = SEED_BASE + rep
            for slot in shuffled(by_band[band], arms, seed):
                arm = arms[slot]
                if arm["max_reps"] is not None and rep > arm["max_reps"]:
                    continue
                runs.append(_materialise(arm, host, rep, seed, index))
                index += 1

    if only:
        runs = [r for r in runs if fnmatch.fnmatch(r["run_id"], only)]

    ids = [r["run_id"] for r in runs]
    if len(set(ids)) != len(ids):
        dupes = sorted({i for i in ids if ids.count(i) > 1})
        raise ValueError(f"run_id collision: {dupes}")
    return runs


def summarise(runs):
    """Counts and wall clock, grouped the way a run plan is read."""
    by_scenario = {}
    for r in runs:
        s = by_scenario.setdefault(r["scenario"], {"runs": 0, "wall_s": 0.0, "peak_disk_bytes": 0})
        s["runs"] += 1
        s["wall_s"] += r["est_wall_s"]
        s["peak_disk_bytes"] = max(s["peak_disk_bytes"], r["est_disk_bytes"])
    return {
        "runs": len(runs),
        "arms": len({r["run_id"].rsplit("__rep", 1)[0] for r in runs}),
        "reps": len({r["rep"] for r in runs}),
        "est_wall_s": round(sum(r["est_wall_s"] for r in runs), 1),
        "est_wall_h": round(sum(r["est_wall_s"] for r in runs) / 3600.0, 2),
        "peak_disk_bytes": max((r["est_disk_bytes"] for r in runs), default=0),
        "by_scenario": {k: {
            "runs": v["runs"],
            "est_wall_h": round(v["wall_s"] / 3600.0, 2),
            "peak_disk_gb": round(v["peak_disk_bytes"] / GIB, 1),
        } for k, v in sorted(by_scenario.items())},
    }


# ---------------------------------------------------------------------------
# Shell emission
# ---------------------------------------------------------------------------

def _sh(value):
    if value is None or value is False:
        return "''"
    if value is True:
        return "1"
    return shlex.quote(str(value))


def emit_shell(run, out=sys.stdout):
    """One run as `KEY=value` lines for `run_suite.sh` to source.

    Every value is shell-quoted, so nothing here can be re-interpreted by the
    shell. Booleans render as `1` / empty, which `[[ -n ]]` reads directly.
    """
    trigger = run["trigger"]
    fields = [
        ("RUN_ID", run["run_id"]),
        ("BAND", run["band"]),
        ("BAND_NAME", run["band_name"]),
        ("SCENARIO", run["scenario"]),
        ("VARIANT", run["variant"]),
        ("HOST", run["host"]),
        ("DISTRO", run["distro"]),
        ("NCORES", run["ncores"]),
        ("REP", run["rep"]),
        ("ORDER_SEED", run["order_seed"]),
        ("ORDER_INDEX", run["order_index"]),
        ("BITRATE_LABEL", run["bitrate"]),
        ("BITRATE_TARGET_MBS", run["bitrate_target_mbs"]),
        ("RUN_CLIPPER", run["run_clipper"]),
        ("CLIP_COMPRESSION", run["effective_clip_compression"]),
        ("EXTRACT_PARALLELISM", run["effective_extract_parallelism"]),
        ("CLIPPER_INTERFACE", run["clipper_interface"]),
        ("CLIPPER_TIME_SOURCE", run["clipper_time_source"]),
        ("CLIPPER_GRACE_SECS", run["clipper_grace_secs"]),
        ("WATCH_OLD_FILES_DURATION", run["watch_old_files_duration"]),
        ("RECORDER_PROFILE", run["recorder_profile"]),
        ("MAX_CACHE_SIZE", run["max_cache_size"]),
        ("MAX_BAG_DURATION", run["max_bag_duration"]),
        ("SNAPSHOT_MODE", run["snapshot_mode"]),
        ("SNAPSHOT_ARM", run["snapshot_arm"]),
        ("MEMORY_MAX_PLANNED_BYTES", run["memory_max_planned_bytes"]),
        ("MEMORY_MAX_FRACTION", run["memory_max_fraction"]),
        ("SNAPSHOT_WINDOW_S", run["snapshot_window_s"]),
        ("TRIGGER_PATTERN", trigger["pattern"]),
        ("TRIGGER_COUNT", trigger["count"]),
        ("PREROLL_NS", trigger["preroll_ns"]),
        ("POSTROLL_NS", trigger["postroll_ns"]),
        ("PREROLL_S", trigger["preroll_s"]),
        ("POSTROLL_S", trigger["postroll_s"]),
        ("STAGGER_MS", trigger["stagger_ms"]),
        ("PERIOD_S", trigger["period_s"]),
        ("TRIGGER_TIMEOUT_S", run["trigger_timeout_s"]),
        ("HOG_CORES", run["hog_cores"]),
        ("POWER_MODE", run["power_mode"]),
        ("PRUNE", run["prune"]),
        ("PRUNE_MAX_AGE_MIN", RETENTION_MAX_AGE_MIN),
        ("PRUNE_INTERVAL_S", RETENTION_PRUNE_INTERVAL_S),
        ("SYNTH_TOPICS", (run["synth"] or {}).get("topics")),
        ("SYNTH_SIZE", (run["synth"] or {}).get("size")),
        ("SYNTH_RATE", (run["synth"] or {}).get("rate")),
        ("WARMUP_S", run["warmup_s"]),
        ("MEASURE_S", run["measure_s"]),
        ("EST_MEASURE_S", run["est_measure_s"]),
        ("EST_RECORD_BYTES", run["est_record_bytes"]),
        ("EST_CLIP_BYTES", run["est_clip_bytes"]),
        ("EST_DISK_BYTES", run["est_disk_bytes"]),
        ("EST_DISK_BYTES_PRUNED", run["est_disk_bytes_pruned"]),
        ("WINDOW_BYTES", run["window_bytes"]),
        ("EST_WALL_S", run["est_wall_s"]),
        ("EXPECT_TRIGGERS", run["expect_triggers"]),
        ("EXPECT_CLIPS", run["expect_clips"]),
        ("IS_SOAK", run["soak"]),
    ]
    for key, value in fields:
        print(f"{key}={_sh(value)}", file=out)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Expand the benchmark scenario matrix into concrete runs.",
        epilog="Prints the expansion as JSON on stdout unless --summary or "
               "--emit-shell is given.",
    )
    parser.add_argument("--host", choices=sorted(HOSTS), help="which board to expand for")
    parser.add_argument("--reps", type=int, default=3, help="repetitions per arm (default 3)")
    parser.add_argument("--only", help="fnmatch pattern filtering run_id")
    parser.add_argument("--include-soak", action="store_true",
                        help="add the 4 h soak arm (excluded by default)")
    parser.add_argument("--nvpmodel-conf", metavar="PATH",
                        help="filter the lowpower arms against the power modes this "
                             "file declares. Pass it only when expanding ON the "
                             "target board — run_suite.sh does. Expanding one host's "
                             "matrix while sitting on the other would otherwise "
                             "filter it against the wrong board's modes.")
    parser.add_argument("--summary", action="store_true",
                        help="print counts and estimated wall clock instead of the runs")
    parser.add_argument("--plan-file", help="read an already-written expansion instead of expanding")
    parser.add_argument("--emit-shell", type=int, metavar="INDEX",
                        help="print run INDEX of the plan as shell assignments")
    args = parser.parse_args(argv)

    if args.plan_file:
        with open(args.plan_file, encoding="utf-8") as fh:
            plan = json.load(fh)
        runs = plan["runs"]
    else:
        if not args.host:
            parser.error("--host is required unless --plan-file is given")
        if args.reps < 1:
            parser.error("--reps must be at least 1")
        runs = expand(args.host, reps=args.reps, only=args.only,
                      include_soak=args.include_soak,
                      nvpmodel_conf=args.nvpmodel_conf)

    if args.emit_shell is not None:
        if not 0 <= args.emit_shell < len(runs):
            parser.error(f"--emit-shell {args.emit_shell} out of range (0..{len(runs) - 1})")
        emit_shell(runs[args.emit_shell])
        return 0

    if args.summary:
        json.dump(summarise(runs), sys.stdout, indent=2)
        print()
        return 0

    json.dump({
        "host": args.host or runs[0]["host"] if runs else args.host,
        "reps": args.reps,
        "seed_base": SEED_BASE,
        "summary": summarise(runs),
        "runs": runs,
    }, sys.stdout, indent=2)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
