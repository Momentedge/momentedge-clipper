#!/usr/bin/env python3
"""Trigger publisher and Recorded-latency matcher for the clipper benchmark
harness.

Publishes momentedge_msgs/Trigger on /events/momentedge/trigger and matches
the resulting momentedge_msgs/Recorded announcements back to them by name,
writing one JSON object per trigger to --out as each trigger's state becomes
known. Four send patterns: a single trigger, ten overlapping ones staggered
--stagger-ms apart, a --count of them --period-s apart for the soak
scenario, or none at all (still subscribing, to confirm no clips happen). See
../CONTRACT.md for the on-disk schema and the CLI this implements, and
../../README.md ("Time source: log or publish" and the trigger interface
table) for what a Trigger message means.

Every trigger this driver sends carries trigger_time = 0. The benchmark runs
clipper with `--interface ros --time-source log`, and in that cell clipper
anchors the clip window on its own subscription instant and rejects — logs an
error, cuts no clip — any trigger whose trigger_time is non-zero.
`trigger_time` is read only under `--interface ros --time-source publish`,
which this harness does not use; see
../../README.md#time-source-log-or-publish and
../../examples/trigger-pub/README.md for the one cell where a producer would
set it.

Python 3 stdlib + rclpy only (no third-party packages), and importing rclpy /
momentedge_msgs is deferred past argument parsing so `--help` and a plain
syntax check both work without a ROS environment sourced.
"""

import argparse
import json
import os
import signal
import sys
import time

TRIGGER_TOPIC = "/events/momentedge/trigger"
RECORDED_TOPIC = "/events/momentedge/recorded"

# How many triggers --pattern ten_overlap fires. Fixed, not a CLI knob: the
# ten_windows scenario (see ../README.md) is defined as exactly ten.
TEN_OVERLAP_COUNT = 10

# Mirrors MAX_TRIGGER_NAME_LEN and validate_name in crates/clipper/src/main.rs.
# A name clipper would reject is logged at error! and dropped silently there
# (no clip, no Recorded) -- checking the same rules here means a bad name
# fails fast, before any ROS I/O, instead of surfacing as a mystery unmatched
# trigger at --timeout-s.
MAX_TRIGGER_NAME_LEN = 128


def _validate_name(name):
    """Raise ValueError if clipper's validate_name (main.rs) would reject
    `name` -- non-empty, at most MAX_TRIGGER_NAME_LEN bytes, and safe to embed
    in the clip pathname `<anchor_ns>_<name>.mcap`."""
    if not name:
        raise ValueError("trigger name is empty")
    if len(name.encode("utf-8")) > MAX_TRIGGER_NAME_LEN:
        raise ValueError(f"trigger name {name!r} exceeds {MAX_TRIGGER_NAME_LEN} bytes")
    if "\0" in name:
        raise ValueError(f"trigger name {name!r} contains a NUL byte")
    if "/" in name or "\\" in name:
        raise ValueError(f"trigger name {name!r} contains a path separator")
    if name.startswith("."):
        raise ValueError(f"trigger name {name!r} starts with a dot")
    if ".." in name:
        raise ValueError(f"trigger name {name!r} contains '..'")


def _default_name_prefix():
    # Unique per invocation (pid + wall time) so trigger names don't collide
    # across separate driver runs that share one clipper --out-dir, without
    # requiring the orchestrator to thread a run_id through.
    return f"bench-{os.getpid()}-{int(time.time())}"


def _plan_names(pattern, name_prefix, count):
    """The ordered list of trigger names this run will send, given --pattern.
    Numbered (rather than left bare) so ordering and identity survive
    independent of send timing; the zero-padding width for `periodic` is
    sized to `count` so names stay lexicographically sortable whether `count`
    is the soak's 240 or a short validation run's 5."""
    if pattern == "single":
        return [name_prefix]
    if pattern == "ten_overlap":
        return [f"{name_prefix}-{i:02d}" for i in range(TEN_OVERLAP_COUNT)]
    if pattern == "periodic":
        width = max(2, len(str(max(count - 1, 0))))
        return [f"{name_prefix}-{i:0{width}d}" for i in range(count)]
    assert pattern == "none"
    return []


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Publish momentedge_msgs/Trigger and match the resulting "
        "Recorded announcements, writing triggers.jsonl."
    )
    p.add_argument(
        "--pattern",
        required=True,
        choices=("single", "ten_overlap", "none", "periodic"),
        help="single: one trigger. ten_overlap: ten triggers --stagger-ms "
        "apart. none: publish nothing, but still subscribe, to confirm no "
        "clips happen. periodic: --count triggers --period-s apart, for the "
        "soak scenario.",
    )
    p.add_argument(
        "--preroll-ns", required=True, type=int, help="Trigger.preroll, nanoseconds"
    )
    p.add_argument(
        "--postroll-ns", required=True, type=int, help="Trigger.postroll, nanoseconds"
    )
    p.add_argument("--out", required=True, help="triggers.jsonl path to write")
    p.add_argument(
        "--stagger-ms",
        type=int,
        default=1000,
        help="gap between successive triggers under --pattern ten_overlap",
    )
    p.add_argument(
        "--period-s",
        type=float,
        default=0.0,
        help="gap between successive triggers under --pattern periodic "
        "(required, > 0, for that pattern)",
    )
    p.add_argument(
        "--count",
        type=int,
        default=0,
        help="number of triggers to send under --pattern periodic "
        "(required, > 0, for that pattern)",
    )
    p.add_argument(
        "--progress-every",
        type=int,
        default=10,
        help="print a sent/matched progress line to stderr every this many "
        "state changes (sends or matches) -- a multi-hour periodic run is "
        "otherwise silent and indistinguishable from a hung one",
    )
    p.add_argument(
        "--timeout-s",
        type=float,
        default=None,
        help="give up waiting once this many seconds have elapsed since the "
        "first trigger (default: postroll_s + 120, or for --pattern "
        "periodic, (count - 1) * period_s + postroll_s + 120 -- the full "
        "schedule plus a settling margin for the last trigger's own window "
        "and clipper's extraction/grace time)",
    )
    p.add_argument(
        "--discovery-timeout-s",
        type=float,
        default=30.0,
        help="give up waiting for the subscriber count to stabilize after "
        "this long (DDS match is asynchronous; publishing into an unmatched "
        "reliable subscription silently drops the message rather than "
        "queuing it). Exceeding it with zero subscribers ever seen, under "
        "--pattern single/ten_overlap/periodic, is a hard failure: no "
        "subscriber means clipper is probably not running, and sending into "
        "the void would just report every trigger as a mysteriously "
        "unmatched one.",
    )
    p.add_argument(
        "--settle-s",
        type=float,
        default=1.5,
        help="once at least one subscriber is seen on --trigger-topic, wait "
        "until the count has been unchanged for this long before publishing "
        "the first trigger. `ros2 bag record --all` subscribes to the "
        "trigger topic too (it records everything), so a bare count of 1 "
        "can be the recorder alone, seen before clipper's own subscription "
        "has matched -- publishing into that window drops the trigger on "
        "the floor under VOLATILE QoS. Waiting for the count to stop "
        "climbing needs no advance knowledge of how many subscribers to "
        "expect, and degrades correctly when only one will ever exist.",
    )
    p.add_argument(
        "--expect-subscribers",
        type=int,
        default=None,
        help="if a caller knows exactly how many subscribers to expect, stop "
        "waiting as soon as this many are seen rather than waiting out the "
        "full --settle-s window once reached. Purely an optimization -- "
        "--settle-s stabilization is always the underlying safety net, on "
        "or off this flag.",
    )
    p.add_argument(
        "--name-prefix",
        default=None,
        help="trigger name prefix; must be unique across any runs sharing "
        "one clipper --out-dir (default: bench-<pid>-<unix time>)",
    )
    p.add_argument(
        "--description", default="", help="Trigger.description to send"
    )
    p.add_argument("--trigger-topic", default=TRIGGER_TOPIC)
    p.add_argument("--recorded-topic", default=RECORDED_TOPIC)
    p.add_argument("--node-name", default="bench_trigger_driver")
    args = p.parse_args(argv)

    if args.timeout_s is None:
        if args.pattern == "periodic":
            # The full schedule (last trigger fires at (count-1)*period_s),
            # plus that trigger's own postroll wait, plus a settling margin
            # for clipper's extraction and grace time -- the same "+120"
            # margin the other patterns use, not a per-trigger cost repeated
            # over the run. run_suite.sh always passes an explicit
            # --timeout-s sized the same way (scenarios.py's
            # trigger_timeout_s) for the real soak run; this default only
            # matters for a standalone invocation.
            args.timeout_s = (
                max(args.count - 1, 0) * args.period_s
                + args.postroll_ns / 1e9
                + 120.0
            )
        else:
            args.timeout_s = args.postroll_ns / 1e9 + 120.0
    if args.name_prefix is None:
        args.name_prefix = _default_name_prefix()
    return args


def _sum_sizes(paths):
    """Best-effort summed on-disk size of `paths`. A stat failure is logged
    and contributes 0 rather than aborting the match -- the latency and file
    list are the primary result; the byte count is secondary."""
    total = 0
    for path in paths:
        try:
            total += os.stat(path).st_size
        except OSError as e:
            print(f"driver: warning: could not stat {path}: {e}", file=sys.stderr)
    return total


def _write_snapshot(out_path, order, records, anomalies):
    """Atomically replace `out_path` with the full, currently-known state --
    one JSON object per line, in send order, followed by any anomaly lines --
    rather than appending.

    Rewriting the file on every state change means it always holds a
    complete, self-consistent snapshot: a trigger's line is written with null
    recorded_ns/latency_ns/files/bytes the moment it is sent and rewritten in
    place once its Recorded arrives, so a process killed between writes
    leaves the previous complete snapshot on disk rather than a half-written
    line or a trigger's fields split across two records. The
    write-temp-then-rename is the same atomic-publish pattern
    crates/clipper/src/clip.rs uses for clip files.

    Cost note: this is O(len(order)) per call and so O(n^2) over a whole run
    of n triggers, each of which rewrites at least twice (sent, matched).
    That is nothing at the soak scenario's n=240 (roughly 10^5 total line
    writes of a ~240-line file over 4 hours) but would not scale to a
    --count in the thousands -- if that is ever needed, this wants to become
    a real per-trigger append/update scheme rather than a bigger n here.
    """
    directory = os.path.dirname(os.path.abspath(out_path)) or "."
    tmp_path = f"{out_path}.tmp.{os.getpid()}"
    with open(tmp_path, "w") as f:
        for name in order:
            f.write(json.dumps(records[name], separators=(",", ":")))
            f.write("\n")
        for a in anomalies:
            f.write(json.dumps(a, separators=(",", ":")))
            f.write("\n")
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp_path, out_path)
    dir_fd = os.open(directory, os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


def main(argv=None):
    args = parse_args(argv)

    if args.pattern == "periodic" and (args.period_s <= 0 or args.count <= 0):
        print(
            "driver: --pattern periodic requires --period-s > 0 and "
            f"--count > 0 (got period-s={args.period_s}, count={args.count})",
            file=sys.stderr,
        )
        return 1

    names = _plan_names(args.pattern, args.name_prefix, args.count)
    try:
        for name in names:
            _validate_name(name)
    except ValueError as e:
        print(f"driver: invalid --name-prefix: {e}", file=sys.stderr)
        return 1

    # Deferred past argument parsing (and the name validation above) so
    # `--help` and a plain syntax check work without a ROS environment
    # sourced -- only this function needs rclpy or the momentedge_msgs
    # typesupport, and both already returned or exited before this line.
    import rclpy
    from momentedge_msgs.msg import Recorded, Trigger
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from rclpy.signals import SignalHandlerOptions

    # Matches the QoS crates/clipper/src/interface.rs builds for both the
    # trigger subscription and the Recorded publisher in RosInterface::new
    # (r2r's QosProfile::default(): KEEP_LAST/depth 10, RELIABLE, VOLATILE).
    # Spelled out explicitly rather than relying on rclpy's own depth-only
    # shorthand resolving to the same policy, so a change on either side is
    # visible here at a glance instead of silently starting to drop messages.
    qos = QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )

    order = list(names)
    records = {
        name: {
            "name": name,
            "sent_ns": None,
            "anchor_hint_ns": None,
            "preroll_ns": args.preroll_ns,
            "postroll_ns": args.postroll_ns,
            # Additive beyond CONTRACT.md's base schema: get_subscription_count()
            # on the trigger publisher at the instant of publish. Distinguishes
            # "clipper never saw this trigger" (0 here) from "clipper saw it and
            # produced nothing" (>=1 here but never matched) -- two unrelated
            # failures that are otherwise indistinguishable in the results.
            "subscribers_at_publish": None,
            "recorded_ns": None,
            "latency_ns": None,
            "files": None,
            "bytes": None,
        }
        for name in names
    }
    matched = {name: False for name in names}
    # Recorded messages that arrived without a matching pending trigger: a
    # duplicate completion, or (the expected use under --pattern none) a
    # clip nobody asked for.
    anomalies = []

    # Progress state: three counters and a scalar, never a growing structure
    # -- a --pattern periodic soak run holds this for four hours, and nothing
    # here is allowed to accumulate with it. `_maybe_progress` is called once
    # per send and once per match; every `--progress-every`'th call prints a
    # one-line summary so `tail -f load.log` shows the run is alive instead of
    # going silent for hours.
    sent_count = 0
    matched_count = 0
    last_latency_ns = None
    progress_events = 0

    def _maybe_progress():
        nonlocal progress_events
        progress_events += 1
        if progress_events % args.progress_every != 0:
            return
        latency_part = (
            f" last_latency_ms={last_latency_ns / 1e6:.1f}"
            if last_latency_ns is not None
            else ""
        )
        print(
            f"driver: progress sent={sent_count}/{len(names)} "
            f"matched={matched_count}/{len(names)}{latency_part}",
            file=sys.stderr,
        )

    stop_requested = False

    def _on_signal(signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # The initial snapshot exists before any trigger is sent, so a kill in
    # the first instant still leaves a valid (empty, for --pattern none;
    # all-null otherwise) triggers.jsonl rather than a missing file.
    _write_snapshot(args.out, order, records, anomalies)

    # SignalHandlerOptions.NO: rclpy's default init() installs its own
    # SIGINT/SIGTERM handlers that shut the context down directly, racing the
    # driver's own handlers above and surfacing mid-spin as an
    # ExternalShutdownException instead of the clean, checked exit the driver
    # wants (flush the current snapshot, report which triggers matched).
    # Disabling rclpy's handlers leaves signal handling solely to
    # `_on_signal`, which only flips `stop_requested`.
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(args.node_name)
    try:
        pub = node.create_publisher(Trigger, args.trigger_topic, qos)

        def _on_recorded(msg):
            recorded_ns = time.time_ns()
            files = list(msg.filenames)
            rec = records.get(msg.name)
            if rec is None or matched[msg.name]:
                anomalies.append(
                    {
                        "unexpected_recorded": True,
                        "name": msg.name,
                        "recorded_ns": recorded_ns,
                        "files": files,
                        "bytes": _sum_sizes(files),
                    }
                )
                _write_snapshot(args.out, order, records, anomalies)
                return
            nonlocal matched_count, last_latency_ns
            sent_ns = rec["sent_ns"]
            rec["recorded_ns"] = recorded_ns
            rec["latency_ns"] = recorded_ns - sent_ns
            rec["files"] = files
            rec["bytes"] = _sum_sizes(files)
            matched[msg.name] = True
            matched_count += 1
            last_latency_ns = rec["latency_ns"]
            _write_snapshot(args.out, order, records, anomalies)
            _maybe_progress()

        # Created before any trigger is published, and before the discovery
        # wait below -- deliberately. clipper's Recorded publisher and this
        # subscription have the same match-before-delivery constraint as the
        # trigger side (reliable + volatile QoS, no late-joiner replay), so a
        # Recorded published before this subscription is matched is lost, the
        # same way an early Trigger publish is. Matching clipper's own
        # publish->announce turnaround is normally seconds, but keep this
        # ahead of the send loop regardless -- do not move it later.
        node.create_subscription(Recorded, args.recorded_topic, _on_recorded, qos)

        # DDS discovery (this publisher <-> the topic's subscribers) is
        # asynchronous and runs on the order of tens to hundreds of
        # milliseconds after the publisher is created, but has no upper bound
        # guarantee. A reliable, volatile publisher has no matched reader to
        # queue for during that window, so a trigger published before a
        # subscription is matched is silently dropped.
        #
        # A bare "count >= 1" is NOT "clipper is ready": `ros2 bag record
        # --all` also subscribes to the trigger topic (it records every
        # topic), and its subscription is typically matched before clipper's
        # own -- so publishing the instant the count first goes non-zero
        # regularly caught only the recorder, dropping the trigger on the
        # floor with no error anywhere (found in production: 14/41 nx runs,
        # concentrated on the first trigger of a batch, every loss at
        # subscribers_at_publish==1 and zero losses at ==2). Waiting for the
        # count to STABILIZE -- unchanged for --settle-s -- instead of merely
        # becoming non-zero needs no advance knowledge of how many
        # subscribers to expect: the recorder's subscription lands, the count
        # holds while clipper's own subscription is still forming, then
        # clipper's lands and the count steps again, then holds for good. A
        # run with no clipper (or any other future single-subscriber
        # scenario) degrades correctly too -- the count simply reaches 1 and
        # stays there, so stabilization proceeds once --settle-s has passed
        # with no further change, without ever having been told to expect 1.
        # --expect-subscribers is a pure optimization on top for a caller
        # that already knows the count, letting it stop waiting the moment
        # that many are seen instead of waiting out the settle window too.
        #
        # Exceeding --discovery-timeout-s with zero subscribers ever seen is
        # a hard failure (no clip run this driver started would ever be
        # believable evidence of clipper's own behaviour) rather than
        # warning-and-proceeding, because sending into a topic nobody is
        # listening on would just report every trigger as a mysteriously
        # unmatched one, indistinguishable from a real clipper bug, hours
        # into an unattended overnight suite. Seeing at least one subscriber
        # but never reaching a stable count is a softer warning, not a hard
        # failure -- refusing to send at all when a real subscriber (just a
        # slow-to-settle one) is known to exist would trade one failure mode
        # for another. Skipped entirely under --pattern none, which never
        # publishes anything and must work with no clipper present.
        if names:
            discovery_start = time.monotonic()
            discovery_deadline = discovery_start + args.discovery_timeout_s
            last_count = pub.get_subscription_count()
            last_change_mono = discovery_start

            def _stabilized():
                if last_count >= 1 and (time.monotonic() - last_change_mono) >= args.settle_s:
                    return True
                return args.expect_subscribers is not None and last_count >= args.expect_subscribers

            stabilized = _stabilized()
            while not stabilized and time.monotonic() < discovery_deadline:
                if stop_requested:
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
                count = pub.get_subscription_count()
                now = time.monotonic()
                if count != last_count:
                    last_count = count
                    last_change_mono = now
                stabilized = _stabilized()

            discovery_elapsed_s = time.monotonic() - discovery_start
            final_count = pub.get_subscription_count()

            if final_count < 1:
                if stop_requested:
                    print(
                        f"driver: interrupted after {discovery_elapsed_s:.2f}s "
                        f"waiting for a subscriber on {args.trigger_topic}",
                        file=sys.stderr,
                    )
                else:
                    print(
                        f"driver: no subscriber appeared on {args.trigger_topic} "
                        f"after {discovery_elapsed_s:.2f}s "
                        f"(--discovery-timeout-s {args.discovery_timeout_s}) -- "
                        "clipper is probably not running or not subscribed to "
                        "this topic; refusing to publish triggers nobody would "
                        "receive",
                        file=sys.stderr,
                    )
                    return 1
            elif not stabilized:
                print(
                    f"driver: warning: subscriber count on {args.trigger_topic} "
                    f"had not stabilized after {discovery_elapsed_s:.2f}s "
                    f"(--discovery-timeout-s {args.discovery_timeout_s}); "
                    f"proceeding anyway with the last-seen count={final_count}",
                    file=sys.stderr,
                )
            else:
                print(
                    f"driver: subscriber count on {args.trigger_topic} "
                    f"stabilized at {final_count} after {discovery_elapsed_s:.2f}s",
                    file=sys.stderr,
                )
            high_water_subscribers = final_count
        else:
            high_water_subscribers = 0

        # Each offset is an absolute target time relative to `start_mono`
        # (below), not a per-iteration gap -- trigger N is due at
        # start_mono + send_offsets[N] regardless of how late any earlier
        # trigger fired. This is what keeps a --pattern periodic soak run
        # from drifting: a `sleep(period_s)` loop accumulates whatever jitter
        # each iteration adds, but comparing against a fixed origin cannot
        # accumulate anything -- trigger 239 lands at start + 239*period_s
        # even if trigger 1 was a few ms late.
        if args.pattern == "ten_overlap":
            send_offsets = [
                i * (args.stagger_ms / 1000.0) for i in range(TEN_OVERLAP_COUNT)
            ]
        elif args.pattern == "periodic":
            send_offsets = [i * args.period_s for i in range(args.count)]
        else:
            send_offsets = [0.0] * len(names)

        def _send(idx):
            nonlocal sent_count, high_water_subscribers
            name = names[idx]
            msg = Trigger()
            msg.name = name
            msg.description = args.description
            # Deliberately zero: see the module docstring and
            # ../../README.md#time-source-log-or-publish. Never stamp this
            # under --time-source log -- clipper rejects a non-zero value.
            msg.trigger_time.sec = 0
            msg.trigger_time.nanosec = 0
            msg.preroll = args.preroll_ns
            msg.postroll = args.postroll_ns
            # Captured right alongside the publish call, not just once at
            # startup: a subscriber discovered during the wait above can still
            # drop out (a clipper crash) before a later trigger in the same
            # ten_overlap/periodic run, and that distinction belongs to the
            # trigger it happened for.
            subscribers_at_publish = pub.get_subscription_count()
            # A drop below the best count seen so far this run is the
            # signature of a subscriber going away mid-run (a clipper crash,
            # most plausibly) -- otherwise invisible, since nothing else
            # here watches the graph between triggers. high_water_subscribers
            # only ever rises, so this compares against the true historical
            # peak, not just the value discovery settled on at the start.
            if subscribers_at_publish < high_water_subscribers:
                print(
                    f"driver: warning: {name} published with "
                    f"subscribers_at_publish={subscribers_at_publish}, below "
                    f"the {high_water_subscribers} seen earlier this run -- "
                    "a subscriber may have dropped out",
                    file=sys.stderr,
                )
            elif subscribers_at_publish > high_water_subscribers:
                high_water_subscribers = subscribers_at_publish
            sent_ns = time.time_ns()
            pub.publish(msg)
            rec = records[name]
            rec["sent_ns"] = sent_ns
            rec["subscribers_at_publish"] = subscribers_at_publish
            # The anchor clipper will actually resolve is `now` at its own
            # subscription instant (--time-source log), which this driver
            # cannot observe directly; sent_ns is the closest hint of it.
            rec["anchor_hint_ns"] = sent_ns
            sent_count += 1
            _write_snapshot(args.out, order, records, anomalies)
            _maybe_progress()

        start_mono = time.monotonic()
        next_idx = 0
        while True:
            if stop_requested:
                break
            now_mono = time.monotonic()
            while next_idx < len(names) and now_mono >= start_mono + send_offsets[next_idx]:
                _send(next_idx)
                next_idx += 1
                now_mono = time.monotonic()

            rclpy.spin_once(node, timeout_sec=0.01)

            if args.pattern != "none" and names and all(matched.values()):
                break
            if time.monotonic() - start_mono > args.timeout_s:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    exit_code = 0
    unmatched = [n for n in names if not matched[n]]
    if unmatched:
        print(
            f"driver: UNMATCHED {len(unmatched)}/{len(names)} triggers never "
            f"got a Recorded: {unmatched}",
            file=sys.stderr,
        )
        # subscribers_at_publish (also in triggers.jsonl) separates two
        # distinct failures: 0 means clipper never saw the trigger at all
        # (a discovery/QoS/topic problem downstream of this driver); >=1
        # means clipper saw it and produced nothing (a clipper-side problem).
        for n in unmatched:
            subs = records[n]["subscribers_at_publish"]
            if subs is None:
                print(f"driver:   {n}: never sent (run stopped first)", file=sys.stderr)
            else:
                print(
                    f"driver:   {n}: subscribers_at_publish={subs}",
                    file=sys.stderr,
                )
        exit_code = 1
    if anomalies:
        print(
            f"driver: ANOMALY {len(anomalies)} unexpected Recorded message(s): "
            f"{[a['name'] for a in anomalies]}",
            file=sys.stderr,
        )
        exit_code = 1
    if not unmatched and not anomalies:
        if names:
            print(f"driver: OK {len(names)}/{len(names)} triggers matched", file=sys.stderr)
        else:
            print("driver: OK no anomalies observed", file=sys.stderr)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
