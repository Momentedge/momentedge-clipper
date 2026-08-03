#!/usr/bin/env python3
"""Publish N synthetic std_msgs/ByteMultiArray topics at a fixed rate each.

Adds message-count/rate load independent of the replayed bag: `--topics`
publishers under `--prefix`, each carrying a `--size`-byte payload, all firing
together on one shared timer at `--rate` Hz (same aggregate rate and message
count as N independent per-publisher timers, less executor overhead).

Argument parsing is stdlib-only, so `--help` and bad-argument errors work
without a ROS2 environment; `rclpy` is only imported once the publishers
actually start, so a missing environment fails there with a clear message
rather than an import traceback.

rclpy's practical rate ceiling is well below what a C++ publisher sustains: a
Python process spinning publisher timers pays real per-tick cost for message
construction, executor scheduling, and the GIL, so jitter and missed ticks
show up long before hardware saturates — a handful of publishers in the
tens-to-low-hundreds of Hz is a realistic ceiling. That is why this benchmark
gets its heavy bitrate tiers from replaying a bag with `ros2 bag play` (a C++
process, see replay.sh) rather than from Python publishers; this script is for
layering extra, independently-controlled message traffic on top of that
replay, not for reaching its bitrate.

Mirrors trigger/driver.py's discovery-visibility pattern, adapted for a
continuously-running publisher rather than driver.py's finite,
matched-by-completion triggers: `get_subscription_count()` per topic is
logged every SUBSCRIBER_LOG_INTERVAL_S, so a recorder that never matches
these topics (a topic-filter mismatch, a discovery race, nobody running)
shows up in load.log as `0` rather than as a clean, silent, zero-message run.
If not one topic ever gets a subscriber for the entire run, that is treated
as this run having contributed no load at all, and the exit code is non-zero.
"""
import argparse
import signal
import sys
import time


SUBSCRIBER_LOG_INTERVAL_S = 5.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--topics", required=True, type=int, help="number of publishers to create")
    parser.add_argument("--size", required=True, type=int, help="payload size in bytes, per message")
    parser.add_argument("--rate", required=True, type=float, help="publish rate in Hz, per publisher")
    parser.add_argument("--prefix", default="/bench", help="topic namespace prefix (default: %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.topics <= 0:
        print("synth_pub.py: --topics must be positive", file=sys.stderr)
        return 1
    if args.size < 0:
        print("synth_pub.py: --size must be non-negative", file=sys.stderr)
        return 1
    if args.rate <= 0:
        print("synth_pub.py: --rate must be positive", file=sys.stderr)
        return 1

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import ByteMultiArray
    except ImportError as exc:
        print(f"synth_pub.py: rclpy/std_msgs unavailable ({exc}) — source a ROS2 environment first:", file=sys.stderr)
        print("  . /opt/ros/<distro>/setup.bash", file=sys.stderr)
        return 1

    stop = {"flag": False}

    def _on_term(signum, frame):
        stop["flag"] = True

    # rclpy.init() installs its own SIGINT/SIGTERM handlers by default, which
    # would shut the context down themselves and race the handlers below —
    # SignalHandlerOptions.NO leaves signal handling entirely to this script,
    # so shutdown always goes through the single, orderly path below.
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)

    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = Node("synth_pub")
    payload = bytes(args.size)
    topic_names = [f"{args.prefix}/synth_{i:02d}" for i in range(args.topics)]
    publishers = [node.create_publisher(ByteMultiArray, name, 10) for name in topic_names]

    def _tick():
        msg = ByteMultiArray()
        msg.data = payload
        for pub in publishers:
            pub.publish(msg)

    node.create_timer(1.0 / args.rate, _tick)
    node.get_logger().info(
        f"synth_pub: {args.topics} publisher(s) under {args.prefix}, {args.size} B @ {args.rate} Hz each"
    )

    # Per-topic subscriber count over time: which topics ever had a matched
    # subscriber (>=1), for the final "contributed no load at all" check
    # below, plus a periodic log line so `0` at every topic is visible in
    # load.log as the run goes rather than only inferred after the fact.
    ever_had_subscriber = [False] * args.topics
    last_log_mono = time.monotonic()

    try:
        while not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.2)
            now_mono = time.monotonic()
            if now_mono - last_log_mono >= SUBSCRIBER_LOG_INTERVAL_S:
                last_log_mono = now_mono
                counts = [pub.get_subscription_count() for pub in publishers]
                for i, count in enumerate(counts):
                    if count >= 1:
                        ever_had_subscriber[i] = True
                node.get_logger().info(f"synth_pub: subscriber counts {dict(zip(topic_names, counts))}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not any(ever_had_subscriber):
        print(
            f"synth_pub.py: no publisher under {args.prefix} ever had a subscriber over the "
            "whole run — this load was never received by anything",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
