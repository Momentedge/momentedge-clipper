#!/usr/bin/env python3
"""Fire a clipper Trigger when a ros2_pulse diagnostic leaves OK.

Subscribes to /diagnostics (diagnostic_msgs/DiagnosticArray), as published by
ros2_pulse's pulse_bridge with diagnostics:=true, and publishes one
momentedge_msgs/Trigger on /events/momentedge/trigger each time a status whose
name starts with `prefix` moves into one of `levels`. A status that stays
non-OK fires once, not once per /diagnostics message; it fires again only after
it has been back at a level outside `levels` (normally OK), and no sooner than
`cooldown_s` after its last trigger.

Parameters (all optional):

    preroll_s    seconds kept before the anchor           (default 30.0)
    postroll_s   seconds kept after the anchor            (default 10.0)
    levels       levels that fire                         (default [WARN, ERROR])
    prefix       only statuses whose name starts with it  (default "ros2_pulse:")
    cooldown_s   least seconds between two triggers of
                 one status                               (default 60.0)

trigger_time is always zero: clipper anchors the window on the instant it
receives the trigger, which is the only anchor accepted outside
--time-source publish.

The edge detection is `step`, a plain function with no ROS in it, so it can be
tested without a ROS 2 environment (see test_pulse_trigger.py).
"""

import sys
import time

LEVEL_NAMES = {0: "OK", 1: "WARN", 2: "ERROR", 3: "STALE"}

# clipper's limits on a trigger (docs/triggers-and-time.md): a name of at most
# 128 bytes, and at most 30 minutes of preroll or postroll.
MAX_NAME_BYTES = 128
MAX_ROLL_S = 30 * 60


def step(state, statuses, now, levels, prefix, cooldown_s):
    """Return the names of the statuses that fire a trigger now.

    `statuses` is one DiagnosticArray as (name, level name) pairs. `state` maps
    a status name to `[level, last_fired]` and is updated in place. A status
    absent from this message keeps its state: other nodes publish on
    /diagnostics too, each with its own statuses only.
    """
    fired = []
    for name, level in statuses:
        if not name.startswith(prefix):
            continue
        previous, last_fired = state.get(name, [None, None])
        entering = level in levels and previous not in levels
        cooled = last_fired is None or now - last_fired >= cooldown_s
        if entering and cooled:
            fired.append(name)
            last_fired = now
        state[name] = [level, last_fired]
    return fired


def trigger_name(status_name):
    """The status name, cut to clipper's 128-byte limit on a character boundary."""
    data = status_name.encode("utf-8")[:MAX_NAME_BYTES]
    return data.decode("utf-8", errors="ignore")


def level_name(level):
    """DiagnosticStatus.level is a one-byte `bytes` in rclpy; accept an int too."""
    value = level if isinstance(level, int) else ord(level)
    return LEVEL_NAMES.get(value, str(value))


def describe(status):
    """Level, message and values, e.g. `WARN rate 0.000 Hz outside [8, inf] (hz=0.000 ...)`."""
    values = " ".join(f"{kv.key}={kv.value}" for kv in status.values)
    text = f"{level_name(status.level)} {status.message}"
    return f"{text} ({values})" if values else text


def main():
    # The ROS imports live here so the functions above import without ROS 2.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from diagnostic_msgs.msg import DiagnosticArray
    from momentedge_msgs.msg import Trigger

    rclpy.init(args=sys.argv)
    node = rclpy.create_node("pulse_trigger")
    preroll_s = node.declare_parameter("preroll_s", 30.0).value
    postroll_s = node.declare_parameter("postroll_s", 10.0).value
    levels = set(node.declare_parameter("levels", ["WARN", "ERROR"]).value)
    prefix = node.declare_parameter("prefix", "ros2_pulse:").value
    cooldown_s = node.declare_parameter("cooldown_s", 60.0).value
    for label, seconds in (("preroll_s", preroll_s), ("postroll_s", postroll_s)):
        if not 0 <= seconds <= MAX_ROLL_S:
            node.get_logger().fatal(f"{label} must be in [0, {MAX_ROLL_S}] s, got {seconds}")
            sys.exit(1)

    # Depth 10, reliable, volatile: rclpy's default, the same profile clipper
    # subscribes with (r2r's QosProfile::default()) and pulse_bridge publishes with.
    publisher = node.create_publisher(Trigger, "/events/momentedge/trigger", 10)
    state = {}

    def on_diagnostics(msg):
        now = time.monotonic()
        by_name = {s.name: s for s in msg.status}
        pairs = [(s.name, level_name(s.level)) for s in msg.status]
        for name in step(state, pairs, now, levels, prefix, cooldown_s):
            trigger = Trigger()
            trigger.name = trigger_name(name)
            trigger.description = describe(by_name[name])
            # trigger_time stays 0: clipper anchors on the instant it receives this.
            trigger.preroll = round(preroll_s * 1e9)
            trigger.postroll = round(postroll_s * 1e9)
            publisher.publish(trigger)
            node.get_logger().info(f"trigger name={trigger.name!r} {trigger.description}")

    node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)
    node.get_logger().info(
        f"watching /diagnostics for '{prefix}*' entering {sorted(levels)}, "
        f"preroll {preroll_s} s, postroll {postroll_s} s"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
