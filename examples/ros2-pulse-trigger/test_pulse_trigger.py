"""Tests for the edge detection in pulse_trigger.py. No ROS 2 needed:

    python3 -m unittest examples/ros2-pulse-trigger/test_pulse_trigger.py
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pulse_trigger import step, trigger_name  # noqa: E402

LEVELS = {"WARN", "ERROR"}
PREFIX = "ros2_pulse:"
TOPIC = "ros2_pulse: /chatter (recv)"


def run(state, level, now, cooldown_s=0.0, name=TOPIC):
    return step(state, [(name, level)], now, LEVELS, PREFIX, cooldown_s)


class StepTest(unittest.TestCase):
    def test_a_status_that_stays_warn_fires_once(self):
        """The bridge republishes every poll, so one stall must not become a clip per message."""
        state = {}
        run(state, "OK", 0)
        self.assertEqual(run(state, "WARN", 1), [TOPIC])
        self.assertEqual(run(state, "WARN", 2), [])
        self.assertEqual(run(state, "WARN", 3), [])

    def test_a_status_first_seen_non_ok_fires(self):
        """A topic already broken when the node starts is still an incident worth a clip."""
        self.assertEqual(run({}, "ERROR", 0), [TOPIC])

    def test_a_status_back_at_ok_fires_again_on_the_next_warn(self):
        """Recovery re-arms the status, so the second stall gets its own clip."""
        state = {}
        run(state, "WARN", 0)
        run(state, "OK", 1)
        self.assertEqual(run(state, "WARN", 2), [TOPIC])

    def test_warn_to_error_is_the_same_incident(self):
        """Getting worse without recovering is not a new event."""
        state = {}
        run(state, "WARN", 0)
        self.assertEqual(run(state, "ERROR", 1), [])

    def test_a_flapping_status_waits_out_the_cooldown(self):
        """A rule on the edge of its bound flips every window; the cooldown bounds the clip rate."""
        state = {}
        self.assertEqual(run(state, "WARN", 0, cooldown_s=60), [TOPIC])
        run(state, "OK", 5, cooldown_s=60)
        self.assertEqual(run(state, "WARN", 10, cooldown_s=60), [])
        run(state, "OK", 61, cooldown_s=60)
        self.assertEqual(run(state, "WARN", 70, cooldown_s=60), [TOPIC])

    def test_statuses_outside_the_prefix_are_ignored(self):
        """Other nodes publish on /diagnostics too; only ros2_pulse's statuses fire by default."""
        self.assertEqual(run({}, "ERROR", 0, name="camera: driver"), [])

    def test_a_status_missing_from_a_message_keeps_its_state(self):
        """Each /diagnostics publisher sends its own statuses only, so absence is not recovery."""
        state = {}
        run(state, "WARN", 0)
        step(state, [("other: status", "OK")], 1, LEVELS, PREFIX, 0.0)
        self.assertEqual(run(state, "WARN", 2), [])


class TriggerNameTest(unittest.TestCase):
    def test_a_long_name_is_cut_to_clippers_128_byte_limit(self):
        """clipper rejects a longer name, and a cut must not split a UTF-8 character."""
        name = trigger_name("ros2_pulse: /" + "é" * 100)
        self.assertLessEqual(len(name.encode("utf-8")), 128)
        self.assertTrue(name.startswith("ros2_pulse: /é"))

    def test_a_short_name_is_kept_as_is(self):
        self.assertEqual(trigger_name(TOPIC), TOPIC)


if __name__ == "__main__":
    unittest.main()
