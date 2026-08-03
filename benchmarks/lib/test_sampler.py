#!/usr/bin/env python3
"""Unit tests for sampler.py's PidTracker, focused on the io-availability
fix: /proc/<pid>/io is absent on kernels built without CONFIG_TASKSTATS
(seen on the nx target, JetPack 5 / L4T 36) for every process, not some --
that must degrade the four io columns to empty, independently of
stat/status/fds, without ever dropping the pid or spamming stderr.

PidTracker takes real pids (so /proc/<pid>/fd -- not faked here -- keeps
working) but reads stat/status/io from an injectable `proc_root` fixture
tree instead of the real /proc, so these tests need no ssh, no real kernel
difference, and no root.

Run with: python3 -m unittest test_sampler -v
"""

import contextlib
import io as io_module
import os
import signal
import stat as stat_module
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock

import procinfo
import sampler


def _stat_line(pid, utime=0, stime=0, threads=1):
    return (
        f"{pid} (proc) S 1 {pid} {pid} 0 -1 4194368 0 0 0 0 {utime} {stime} "
        f"0 0 20 0 {threads} 0 999 9000000 200 18446744073709551615 0 0 0 0 "
        "0 0 0 0 0 0 17 3 0 0 0 0 0 0 0 0 0 0 0 0 0\n"
    )


def _status_text(rss_kb=1000, vmhwm_kb=1200):
    return f"Name:\tproc\nVmHWM:\t{vmhwm_kb} kB\nVmRSS:\t{rss_kb} kB\n"


def _io_text_full(rchar=100, wchar=200, read_bytes=300, write_bytes=400):
    return (
        f"rchar: {rchar}\nwchar: {wchar}\nsyscr: 1\nsyscw: 1\n"
        f"read_bytes: {read_bytes}\nwrite_bytes: {write_bytes}\n"
        "cancelled_write_bytes: 0\n"
    )


def _io_text_partial(read_bytes=300, write_bytes=400):
    # A real gotcha (see procinfo.parse_io): some kernels/containers omit
    # rchar/wchar specifically while keeping the accounting fields.
    return f"syscr: 1\nsyscw: 1\nread_bytes: {read_bytes}\nwrite_bytes: {write_bytes}\n"


def _make_fake_proc(root, pid, utime=0, stime=0, threads=1, rss_kb=1000,
                     vmhwm_kb=1200, io_text=None):
    """Populate <root>/<pid>/{stat,status[,io]}. io_text=None means the io
    file is not created at all -- the condition this fix is about.
    """
    d = os.path.join(root, str(pid))
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "stat"), "w") as f:
        f.write(_stat_line(pid, utime, stime, threads))
    with open(os.path.join(d, "status"), "w") as f:
        f.write(_status_text(rss_kb, vmhwm_kb))
    if io_text is not None:
        with open(os.path.join(d, "io"), "w") as f:
            f.write(io_text)


# Column indices in a samples.csv row (see sampler.SAMPLES_HEADER).
COL_RCHAR, COL_WCHAR, COL_READ_BYTES, COL_WRITE_BYTES = 8, 9, 10, 11


class IoPresentTests(unittest.TestCase):
    def test_full_io_populates_numeric_fields(self):
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            _make_fake_proc(root, pid, io_text=_io_text_full(
                rchar=111, wchar=222, read_bytes=333, write_bytes=444))
            tracker = sampler.PidTracker(
                [("clipper", pid)], io_available=True, proc_root=root
            )
            stderr = io_module.StringIO()
            with contextlib.redirect_stderr(stderr):
                rows = tracker.sample(1_000_000_000)
            tracker.close()

        self.assertEqual(len(rows), 1)
        row = rows[0]
        self.assertEqual(row[COL_RCHAR], 111)
        self.assertEqual(row[COL_WCHAR], 222)
        self.assertEqual(row[COL_READ_BYTES], 333)
        self.assertEqual(row[COL_WRITE_BYTES], 444)
        self.assertNotIn("io", stderr.getvalue().lower())

    def test_partial_io_missing_fields_are_empty_present_fields_are_real(self):
        """A present io file missing a couple of fields (a real container/
        kernel quirk) reports those specific fields empty (None -> blank
        CSV cell) -- 0 would claim "measured zero bytes" for something
        never measured at all. The fields that ARE present still carry
        their real values, including a genuine 0 (see the sibling test
        below): only *absence* renders empty, not "happens to be zero".
        """
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            _make_fake_proc(root, pid, io_text=_io_text_partial(
                read_bytes=333, write_bytes=444))
            tracker = sampler.PidTracker(
                [("clipper", pid)], io_available=True, proc_root=root
            )
            rows = tracker.sample(1_000_000_000)
            tracker.close()

        row = rows[0]
        self.assertIsNone(row[COL_RCHAR])
        self.assertIsNone(row[COL_WCHAR])
        self.assertEqual(row[COL_READ_BYTES], 333)
        self.assertEqual(row[COL_WRITE_BYTES], 444)

    def test_genuine_zero_io_field_is_not_rendered_empty(self):
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            _make_fake_proc(root, pid, io_text=_io_text_full(
                rchar=0, wchar=0, read_bytes=0, write_bytes=0))
            tracker = sampler.PidTracker(
                [("clipper", pid)], io_available=True, proc_root=root
            )
            rows = tracker.sample(1_000_000_000)
            tracker.close()

        row = rows[0]
        for col in (COL_RCHAR, COL_WCHAR, COL_READ_BYTES, COL_WRITE_BYTES):
            self.assertEqual(row[col], 0)
            self.assertIsNotNone(row[col])


class IoAbsentTests(unittest.TestCase):
    def test_per_pid_io_missing_from_start_emits_empty_fields(self):
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            _make_fake_proc(root, pid, io_text=None)  # no io file at all
            tracker = sampler.PidTracker(
                [("clipper", pid)], io_available=True, proc_root=root
            )
            rows = tracker.sample(1_000_000_000)
            tracker.close()

        row = rows[0]
        self.assertIsNone(row[COL_RCHAR])
        self.assertIsNone(row[COL_WCHAR])
        self.assertIsNone(row[COL_READ_BYTES])
        self.assertIsNone(row[COL_WRITE_BYTES])
        # stat/status/fds are still real data -- io absence must not
        # blank out the rest of the row.
        self.assertEqual(row[2], pid)
        self.assertIsInstance(row[6], int)  # rss_kb

    def test_warns_exactly_once_across_multiple_pids_and_ticks(self):
        # Two distinct, genuinely running pids (so /proc/<pid>/fd is real
        # for both) whose fake proc dirs both lack an io file.
        proc_a = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(5)"])
        proc_b = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(5)"])
        try:
            with tempfile.TemporaryDirectory() as root:
                _make_fake_proc(root, proc_a.pid, io_text=None)
                _make_fake_proc(root, proc_b.pid, io_text=None)
                stderr = io_module.StringIO()
                with contextlib.redirect_stderr(stderr):
                    tracker = sampler.PidTracker(
                        [("a", proc_a.pid), ("b", proc_b.pid)],
                        io_available=True, proc_root=root,
                    )
                    for tick_ns in (1_000_000_000, 1_100_000_000, 1_200_000_000):
                        rows = tracker.sample(tick_ns)
                        self.assertEqual(len(rows), 2)
                        for row in rows:
                            self.assertIsNone(row[COL_RCHAR])
                    tracker.close()
        finally:
            proc_a.terminate()
            proc_b.terminate()
            proc_a.wait(timeout=5)
            proc_b.wait(timeout=5)

        occurrences = stderr.getvalue().lower().count("unavailable on this kernel")
        self.assertEqual(
            occurrences, 1,
            f"expected exactly one io-unavailable warning, got {occurrences} "
            f"in:\n{stderr.getvalue()}",
        )

    def test_global_hint_false_skips_per_pid_open_and_does_not_warn_again(self):
        """When the up-front /proc/self/io probe already found io
        unavailable (io_available=False passed in), PidTracker must not
        even attempt to open a per-pid io file (even if one happens to
        exist in the fixture) and must not print its own redundant
        warning -- run() already prints the one warning for the whole run
        in that case, before constructing PidTracker at all.
        """
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            # io file DOES exist in the fixture; io_available=False must
            # still short-circuit ever opening it.
            _make_fake_proc(root, pid, io_text=_io_text_full())
            stderr = io_module.StringIO()
            with contextlib.redirect_stderr(stderr):
                tracker = sampler.PidTracker(
                    [("clipper", pid)], io_available=False, proc_root=root
                )
                rows = tracker.sample(1_000_000_000)
                tracker.close()

        row = rows[0]
        self.assertIsNone(row[COL_RCHAR])
        self.assertEqual(stderr.getvalue(), "")


class StatStatusFatalTests(unittest.TestCase):
    def test_missing_stat_or_status_drops_pid_without_crashing(self):
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            # Deliberately do not call _make_fake_proc: no stat/status/io
            # at all, as if the pid never existed under this proc_root.
            stderr = io_module.StringIO()
            with contextlib.redirect_stderr(stderr):
                tracker = sampler.PidTracker(
                    [("clipper", pid)], io_available=True, proc_root=root
                )
                rows = tracker.sample(1_000_000_000)
        self.assertEqual(rows, [])
        self.assertIn("skipping", stderr.getvalue())


class MidRunIoFailureTests(unittest.TestCase):
    class _FlakyIo:
        """A stand-in for an open io file object that always raises on
        read -- exercises the "io was open but broke" branch in
        _sample_one that a plain missing-file fixture can't reach, since
        Linux keeps an already-open fd readable even after its directory
        entry vanishes. Swapped in *after* a real, successful first tick
        to represent "io became unreadable starting now".
        """

        def __init__(self):
            self.closed = False

        def seek(self, pos):
            pass

        def read(self):
            raise OSError("simulated: io became unreadable")

        def close(self):
            self.closed = True

    def test_io_failure_after_first_tick_drops_only_io_not_the_pid(self):
        pid = os.getpid()
        with tempfile.TemporaryDirectory() as root:
            _make_fake_proc(root, pid, io_text=_io_text_full(
                rchar=1, wchar=2, read_bytes=3, write_bytes=4))
            tracker = sampler.PidTracker(
                [("clipper", pid)], io_available=True, proc_root=root
            )

            rows1 = tracker.sample(1_000_000_000)
            self.assertEqual(rows1[0][COL_RCHAR], 1)

            tracker._handles[pid]["io"].close()  # the real handle from _open()
            flaky = self._FlakyIo()
            tracker._handles[pid]["io"] = flaky

            stderr = io_module.StringIO()
            with contextlib.redirect_stderr(stderr):
                rows2 = tracker.sample(1_100_000_000)
            self.assertIsNone(rows2[0][COL_RCHAR])
            self.assertEqual(rows2[0][2], pid)  # pid itself was NOT dropped
            self.assertIn("unavailable on this kernel", stderr.getvalue())
            self.assertTrue(flaky.closed)

            # Third tick: io handle is now None: no repeated attempt to
            # call the (already failing) flaky object, no crash, pid
            # still sampled, and no second warning.
            stderr2 = io_module.StringIO()
            with contextlib.redirect_stderr(stderr2):
                rows3 = tracker.sample(1_200_000_000)
            self.assertIsNone(rows3[0][COL_RCHAR])
            self.assertEqual(rows3[0][2], pid)
            self.assertEqual(stderr2.getvalue(), "")

            tracker.close()


class TegrastatsCommandsTests(unittest.TestCase):
    """Pure command-construction tests -- no sudo or tegrastats needed."""

    def test_direct_invocation_never_touches_sudo_or_env(self):
        cmds = sampler.tegrastats_commands(
            "/usr/bin/tegrastats", 1000, {"BENCH_SUITE": "momentedge-bench"}
        )
        self.assertEqual(cmds[0], ["/usr/bin/tegrastats", "--interval", "1000"])

    def test_sudo_invocation_reinjects_bench_suite_via_env(self):
        cmds = sampler.tegrastats_commands(
            "/usr/bin/tegrastats", 1000, {"BENCH_SUITE": "momentedge-bench"}
        )
        self.assertEqual(
            cmds[1],
            ["sudo", "-n", "env", "BENCH_SUITE=momentedge-bench",
             "/usr/bin/tegrastats", "--interval", "1000"],
        )

    def test_sudo_invocation_without_bench_suite_set_skips_env_wrapper(self):
        cmds = sampler.tegrastats_commands("/usr/bin/tegrastats", 1000, {})
        self.assertEqual(
            cmds[1], ["sudo", "-n", "/usr/bin/tegrastats", "--interval", "1000"]
        )

    def test_never_uses_preserve_env_flag(self):
        # --preserve-env silently no-ops unless the target's own sudoers
        # config already allows preserving that var -- something this
        # script cannot assume, which is why it isn't used at all.
        cmds = sampler.tegrastats_commands(
            "/usr/bin/tegrastats", 1000, {"BENCH_SUITE": "x"}
        )
        joined = " ".join(cmds[1])
        self.assertNotIn("--preserve-env", joined)
        self.assertNotIn("-E", cmds[1])


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False


class TegraStatsProcessTreeTests(unittest.TestCase):
    """Live process-tree tests against a stub `tegrastats` binary that
    itself forks a child -- simulating the real sudo-wraps-tegrastats
    topology -- placed on PATH so shutil.which("tegrastats") finds it.
    No real sudo or tegrastats needed.
    """

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        root = self._tmp.name
        self.bench_suite_out = os.path.join(root, "bench_suite_seen")
        self.child_pid_file = os.path.join(root, "child_pid")
        stub_path = os.path.join(root, "tegrastats")
        with open(stub_path, "w") as f:
            f.write(f"""#!/usr/bin/env bash
echo "$BENCH_SUITE" > {self.bench_suite_out}
sleep 100 &
echo $! > {self.child_pid_file}
trap 'exit 0' TERM
while true; do echo "fake tegrastats line"; sleep 0.1; done
""")
        os.chmod(
            stub_path,
            os.stat(stub_path).st_mode | stat_module.S_IXUSR | stat_module.S_IRUSR,
        )
        self._old_path = os.environ.get("PATH", "")
        os.environ["PATH"] = root + os.pathsep + self._old_path
        self._old_bench_suite = os.environ.get("BENCH_SUITE")
        os.environ["BENCH_SUITE"] = "momentedge-bench"

    def tearDown(self):
        os.environ["PATH"] = self._old_path
        if self._old_bench_suite is None:
            os.environ.pop("BENCH_SUITE", None)
        else:
            os.environ["BENCH_SUITE"] = self._old_bench_suite
        self._tmp.cleanup()

    def _wait_for_file(self, path, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if os.path.exists(path) and os.path.getsize(path) > 0:
                return
            time.sleep(0.02)
        self.fail(f"{path} was never written")

    def test_launched_as_plain_child_same_group_as_this_process(self):
        """No setsid: the orchestrator finds harness processes via an
        inherited env var and/or process-group membership; either is
        defeated if tegrastats gets its own session.
        """
        tg = sampler.TegraStats(self._tmp.name, 1000)
        tg.start()
        self.addCleanup(tg.stop)
        self.assertIsNotNone(tg._proc)
        self.assertEqual(os.getpgid(tg._proc.pid), os.getpgid(os.getpid()))

    def test_bench_suite_env_var_reaches_the_child(self):
        tg = sampler.TegraStats(self._tmp.name, 1000)
        tg.start()
        self.addCleanup(tg.stop)
        self._wait_for_file(self.bench_suite_out)
        with open(self.bench_suite_out) as f:
            self.assertEqual(f.read().strip(), "momentedge-bench")

    def test_stop_cleans_up_the_whole_tree_including_the_grandchild(self):
        tg = sampler.TegraStats(self._tmp.name, 1000)
        tg.start()
        self._wait_for_file(self.child_pid_file)
        with open(self.child_pid_file) as f:
            child_pid = int(f.read().strip())
        stub_pid = tg._proc.pid

        self.assertTrue(_alive(stub_pid))
        self.assertTrue(_alive(child_pid))

        tg.stop()

        self.assertFalse(_alive(stub_pid), "tegrastats stub still running after stop()")
        self.assertFalse(_alive(child_pid), "the stub's forked child leaked past stop()")


class _FakeProc:
    """Stands in for the subprocess.Popen object TegraStats._proc holds,
    for tests that only exercise signal/liveness logic and never call
    start() or spawn anything real.
    """

    def __init__(self, pid, poll_result=None):
        self.pid = pid
        self._poll_result = poll_result

    def poll(self):
        return self._poll_result


class TegraStatsPermissionRetryTests(unittest.TestCase):
    """A root-owned tegrastats (started via `sudo -n`, per sampler.py:314)
    cannot be signalled by os.kill() from this unprivileged process --
    that raises PermissionError, not ProcessLookupError, since signal
    delivery is gated on uid/CAP_KILL. stop() must retry through sudo
    rather than let that exception escape (which previously could abort
    stop() before the whole descendant set was signalled).
    """

    def setUp(self):
        self.tg = sampler.TegraStats("/tmp", 1000)
        self.tg._proc = _FakeProc(pid=4242)

    def test_permission_error_falls_back_to_sudo_kill_with_numeric_signal(self):
        run_calls = []

        def fake_run(cmd, **kwargs):
            run_calls.append(cmd)
            return subprocess.CompletedProcess(cmd, 0)

        with mock.patch.object(sampler.os, "kill", side_effect=PermissionError), \
             mock.patch.object(sampler.subprocess, "run", side_effect=fake_run):
            self.tg._signal_one(4242, signal.SIGTERM)

        self.assertEqual(run_calls, [["sudo", "-n", "kill", "-15", "4242"]])

    def test_process_lookup_error_does_not_fall_back_to_sudo(self):
        with mock.patch.object(sampler.os, "kill", side_effect=ProcessLookupError), \
             mock.patch.object(sampler.subprocess, "run") as mock_run:
            self.tg._signal_one(4242, signal.SIGTERM)
        mock_run.assert_not_called()

    def test_successful_direct_kill_does_not_fall_back_to_sudo(self):
        with mock.patch.object(sampler.os, "kill") as mock_kill, \
             mock.patch.object(sampler.subprocess, "run") as mock_run:
            self.tg._signal_one(4242, signal.SIGTERM)
        mock_kill.assert_called_once_with(4242, signal.SIGTERM)
        mock_run.assert_not_called()

    def test_sudo_retry_timeout_or_missing_sudo_does_not_raise(self):
        with mock.patch.object(sampler.os, "kill", side_effect=PermissionError):
            with mock.patch.object(
                sampler.subprocess, "run",
                side_effect=subprocess.TimeoutExpired(cmd="sudo", timeout=2),
            ):
                self.tg._signal_one(4242, signal.SIGTERM)  # must not raise
            with mock.patch.object(sampler.subprocess, "run", side_effect=OSError):
                self.tg._signal_one(4242, signal.SIGTERM)  # must not raise

    def test_one_pid_refused_does_not_stop_the_rest_being_signalled(self):
        """The set-based iteration in _signal_descendants must not let a
        PermissionError on one descendant abort signalling the others.
        """
        self.tg._proc = _FakeProc(pid=100)
        attempted = []

        def fake_descendants_of(pid, proc_root="/proc"):
            return [200]

        def fake_kill(pid, sig):
            attempted.append(pid)
            if pid == 100:
                raise PermissionError
            # pid 200 succeeds directly.

        run_calls = []
        with mock.patch.object(procinfo, "descendants_of", side_effect=fake_descendants_of), \
             mock.patch.object(sampler.os, "kill", side_effect=fake_kill), \
             mock.patch.object(
                 sampler.subprocess, "run",
                 side_effect=lambda cmd, **kw: run_calls.append(cmd),
             ):
            self.tg._signal_descendants(signal.SIGTERM)

        self.assertEqual(sorted(attempted), [100, 200])
        self.assertEqual(run_calls, [["sudo", "-n", "kill", "-15", "100"]])

    def test_signals_reach_a_grandchild_two_levels_under_a_sudo_monitor(self):
        """Regression test for a real gap found via live testing against
        actual sudo: self._proc is a monitor whose own real uid is still
        this process's, so it is directly killable, but the process that
        actually becomes root (and anything *it* forks) can be two levels
        down, not one -- children_of alone would miss it, which is why
        _descendant_pids uses descendants_of (the full subtree).
        """
        self.tg._proc = _FakeProc(pid=1)  # sudo monitor, killable directly

        def fake_descendants_of(pid, proc_root="/proc"):
            # 1 -> 2 (execs the real command) -> 3 (a further fork)
            return [2, 3]

        attempted = []

        def fake_kill(pid, sig):
            attempted.append(pid)
            if pid in (2, 3):
                raise PermissionError  # genuinely root-owned

        run_calls = []
        with mock.patch.object(procinfo, "descendants_of", side_effect=fake_descendants_of), \
             mock.patch.object(sampler.os, "kill", side_effect=fake_kill), \
             mock.patch.object(
                 sampler.subprocess, "run",
                 side_effect=lambda cmd, **kw: run_calls.append(cmd),
             ):
            self.tg._signal_descendants(signal.SIGTERM)

        self.assertEqual(sorted(attempted), [1, 2, 3])
        self.assertEqual(
            sorted(run_calls),
            [["sudo", "-n", "kill", "-15", "2"], ["sudo", "-n", "kill", "-15", "3"]],
        )

    def test_still_running_uses_poll_not_signal_zero(self):
        """_still_running must never probe self._proc via os.kill(pid, 0):
        that is just as uid-gated as a real signal, so it would raise
        PermissionError against a root-owned process for the same reason
        a plain terminate/kill does.
        """
        self.tg._proc = _FakeProc(pid=100, poll_result=None)  # still alive
        with mock.patch.object(procinfo, "descendants_of", return_value=[]), \
             mock.patch.object(sampler.os, "kill", side_effect=AssertionError(
                 "must not call os.kill for a liveness probe")):
            self.assertTrue(self.tg._still_running())

        self.tg._proc = _FakeProc(pid=100, poll_result=0)  # exited
        with mock.patch.object(procinfo, "descendants_of", return_value=[]), \
             mock.patch.object(sampler.os, "kill", side_effect=AssertionError(
                 "must not call os.kill for a liveness probe")):
            self.assertFalse(self.tg._still_running())


if __name__ == "__main__":
    unittest.main()
