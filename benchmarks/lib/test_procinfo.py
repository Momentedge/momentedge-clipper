#!/usr/bin/env python3
"""Unit tests for procinfo.py against synthetic /proc fixture text.

Run with: python3 -m unittest test_procinfo -v
(or just: python3 test_procinfo.py)

stdlib-only (unittest, tempfile, os) so this runs the same way on the
Jetsons as on a workstation -- no pytest, no third-party assertion libs.
"""

import os
import tempfile
import unittest

import procinfo


# A realistic /proc/<pid>/stat line, comm containing both a space and an
# internal, unbalanced-looking ')' -- exercises the "split on the *last*
# ')'" requirement from proc(5).
STAT_LINE = (
    "12345 (my (weird) proc) S 1 12345 12345 0 -1 4194304 100 200 5 10 "
    "300 150 30 20 20 0 5 0 987654 123456789 4321 18446744073709551615 "
    "4194304 4198400 140737488347136 140737488346464 140617355173387 0 0 "
    "4096 0 0 0 17 2 0 0 0 0 0 6295552 6295552 21491712 140737488350528 "
    "140737488350588 140737488350588 140737488353771 0\n"
)

STAT_LINE_SIMPLE = (
    "42 (sleep) S 1 42 42 0 -1 4194368 0 0 0 0 0 0 0 0 20 0 1 0 999 "
    "9000000 200 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 3 0 0 0 "
    "0 0 0 0 0 0 0 0 0 0\n"
)

STATUS_TEXT = """\
Name:\tsleep
State:\tS (sleeping)
Tgid:\t12345
Pid:\t12345
PPid:\t1
Threads:\t5
VmPeak:\t   12876 kB
VmSize:\t   12876 kB
VmHWM:\t    3344 kB
VmRSS:\t    3200 kB
VmData:\t    500 kB
Threads:\t5
"""

STATUS_TEXT_MISSING_HWM = """\
Name:\tweird
State:\tR (running)
VmRSS:\t    128 kB
"""

IO_TEXT_FULL = """\
rchar: 108233
wchar: 2048
syscr: 45
syscw: 12
read_bytes: 4096
write_bytes: 0
cancelled_write_bytes: 0
"""

# A real gotcha: some kernels/containers omit rchar/wchar from
# /proc/<pid>/io entirely (e.g. certain hardened configs), leaving only
# the accounting fields that don't require CONFIG_TASK_IO_ACCOUNTING.
IO_TEXT_PARTIAL = """\
syscr: 45
syscw: 12
read_bytes: 8192
write_bytes: 1024
"""

PROC_STAT_TEXT = """\
cpu  1000 200 500 8000 300 0 50 0 0 0
cpu0 500 100 250 4000 150 0 25 0 0 0
cpu1 500 100 250 4000 150 0 25 0 0 0
intr 12345 0 0 0
ctxt 98765
btime 1700000000
processes 4321
"""

# Older-kernel form missing the trailing guest/guest_nice fields.
PROC_STAT_TEXT_SHORT = """\
cpu  1000 200 500 8000 300 0 50
"""

MEMINFO_TEXT = """\
MemTotal:        8000000 kB
MemFree:         2000000 kB
MemAvailable:    5000000 kB
Buffers:          100000 kB
Cached:          1200000 kB
SwapCached:            0 kB
SReclaimable:     300000 kB
Shmem:             50000 kB
"""

DISKSTATS_TEXT = """\
  259       0 nvme0n1 123456 100 987654 4000 654321 200 5432100 8000 0 12000 12000
  259       1 nvme0n1p1 100000 90 800000 3500 600000 180 5000000 7500 0 11000 11000
  259       2 nvme0n1p2 20000 5 150000 400 50000 10 400000 400 0 900 900
   8       0 sda 10 0 100 5 0 0 0 0 0 5 5
"""

# Newer kernels append discard + flush counters after the classic 14
# fields; the parser must keep indexing sectors_read/written positionally
# from the front rather than assuming a fixed line length.
DISKSTATS_TEXT_WITH_DISCARDS = """\
259 1 nvme0n1p1 100100 90 800800 3500 600600 180 5000500 7500 0 11000 11000 5 5 40 20 0 0 0
"""


class ParseStatTests(unittest.TestCase):
    def test_simple(self):
        d = procinfo.parse_stat(STAT_LINE_SIMPLE)
        self.assertEqual(d["pid"], 42)
        self.assertEqual(d["comm"], "sleep")
        self.assertEqual(d["state"], "S")
        self.assertEqual(d["utime_ticks"], 0)
        self.assertEqual(d["stime_ticks"], 0)
        self.assertEqual(d["num_threads"], 1)
        self.assertEqual(d["starttime_ticks"], 999)

    def test_comm_with_spaces_and_parens(self):
        d = procinfo.parse_stat(STAT_LINE)
        self.assertEqual(d["pid"], 12345)
        self.assertEqual(d["comm"], "my (weird) proc")
        self.assertEqual(d["state"], "S")
        self.assertEqual(d["utime_ticks"], 300)
        self.assertEqual(d["stime_ticks"], 150)
        self.assertEqual(d["num_threads"], 5)
        self.assertEqual(d["starttime_ticks"], 987654)

    def test_read_pid_stat_from_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "stat")
            with open(path, "w") as f:
                f.write(STAT_LINE_SIMPLE)
            with open(path, "r") as f:
                parsed = procinfo.read_pid_stat(f)
                # exercise the seek(0)-and-reread path used by the sampler
                parsed_again = procinfo.read_pid_stat(f)
            self.assertEqual(parsed, parsed_again)
            self.assertEqual(parsed["pid"], 42)


class ParseStatusTests(unittest.TestCase):
    def test_full(self):
        d = procinfo.parse_status(STATUS_TEXT)
        self.assertEqual(d["vmrss_kb"], 3200)
        self.assertEqual(d["vmhwm_kb"], 3344)

    def test_missing_field_defaults_zero(self):
        d = procinfo.parse_status(STATUS_TEXT_MISSING_HWM)
        self.assertEqual(d["vmrss_kb"], 128)
        self.assertEqual(d["vmhwm_kb"], 0)


class ParseIoTests(unittest.TestCase):
    def test_full(self):
        d = procinfo.parse_io(IO_TEXT_FULL)
        self.assertEqual(d["rchar"], 108233)
        self.assertEqual(d["wchar"], 2048)
        self.assertEqual(d["read_bytes"], 4096)
        self.assertEqual(d["write_bytes"], 0)

    def test_partial_missing_fields_are_none_not_zero(self):
        # A field absent from a present io file means "not measurable
        # here", the same fact the whole-file-absent case represents --
        # not "measured zero bytes". None (not 0) is how that distinction
        # survives into the row the sampler builds.
        d = procinfo.parse_io(IO_TEXT_PARTIAL)
        self.assertIsNone(d["rchar"])
        self.assertIsNone(d["wchar"])
        self.assertEqual(d["read_bytes"], 8192)
        self.assertEqual(d["write_bytes"], 1024)
        self.assertEqual(d["syscr"], 45)

    def test_genuine_zero_is_distinct_from_absent(self):
        d = procinfo.parse_io(IO_TEXT_FULL)
        # write_bytes: 0 in the fixture is a real measurement, not absence.
        self.assertEqual(d["write_bytes"], 0)
        self.assertIsNotNone(d["write_bytes"])


class ProbeIoAvailableTests(unittest.TestCase):
    def test_present_file_is_available(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "io")
            with open(path, "w") as f:
                f.write(IO_TEXT_FULL)
            self.assertTrue(procinfo.probe_io_available(path))

    def test_missing_file_is_unavailable(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "io")  # never created
            self.assertFalse(procinfo.probe_io_available(path))

    def test_live_self_io_on_this_workstation(self):
        # This dev box's kernel does have CONFIG_TASKSTATS (unlike the nx
        # target, JetPack 5 / L4T 36) -- a live sanity check that the
        # default path argument actually resolves against the real /proc.
        self.assertTrue(procinfo.probe_io_available())
        self.assertTrue(procinfo.probe_io_available("/proc/self/io"))


class CountFdsTests(unittest.TestCase):
    def test_matches_scandir_and_grows(self):
        pid = os.getpid()
        before = procinfo.count_fds(pid)
        with tempfile.TemporaryFile() as extra:
            after = procinfo.count_fds(pid)
            self.assertEqual(after, before + 1)
            self.assertEqual(
                after, len(os.listdir("/proc/%d/fd" % pid))
            )

    def test_dead_pid_raises(self):
        # PID 1 exists but some huge unused PID almost certainly doesn't;
        # find one that doesn't exist without relying on a fixed number.
        dead_pid = 2
        while os.path.exists("/proc/%d" % dead_pid):
            dead_pid *= 7
        with self.assertRaises(OSError):
            procinfo.count_fds(dead_pid)


class CpuPctTests(unittest.TestCase):
    def test_basic_delta(self):
        tck = procinfo.CLK_TCK
        prev_ticks = 1000
        cur_ticks = prev_ticks + int(1.5 * tck)  # 1.5 core-seconds of work
        pct = procinfo.cpu_pct(prev_ticks, 0, cur_ticks, 1_000_000_000)
        self.assertAlmostEqual(pct, 150.0, delta=1.0)

    def test_can_exceed_100_uncapped(self):
        tck = procinfo.CLK_TCK
        pct = procinfo.cpu_pct(0, 0, int(3 * tck), 1_000_000_000)
        self.assertGreater(pct, 200.0)

    def test_zero_interval_is_safe(self):
        self.assertEqual(procinfo.cpu_pct(0, 1000, 100, 1000), 0.0)

    def test_negative_delta_is_safe(self):
        # Should never happen (monotonic counters) but must not go negative
        # or raise if it somehow did (e.g. pid reuse race).
        self.assertEqual(procinfo.cpu_pct(500, 0, 100, 1_000_000_000), 0.0)


class ProcStatCpuTests(unittest.TestCase):
    def test_parse(self):
        d = procinfo.parse_proc_stat_cpu(PROC_STAT_TEXT)
        self.assertEqual(d["user"], 1000)
        self.assertEqual(d["idle"], 8000)
        self.assertEqual(d["iowait"], 300)
        self.assertEqual(d["guest_nice"], 0)

    def test_parse_short_line_defaults_missing_fields(self):
        d = procinfo.parse_proc_stat_cpu(PROC_STAT_TEXT_SHORT)
        self.assertEqual(d["softirq"], 50)
        self.assertEqual(d["steal"], 0)
        self.assertEqual(d["guest"], 0)
        self.assertEqual(d["guest_nice"], 0)

    def test_missing_cpu_line_raises(self):
        with self.assertRaises(ValueError):
            procinfo.parse_proc_stat_cpu("intr 0\nctxt 0\n")

    def test_cpu_total_pct(self):
        prev = procinfo.parse_proc_stat_cpu(PROC_STAT_TEXT)
        # advance idle by 100 and user by 100 over a total of +200 jiffies
        cur = dict(prev)
        cur["idle"] += 100
        cur["user"] += 100
        pct = procinfo.cpu_total_pct(prev, cur)
        self.assertAlmostEqual(pct, 50.0)

    def test_cpu_total_pct_no_progress_is_safe(self):
        prev = procinfo.parse_proc_stat_cpu(PROC_STAT_TEXT)
        cur = dict(prev)
        self.assertEqual(procinfo.cpu_total_pct(prev, cur), 0.0)


class MemInfoTests(unittest.TestCase):
    def test_parse(self):
        d = procinfo.parse_meminfo(MEMINFO_TEXT)
        self.assertEqual(d["MemTotal"], 8000000)
        self.assertEqual(d["Cached"], 1200000)
        self.assertEqual(d["SReclaimable"], 300000)

    def test_mem_used_kb(self):
        d = procinfo.parse_meminfo(MEMINFO_TEXT)
        self.assertEqual(procinfo.mem_used_kb(d), 8000000 - 5000000)

    def test_mem_cached_kb(self):
        d = procinfo.parse_meminfo(MEMINFO_TEXT)
        self.assertEqual(procinfo.mem_cached_kb(d), 1200000 + 300000)

    def test_mem_used_kb_falls_back_without_memavailable(self):
        text = "MemTotal: 1000 kB\nMemFree: 400 kB\n"
        d = procinfo.parse_meminfo(text)
        self.assertEqual(procinfo.mem_used_kb(d), 600)


class DiskStatsTests(unittest.TestCase):
    def test_parse_picks_right_device(self):
        d = procinfo.parse_diskstats(DISKSTATS_TEXT)
        self.assertIn("nvme0n1p1", d)
        self.assertEqual(d["nvme0n1p1"], (800000, 5000000))
        self.assertEqual(d["sda"], (100, 0))

    def test_parse_tolerates_discard_and_flush_fields(self):
        d = procinfo.parse_diskstats(DISKSTATS_TEXT_WITH_DISCARDS)
        self.assertEqual(d["nvme0n1p1"], (800800, 5000500))


class DiskDeviceForPathTests(unittest.TestCase):
    def test_resolves_via_mount_table_dev_source(self):
        with tempfile.TemporaryDirectory() as d:
            mounts = os.path.join(d, "mounts")
            bench = os.path.join(d, "bench")
            os.mkdir(bench)
            with open(mounts, "w") as f:
                f.write("/dev/sda1 / ext4 rw 0 0\n")
                f.write("/dev/nvme0n1p1 %s ext4 rw 0 0\n" % bench)
            dev = procinfo.disk_device_for_path(
                bench, mounts_path=mounts, sys_block_dir="/nonexistent"
            )
            self.assertEqual(dev, "nvme0n1p1")

    def test_longest_prefix_match_wins(self):
        with tempfile.TemporaryDirectory() as d:
            mounts = os.path.join(d, "mounts")
            home = os.path.join(d, "home")
            bench = os.path.join(home, "bench")
            os.makedirs(bench)
            with open(mounts, "w") as f:
                f.write("/dev/sda1 / ext4 rw 0 0\n")
                f.write("/dev/sda2 %s ext4 rw 0 0\n" % home)
                f.write("/dev/nvme0n1p1 %s ext4 rw 0 0\n" % bench)
            dev = procinfo.disk_device_for_path(
                bench, mounts_path=mounts, sys_block_dir="/nonexistent"
            )
            self.assertEqual(dev, "nvme0n1p1")

    def test_unescapes_octal_space_in_mountpoint(self):
        with tempfile.TemporaryDirectory() as d:
            mounts = os.path.join(d, "mounts")
            bench = os.path.join(d, "my bench")
            os.mkdir(bench)
            mnt_escaped = os.path.join(d, "my\\040bench")
            with open(mounts, "w") as f:
                f.write("/dev/nvme0n1p1 %s ext4 rw 0 0\n" % mnt_escaped)
            dev = procinfo.disk_device_for_path(
                bench, mounts_path=mounts, sys_block_dir="/nonexistent"
            )
            self.assertEqual(dev, "nvme0n1p1")

    def test_falls_back_to_sysfs_for_dev_root_alias(self):
        with tempfile.TemporaryDirectory() as d:
            mounts = os.path.join(d, "mounts")
            bench = os.path.join(d, "bench")
            os.mkdir(bench)
            with open(mounts, "w") as f:
                f.write("/dev/root %s ext4 rw 0 0\n" % bench)

            st = os.stat(bench)
            major, minor = os.major(st.st_dev), os.minor(st.st_dev)
            sys_block_dir = os.path.join(d, "sys_dev_block")
            os.mkdir(sys_block_dir)
            # A real /sys/dev/block entry is a symlink to something under
            # .../devices/... whose basename is the kernel device name.
            fake_devices_dir = os.path.join(d, "devices", "fake_nvme")
            os.makedirs(fake_devices_dir)
            os.symlink(
                fake_devices_dir,
                os.path.join(sys_block_dir, "%d:%d" % (major, minor)),
            )

            dev = procinfo.disk_device_for_path(
                bench, mounts_path=mounts, sys_block_dir=sys_block_dir
            )
            self.assertEqual(dev, "fake_nvme")

    def test_no_match_returns_none(self):
        with tempfile.TemporaryDirectory() as d:
            mounts = os.path.join(d, "mounts")
            with open(mounts, "w") as f:
                f.write("")
            dev = procinfo.disk_device_for_path(
                os.path.join(d, "bench"),
                mounts_path=mounts,
                sys_block_dir=os.path.join(d, "nonexistent_sys_block"),
            )
            self.assertIsNone(dev)


class ChildrenOfTests(unittest.TestCase):
    def _write_stat(self, root, pid, ppid):
        d = os.path.join(root, str(pid))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "stat"), "w") as f:
            f.write(
                f"{pid} (proc) S {ppid} {pid} {pid} 0 -1 4194368 0 0 0 0 0 0 "
                "0 0 20 0 1 0 999 9000000 200 18446744073709551615 0 0 0 0 "
                "0 0 0 0 0 0 17 3 0 0 0 0 0 0 0 0 0 0 0 0 0\n"
            )

    def test_finds_direct_children_only(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 100, ppid=1)     # unrelated
            self._write_stat(root, 200, ppid=100)    # sudo, child of 100
            self._write_stat(root, 300, ppid=200)    # tegrastats, child of sudo
            self._write_stat(root, 400, ppid=1)      # unrelated
            self.assertEqual(
                sorted(procinfo.children_of(200, proc_root=root)), [300]
            )
            self.assertEqual(
                sorted(procinfo.children_of(100, proc_root=root)), [200]
            )
            self.assertEqual(procinfo.children_of(300, proc_root=root), [])

    def test_no_matches_returns_empty_list(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 100, ppid=1)
            self.assertEqual(procinfo.children_of(999, proc_root=root), [])

    def test_missing_proc_root_returns_empty_list(self):
        self.assertEqual(
            procinfo.children_of(1, proc_root="/nonexistent/proc/root"), []
        )

    def test_non_numeric_entries_are_ignored(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 100, ppid=1)
            os.makedirs(os.path.join(root, "self"), exist_ok=True)  # like real /proc
            self.assertEqual(procinfo.children_of(1, proc_root=root), [100])

    def test_stat_disappearing_mid_scan_is_skipped_not_raised(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 100, ppid=1)
            os.makedirs(os.path.join(root, "999"))  # dir exists, no stat file
            self.assertEqual(procinfo.children_of(1, proc_root=root), [100])


class DescendantsOfTests(unittest.TestCase):
    def _write_stat(self, root, pid, ppid):
        d = os.path.join(root, str(pid))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "stat"), "w") as f:
            f.write(
                f"{pid} (proc) S {ppid} {pid} {pid} 0 -1 4194368 0 0 0 0 0 0 "
                "0 0 20 0 1 0 999 9000000 200 18446744073709551615 0 0 0 0 "
                "0 0 0 0 0 0 17 3 0 0 0 0 0 0 0 0 0 0 0 0 0\n"
            )

    def test_finds_the_whole_subtree_not_just_direct_children(self):
        # 1 (sudo monitor) -> 2 (execs the real command) -> 3 (a further
        # fork, e.g. the command is itself a wrapping shell script) ->
        # 4, 5 (two of its own children). Mirrors the real depth found
        # testing against actual `sudo -n` in sampler.py's TegraStats.
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 1, ppid=0)
            self._write_stat(root, 2, ppid=1)
            self._write_stat(root, 3, ppid=2)
            self._write_stat(root, 4, ppid=3)
            self._write_stat(root, 5, ppid=3)
            self._write_stat(root, 99, ppid=0)  # unrelated, no relation to 1
            self.assertEqual(
                sorted(procinfo.descendants_of(1, proc_root=root)),
                [2, 3, 4, 5],
            )
            self.assertEqual(
                sorted(procinfo.descendants_of(2, proc_root=root)), [3, 4, 5]
            )
            self.assertEqual(procinfo.descendants_of(4, proc_root=root), [])

    def test_leaf_with_no_descendants_returns_empty_list(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 1, ppid=0)
            self.assertEqual(procinfo.descendants_of(1, proc_root=root), [])

    def test_no_such_pid_returns_empty_list(self):
        with tempfile.TemporaryDirectory() as root:
            self._write_stat(root, 1, ppid=0)
            self.assertEqual(procinfo.descendants_of(999, proc_root=root), [])

    def test_missing_proc_root_returns_empty_list(self):
        self.assertEqual(
            procinfo.descendants_of(1, proc_root="/nonexistent/proc/root"), []
        )


class LoadAvgTests(unittest.TestCase):
    def test_parse(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "loadavg")
            with open(path, "w") as f:
                f.write("1.23 0.98 0.50 2/345 6789\n")
            self.assertAlmostEqual(procinfo.load1(path), 1.23)


if __name__ == "__main__":
    unittest.main()
