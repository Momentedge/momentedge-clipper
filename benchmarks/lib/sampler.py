#!/usr/bin/env python3
"""10 Hz /proc sampler + 1 Hz tegrastats capture.

Writes samples.csv (per-process) and system.csv (system-wide) at
--interval-ms cadence, and tees raw `tegrastats` output to tegrastats.log,
into --out DIR. See ../CONTRACT.md ("samples.csv", "system.csv",
"tegrastats", "Component CLI contracts") for the exact format this
implements.

Self-sampling: this process adds its own pid to the tracked set under
role "sampler" by default (see --no-self-sample). The measurement
apparatus runs on the same board whose CPU is attributed to clipper, so
its own cost belongs in samples.csv rather than being an unquantified
charge levied equally against every arm. The orchestrator cannot pass
this pid in -- it does not exist before this process starts -- so this
is the one role the sampler adds to its own --pid list rather than
receiving from run_suite.sh.

Python 3 standard library only, compatible with Python 3.10 and 3.12 --
this runs on the Jetsons being measured, which have no pip.
"""

import argparse
import csv
import json
import os
import shutil
import signal
import subprocess
import sys
import threading
import time

import procinfo

SAMPLES_HEADER = [
    "ts_ns", "role", "pid", "cpu_pct", "utime_s", "stime_s", "rss_kb",
    "vmhwm_kb", "rchar", "wchar", "read_bytes", "write_bytes", "threads",
    "fds",
]

SYSTEM_HEADER = [
    "ts_ns", "cpu_total_pct", "mem_used_kb", "mem_cached_kb", "disk_read_kb",
    "disk_write_kb", "load1",
]

_stop = False


def _on_signal(signum, frame):
    global _stop
    _stop = True


class PidTracker:
    """Samples a fixed set of (role, pid) pairs into samples.csv rows.

    File handles for stat/status/io are opened once per pid and re-read
    via seek(0) every tick. A pid that disappears mid-run (process exit)
    is logged once and dropped permanently -- the orchestrator does not
    support adding pids back once the sampler has started. Any other
    stat/status read error is treated as a skipped row for that tick
    only, with at most one warning per pid so a persistent problem never
    spams stderr.

    io is tracked independently of stat/status/fds, deliberately: some
    kernels (built without CONFIG_TASKSTATS, e.g. JetPack 5 / L4T 36)
    never expose /proc/<pid>/io for *any* process. That is a fact about
    the host, not about a given pid being alive, so it must not drop the
    pid -- stat/status/fds still work fine and are the majority of what
    samples.csv carries. When io is unavailable the four io columns
    (rchar, wchar, read_bytes, write_bytes) are left empty, not zero:
    zero would claim "measured no IO", which on this host would be a
    fabricated result -- empty says "not measurable here". `io_available`
    (an up-front /proc/self/io probe, not something inferred lazily from
    a failed per-pid open) short-circuits every per-pid io open attempt
    once it is known false, and the one-and-only "io unavailable" warning
    for the whole run is guaranteed regardless of which pid happens to
    be sampled first.

    `proc_root` defaults to "/proc" and exists purely so tests can point
    this at a fabricated directory tree without real processes or a
    kernel missing CONFIG_TASKSTATS.
    """

    def __init__(self, pids, io_available=True, proc_root="/proc"):
        # pids: list of (role, pid) tuples, as parsed from --pid role=PID.
        self._roles = {pid: role for role, pid in pids}
        self._handles = {}
        self._prev_cpu = {}
        self._warned = set()
        self._proc_root = proc_root
        self._io_available = io_available
        # If the up-front probe already found io unavailable, that is the
        # one warning for the run -- suppress the (redundant) per-pid one.
        self._io_warned = not io_available
        for pid, role in self._roles.items():
            self._open(pid, role)

    def _path(self, pid, name):
        return os.path.join(self._proc_root, str(pid), name)

    def _open(self, pid, role):
        try:
            stat_f = open(self._path(pid, "stat"), "r")
            status_f = open(self._path(pid, "status"), "r")
        except OSError as exc:
            print(
                "sampler: pid %d (%s) unreadable at start (%s), skipping"
                % (pid, role, exc),
                file=sys.stderr,
            )
            return
        io_f = None
        if self._io_available:
            try:
                io_f = open(self._path(pid, "io"), "r")
            except OSError as exc:
                self._warn_io_once(exc)
        self._handles[pid] = {"stat": stat_f, "status": status_f, "io": io_f}

    def _warn_io_once(self, exc):
        if self._io_warned:
            return
        self._io_warned = True
        print(
            "sampler: /proc/<pid>/io is unavailable on this kernel (%s); "
            "this is expected when CONFIG_TASKSTATS is unset (seen on "
            "JetPack 5 / L4T 36, kernel 5.15). rchar/wchar/read_bytes/"
            "write_bytes will be left EMPTY (not zero) in samples.csv for "
            "every process for the rest of this run -- they are not "
            "measurable here, which is not the same as measuring zero "
            "bytes." % exc,
            file=sys.stderr,
        )

    def sample(self, ts_ns):
        rows = []
        for pid in list(self._handles):
            row = self._sample_one(pid, ts_ns)
            if row is not None:
                rows.append(row)
        return rows

    def _sample_one(self, pid, ts_ns):
        role = self._roles[pid]
        h = self._handles[pid]
        try:
            stat = procinfo.read_pid_stat(h["stat"])
            status = procinfo.read_pid_status(h["status"])
            fds = procinfo.count_fds(pid)
        except (FileNotFoundError, ProcessLookupError):
            print(
                "sampler: pid %d (%s) exited, dropping from sampling"
                % (pid, role),
                file=sys.stderr,
            )
            self._drop(pid)
            return None
        except OSError as exc:
            if pid not in self._warned:
                print(
                    "sampler: pid %d (%s) sample error (%s); will keep "
                    "retrying without further warnings" % (pid, role, exc),
                    file=sys.stderr,
                )
                self._warned.add(pid)
            return None

        io = None
        if h["io"] is not None:
            try:
                io = procinfo.read_pid_io(h["io"])
            except OSError as exc:
                # io was open but became unreadable -- drop just the io
                # handle for this pid, not the whole pid: stat/status/fds
                # (already read above, successfully) are unaffected.
                self._warn_io_once(exc)
                try:
                    h["io"].close()
                except OSError:
                    pass
                h["io"] = None

        cur_ticks = stat["utime_ticks"] + stat["stime_ticks"]
        prev = self._prev_cpu.get(pid)
        pct = (
            procinfo.cpu_pct(prev[0], prev[1], cur_ticks, ts_ns)
            if prev is not None
            else 0.0
        )
        self._prev_cpu[pid] = (cur_ticks, ts_ns)

        return (
            ts_ns,
            role,
            pid,
            round(pct, 3),
            round(stat["utime_ticks"] / procinfo.CLK_TCK, 3),
            round(stat["stime_ticks"] / procinfo.CLK_TCK, 3),
            status["vmrss_kb"],
            status["vmhwm_kb"],
            # io["rchar"] etc. can themselves be None too -- a present io
            # file missing that one field (see procinfo.parse_io). Both
            # None cases render as an empty CSV field via csv.writer.
            io["rchar"] if io is not None else None,
            io["wchar"] if io is not None else None,
            io["read_bytes"] if io is not None else None,
            io["write_bytes"] if io is not None else None,
            stat["num_threads"],
            fds,
        )

    def _drop(self, pid):
        for f in self._handles[pid].values():
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass
        del self._handles[pid]

    def close(self):
        for pid in list(self._handles):
            self._drop(pid)


class SystemSampler:
    """Tracks previous-tick state to turn cumulative /proc counters into
    per-interval deltas for one system.csv row.
    """

    def __init__(self, disk_device):
        self._stat_f = open("/proc/stat", "r")
        self._meminfo_f = open("/proc/meminfo", "r")
        self._diskstats_f = open("/proc/diskstats", "r")
        self._loadavg_f = open("/proc/loadavg", "r")
        self._disk_device = disk_device
        self._prev_cpu = None
        self._prev_disk = None

    def sample(self, ts_ns):
        cur_cpu = procinfo.read_proc_stat_cpu(self._stat_f)
        pct = (
            procinfo.cpu_total_pct(self._prev_cpu, cur_cpu)
            if self._prev_cpu is not None
            else 0.0
        )
        self._prev_cpu = cur_cpu

        meminfo = procinfo.read_meminfo(self._meminfo_f)
        mem_used = procinfo.mem_used_kb(meminfo)
        mem_cached = procinfo.mem_cached_kb(meminfo)

        disk_read_kb = 0.0
        disk_write_kb = 0.0
        if self._disk_device:
            sectors = procinfo.read_diskstats(self._diskstats_f).get(
                self._disk_device
            )
            if sectors is not None:
                if self._prev_disk is not None:
                    disk_read_kb = (sectors[0] - self._prev_disk[0]) * 512 / 1024.0
                    disk_write_kb = (sectors[1] - self._prev_disk[1]) * 512 / 1024.0
                self._prev_disk = sectors

        load1 = procinfo.load1(self._loadavg_f)

        return (
            ts_ns,
            round(pct, 3),
            mem_used,
            mem_cached,
            round(disk_read_kb, 3),
            round(disk_write_kb, 3),
            load1,
        )

    def close(self):
        for f in (self._stat_f, self._meminfo_f, self._diskstats_f, self._loadavg_f):
            try:
                f.close()
            except OSError:
                pass


def tegrastats_commands(exe, interval_ms, environ):
    """The ordered list of argv candidates TegraStats.start() tries.

    Pure and separated from actually spawning anything so the exact
    argv -- especially the sudo/env-passthrough details below -- is
    testable without sudo or tegrastats installed.

    The orchestrator identifies every harness process (for its straggler
    sweep, run after this sampler exits or is killed) by a BENCH_SUITE
    environment variable it sets on this process; a plain child inherits
    it automatically. Going through `sudo`, though, does not: sudo's
    default env_reset policy drops arbitrary environment variables
    before exec'ing the target, and BENCH_SUITE is not in any default
    env_keep list. `--preserve-env=BENCH_SUITE` would only work if the
    target's own sudoers config already allows preserving that specific
    var -- a target-side setting this script cannot assume. Re-injecting
    it via `env VAR=value tegrastats ...` (run *as* the elevated command)
    sidesteps that: it needs nothing from sudoers, only that `env` is on
    PATH, which it always is.
    """
    cmds = [[exe, "--interval", str(interval_ms)]]
    bench_suite = environ.get("BENCH_SUITE")
    sudo_cmd = ["sudo", "-n"]
    if bench_suite is not None:
        sudo_cmd += ["env", "BENCH_SUITE=" + bench_suite]
    sudo_cmd += [exe, "--interval", str(interval_ms)]
    cmds.append(sudo_cmd)
    return cmds


class TegraStats:
    """Runs `tegrastats --interval N`, tee'ing its raw stdout to
    tegrastats.log. Parsing happens later on the workstation (JetPack 5
    and 7 differ in line format); this only captures the raw lines.

    tegrastats may need root. start() tries a plain invocation first,
    then `sudo -n` (both target hosts have passwordless sudo); if neither
    works -- e.g. the binary is missing entirely, as on a dev workstation
    -- it logs one warning and leaves tegrastats.log empty rather than
    failing the run.

    Deliberately launched as a plain child (no setsid / new process
    group/session): the orchestrator finds every harness process via an
    inherited BENCH_SUITE environment variable and, separately, may sweep
    by process-group membership for stragglers left behind by a killed
    sampler -- putting tegrastats in its own session would make it
    invisible to both. That means stop() cannot rely on a single
    `killpg` the way an isolated process group would allow: if sudo was
    used, the direct child is `sudo`, not tegrastats, and this process's
    own group also holds the rest of the harness, so signalling the whole
    group would be wrong. Instead stop() explicitly resolves tegrastats
    itself (a child of the direct child, when sudo is in the way) via
    /proc and signals both by pid.
    """

    def __init__(self, out_dir, interval_ms):
        self._log_path = os.path.join(out_dir, "tegrastats.log")
        self._interval_ms = interval_ms
        self._proc = None
        self._reader = None
        self._log_file = None

    def start(self):
        # Always create tegrastats.log, even empty -- the run's output
        # directory should have a stable set of files regardless of
        # whether tegrastats itself was available on this host.
        self._log_file = open(self._log_path, "w")

        exe = shutil.which("tegrastats")
        if exe is None:
            print(
                "sampler: tegrastats not found on PATH, continuing without "
                "tegrastats capture",
                file=sys.stderr,
            )
            return

        for cmd in tegrastats_commands(exe, self._interval_ms, os.environ):
            try:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
            except OSError:
                continue
            # Give an immediate failure (permission denied, sudo password
            # required with no tty, ...) a moment to surface.
            time.sleep(0.2)
            if proc.poll() is None:
                self._proc = proc
                self._reader = threading.Thread(
                    target=self._pump, args=(proc,), daemon=True
                )
                self._reader.start()
                return
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()

        print(
            "sampler: could not start tegrastats (tried direct invocation "
            "and 'sudo -n'), continuing without it",
            file=sys.stderr,
        )
        self._log_file.close()
        self._log_file = None

    def _pump(self, proc):
        try:
            for line in proc.stdout:
                self._log_file.write(line)
                self._log_file.flush()
        except (OSError, ValueError):
            pass

    def _descendant_pids(self):
        """self._proc's pid plus its whole process subtree.

        `sudo -n cmd` commonly means self._proc (the pid this process
        directly spawned) is a *monitor* that is not, itself, meaningfully
        root-owned (its real uid stays this process's own, which is
        exactly why a plain os.kill against it succeeds) -- the monitor
        forks a separate child that fully becomes root and execs `cmd`,
        which can fork further (e.g. `env VAR=x cmd` execs through `env`,
        and `cmd` may itself be a wrapping shell script). Verified live,
        against real `sudo`: with the stub tegrastats used in testing,
        the actually root-owned processes were two levels below
        self._proc.pid, not one -- children_of alone would have missed
        them, so this walks the full subtree via descendants_of.
        """
        return {self._proc.pid} | set(procinfo.descendants_of(self._proc.pid))

    def _signal_descendants(self, sig):
        for pid in self._descendant_pids():
            self._signal_one(pid, sig)

    def _signal_one(self, pid, sig):
        """Signal one pid, retrying through sudo if a direct signal is
        refused.

        When tegrastats was started via `sudo -n` (both target hosts
        need it), sudo may exec into tegrastats in place -- same pid,
        now owned by root -- or leave tegrastats as a separate root-owned
        child. Either way, `os.kill()` from this unprivileged process
        raises PermissionError, not ProcessLookupError, against a
        root-owned pid: signal delivery is gated on matching uid (or
        CAP_KILL), a completely different check from readability (which
        is why procinfo.descendants_of, a plain /proc read, still works
        fine here regardless of ownership). Retrying via `sudo -n kill`
        mirrors how the process was started in the first place. One pid
        being refused must not stop the rest from being signalled --
        each pid in the set is independent.
        """
        try:
            os.kill(pid, sig)
            return
        except ProcessLookupError:
            return
        except PermissionError:
            pass
        try:
            subprocess.run(
                ["sudo", "-n", "kill", "-%d" % int(sig), str(pid)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=2,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass

    def _still_running(self):
        if procinfo.descendants_of(self._proc.pid):
            return True
        # Not os.kill(pid, 0): signal 0 is still gated on uid/CAP_KILL, so
        # it raises PermissionError (not just ProcessLookupError) against
        # a root-owned self._proc for the same reason _signal_one does.
        # poll() is waitpid-based, which this process is always allowed
        # to do on its own direct child regardless of what uid that child
        # later assumed via sudo/exec.
        return self._proc.poll() is None

    def stop(self):
        if self._proc is not None:
            self._signal_descendants(signal.SIGTERM)
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass
            # Re-check regardless of whether the direct child (which may
            # be `sudo`, not tegrastats) exited in time: sudo can forward
            # the signal and exit immediately while tegrastats is still
            # briefly alive, reparented -- don't assume the direct
            # child's exit means the whole tree is gone.
            if self._still_running():
                self._signal_descendants(signal.SIGKILL)
                try:
                    self._proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
            if self._reader is not None:
                self._reader.join(timeout=2)
            if self._proc.stdout is not None:
                self._proc.stdout.close()
        if self._log_file is not None:
            self._log_file.close()


def _write_io_accounting_fact(out_dir, available):
    """Record whether /proc/<pid>/io exists on this kernel as an explicit,
    machine-readable fact about the host -- not something the orchestrator
    or analysis has to infer from every io column in samples.csv coming
    back empty. Always written, so its absence never itself has to be
    interpreted as "unknown".

    Not yet in CONTRACT.md's per-run file list (samples.csv/system.csv/
    tegrastats.log) -- this is a new, small sidecar this fix introduces;
    flagging that in case the contract should record it explicitly.
    """
    fact = {
        "io_accounting_available": available,
        "probed_via": "/proc/self/io",
    }
    with open(os.path.join(out_dir, "io_accounting.json"), "w") as f:
        json.dump(fact, f)
        f.write("\n")


def _parse_pid_arg(value):
    role, sep, pid_str = value.partition("=")
    if not sep or not role:
        raise argparse.ArgumentTypeError(
            "expected role=PID, got %r" % value
        )
    try:
        pid = int(pid_str)
    except ValueError:
        raise argparse.ArgumentTypeError(
            "PID must be an integer, got %r" % value
        )
    return role, pid


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Sample /proc for a fixed set of pids at --interval-ms, plus "
            "system-wide /proc counters and raw tegrastats, until SIGTERM."
        ),
    )
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument(
        "--pid",
        dest="pid",
        action="append",
        type=_parse_pid_arg,
        required=True,
        metavar="role=PID",
        help="one per process to sample, e.g. --pid clipper=1234; repeatable",
    )
    parser.add_argument(
        "--interval-ms", type=int, default=100,
        help="sample interval for samples.csv/system.csv (default: 100)",
    )
    parser.add_argument(
        "--tegrastats-interval-ms", type=int, default=1000,
        help="tegrastats --interval (default: 1000)",
    )
    parser.add_argument(
        "--no-self-sample",
        dest="self_sample",
        action="store_false",
        default=True,
        help="do not add the sampler's own pid to samples.csv under role "
             "'sampler' (default: on -- the measurement apparatus's own "
             "cost is otherwise an unquantified charge against every arm)",
    )
    return parser


def run(args):
    if args.interval_ms <= 0:
        print("sampler: --interval-ms must be positive", file=sys.stderr)
        return 1

    os.makedirs(args.out, exist_ok=True)
    interval_s = args.interval_ms / 1000.0

    io_available = procinfo.probe_io_available()
    _write_io_accounting_fact(args.out, io_available)
    if not io_available:
        print(
            "sampler: /proc/self/io does not exist on this kernel -- "
            "CONFIG_TASKSTATS is likely unset (seen on JetPack 5 / L4T 36, "
            "kernel 5.15). rchar/wchar/read_bytes/write_bytes will be left "
            "EMPTY (not zero) in samples.csv for every process this run; "
            "see io_accounting.json.",
            file=sys.stderr,
        )

    pids = list(args.pid)
    if args.self_sample:
        # The sampler is a 10 Hz Python process reading four files per pid
        # plus several system-wide files, running on the same board whose
        # CPU is being attributed to clipper -- its own cost is otherwise
        # an unquantified charge levied equally against every arm. It adds
        # its own pid here, not in run_suite.sh: the orchestrator cannot
        # know this process's pid before launching it. Reuses the exact
        # same PidTracker path as every other role -- no extra reads, no
        # special-cased row shape, and a read failure against its own
        # /proc entries degrades exactly like any other pid rather than
        # taking the run down.
        pids.append(("sampler", os.getpid()))
    tracker = PidTracker(pids, io_available=io_available)

    disk_device = procinfo.disk_device_for_path(os.path.expanduser("~/bench"))
    if disk_device is None:
        print(
            "sampler: could not resolve the block device backing ~/bench; "
            "disk_read_kb/disk_write_kb will be 0",
            file=sys.stderr,
        )
    sysmon = SystemSampler(disk_device)

    samples_f = open(os.path.join(args.out, "samples.csv"), "w", newline="")
    system_f = open(os.path.join(args.out, "system.csv"), "w", newline="")
    samples_w = csv.writer(samples_f)
    system_w = csv.writer(system_f)
    samples_w.writerow(SAMPLES_HEADER)
    system_w.writerow(SYSTEM_HEADER)

    tegrastats = TegraStats(args.out, args.tegrastats_interval_ms)
    tegrastats.start()

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)

    start = time.monotonic()
    tick = 0
    try:
        while not _stop:
            ts_ns = time.time_ns()

            for row in tracker.sample(ts_ns):
                samples_w.writerow(row)
            system_w.writerow(sysmon.sample(ts_ns))

            samples_f.flush()
            system_f.flush()

            tick += 1
            sleep_for = (start + tick * interval_s) - time.monotonic()
            if sleep_for > 0 and not _stop:
                time.sleep(sleep_for)
    finally:
        samples_f.flush()
        samples_f.close()
        system_f.flush()
        system_f.close()
        tracker.close()
        sysmon.close()
        tegrastats.stop()

    return 0


def main(argv=None):
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
