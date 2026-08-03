#!/usr/bin/env python3
"""Busy-loop CPU contention generator, pinned one core each.

Forks `--cores` children, each pinned via `os.sched_setaffinity` to a distinct
core and spinning a tight loop at 100% of that core, to simulate a perception
stack occupying cores in the `contention` scenario. Each busy loop has no exit
condition of its own — shutdown relies entirely on the parent forwarding
SIGTERM/SIGINT — so the signal handler is installed, and `children` exists,
before the fork loop runs: a signal landing mid-fork-loop, or `os.fork()`
itself failing partway (e.g. EAGAIN under process pressure), must still have
somewhere to route already-forked pids for cleanup rather than orphan them to
spin their core at 100% forever.

While running, `children` is polled roughly once a second for an early exit
(crash, OOM kill, a `sched_setaffinity` failure). An unnoticed one would
silently turn `--cores N` into N-1 for the rest of the run while hog.py still
exits 0 — a contention scenario recorded as more contention than it actually
measured. An unexpected child exit is logged and makes the final exit code
non-zero.
"""
import argparse
import os
import signal
import sys
import time


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--cores", required=True, type=int, help="number of cores to occupy")
    return parser.parse_args(argv)


def _busy_loop(core):
    # Restore default disposition for the signals the parent handles: fork()
    # copies the parent's handler registration too, and the parent registers
    # its SIGTERM/SIGINT handler before forking (see main()). Left unreset, a
    # child would inherit that handler, so the parent's SIGTERM would run the
    # handler harmlessly instead of killing the child — the loop below has no
    # exit condition of its own, so it would spin forever, immune to SIGTERM.
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    os.sched_setaffinity(0, {core})
    while True:
        pass


def main(argv=None):
    args = parse_args(argv)
    if args.cores <= 0:
        print("hog.py: --cores must be positive", file=sys.stderr)
        return 1

    available = sorted(os.sched_getaffinity(0))
    if args.cores > len(available):
        print(
            f"hog.py: --cores {args.cores} exceeds {len(available)} available core(s) ({available})",
            file=sys.stderr,
        )
        return 1

    cores = available[: args.cores]

    children = []
    stopping = {"flag": False}

    def _stop(signum, frame):
        stopping["flag"] = True
        for pid in children:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        for core in cores:
            pid = os.fork()
            if pid == 0:
                _busy_loop(core)
                os._exit(0)  # unreachable — _busy_loop never returns
            children.append(pid)
    except OSError as exc:
        print(f"hog.py: fork() failed after {len(children)}/{len(cores)} busy loop(s) ({exc})", file=sys.stderr)
    finally:
        if len(children) < len(cores):
            _stop(None, None)

    if len(children) < len(cores):
        for pid in list(children):
            try:
                os.waitpid(pid, 0)
            except ChildProcessError:
                pass
        return 1

    pid_to_core = dict(zip(children, cores))
    print(f"hog.py: {len(children)} busy loop(s) pinned to cores {cores} — Ctrl-C to stop", file=sys.stderr)

    unexpected_exit = False
    while children and not stopping["flag"]:
        for pid in list(children):
            reaped_pid, status = os.waitpid(pid, os.WNOHANG)
            if reaped_pid == 0:
                continue
            children.remove(pid)
            if not stopping["flag"]:
                unexpected_exit = True
                exit_code = os.waitstatus_to_exitcode(status)
                print(
                    f"hog.py: busy loop for core {pid_to_core[pid]} (pid {pid}) exited "
                    f"unexpectedly (exit code {exit_code}) — contention on that core has silently dropped out",
                    file=sys.stderr,
                )
        if children and not stopping["flag"]:
            time.sleep(1)

    _stop(None, None)  # idempotent: forwards SIGTERM to whatever is still left
    for pid in list(children):
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass

    return 1 if unexpected_exit else 0


if __name__ == "__main__":
    sys.exit(main())
