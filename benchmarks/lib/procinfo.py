"""Parsers for /proc: per-process stat/status/io/fd, and system-wide
/proc/stat, /proc/meminfo, /proc/diskstats, /proc/loadavg.

Python 3 standard library only, and syntax compatible with both Python 3.10
(Ubuntu 22.04 / Humble host) and 3.12 (Ubuntu 24.04 / Jazzy host) -- this
module runs on the Jetsons being measured, which have no pip and no
third-party packages.

Every function here is a pure parser: it takes either a path or an already
open file object and returns a plain dict/tuple. Callers that sample at
10 Hz (see sampler.py) open each /proc file once and re-read it via
seek(0) on every tick rather than paying an open()/close() pair per
sample; passing a path instead (as the unit tests mostly do) works too.
"""

import os

CLK_TCK = os.sysconf("SC_CLK_TCK")


def _read_all(path_or_file):
    """Return the full text of a /proc file, given a path or an open file.

    If given an open file object, seeks to 0 first so the same handle can
    be re-read on every tick instead of being reopened.
    """
    if hasattr(path_or_file, "seek"):
        path_or_file.seek(0)
        return path_or_file.read()
    with open(path_or_file, "r") as f:
        return f.read()


def _first_int(text):
    """Pull the first run of digits out of e.g. 'VmRSS:   1234 kB'."""
    for tok in text.split():
        if tok.lstrip("-").isdigit():
            return int(tok)
    return 0


# ---------------------------------------------------------------------------
# /proc/<pid>/stat
# ---------------------------------------------------------------------------

def parse_stat(text):
    """Parse /proc/<pid>/stat text.

    Returns a dict with pid, comm, state, utime_ticks, stime_ticks,
    num_threads, starttime_ticks (all ticks are raw jiffies, i.e. divide
    by CLK_TCK for seconds).

    The comm field is the process name in parentheses and may itself
    contain spaces or unbalanced parentheses (e.g. a script named
    "weird)name"), so the split point used here is the *last* ')' on the
    line -- not the first -- exactly as the proc(5) man page recommends.
    """
    open_paren = text.index("(")
    close_paren = text.rindex(")")
    pid = int(text[:open_paren].strip())
    comm = text[open_paren + 1:close_paren]
    rest = text[close_paren + 1:].split()
    # rest[0] is field 3 (state); fields below are indexed from there.
    return {
        "pid": pid,
        "comm": comm,
        "state": rest[0],
        "ppid": int(rest[1]),              # field 4
        "utime_ticks": int(rest[11]),      # field 14
        "stime_ticks": int(rest[12]),      # field 15
        "num_threads": int(rest[17]),      # field 20
        "starttime_ticks": int(rest[19]),  # field 22
    }


def read_pid_stat(pid_or_file):
    """parse_stat() given a pid (int) or an open /proc/<pid>/stat file."""
    src = "/proc/%d/stat" % pid_or_file if isinstance(pid_or_file, int) else pid_or_file
    return parse_stat(_read_all(src))


# ---------------------------------------------------------------------------
# /proc/<pid>/status
# ---------------------------------------------------------------------------

def parse_status(text):
    """Parse /proc/<pid>/status text into a dict with vmrss_kb, vmhwm_kb.

    Both are '<N> kB' lines. A field that is genuinely absent (seen on
    some kernel configs) comes back as 0 rather than raising.
    """
    out = {"vmrss_kb": 0, "vmhwm_kb": 0}
    for line in text.splitlines():
        if line.startswith("VmRSS:"):
            out["vmrss_kb"] = _first_int(line)
        elif line.startswith("VmHWM:"):
            out["vmhwm_kb"] = _first_int(line)
    return out


def read_pid_status(pid_or_file):
    src = "/proc/%d/status" % pid_or_file if isinstance(pid_or_file, int) else pid_or_file
    return parse_status(_read_all(src))


# ---------------------------------------------------------------------------
# /proc/<pid>/io
# ---------------------------------------------------------------------------

_IO_FIELDS = ("rchar", "wchar", "syscr", "syscw", "read_bytes", "write_bytes",
              "cancelled_write_bytes")


def parse_io(text):
    """Parse /proc/<pid>/io text.

    Real /proc/<pid>/io files can lack fields -- some kernels/containers
    omit rchar/wchar, or omit the CONFIG_TASK_IO_ACCOUNTING fields
    (read_bytes/write_bytes/cancelled_write_bytes) entirely. A field
    missing from the text comes back as None, not 0: 0 would claim
    "measured zero bytes" for something that was never measured at all --
    exactly the fabrication the whole-file-absent case (see
    probe_io_available / the sampler's io_available handling) already
    takes care to avoid. None renders as an empty CSV field, the same as
    the whole-file-absent case, so both stay distinguishable from a
    genuine zero measurement while looking identical in samples.csv --
    which is the correct outcome, since from the analyst's side "this
    kernel never reports it" and "this field isn't in this particular
    io file" are the same fact: not measurable here.
    """
    out = {k: None for k in _IO_FIELDS}
    for line in text.splitlines():
        key, sep, value = line.partition(":")
        if not sep:
            continue
        key = key.strip()
        if key in out:
            try:
                out[key] = int(value.strip())
            except ValueError:
                pass
    return out


def read_pid_io(pid_or_file):
    src = "/proc/%d/io" % pid_or_file if isinstance(pid_or_file, int) else pid_or_file
    return parse_io(_read_all(src))


def probe_io_available(path="/proc/self/io"):
    """Whether /proc/<pid>/io exists on this kernel at all.

    Some kernels (built without CONFIG_TASKSTATS -- seen on JetPack 5 /
    L4T 36, kernel 5.15) never expose /proc/<pid>/io for any process, not
    just some. Probing /proc/self/io once gives an authoritative,
    host-wide answer up front, instead of discovering the same fact
    independently -- and easy to miss -- for every sampled pid. This is
    a *different* condition from the one parse_io() tolerates: a present
    io file missing a handful of fields still has real per-field data;
    a host with no io file at all has none, for anyone.
    """
    try:
        _read_all(path)
        return True
    except OSError:
        return False


# ---------------------------------------------------------------------------
# /proc/<pid>/fd
# ---------------------------------------------------------------------------

def count_fds(pid):
    """Count open file descriptors for pid via /proc/<pid>/fd.

    Raises whatever os.scandir raises (FileNotFoundError, ProcessLookupError,
    PermissionError, ...) if the process is gone or unreadable -- callers
    treat any exception here the same as a failed stat/status/io read.
    """
    n = 0
    with os.scandir("/proc/%d/fd" % pid) as it:
        for _ in it:
            n += 1
    return n


# ---------------------------------------------------------------------------
# per-process CPU percent
# ---------------------------------------------------------------------------

def cpu_pct(prev_ticks, prev_ts_ns, cur_ticks, cur_ts_ns):
    """Percent of one core used between two (utime+stime) samples.

    Not clamped to 100: a busy multithreaded process legitimately reports
    more than one core's worth of work.
    """
    dt_s = (cur_ts_ns - prev_ts_ns) / 1e9
    if dt_s <= 0:
        return 0.0
    d_ticks = cur_ticks - prev_ticks
    if d_ticks < 0:
        return 0.0
    return 100.0 * (d_ticks / CLK_TCK) / dt_s


# ---------------------------------------------------------------------------
# /proc/stat (system-wide CPU)
# ---------------------------------------------------------------------------

_CPU_STAT_FIELDS = ("user", "nice", "system", "idle", "iowait", "irq",
                     "softirq", "steal", "guest", "guest_nice")


def parse_proc_stat_cpu(text):
    """Parse the aggregate 'cpu ' line of /proc/stat into a dict of jiffies.

    This first line is already summed across all cores, so a percentage
    derived from it is naturally normalised to 0-100% regardless of core
    count (unlike the per-core 'cpu0', 'cpu1', ... lines).
    """
    for line in text.splitlines():
        if line.startswith("cpu "):
            parts = line.split()[1:]
            out = {}
            for i, name in enumerate(_CPU_STAT_FIELDS):
                out[name] = int(parts[i]) if i < len(parts) else 0
            return out
    raise ValueError("no aggregate 'cpu ' line in /proc/stat")


def read_proc_stat_cpu(path_or_file="/proc/stat"):
    return parse_proc_stat_cpu(_read_all(path_or_file))


def cpu_total_pct(prev, cur):
    """System-wide CPU busy percent (0-100) between two /proc/stat samples."""
    prev_idle = prev["idle"] + prev["iowait"]
    cur_idle = cur["idle"] + cur["iowait"]
    d_total = sum(cur.values()) - sum(prev.values())
    if d_total <= 0:
        return 0.0
    d_idle = cur_idle - prev_idle
    return 100.0 * (d_total - d_idle) / d_total


# ---------------------------------------------------------------------------
# /proc/meminfo
# ---------------------------------------------------------------------------

def parse_meminfo(text):
    """Parse /proc/meminfo into {field_name: kB_value} (int), e.g.
    {'MemTotal': 8318000, 'MemAvailable': 5000000, 'Cached': 900000, ...}.
    """
    out = {}
    for line in text.splitlines():
        key, sep, value = line.partition(":")
        if sep:
            out[key.strip()] = _first_int(value)
    return out


def read_meminfo(path_or_file="/proc/meminfo"):
    return parse_meminfo(_read_all(path_or_file))


def mem_used_kb(meminfo):
    """'Used' memory the way free(1) reports it: total minus what the
    kernel considers available for new allocations. This already accounts
    for reclaimable caches, unlike a naive MemTotal-MemFree.
    """
    total = meminfo.get("MemTotal", 0)
    avail = meminfo.get("MemAvailable", meminfo.get("MemFree", 0))
    return max(0, total - avail)


def mem_cached_kb(meminfo):
    """Page cache plus reclaimable slab -- free(1)'s 'buff/cache' minus
    Buffers, i.e. the cache that shrinks first under memory pressure.
    """
    return meminfo.get("Cached", 0) + meminfo.get("SReclaimable", 0)


# ---------------------------------------------------------------------------
# /proc/diskstats
# ---------------------------------------------------------------------------

def parse_diskstats(text):
    """Parse /proc/diskstats into {device_name: (sectors_read, sectors_written)}.

    Sectors are always 512 bytes regardless of the device's actual block
    size (a /proc/diskstats convention, not a hardware fact). Indexing is
    positional from the start of the line (major, minor, name, then the
    counters), so it does not care whether the kernel also emits the
    newer discard/flush fields at the end of the line.
    """
    out = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 10:
            continue
        name = parts[2]
        sectors_read = int(parts[5])
        sectors_written = int(parts[9])
        out[name] = (sectors_read, sectors_written)
    return out


def read_diskstats(path_or_file="/proc/diskstats"):
    return parse_diskstats(_read_all(path_or_file))


def _unescape_mount_field(field):
    """Undo the octal escaping /proc/mounts uses for spaces etc. in paths."""
    return (field.replace("\\040", " ").replace("\\011", "\t")
                 .replace("\\012", "\n").replace("\\134", "\\"))


def disk_device_for_path(path, mounts_path="/proc/mounts",
                          sys_block_dir="/sys/dev/block"):
    """Resolve the /proc/diskstats device name backing `path`'s filesystem.

    Primary method: parse /proc/mounts (the mount table) for the longest
    matching mountpoint and take its source device. Falls back to
    resolving the mountpoint's backing device through
    <sys_block_dir>/<major>:<minor>, which also copes with "/dev/root"
    -style aliases some board bootloaders substitute for the real device
    node. Returns None (never raises) if neither resolves, e.g. because
    `path` is not on a real block device at all.
    """
    target = os.path.realpath(path)
    best_mnt = None
    best_src = None
    try:
        with open(mounts_path, "r") as f:
            for line in f:
                fields = line.split()
                if len(fields) < 2:
                    continue
                src, mnt = fields[0], _unescape_mount_field(fields[1])
                mnt = mnt.rstrip("/") or "/"
                if target == mnt or target.startswith(mnt + ("" if mnt == "/" else "/")):
                    if best_mnt is None or len(mnt) > len(best_mnt):
                        best_mnt, best_src = mnt, src
    except OSError:
        pass

    if best_src and best_src.startswith("/dev/") and best_src != "/dev/root":
        return os.path.basename(os.path.realpath(best_src))

    probe = best_mnt or target
    try:
        st = os.stat(probe)
        major, minor = os.major(st.st_dev), os.minor(st.st_dev)
        link = os.readlink("%s/%d:%d" % (sys_block_dir, major, minor))
        return os.path.basename(link)
    except OSError:
        return None


# ---------------------------------------------------------------------------
# /proc/loadavg
# ---------------------------------------------------------------------------

def load1(path_or_file="/proc/loadavg"):
    return float(_read_all(path_or_file).split()[0])


# ---------------------------------------------------------------------------
# process tree (cleanup helper, not part of the per-tick sampling path)
# ---------------------------------------------------------------------------

def _stat_by_pid(proc_root):
    """{pid: parse_stat() dict} for every currently-scannable pid under
    proc_root, via one pass over the directory. Shared by children_of and
    descendants_of so both cost exactly one /proc scan regardless of how
    many pids they end up asking about. A pid that disappears mid-scan,
    or whose stat is briefly unparseable, is skipped rather than raising:
    both are normal races, not faults.
    """
    out = {}
    try:
        entries = os.listdir(proc_root)
    except OSError:
        return out
    for name in entries:
        if not name.isdigit():
            continue
        try:
            out[int(name)] = parse_stat(_read_all(os.path.join(proc_root, name, "stat")))
        except (OSError, ValueError, IndexError):
            continue
    return out


def children_of(ppid, proc_root="/proc"):
    """Direct children of ppid only -- see descendants_of for the whole
    subtree. This is a one-off, whole-/proc scan -- fine for cleanup, not
    something to call every sampling tick.
    """
    stats = _stat_by_pid(proc_root)
    return [pid for pid, st in stats.items() if st["ppid"] == ppid]


def descendants_of(pid, proc_root="/proc"):
    """Every descendant of pid -- children, grandchildren, and so on --
    from a single /proc scan.

    children_of alone is not enough for cleanup through an intermediate
    wrapper: `sudo -n cmd` commonly means the pid this process directly
    spawned is a *monitor* that itself forks a further child to actually
    become the target user and exec `cmd` -- and `cmd` may itself be a
    shell script that forks more. Signalling only direct children can
    miss the actual work two or three levels down. Not for the per-tick
    sampling path -- this is a whole-tree cleanup helper.
    """
    stats = _stat_by_pid(proc_root)
    children_map = {}
    for cpid, st in stats.items():
        children_map.setdefault(st["ppid"], []).append(cpid)

    result = []
    frontier = [pid]
    while frontier:
        next_frontier = []
        for p in frontier:
            for child in children_map.get(p, []):
                result.append(child)
                next_frontier.append(child)
        frontier = next_frontier
    return result
