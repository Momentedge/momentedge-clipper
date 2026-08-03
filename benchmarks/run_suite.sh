#!/usr/bin/env bash
# Resumable benchmark orchestrator: runs the scenario matrix, one run at a time.
#
# `scenarios.py` decides *what* runs; this script owns *how* a run is measured
# and whether the result counts. One run is a fixed sequence — clean state,
# recorder, load, clipper, warmup, sampler, triggers, measure, teardown — and
# it produces a directory under ~/bench/results/<run_id>/ whose `run.json` is
# marked `"complete": true` only when the run actually produced what its
# scenario promised. A run that did not is left incomplete with a reason; it is
# never quietly recorded as data.
#
# Run it under tmux — the suite ignores SIGHUP and reads no stdin, so an ssh
# disconnect does not stop it, and `--resume` picks up where a kill left off.
#
#   ./run_suite.sh --host nano --reps 3 --resume
#   ./run_suite.sh --host nano --only '*ten_windows*heavy*' --dry-run
#
# The caller provides the ROS 2 environment (`. /opt/ros/<distro>/setup.bash`,
# or the Nix dev shell); this script never creates one.
set -euo pipefail

# Job control, which this script needs for one specific reason: without it a
# shell starts every background job with SIGINT and SIGQUIT set to *ignored*,
# and that disposition is inherited across exec and cannot be restored by a
# handler in the child. `kill -INT` to the recorder would then be a silent
# no-op — the signal that makes rosbag2 write its mcap summary would never
# arrive, and every recording would end up closed by SIGTERM or SIGKILL with
# no summary section. With job control on, each background job gets its own
# process group and the default dispositions, and SIGINT works as intended.
set -m

# EPOCHREALTIME and every numeric comparison below assume a C locale decimal
# point, and the log is read by machine.
export LC_ALL=C

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# --------------------------------------------------------------------------
# Defaults — every path is overridable so the suite can run out of a scratch
# tree without editing the script.
# --------------------------------------------------------------------------
BENCH_ROOT="${BENCH_ROOT:-$HOME/bench}"
RESULTS_DIR="${RESULTS_DIR:-$BENCH_ROOT/results}"
RECORD_DIR="${RECORD_DIR:-$BENCH_ROOT/record}"
CLIP_DIR="${CLIP_DIR:-$BENCH_ROOT/clipped}"
BAG="${BAG:-$BENCH_ROOT/bag}"
CLIPPER_BIN="${CLIPPER_BIN:-/opt/momentedge-clipper/bin/clipper}"

# The harness components. Overridable so the orchestrator can be exercised
# against stand-ins without moving anything in the tree.
SAMPLER="${SAMPLER:-$HERE/lib/sampler.py}"
REPLAY="${REPLAY:-$HERE/load/replay.sh}"
CALIBRATE="${CALIBRATE:-$HERE/load/calibrate.py}"
SYNTH="${SYNTH:-$HERE/load/synth_pub.py}"
HOG="${HOG:-$HERE/load/hog.py}"
DRIVER="${DRIVER:-$HERE/trigger/driver.py}"
PRUNER="${PRUNER:-$HERE/../scripts/prune-record.sh}"

SAMPLE_INTERVAL_MS=100
TEGRASTATS_INTERVAL_MS=1000
# The sampler needs one interval before its first CPU delta is real, so it
# starts a few seconds inside the warmup rather than exactly at the boundary.
SAMPLER_LEAD_S=3
# Free space demanded before a run starts, as a multiple of its own estimate,
# plus a fixed reserve so a full disk never becomes the board's problem. A run
# that does not fit is skipped and says so; it never starts and fills the disk
# under every run that would have followed it.
DISK_MARGIN_PCT=150
DISK_RESERVE_BYTES=$((2 * 1024 * 1024 * 1024))

# Grace given to each process to exit on its own before SIGKILL.
STOP_TIMEOUT_S=25

# The recorder is the one process whose shutdown is itself expensive: SIGINT is
# what makes rosbag2 write the mcap's summary and index, and for an unchunked
# multi-gigabyte file that takes far longer than any other teardown. Killing it
# mid-flush leaves a file with no summary section — which costs the clean
# recording *and* the message counts the drop metric is derived from. The grace
# is therefore a base plus an allowance that scales with what has to be closed.
# How long to wait for the recorder to ACKNOWLEDGE a stop signal. Short on
# purpose: `ros2 bag record` has been observed on both boards ignoring a
# script-sent SIGINT indefinitely, and waiting out a long grace on a signal
# that is not being serviced wastes that time on every one of 138 runs.
RECORDER_ACK_S=15
# How long to wait for the mcap to actually CLOSE once a signal has landed.
# This is the wait that must be generous.
RECORDER_CLOSE_BASE_S=120
RECORDER_CLOSE_MBS=200          # assumed summary-write throughput
RECORDER_CLOSE_CAP_S=600
RECORDER_SIGKILL_GRACE_S=15

# Below this fraction of the pinned maximum, a CPU is throttling rather than
# merely being sampled between transitions.
CPU_FREQ_THROTTLE_TOLERANCE_PCT=95

HOST=""
REPS=3
ONLY=""
DRY_RUN=""
RESUME=""
INCLUDE_SOAK=""
LIST_ONLY=""
PRUNE_CLIPS_MODE="auto"

usage() {
  cat <<'EOF'
run_suite.sh — run the benchmark scenario matrix, one run at a time.

  --host nano|nx     which board's matrix to expand (required)
  --only GLOB        fnmatch pattern on run_id; runs only the matches
  --reps N           repetitions per arm (default 3)
  --resume           skip runs whose run.json already says "complete": true
  --include-soak     add the 4 h soak arm (excluded by default)
  --dry-run          print what each run would execute; touch nothing
  --list             print the run ids in order and exit
  --prune-clips      unlink each clip once it is announced and its size is
                     recorded, bounding the clipped directory to the windows
                     being cut concurrently (default: auto — on only when the
                     run would otherwise not fit)
  --no-prune-clips   never prune; a run that then does not fit is skipped
  -h, --help         this text

Environment (all optional):
  BENCH_ROOT    root of the bench tree          (default ~/bench)
  RESULTS_DIR   one directory per run           (default $BENCH_ROOT/results)
  RECORD_DIR    the recorder's output           (default $BENCH_ROOT/record)
  CLIP_DIR      clipper's output                (default $BENCH_ROOT/clipped)
  BAG           replay source bag               (default $BENCH_ROOT/bag)
  CLIPPER_BIN   clipper binary                  (default /opt/momentedge-clipper/bin/clipper)

Requires a sourced ROS 2 environment. Progress goes to stdout and is appended
to $RESULTS_DIR/suite.log.
EOF
}

while (($#)); do
  case "$1" in
    --host) HOST="${2:?--host needs a value}"; shift 2 ;;
    --only) ONLY="${2:?--only needs a value}"; shift 2 ;;
    --reps) REPS="${2:?--reps needs a value}"; shift 2 ;;
    --resume) RESUME=1; shift ;;
    --include-soak) INCLUDE_SOAK=1; shift ;;
    --dry-run) DRY_RUN=1; shift ;;
    --list) LIST_ONLY=1; shift ;;
    --prune-clips) PRUNE_CLIPS_MODE=always; shift ;;
    --no-prune-clips) PRUNE_CLIPS_MODE=never; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n $HOST ]] || { echo "--host is required" >&2; usage >&2; exit 2; }

# --------------------------------------------------------------------------
# Small utilities
# --------------------------------------------------------------------------

now_ns() {
  local e=${EPOCHREALTIME:-}
  if [[ -n $e && $e == *.* ]]; then
    printf '%s%s000\n' "${e%%.*}" "${e#*.}"
  else
    date +%s%N
  fi
}
now_s() { printf '%s\n' "$(( $(now_ns) / 1000000000 ))"; }

log() { printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$*"; }

# Progress lines land on stdout and in the suite log. The log is opened per
# write so a rotated or truncated file never wedges a multi-hour suite.
say() {
  local line
  line="$(log "$@")"
  printf '%s\n' "$line"
  [[ -n $DRY_RUN ]] || printf '%s\n' "$line" >>"$RESULTS_DIR/suite.log"
}

die() { printf 'run_suite: %s\n' "$*" >&2; exit 1; }

quote_cmd() { local out=(); local a; for a in "$@"; do out+=("$(printf '%q' "$a")"); done; printf '%s' "${out[*]}"; }

# A pid can exit between being listed and being read, which is ordinary while
# polling for one to appear; the whole function is quiet so the redirection
# failure never reaches the log.
cmdline_of() {
  [[ -r /proc/$1/cmdline ]] || return 0
  tr '\0' ' ' <"/proc/$1/cmdline" || true
} 2>/dev/null

# Every child is exec'd through this, which restores the default SIGINT and
# SIGQUIT disposition before running the real command.
#
# It has to be done in the child, between fork and exec, because a shell cannot
# do it: a background job inherits SIGINT and SIGQUIT set to *ignored*, an
# inherited SIG_IGN survives exec, and no `trap` in the child can undo it. If
# this shell was itself started in the background — from CI, from another
# script, with `&` — then without this the recorder runs with SIGINT ignored
# and `kill -INT` is a silent no-op, so rosbag2 never writes its mcap summary
# and every recording ends up closed by SIGKILL instead.
#
# SIGHUP is deliberately left as inherited: the suite ignores it so an ssh
# disconnect cannot end a multi-hour run, and its children should too.
# `execvp` replaces this process, so the pid bash reports is the workload's.
SPAWN=(python3 -c '
import os, signal, sys
for sig in (signal.SIGINT, signal.SIGQUIT):
    signal.signal(sig, signal.SIG_DFL)
os.execvp(sys.argv[1], sys.argv[1:])
')

# A pid is "alive" only while it can still do work: a zombie child has already
# finished and must not hold up a measurement window.
proc_alive() {
  local p=$1 st
  [[ -n $p && -d /proc/$p ]] || return 1
  read -r st </proc/"$p"/stat 2>/dev/null || return 1
  st=${st#*") "}
  st=${st%% *}
  [[ $st != Z ]]
}

# Depth-first descendant search. `systemd-run --scope` and any other launcher
# that forks means the pid we spawned is not always the pid we must sample.
find_descendant() {
  local root=$1 pattern=$2 child cmd
  for child in $(pgrep -P "$root" 2>/dev/null || true); do
    cmd=$(cmdline_of "$child")
    if [[ $cmd == *"$pattern"* ]]; then printf '%s\n' "$child"; return 0; fi
    if find_descendant "$child" "$pattern"; then return 0; fi
  done
  return 1
}

# How long a workload process may take to appear. Generous because the bag
# replay copies the bag to /dev/shm on first use before it execs the player.
RESOLVE_TIMEOUT_S=180

# The pid to sample for a role: the launcher itself once it *is* the workload
# (replay.sh execs the player into its own pid), otherwise the descendant that
# is. A wrapper's own cmdline repeats the command it wraps, so `systemd-run`
# and `sudo` are never mistaken for the workload they launched.
resolve_pid() {
  local launcher=$1 pattern=$2 deadline cmd found
  deadline=$(( $(now_s) + RESOLVE_TIMEOUT_S ))
  while (( $(now_s) < deadline )); do
    cmd=$(cmdline_of "$launcher")
    case "$cmd" in
      *systemd-run*|"sudo "*|*" sudo "*) ;;
      *"$pattern"*) printf '%s\n' "$launcher"; return 0 ;;
    esac
    if found=$(find_descendant "$launcher" "$pattern"); then
      printf '%s\n' "$found"; return 0
    fi
    proc_alive "$launcher" || return 1
    sleep 0.5
  done
  return 1
}

bytes_of_dir() {
  local d=$1 out
  [[ -d $d ]] || { printf '0\n'; return 0; }
  out=$(du -sb "$d" 2>/dev/null | awk '{print $1; exit}') || out=0
  printf '%s\n' "${out:-0}"
}

free_bytes() {
  local out
  out=$(df -B1 --output=avail "$1" 2>/dev/null | awk 'NR==2 {print $1}') || out=""
  printf '%s\n' "$out"
}

# --------------------------------------------------------------------------
# Prerequisites
# --------------------------------------------------------------------------

# Everything below runs ROS 2 executables; setup.bash exports ROS_DISTRO, so
# its absence means nothing is sourced yet — fail fast rather than emit an
# opaque "ros2: command not found" halfway into a run.
if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "ROS_DISTRO is unset — source a ROS2 environment first:" >&2
  echo "  . /opt/ros/<distro>/setup.bash" >&2
  exit 1
fi

command -v python3 >/dev/null 2>&1 || die "python3 not on PATH"
command -v ros2 >/dev/null 2>&1 || die "ros2 not on PATH (source setup.bash)"
command -v pgrep >/dev/null 2>&1 || die "pgrep not on PATH (install procps)"
[[ -f "$HERE/scenarios.py" ]] || die "scenarios.py missing next to $0"

if [[ -z $DRY_RUN ]]; then
  mkdir -p "$RESULTS_DIR" "$BENCH_ROOT"
fi

EXPECTED_DISTRO=$(python3 -c '
import sys
sys.path.insert(0, sys.argv[1])
import scenarios
print(scenarios.HOSTS[sys.argv[2]]["distro"])' "$HERE" "$HOST")

if [[ "$ROS_DISTRO" != "$EXPECTED_DISTRO" ]]; then
  say "WARNING: --host $HOST expects ROS_DISTRO=$EXPECTED_DISTRO, sourced env is $ROS_DISTRO"
fi

# --------------------------------------------------------------------------
# Suite-level setup
# --------------------------------------------------------------------------

# Runs are strictly serial per host. Two suites sharing one bench root would
# wipe each other's record directory mid-measurement and record the wreckage as
# data, so the second one refuses to start. The lock is held on a file
# descriptor and released when the process ends, however it ends.
if [[ -z $DRY_RUN ]] && command -v flock >/dev/null 2>&1; then
  exec 9>"$RESULTS_DIR/.suite.lock"
  flock -n 9 || die "another run_suite.sh holds $RESULTS_DIR/.suite.lock — one suite at a time per host"
fi

TMPDIR_SUITE=$(mktemp -d "${TMPDIR:-/tmp}/run_suite.XXXXXX")
# An interrupted suite tears down its own run rather than orphaning it: a
# recorder or clipper left behind would poison the next invocation and keep
# writing to a disk nobody is watching.
cleanup_suite() {
  # A run executes in its own backgrounded subshell, so ending it is what stops
  # the processes it owns; the marker sweep then catches anything it left.
  if [[ -n ${CURRENT_RUN_PID:-} ]]; then
    kill -TERM "$CURRENT_RUN_PID" 2>/dev/null || true
    sleep 2
  fi
  if declare -F stop_all >/dev/null; then stop_all || true; fi
  if [[ -z $DRY_RUN ]] && declare -F kill_stragglers >/dev/null; then
    kill_stragglers >/dev/null || true
  fi
  rm -rf "$TMPDIR_SUITE"
  if declare -F restore_power_mode >/dev/null; then restore_power_mode || true; fi
}
trap cleanup_suite EXIT
# An ssh disconnect must not end a multi-hour suite; tmux keeps the process,
# and ignoring SIGHUP keeps it even without one.
trap '' HUP

PLAN_FILE="$TMPDIR_SUITE/plan.json"
plan_args=(--host "$HOST" --reps "$REPS")
[[ -n $ONLY ]] && plan_args+=(--only "$ONLY")
[[ -n $INCLUDE_SOAK ]] && plan_args+=(--include-soak)
# Only the orchestrator knows the expansion is happening ON the board whose
# matrix it is expanding, so only it may ask for the lowpower modes to be
# resolved against the local nvpmodel configuration. Expanding one host's
# matrix from the other, or from a workstation, must not filter against the
# wrong board's modes — hence a flag rather than autodetection inside
# scenarios.py.
[[ -r /etc/nvpmodel.conf ]] && plan_args+=(--nvpmodel-conf /etc/nvpmodel.conf)
python3 "$HERE/scenarios.py" "${plan_args[@]}" >"$PLAN_FILE"

RUN_COUNT=$(python3 -c 'import json,sys; print(len(json.load(open(sys.argv[1]))["runs"]))' "$PLAN_FILE")
(( RUN_COUNT > 0 )) || die "the expansion selected no runs (--only '$ONLY'?)"

if [[ -n $LIST_ONLY ]]; then
  python3 -c '
import json,sys
for r in json.load(open(sys.argv[1]))["runs"]:
    print(r["order_index"], r["run_id"], sep="\t")' "$PLAN_FILE"
  exit 0
fi

[[ -n $DRY_RUN ]] || cp "$PLAN_FILE" "$RESULTS_DIR/plan-$HOST.json"

# --------------------------------------------------------------------------
# Power state
# --------------------------------------------------------------------------

ORIGINAL_POWER_MODE=""
CURRENT_POWER_MODE=""

read_power_mode() {
  local out
  if command -v nvpmodel >/dev/null 2>&1; then
    out=$(sudo -n nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null || true)
    printf '%s\n' "$out" | awk -F': *' '/NV Power Mode/ {print $2; exit}'
    return 0
  fi
  printf '\n'
}

# nvpmodel mode ids are board-specific, so a mode is looked up by the name the
# board's own nvpmodel.conf gives it. A name the board does not define is a
# hard error for the runs that asked for it, never a silent substitution.
power_mode_id() {
  local name=$1
  [[ $name == id:* ]] && { printf '%s\n' "${name#id:}"; return 0; }
  [[ -r /etc/nvpmodel.conf ]] || return 1
  awk -v want="$name" '
    /< *POWER_MODEL/ {
      id=""; nm="";
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^ID=/) { id = substr($i, 4) }
        if ($i ~ /^NAME=/) { nm = substr($i, 6) }
      }
      gsub(/[<> ]/, "", nm)
      if (nm == want) { print id; exit }
    }' /etc/nvpmodel.conf | head -1
}

set_power_mode() {
  local name=$1 id readback
  id=$(power_mode_id "$name") || true
  [[ -n $id ]] || { say "power mode '$name' not defined in /etc/nvpmodel.conf"; return 1; }
  sudo -n nvpmodel -m "$id" >/dev/null 2>&1 || { say "nvpmodel -m $id failed (passwordless sudo?)"; return 1; }
  sleep 3
  readback=$(read_power_mode)
  CURRENT_POWER_MODE="$readback"
  [[ -n $readback ]] || return 1
  return 0
}

restore_power_mode() {
  [[ -n $ORIGINAL_POWER_MODE && -n $CURRENT_POWER_MODE ]] || return 0
  [[ "$CURRENT_POWER_MODE" != "$ORIGINAL_POWER_MODE" ]] || return 0
  set_power_mode "$ORIGINAL_POWER_MODE" || say "WARNING: could not restore power mode $ORIGINAL_POWER_MODE"
}

jetson_clocks_on() {
  local minf maxf
  read -r minf </sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq 2>/dev/null || { printf 'unknown\n'; return 0; }
  read -r maxf </sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null || { printf 'unknown\n'; return 0; }
  [[ "$minf" == "$maxf" ]] && printf 'true\n' || printf 'false\n'
}

if [[ -z $DRY_RUN ]]; then
  ORIGINAL_POWER_MODE=$(read_power_mode)
  CURRENT_POWER_MODE="$ORIGINAL_POWER_MODE"
fi

CLIPPER_VERSION="unknown"
if [[ -x $CLIPPER_BIN ]]; then
  CLIPPER_VERSION=$("$CLIPPER_BIN" --version 2>/dev/null | awk '{print $NF; exit}') || CLIPPER_VERSION="unknown"
fi

# --------------------------------------------------------------------------
# Load calibration — cached, because the mapping depends only on the bag
# --------------------------------------------------------------------------

CALIB_DIR="$RESULTS_DIR/calibration"
# Achieved write rates from every completed run, by bitrate label. The snapshot
# arm sizes its cache from these rather than from the nominal target — the
# target is only how the replay rate was chosen, and the rate actually reaching
# the disk differs from it (the source bag's chunks are compressed, the
# recorder writes them out uncompressed). Band ordering puts the mid-bitrate
# runs long before the sweep, so by then this is populated from real runs.
ACHIEVED_DB="$RESULTS_DIR/achieved.json"

record_achieved() {
  local label=$1 mbs=$2
  [[ -n $mbs && $mbs != null ]] || return 0
  python3 - "$ACHIEVED_DB" "$label" "$mbs" <<'PY'
import json, os, sys
path, label, mbs = sys.argv[1], sys.argv[2], float(sys.argv[3])
try:
    db = json.load(open(path, encoding="utf-8"))
except (OSError, ValueError):
    db = {}
db.setdefault(label, []).append(mbs)
tmp = path + ".tmp"
with open(tmp, "w", encoding="utf-8") as fh:
    json.dump(db, fh, indent=2)
os.replace(tmp, path)
PY
}

# The write rate to size the snapshot cache from, and where the figure came
# from. Falls back through the calibration estimate to the nominal target, and
# always says which was used — a cache sized from the wrong number is the
# difference between the sweep bracketing its kill point and missing it.
achieved_mbs_for() {
  local label=$1 target=$2
  python3 - "$ACHIEVED_DB" "$label" "$CALIB_DIR/${target}.json" "$target" <<'PY'
import json, sys
db_path, label, calib_path, target = sys.argv[1:5]
try:
    runs = json.load(open(db_path, encoding="utf-8")).get(label) or []
except (OSError, ValueError):
    runs = []
if runs:
    runs = sorted(runs)
    mid = len(runs) // 2
    median = runs[mid] if len(runs) % 2 else (runs[mid - 1] + runs[mid]) / 2
    print(f"{median}\tmeasured_median_of_{len(runs)}")
else:
    try:
        print(f"{float(json.load(open(calib_path, encoding='utf-8'))['est_mbs'])}\tcalibration_estimate")
    except (OSError, ValueError, KeyError):
        print(f"{float(target)}\tnominal_target")
PY
}

# The board's real usable memory. The nameplate figure ("8 GB") is not what
# MemTotal reports once the carveouts are taken, and a memory bound sized from
# the wrong one either never fires or fires far too early.
mem_total_bytes() {
  local kb
  kb=$(awk '/^MemTotal:/ {print $2; exit}' /proc/meminfo 2>/dev/null) || kb=""
  printf '%s\n' "$(( ${kb:-0} * 1024 ))"
}

calibrated_rate() {
  local target=$1
  local cache="$CALIB_DIR/${target}.json"
  if [[ -n $DRY_RUN ]]; then
    [[ -r $cache ]] || { printf 'CALIBRATED-RATE\n'; return 0; }
  fi
  if [[ ! -r $cache ]]; then
    mkdir -p "$CALIB_DIR"
    python3 "$CALIBRATE" --bag "$BAG" --target-mbs "$target" >"$cache.tmp" \
      || { rm -f "$cache.tmp"; return 1; }
    mv "$cache.tmp" "$cache"
  fi
  python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["rate"])' "$cache"
}

# --------------------------------------------------------------------------
# Straggler control
# --------------------------------------------------------------------------

# Every *workload* the suite starts is launched with this marker in its
# environment, and it survives both an `exec` (replay.sh becomes the player)
# and a transient scope (systemd-run), so a leak is identified by what started
# it rather than by what its command line happens to look like. Matching on
# command lines instead would sweep up any `ros2 bag record` the same user
# happens to be running for entirely unrelated reasons.
#
# It is deliberately not exported from the suite itself: the shell's own
# `sleep`, `du` and `df` helpers must not look like workloads.
BENCH_MARKER=momentedge-bench

# One `pid<TAB>command` line per survivor. Swept before every run as well as
# after: a process leaked by run N poisons run N+1.
#
# The scan runs in one python process rather than as a shell loop over /proc:
# a bash-level walk of every pid's environ costs the better part of a minute on
# a busy machine, and this sits between two measured runs.
find_stragglers() {
  python3 - "$BENCH_MARKER" "$$" <<'PY'
import os, sys

marker = ("BENCH_SUITE=" + sys.argv[1]).encode()
me = sys.argv[2]
for pid in os.listdir("/proc"):
    # Another user's environ is unreadable and a pid can vanish mid-scan; both
    # are ordinary during a sweep.
    if not pid.isdigit() or pid == me:
        continue
    try:
        with open(f"/proc/{pid}/environ", "rb") as fh:
            if marker not in fh.read().split(b"\0"):
                continue
        with open(f"/proc/{pid}/cmdline", "rb") as fh:
            cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        continue
    # The suite's own shell and its subshells inherit the marker too.
    if "run_suite.sh" in cmd:
        continue
    print(pid, cmd[:100], sep="\t")
PY
}

# tegrastats needs root on a Jetson, so the sampler starts it through `sudo -n`
# and in its own session. A root process's environ is unreadable to this user,
# which makes it invisible to the marker sweep, and it outlives a sampler that
# had to be hard-killed — where it would keep polling and keep the previous
# run's tegrastats.log open. It is therefore swept by process name, and through
# sudo when a plain signal is refused. `pgrep -x` matches the executable name
# exactly, so nothing else can be caught by it.
sweep_tegrastats() {
  local pid found=""
  for pid in $(pgrep -x tegrastats 2>/dev/null || true); do
    found="$found $pid(tegrastats)"
    kill -TERM "$pid" 2>/dev/null || sudo -n kill -TERM "$pid" 2>/dev/null || true
  done
  [[ -n $found ]] || return 0
  sleep 2
  for pid in $(pgrep -x tegrastats 2>/dev/null || true); do
    kill -KILL "$pid" 2>/dev/null || sudo -n kill -KILL "$pid" 2>/dev/null || true
  done
  printf '%s\n' "${found# }"
}

# Kills every survivor and prints what it killed, so the caller can name them
# in the run's record and in the suite log.
kill_stragglers() {
  local pid cmd found=""
  while IFS=$'\t' read -r pid cmd; do
    [[ -n $pid ]] || continue
    found="$found $pid(${cmd// /_})"
    kill -TERM "$pid" 2>/dev/null || true
  done < <(find_stragglers)
  local tegra
  tegra=$(sweep_tegrastats)
  found="$found $tegra"
  found=${found## }
  [[ -n ${found// /} ]] || return 0
  sleep 5
  while IFS=$'\t' read -r pid cmd; do
    [[ -n $pid ]] || continue
    kill -KILL "$pid" 2>/dev/null || true
  done < <(find_stragglers)
  sleep 2
  printf '%s\n' "$found"
}

# --------------------------------------------------------------------------
# Per-run process bookkeeping
# --------------------------------------------------------------------------

declare -A LAUNCH_PIDS   # role -> pid we spawned and can wait(1) on
declare -A ROLE_PIDS     # role -> pid that actually does the work
declare -A EXIT_CODES    # role -> exit status once reaped
declare -A KILL_PREFIX   # role -> command prefix needed to signal it
RUN_STRAGGLERS=""
KILLED_HARD=""

reset_procs() {
  LAUNCH_PIDS=(); ROLE_PIDS=(); EXIT_CODES=(); KILL_PREFIX=()
  RUN_STRAGGLERS=""; KILLED_HARD=""
}

# Signals a role's pids. A scope launched through sudo runs as root, so the
# signal has to travel the same way the launch did.
signal_role() {
  local role=$1 sig=$2 pid=$3
  ${KILL_PREFIX[$role]:-} kill -"$sig" "$pid" 2>/dev/null || true
}

# A role is alive while EITHER of its pids is, matching what signal_role
# signals. Under `systemd-run --scope` the launcher and the workload are
# different processes, and watching only the launcher would call an
# actively-closing multi-gigabyte recorder finished — handing it to the
# straggler sweep's blunt 5 s SIGTERM/SIGKILL instead of the scaled grace that
# exists precisely to let it finish.
role_alive() {
  local role=$1
  proc_alive "${LAUNCH_PIDS[$role]:-}" && return 0
  proc_alive "${ROLE_PIDS[$role]:-}" && return 0
  return 1
}

# start_bg ROLE LOGFILE MATCH -- cmd...
#   MATCH is the cmdline fragment identifying the workload process, used when
#   the spawned pid is only a launcher.
start_bg() {
  local role=$1 logfile=$2 match=$3; shift 3
  [[ $1 == "--" ]] && shift
  if [[ -n $DRY_RUN ]]; then
    printf '    %-9s %s\n' "$role" "$(quote_cmd "$@")"
    printf '    %-9s   > %s\n' "" "$logfile"
    return 0
  fi
  BENCH_SUITE="$BENCH_MARKER" BENCH_RUN_ID="${RUN_ID:-}" \
    "${SPAWN[@]}" "$@" </dev/null >>"$logfile" 2>&1 &
  local pid=$!
  LAUNCH_PIDS[$role]=$pid
  local resolved
  if resolved=$(resolve_pid "$pid" "$match"); then
    ROLE_PIDS[$role]=$resolved
  else
    ROLE_PIDS[$role]=""
    return 1
  fi
  return 0
}

# stop_proc ROLE SIGNAL — signal, wait out STOP_TIMEOUT_S, then SIGKILL.
stop_proc() {
  local role=$1 sig=${2:-TERM}
  local pid=${LAUNCH_PIDS[$role]:-} rpid=${ROLE_PIDS[$role]:-}
  [[ -n $pid ]] || return 0
  if [[ -n $DRY_RUN ]]; then printf '    stop %-9s SIG%s\n' "$role" "$sig"; return 0; fi

  if proc_alive "$pid"; then signal_role "$role" "$sig" "$pid"; fi
  if [[ -n $rpid && $rpid != "$pid" ]] && proc_alive "$rpid"; then
    signal_role "$role" "$sig" "$rpid"
  fi

  local limit=$STOP_TIMEOUT_S
  local waited=0
  while proc_alive "$pid" && (( waited < limit )); do sleep 1; waited=$((waited + 1)); done
  if proc_alive "$pid"; then
    say "  !! $role (pid $pid) ignored SIG$sig — SIGKILL"
    RUN_STRAGGLERS="$RUN_STRAGGLERS $role:$pid"
    KILLED_HARD=1
    signal_role "$role" KILL "$pid"
    if [[ -n $rpid && $rpid != "$pid" ]]; then signal_role "$role" KILL "$rpid"; fi
    sleep 2
  fi

  local status=0
  wait "$pid" 2>/dev/null || status=$?
  EXIT_CODES[$role]=$status
  unset 'LAUNCH_PIDS[$role]'
}

# The recorder's own stop: SIGINT, then a bounded escalation.
#
# SIGINT is what makes rosbag2 write the mcap's summary and index, so it is
# given a long grace — scaled by how much file there is to close. But the wait
# is always bounded: `ros2 bag record` has been seen ignoring a programmatic
# SIGINT for minutes while continuing to record, so an unbounded wait would
# stall the suite silently, with the process looking busy rather than stuck.
RECORDER_STOP_SIGNAL=""
RECORDER_STOP_WAIT_S=0

stop_recorder() {
  local pid=${LAUNCH_PIDS[recorder]:-} rpid=${ROLE_PIDS[recorder]:-}
  RECORDER_STOP_SIGNAL=""
  RECORDER_STOP_WAIT_S=0
  [[ -n $pid ]] || return 0
  if [[ -n $DRY_RUN ]]; then printf '    stop %-9s SIGINT then SIGTERM then SIGKILL\n' recorder; return 0; fi

  # Two different waits, and conflating them wastes minutes per run. The
  # ACKNOWLEDGEMENT wait is short: rosbag2 has been observed on both boards and
  # both distros ignoring a script-sent SIGINT indefinitely, so there is no
  # point spending a size-scaled grace on a signal that is not being serviced —
  # escalate quickly to SIGTERM, which does land. The CLOSE wait is the long
  # one, and only starts once a signal has been acknowledged: writing the
  # summary and index of a multi-gigabyte unchunked mcap genuinely takes
  # minutes, and cutting it short costs the recording's summary section.
  local bytes close_grace waited=0 sig
  bytes=$(bytes_of_dir "$RECORD_DIR")
  close_grace=$(( RECORDER_CLOSE_BASE_S + bytes / (RECORDER_CLOSE_MBS * 1000000) ))
  (( close_grace > RECORDER_CLOSE_CAP_S )) && close_grace=$RECORDER_CLOSE_CAP_S
  say "  .. stopping recorder: SIGINT (ack within ${RECORDER_ACK_S}s), then up to ${close_grace}s to close $(( bytes / 1000000 )) MB"

  for sig in INT TERM KILL; do
    local limit
    case "$sig" in
      # Long enough to tell "ignoring the signal" from "servicing it", short
      # enough that being ignored costs seconds rather than minutes.
      INT)  limit=$RECORDER_ACK_S ;;
      # SIGTERM lands where SIGINT does not, so this one gets the close grace.
      TERM) limit=$close_grace ;;
      KILL) limit=$RECORDER_SIGKILL_GRACE_S ;;
    esac
    role_alive recorder || break
    if [[ $sig != INT ]]; then
      say "  !! recorder did not exit on SIG${RECORDER_STOP_SIGNAL} after ${waited}s — escalating to SIG$sig"
    fi
    RECORDER_STOP_SIGNAL=$sig
    signal_role recorder "$sig" "$pid"
    if [[ -n $rpid && $rpid != "$pid" ]]; then signal_role recorder "$sig" "$rpid"; fi
    local spent=0
    while role_alive recorder && (( spent < limit )); do sleep 1; spent=$((spent + 1)); done
    waited=$((waited + spent))
    role_alive recorder || break
  done

  RECORDER_STOP_WAIT_S=$waited
  if [[ $RECORDER_STOP_SIGNAL == KILL ]]; then
    # A hard-killed recorder leaves an mcap with no summary section, which
    # costs the clean recording and the message counts derived from it.
    say "  !! recorder SIGKILLED after ${waited}s — its mcap has no summary section"
    RUN_STRAGGLERS="$RUN_STRAGGLERS recorder:$pid"
    KILLED_HARD=1
  fi

  local status=0
  wait "$pid" 2>/dev/null || status=$?
  EXIT_CODES[recorder]=$status
  unset 'LAUNCH_PIDS[recorder]'
}

# Teardown is the reverse of startup: the trigger driver first, the recorder
# last. Nothing that produced data is stopped while something that consumes it
# is still running.
stop_all() {
  local role
  for role in driver sampler throttle clip_pruner pruner hog synth player clipper; do
    if [[ -n ${LAUNCH_PIDS[$role]:-} ]]; then stop_proc "$role" TERM; fi
  done
  stop_recorder
  return 0
}

# --------------------------------------------------------------------------
# Throttle evidence
#
# `throttled` is derived from CPU frequency, not from temperature. The boards
# run pinned with `jetson_clocks`, which sets scaling_min_freq to
# scaling_max_freq, so any sustained drop below that pinned maximum *is* the
# throttle — direct and unambiguous. Trip points cannot carry this: a Jetson's
# thermal zones expose fan trips alongside throttle trips, and the fan trip on
# `tj-thermal` sits at 35 C, which every zone exceeds permanently while sitting
# 40 C below the real 99 C limit.
#
# Thermal zones are still logged, because temperature is useful evidence for
# reading a result — but nothing is inferred from them here.
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# Clip pruning
#
# A ten-window heavy run asks for ~40 GB of clips, which does not fit on every
# board. Pruning bounds the clipped directory to the windows being cut at any
# one moment, without costing a single measurement: the trigger driver records
# each clip's size in triggers.jsonl at announcement time, so a clip's bytes
# are already captured before it can be considered for unlinking.
#
# The ordering guarantee is the whole point: nothing is unlinked until its own
# line in triggers.jsonl carries both a `recorded_ns` and a `bytes`, and the
# byte totals reported afterwards come from that line, never from the file. It
# unlinks only inside the clipped directory — clipper's own `--out-dir` — and
# refuses any path that resolves outside it, so the recording is never at risk.
# --------------------------------------------------------------------------

# `exec` is load-bearing. Backgrounding a function whose body is a single
# heredoc-fed command makes bash fork an extra subshell, so `$!` would name
# that wrapper rather than the python process: stopping the role would kill the
# wrapper, the real pruner would be orphaned to init, and its SIGTERM handler
# and final accounting sweep could never run. `exec` collapses the wrapper, so
# the pid the caller records is the pruner's own.
clip_pruner() {
  local jsonl=$1 clipdir=$2 state=$3
  exec python3 - "$jsonl" "$clipdir" "$state" <<'PY'
import json, os, signal, sys, time

jsonl, clipdir, state_path = sys.argv[1:4]
clipdir = os.path.realpath(clipdir) + os.sep
pruned_bytes, pruned_files, seen = 0, [], set()
running = [True]
signal.signal(signal.SIGTERM, lambda *_: running.__setitem__(0, False))
signal.signal(signal.SIGINT, lambda *_: running.__setitem__(0, False))


def flush():
    tmp = state_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump({"pruned_bytes": pruned_bytes,
                   "pruned_count": len(pruned_files),
                   "pruned_files": pruned_files}, fh, indent=2)
    os.replace(tmp, state_path)


def sweep():
    global pruned_bytes
    try:
        lines = open(jsonl, encoding="utf-8").read().splitlines()
    except OSError:
        return
    changed = False
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except ValueError:
            continue          # a line still being written; it will be read again
        name = rec.get("name")
        # Both conditions are required: the clip has been announced, and its
        # size is already recorded. Either one alone is not enough to unlink.
        if name in seen or rec.get("recorded_ns") is None or rec.get("bytes") is None:
            continue
        files = rec.get("files") or []
        if not files:
            continue
        for path in files:
            real = os.path.realpath(path)
            if not real.startswith(clipdir):
                print(f"clip_pruner: refusing {path}: outside {clipdir}", file=sys.stderr)
                continue
            try:
                os.unlink(real)
            except FileNotFoundError:
                pass
            except OSError as exc:
                print(f"clip_pruner: {real}: {exc}", file=sys.stderr)
                continue
            pruned_files.append(real)
        seen.add(name)
        pruned_bytes += int(rec["bytes"])
        changed = True
    if changed:
        flush()


flush()
while running[0]:
    sweep()
    for _ in range(20):
        if not running[0]:
            break
        time.sleep(0.1)
sweep()   # a final pass, so the last announced clip is accounted for
flush()
PY
}

throttle_watch() {
  local out=$1 z zone temp cpu cur max ts
  printf 'ts_ns,kind,id,value,limit\n' >"$out"
  while :; do
    ts=$(now_ns)
    for cpu in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/; do
      # An offline core exposes no cpufreq directory; skip rather than guess.
      [[ -r "$cpu/scaling_cur_freq" && -r "$cpu/scaling_max_freq" ]] || continue
      read -r cur <"$cpu/scaling_cur_freq" 2>/dev/null || continue
      read -r max <"$cpu/scaling_max_freq" 2>/dev/null || continue
      z=${cpu%/cpufreq/}
      printf '%s,cpufreq,%s,%s,%s\n' "$ts" "${z##*/}" "$cur" "$max"
    done
    for z in /sys/class/thermal/thermal_zone*/; do
      [[ -r "$z/temp" && -r "$z/type" ]] || continue
      read -r temp <"$z/temp" 2>/dev/null || continue
      read -r zone <"$z/type" 2>/dev/null || continue
      printf '%s,thermal,%s,%s,\n' "$ts" "$zone" "$temp"
    done
    sleep 2
  done >>"$out"
}

# --------------------------------------------------------------------------
# One run
# --------------------------------------------------------------------------

run_one() {
  local index=$1
  local varfile="$TMPDIR_SUITE/run-$index.sh"
  python3 "$HERE/scenarios.py" --plan-file "$PLAN_FILE" --emit-shell "$index" >"$varfile"
  # shellcheck disable=SC1090
  source "$varfile"

  local run_dir="$RESULTS_DIR/$RUN_ID"

  if [[ -n $RESUME && -f "$run_dir/run.json" ]]; then
    if python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("complete") else 1)' \
        "$run_dir/run.json" 2>/dev/null; then
      say "SKIP  $RUN_ID (complete)"
      return 0
    fi
  fi

  reset_procs
  local fail_reason=""
  local t0_ns t1_ns t2_ns t3_ns
  local rec_bytes_t1=0 rec_bytes_t2=0 clip_bytes=0
  local oom_killed="" measure_timeout="" snapshot_call_ns="" snapshot_anchor_ns=""
  SKIPPED=""; PRUNE_CLIPS=""; FREE_AT_START=0
  MAX_CACHE_EFFECTIVE=$MAX_CACHE_SIZE; MAX_CACHE_SOURCE="fixed"
  MEMORY_MAX_EFFECTIVE=0; MEM_TOTAL_BYTES=0; RATE_USED=""
  local pre_stragglers=""

  if [[ -n $DRY_RUN ]]; then
    printf '\n=== [%s/%s] %s\n' "$((index + 1))" "$RUN_COUNT" "$RUN_ID"
    printf '    scenario=%s variant=%s bitrate=%s (%s MB/s) comp=%s par=%s rep=%s band=%s seed=%s\n' \
      "$SCENARIO" "${VARIANT:--}" "$BITRATE_LABEL" "$BITRATE_TARGET_MBS" \
      "${CLIP_COMPRESSION:-n/a}" "${EXTRACT_PARALLELISM:-n/a}" "$REP" "$BAND_NAME" "$ORDER_SEED"
    printf '    warmup=%ss measure>=%ss (est %ss) est_disk=%s GiB\n' \
      "$WARMUP_S" "$MEASURE_S" "$EST_MEASURE_S" \
      "$(awk -v b="$EST_DISK_BYTES" 'BEGIN{printf "%.1f", b/1073741824}')"
    printf '    results -> %s\n' "$run_dir"
  else
    say "RUN   [$((index + 1))/$RUN_COUNT] $RUN_ID"
  fi

  # ---- 1. clean state -----------------------------------------------------
  if [[ -n $DRY_RUN ]]; then
    printf '    clean    rm -rf %s %s; mkdir -p %s %s  (record dir left for rosbag2 to create)\n' \
      "$RECORD_DIR" "$CLIP_DIR" "$CLIP_DIR" "$run_dir"
    printf '    clean    sweep leaked clipper / bag record / bag play / tegrastats / hog\n'
  else
    # Guard the wipe: these are computed paths, and rm -rf deserves a floor.
    [[ $RECORD_DIR == "$BENCH_ROOT"/* && $CLIP_DIR == "$BENCH_ROOT"/* ]] \
      || die "RECORD_DIR/CLIP_DIR must live under BENCH_ROOT ($BENCH_ROOT)"
    rm -rf "$RECORD_DIR" "$CLIP_DIR"
    mkdir -p "$CLIP_DIR" "$run_dir"
    # rosbag2 refuses to record into an existing bag directory, so RECORD_DIR
    # is left absent for it to create.
    : >"$run_dir/clipper.log"; : >"$run_dir/rosbag2.log"; : >"$run_dir/load.log"

    # A process leaked by run N-1 would contaminate this one, so the sweep
    # happens before the run starts as well as after it ends.
    pre_stragglers=$(kill_stragglers)
    if [[ -n $pre_stragglers ]]; then
      say "  !! killed leaked processes before start: $pre_stragglers"
    fi
  fi

  # ---- 2. disk guard ------------------------------------------------------
  #
  # Enforced, not advisory. A run that does not fit never starts: it is
  # skipped with a reason, because a skipped run that says why costs one data
  # point, and a filled disk costs every run that would have followed it.
  local need need_pruned
  need=$(( EST_DISK_BYTES * DISK_MARGIN_PCT / 100 + DISK_RESERVE_BYTES ))
  need_pruned=$(( EST_DISK_BYTES_PRUNED * DISK_MARGIN_PCT / 100 + DISK_RESERVE_BYTES ))
  PRUNE_CLIPS=""
  [[ $PRUNE_CLIPS_MODE == always && -n $EXPECT_CLIPS ]] && PRUNE_CLIPS=1

  if [[ -n $DRY_RUN ]]; then
    printf '    disk     require %s GiB free under %s (%s GiB with clip pruning)\n' \
      "$(awk -v b="$need" 'BEGIN{printf "%.1f", b/1073741824}')" "$BENCH_ROOT" \
      "$(awk -v b="$need_pruned" 'BEGIN{printf "%.1f", b/1073741824}')"
    [[ -n $PRUNE_CLIPS ]] && printf '    disk     clip pruning ON\n'
  else
    FREE_AT_START=$(free_bytes "$BENCH_ROOT")
    FREE_AT_START=${FREE_AT_START:-0}
    if (( FREE_AT_START < need )) && [[ $PRUNE_CLIPS_MODE != never && -n $EXPECT_CLIPS ]] \
       && (( FREE_AT_START >= need_pruned )); then
      PRUNE_CLIPS=1
      say "  .. clip pruning enabled: $(( FREE_AT_START / 1000000 )) MB free, "\
"$(( need / 1000000 )) MB needed unpruned, $(( need_pruned / 1000000 )) MB pruned"
    fi
    local required=$need
    [[ -n $PRUNE_CLIPS ]] && required=$need_pruned
    if (( FREE_AT_START < required )); then
      SKIPPED=1
      fail_reason="does not fit: needs $(( required / 1000000 )) MB free "\
"(estimate x${DISK_MARGIN_PCT}% + reserve), has $(( FREE_AT_START / 1000000 )) MB"
      say "  !! SKIP $RUN_ID — $fail_reason"
      t0_ns=$(now_ns); t1_ns=$t0_ns; t2_ns=$t0_ns; t3_ns=$t0_ns
      write_run_json "$index" "$run_dir" "$fail_reason"
      return 0
    fi
  fi

  # ---- 3. power mode ------------------------------------------------------
  if [[ -n $POWER_MODE ]]; then
    if [[ -n $DRY_RUN ]]; then
      printf '    power    sudo -n nvpmodel -m <id of %s>  (restored afterwards)\n' "$POWER_MODE"
    elif ! set_power_mode "$POWER_MODE"; then
      fail_reason="could not select nvpmodel mode $POWER_MODE"
      say "  !! $fail_reason"
      t0_ns=$(now_ns); t1_ns=$t0_ns; t2_ns=$t0_ns; t3_ns=$t0_ns
      write_run_json "$index" "$run_dir" "$fail_reason"
      return 0
    fi
  fi

  t0_ns=$(now_ns)

  # ---- 4. recorder --------------------------------------------------------
  # The snapshot arm's cache has to hold the whole window, so it is sized from
  # a measured write rate rather than the nominal target.
  if [[ -n $SNAPSHOT_MODE && -z $DRY_RUN ]]; then
    local ach
    ach=$(achieved_mbs_for "$BITRATE_LABEL" "$BITRATE_TARGET_MBS")
    MAX_CACHE_SOURCE=${ach#*$'\t'}
    ach=${ach%%$'\t'*}
    MAX_CACHE_EFFECTIVE=$(awk -v w="$SNAPSHOT_WINDOW_S" -v m="$ach" 'BEGIN{printf "%d", w*m*1000000}')
    MEM_TOTAL_BYTES=$(mem_total_bytes)
    MEMORY_MAX_EFFECTIVE=$(awk -v t="$MEM_TOTAL_BYTES" -v f="$MEMORY_MAX_FRACTION" 'BEGIN{printf "%d", t*f}')
    say "  .. snapshot cache ${MAX_CACHE_EFFECTIVE}B from ${ach} MB/s ($MAX_CACHE_SOURCE); "\
"MemoryMax ${MEMORY_MAX_EFFECTIVE}B of ${MEM_TOTAL_BYTES}B MemTotal"
  elif [[ -n $SNAPSHOT_MODE ]]; then
    MAX_CACHE_EFFECTIVE=$MAX_CACHE_SIZE
    MEMORY_MAX_EFFECTIVE=$MEMORY_MAX_PLANNED_BYTES
  fi

  local rec_cmd=(ros2 bag record --all
                 --storage mcap
                 --storage-preset-profile "$RECORDER_PROFILE"
                 --max-cache-size "$MAX_CACHE_EFFECTIVE"
                 --output "$RECORD_DIR")
  # --max-bag-duration and --snapshot-mode both exist on Humble and Jazzy;
  # nothing here uses a Jazzy-only topic-selection flag.
  if (( MAX_BAG_DURATION > 0 )); then rec_cmd+=(--max-bag-duration "$MAX_BAG_DURATION"); fi
  if [[ -n $SNAPSHOT_MODE ]]; then rec_cmd+=(--snapshot-mode); fi

  if [[ -n $SNAPSHOT_MODE ]]; then
    # An over-large snapshot cache must be OOM-killed inside its own scope
    # rather than take the board down. Without a working scope the run does
    # not start: an unbounded cache is the one failure mode worth preventing.
    local scope_cmd=(systemd-run --user --scope --quiet
                     -p "MemoryMax=$MEMORY_MAX_EFFECTIVE" -p MemorySwapMax=0
                     --unit "bench-rec-$$-$index" --)
    if [[ -z $DRY_RUN ]] && ! systemd-run --user --scope --quiet -p "MemoryMax=$MEMORY_MAX_EFFECTIVE" -- true >/dev/null 2>&1; then
      if sudo -n systemd-run --scope --quiet -p "MemoryMax=$MEMORY_MAX_EFFECTIVE" -- true >/dev/null 2>&1; then
        scope_cmd=(sudo -n systemd-run --scope --quiet
                   -p "MemoryMax=$MEMORY_MAX_EFFECTIVE" -p MemorySwapMax=0
                   --unit "bench-rec-$$-$index" --)
        # The recorder then runs as root and cannot be signalled directly.
        KILL_PREFIX[recorder]="sudo -n"
      else
        fail_reason="no usable systemd-run scope for MemoryMax=$MEMORY_MAX_EFFECTIVE"
        say "  !! $fail_reason"
        t1_ns=$t0_ns; t2_ns=$t0_ns; t3_ns=$t0_ns
        write_run_json "$index" "$run_dir" "$fail_reason"
        return 0
      fi
    fi
    rec_cmd=("${scope_cmd[@]}" "${rec_cmd[@]}")
  fi

  if ! start_bg recorder "$run_dir/rosbag2.log" "bag record" -- "${rec_cmd[@]}"; then
    fail_reason="recorder did not come up"
  fi

  # ---- 5. load ------------------------------------------------------------
  local rate=""
  if [[ -z $fail_reason ]]; then
    if ! rate=$(calibrated_rate "$BITRATE_TARGET_MBS"); then
      fail_reason="calibration failed for ${BITRATE_TARGET_MBS} MB/s"
    fi
    RATE_USED=$rate
  fi
  if [[ -z $fail_reason ]]; then
    start_bg player "$run_dir/load.log" "bag play" -- \
      "$REPLAY" --bag "$BAG" --rate "$rate" --loop \
      || fail_reason="bag replay did not come up"
  fi

  if [[ -z $fail_reason && -n $SYNTH_TOPICS ]]; then
    start_bg synth "$run_dir/load.log" "synth_pub.py" -- \
      python3 "$SYNTH" --topics "$SYNTH_TOPICS" --size "$SYNTH_SIZE" --rate "$SYNTH_RATE" \
      || fail_reason="synth publishers did not come up"
  fi

  if [[ -z $fail_reason ]] && (( HOG_CORES > 0 )); then
    start_bg hog "$run_dir/load.log" "hog.py" -- \
      python3 "$HOG" --cores "$HOG_CORES" \
      || fail_reason="hog did not come up"
  fi

  if [[ -z $fail_reason && -n $PRUNE ]]; then
    if [[ -n $DRY_RUN ]]; then
      printf '    pruner   RECORD_DIR=%s MAX_AGE_MIN=%s PRUNE_INTERVAL=%s %s\n' \
        "$RECORD_DIR" "$PRUNE_MAX_AGE_MIN" "$PRUNE_INTERVAL_S" "$PRUNER"
    else
      BENCH_SUITE="$BENCH_MARKER" BENCH_RUN_ID="$RUN_ID" \
        RECORD_DIR="$RECORD_DIR" MAX_AGE_MIN="$PRUNE_MAX_AGE_MIN" \
        PRUNE_INTERVAL="$PRUNE_INTERVAL_S" "$PRUNER" </dev/null \
        >>"$run_dir/load.log" 2>&1 &
      LAUNCH_PIDS[pruner]=$!
      ROLE_PIDS[pruner]=$!
    fi
  fi

  # ---- 6. clipper ---------------------------------------------------------
  if [[ -z $fail_reason && -n $RUN_CLIPPER ]]; then
    [[ -x $CLIPPER_BIN || -n $DRY_RUN ]] || fail_reason="clipper binary not executable: $CLIPPER_BIN"
    if [[ -z $fail_reason ]]; then
      start_bg clipper "$run_dir/clipper.log" "clipper" -- \
        "$CLIPPER_BIN" \
        --record-dir "$RECORD_DIR" \
        --out-dir "$CLIP_DIR" \
        --interface "$CLIPPER_INTERFACE" \
        --time-source "$CLIPPER_TIME_SOURCE" \
        --grace-secs "$CLIPPER_GRACE_SECS" \
        --clip-compression "$CLIP_COMPRESSION" \
        --extract-parallelism "$EXTRACT_PARALLELISM" \
        --watch-old-files-duration "$WATCH_OLD_FILES_DURATION" \
        || fail_reason="clipper did not come up"
    fi
  fi

  if [[ -n $fail_reason && -z $DRY_RUN ]]; then
    say "  !! $fail_reason"
    stop_all
    t1_ns=$(now_ns); t2_ns=$t1_ns; t3_ns=$(now_ns)
    write_run_json "$index" "$run_dir" "$fail_reason"
    return 0
  fi

  # ---- 7. warmup ----------------------------------------------------------
  # Discarded from the measurement: it exists so the page cache, DDS discovery
  # and the recorder's steady state settle before anything is counted.
  local lead=$SAMPLER_LEAD_S
  (( lead < WARMUP_S )) || lead=0
  if [[ -n $DRY_RUN ]]; then
    printf '    warmup   sleep %ss  (sampler starts %ss before the boundary)\n' "$WARMUP_S" "$lead"
  else
    sleep "$((WARMUP_S - lead))"
  fi

  # ---- 8. sampler ---------------------------------------------------------
  # Every pid exists by now; the sampler takes them all at once because pids
  # cannot be added after it starts.
  local sampler_cmd=(python3 "$SAMPLER" --out "$run_dir"
                     --interval-ms "$SAMPLE_INTERVAL_MS"
                     --tegrastats-interval-ms "$TEGRASTATS_INTERVAL_MS")
  local role
  for role in clipper recorder player synth hog; do
    local pid=${ROLE_PIDS[$role]:-}
    [[ -n $pid ]] || continue
    local sampler_role=$role
    [[ $role == recorder ]] && sampler_role=rosbag2
    sampler_cmd+=(--pid "$sampler_role=$pid")
  done
  if [[ -n $DRY_RUN ]]; then
    sampler_cmd+=(--pid "rosbag2=PID" --pid "player=PID")
    if [[ -n $RUN_CLIPPER ]]; then sampler_cmd+=(--pid "clipper=PID"); fi
    if [[ -n $SYNTH_TOPICS ]]; then sampler_cmd+=(--pid "synth=PID"); fi
    if (( HOG_CORES > 0 )); then sampler_cmd+=(--pid "hog=PID"); fi
    printf '    sampler  %s\n' "$(quote_cmd "${sampler_cmd[@]}")"
    printf '    throttle poll cpufreq + /sys/class/thermal every 2s > %s\n' "$run_dir/throttle.log"
    [[ -n $PRUNE_CLIPS ]] && printf '    prune    unlink each clip once announced and sized > %s\n' "$run_dir/pruned_clips.json"
  else
    BENCH_SUITE="$BENCH_MARKER" BENCH_RUN_ID="$RUN_ID" \
      "${sampler_cmd[@]}" </dev/null >>"$run_dir/sampler.log" 2>&1 &
    LAUNCH_PIDS[sampler]=$!
    ROLE_PIDS[sampler]=$!
    throttle_watch "$run_dir/throttle.log" </dev/null >/dev/null 2>&1 &
    LAUNCH_PIDS[throttle]=$!
    ROLE_PIDS[throttle]=$!
    if [[ -n $PRUNE_CLIPS ]]; then
      BENCH_SUITE="$BENCH_MARKER" BENCH_RUN_ID="$RUN_ID" \
        clip_pruner "$run_dir/triggers.jsonl" "$CLIP_DIR" "$run_dir/pruned_clips.json" \
        </dev/null >>"$run_dir/clip_pruner.log" 2>&1 &
      LAUNCH_PIDS[clip_pruner]=$!
      ROLE_PIDS[clip_pruner]=$!
    fi
    sleep "$lead"
  fi

  # ---- 9. measurement window ---------------------------------------------
  t1_ns=$(now_ns)
  [[ -n $DRY_RUN ]] || rec_bytes_t1=$(bytes_of_dir "$RECORD_DIR")

  if [[ $TRIGGER_PATTERN == snapshot_service ]]; then
    # No clipper, so no `Recorded` to wait on: the anchor is the decision
    # instant and the completion is the snapshot service returning.
    if [[ -n $DRY_RUN ]]; then
      printf '    trigger  anchor=now; sleep %ss; ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot\n' "$POSTROLL_S"
    else
      snapshot_anchor_ns=$(now_ns)
      sleep "$POSTROLL_S"
      local call_start call_end
      call_start=$(now_ns)
      ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot "{}" \
        </dev/null >>"$run_dir/load.log" 2>&1 || say "  !! snapshot service call failed"
      call_end=$(now_ns)
      snapshot_call_ns=$((call_end - call_start))
      python3 - "$run_dir/triggers.jsonl" "$snapshot_anchor_ns" "$call_start" "$call_end" \
        "$PREROLL_NS" "$POSTROLL_NS" <<'PY'
import json, sys
path, anchor, start, end, pre, post = sys.argv[1:7]
anchor, start, end = int(anchor), int(start), int(end)
with open(path, "w", encoding="utf-8") as fh:
    fh.write(json.dumps({
        "name": "bench-snapshot", "sent_ns": anchor, "anchor_hint_ns": anchor,
        "preroll_ns": int(pre), "postroll_ns": int(post),
        "recorded_ns": end, "latency_ns": end - anchor,
        "service_call_ns": end - start, "files": [], "bytes": None,
    }) + "\n")
PY
    fi
  elif (( TRIGGER_COUNT > 0 )); then
    local drv_cmd=(python3 "$DRIVER" --pattern "$TRIGGER_PATTERN"
                   --preroll-ns "$PREROLL_NS" --postroll-ns "$POSTROLL_NS"
                   --out "$run_dir/triggers.jsonl" --timeout-s "$TRIGGER_TIMEOUT_S")
    [[ $TRIGGER_PATTERN == ten_overlap ]] && drv_cmd+=(--stagger-ms "$STAGGER_MS")
    # The soak arm needs a repeating pattern; driver.py grows --period-s and
    # --count for it. Until it does, the soak run fails loudly here.
    [[ $TRIGGER_PATTERN == periodic ]] && drv_cmd+=(--period-s "$PERIOD_S" --count "$TRIGGER_COUNT")
    if [[ -n $DRY_RUN ]]; then
      printf '    trigger  %s\n' "$(quote_cmd "${drv_cmd[@]}")"
    else
      BENCH_SUITE="$BENCH_MARKER" BENCH_RUN_ID="$RUN_ID" \
        "${drv_cmd[@]}" </dev/null >>"$run_dir/load.log" 2>&1 &
      LAUNCH_PIDS[driver]=$!
      ROLE_PIDS[driver]=$!
    fi
  fi

  # The measure window is a floor, never a cap: it does not end while a clip
  # is still being cut, or a truncated extraction would be recorded as data.
  if [[ -n $DRY_RUN ]]; then
    printf '    measure  >= %ss, extended until every trigger is matched (hard cap %ss)\n' \
      "$MEASURE_S" "$((MEASURE_S + TRIGGER_TIMEOUT_S + 60))"
  else
    local floor_s hard_s nowsec
    floor_s=$(( $(now_s) + MEASURE_S ))
    hard_s=$(( $(now_s) + MEASURE_S + TRIGGER_TIMEOUT_S + 60 ))
    local driver_done
    while :; do
      nowsec=$(now_s)
      driver_done=1
      if [[ -n ${LAUNCH_PIDS[driver]:-} ]] && proc_alive "${LAUNCH_PIDS[driver]}"; then
        driver_done=0
      fi
      if (( nowsec >= floor_s && driver_done == 1 )); then break; fi
      if (( nowsec >= hard_s )); then
        measure_timeout=1
        fail_reason="measurement window hit its hard cap with triggers outstanding"
        say "  !! $fail_reason"
        break
      fi
      # A recorder or clipper that dies mid-window makes the rest of the
      # window meaningless — stop measuring rather than average over a hole.
      if ! role_alive recorder; then
        # Whether this was the memory cap firing is decided from the exit
        # status after the wait, never guessed here: a recorder that died for
        # an unrelated reason must not be recorded as an OOM kill, because
        # that field is what the snapshot sweep's kill-point curve is built
        # from.
        fail_reason="recorder exited during the measurement window"
        say "  !! $fail_reason"
        break
      fi
      if [[ -n $RUN_CLIPPER ]] && ! proc_alive "${ROLE_PIDS[clipper]:-}"; then
        fail_reason="clipper exited during the measurement window"
        say "  !! $fail_reason"
        break
      fi
      sleep 1
    done
  fi

  t2_ns=$(now_ns)
  if [[ -z $DRY_RUN ]]; then
    rec_bytes_t2=$(bytes_of_dir "$RECORD_DIR")
    clip_bytes=$(bytes_of_dir "$CLIP_DIR")
  fi

  # ---- 10. teardown, reverse order ---------------------------------------
  if [[ -n $DRY_RUN ]]; then
    printf '    teardown SIGTERM driver, sampler, throttle, pruner, hog, synth, player, clipper\n'
    printf '    teardown SIGINT  recorder (closes the mcap cleanly), then wait for exit\n'
    printf '    verify   no clipper / bag record / bag play / tegrastats / hog survives\n'
  fi
  stop_all

  if [[ -z $DRY_RUN ]]; then
    # A survivor here would poison the next run, so it is killed and named.
    local post
    post=$(kill_stragglers)
    if [[ -n $post ]]; then
      say "  !! stragglers survived teardown, killed: $post"
      RUN_STRAGGLERS="$RUN_STRAGGLERS $post"
      KILLED_HARD=1
    fi
    # Preserve what the clips looked like; the bytes themselves are not kept.
    ls -l "$CLIP_DIR" >"$run_dir/clips.txt" 2>/dev/null || true
    ls -l "$RECORD_DIR" >"$run_dir/record.txt" 2>/dev/null || true
  fi

  # The recorder's exit status is how a MemoryMax kill announces itself.
  if [[ -n $SNAPSHOT_MODE && -z $DRY_RUN ]]; then
    local rec_status=${EXIT_CODES[recorder]:-0}
    if (( rec_status == 137 )); then oom_killed=1; fi
  fi

  if [[ -n $POWER_MODE && -z $DRY_RUN ]]; then restore_power_mode; fi

  t3_ns=$(now_ns)

  if [[ -n $DRY_RUN ]]; then
    printf '    finish   write %s\n' "$run_dir/run.json"
    return 0
  fi

  write_run_json "$index" "$run_dir" "$fail_reason"

  # Feed the achieved write rate forward: the snapshot arm sizes its cache
  # from these, and band ordering guarantees these runs happen first.
  if [[ -z $SNAPSHOT_MODE ]]; then
    local ach_mbs
    ach_mbs=$(python3 -c '
import json,sys
d=json.load(open(sys.argv[1]))
print(d["bitrate_achieved_mbs"] if d.get("complete") and d.get("bitrate_achieved_mbs") else "")' \
      "$run_dir/run.json" 2>/dev/null) || ach_mbs=""
    [[ -n $ach_mbs ]] && record_achieved "$BITRATE_LABEL" "$ach_mbs"
  fi
}

# --------------------------------------------------------------------------
# Per-run containment
#
# One bad arm must cost one data point, not the night. A run executes inside
# its own backgrounded subshell, so `set -e` still aborts *that run* at the
# first unexpected failure while the suite carries on to the next. Only
# genuinely suite-level conditions — the lock held, no ROS environment, a
# missing bag, no plan — abort the whole thing, and those are all checked
# before the first run starts.
#
# The subshell means the parent no longer holds the run's pids, so suite-level
# cleanup works through the BENCH_SUITE marker sweep instead, which finds the
# run's processes whoever started them.
# --------------------------------------------------------------------------

CURRENT_RUN_PID=""

contain_cleanup() {
  stop_all || true
  restore_power_mode || true
}

# The subshell is spawned by the main loop itself, NOT from inside a helper
# function. Calling a function as `helper "$i" || status=$?` puts it in a
# tested context, and bash then suppresses errexit for the whole call —
# including any subshell it creates — so a failing command inside the run would
# be stepped over instead of ending it. Spawning at the top level and testing
# only `wait` keeps errexit live where it matters.

# A run that aborted before writing its own run.json still gets one, so
# `--resume` can tell "failed" from "never ran" and analysis does not silently
# skip a directory full of real samples.
ensure_run_json() {
  local index=$1 reason=$2 run_id run_dir
  run_id=$(python3 -c '
import json, sys
print(json.load(open(sys.argv[1]))["runs"][int(sys.argv[2])]["run_id"])' \
    "$PLAN_FILE" "$index" 2>/dev/null) || return 0
  run_dir="$RESULTS_DIR/$run_id"
  if python3 -c 'import json,sys; json.load(open(sys.argv[1]))' "$run_dir/run.json" 2>/dev/null; then
    return 0        # the run wrote its own record before it died
  fi
  mkdir -p "$run_dir"
  RJ_REASON="$reason" python3 - "$PLAN_FILE" "$index" "$run_dir" <<'ENSUREJSON'
import json, os, sys
plan_path, idx, run_dir = sys.argv[1], int(sys.argv[2]), sys.argv[3]
try:
    cfg = json.load(open(plan_path, encoding="utf-8"))["runs"][idx]
except Exception:
    cfg = {}
out = {"run_id": cfg.get("run_id", os.path.basename(run_dir)),
       "host": cfg.get("host"), "distro": cfg.get("distro"),
       "scenario": cfg.get("scenario"), "variant": cfg.get("variant"),
       "rep": cfg.get("rep"), "band_name": cfg.get("band_name"),
       "complete": False, "skipped": False, "skipped_reason": None,
       "oom_killed": False,
       "incomplete_reasons": [os.environ.get("RJ_REASON", "the run aborted")],
       "config": cfg}
tmp = os.path.join(run_dir, "run.json.tmp")
with open(tmp, "w", encoding="utf-8") as fh:
    json.dump(out, fh, indent=2)
    fh.write("\n")
os.replace(tmp, os.path.join(run_dir, "run.json"))
ENSUREJSON
  say "  .. wrote a placeholder run.json for $run_id so resume can see it failed"
}

# --------------------------------------------------------------------------
# run.json
#
# The run's own configuration comes straight out of the plan, so a result
# directory carries the exact expansion that produced it. Everything else is
# measured, and `complete` is decided from the evidence — never assumed.
# --------------------------------------------------------------------------

write_run_json() {
  local index=$1 run_dir=$2 reason=$3
  mkdir -p "$run_dir"
  # The analysis below reads files a run produced, and any of them can be
  # malformed in a way not anticipated here. An unhandled exception must not
  # cost the whole run record, so a failure falls through to the minimal one
  # written after this block.
  if ! \
  RJ_PLAN="$PLAN_FILE" RJ_INDEX="$index" RJ_DIR="$run_dir" RJ_REASON="$reason" \
  RJ_T0="${t0_ns:-0}" RJ_T1="${t1_ns:-0}" RJ_T2="${t2_ns:-0}" RJ_T3="${t3_ns:-0}" \
  RJ_REC_T1="${rec_bytes_t1:-0}" RJ_REC_T2="${rec_bytes_t2:-0}" RJ_CLIP="${clip_bytes:-0}" \
  RJ_CLIPPER_VERSION="$CLIPPER_VERSION" \
  RJ_NVPMODEL="${CURRENT_POWER_MODE:-unknown}" RJ_JETSON_CLOCKS="$(jetson_clocks_on)" \
  RJ_STRAGGLERS="${RUN_STRAGGLERS# }" RJ_KILLED_HARD="${KILLED_HARD:-}" \
  RJ_OOM="${oom_killed:-}" RJ_MEASURE_TIMEOUT="${measure_timeout:-}" \
  RJ_CLIP_DIR="$CLIP_DIR" RJ_RECORD_DIR="$RECORD_DIR" \
  RJ_SKIPPED="${SKIPPED:-}" RJ_FREE_AT_START="${FREE_AT_START:-0}" \
  RJ_PRUNE_CLIPS="${PRUNE_CLIPS:-}" \
  RJ_MAX_CACHE="${MAX_CACHE_EFFECTIVE:-0}" RJ_MAX_CACHE_SOURCE="${MAX_CACHE_SOURCE:-fixed}" \
  RJ_MEMORY_MAX="${MEMORY_MAX_EFFECTIVE:-0}" RJ_MEM_TOTAL="${MEM_TOTAL_BYTES:-0}" \
  RJ_REC_SIGNAL="${RECORDER_STOP_SIGNAL:-}" RJ_REC_WAIT_S="${RECORDER_STOP_WAIT_S:-0}" \
  RJ_RATE="${RATE_USED:-}" RJ_BAG="$BAG" RJ_CALIBRATE="$CALIBRATE" \
  RJ_FREQ_TOL_PCT="$CPU_FREQ_THROTTLE_TOLERANCE_PCT" \
  python3 - <<'PY'
import glob, json, os, re, struct, sys, time

env = os.environ
run_dir = env["RJ_DIR"]
plan = json.load(open(env["RJ_PLAN"], encoding="utf-8"))
cfg = plan["runs"][int(env["RJ_INDEX"])]

def n(key):
    return int(env.get(key) or 0)

t0, t1, t2, t3 = n("RJ_T0"), n("RJ_T1"), n("RJ_T2"), n("RJ_T3")
elapsed_s = (t2 - t1) / 1e9 if t2 > t1 else 0.0
written = n("RJ_REC_T2") - n("RJ_REC_T1")
# Bytes reaching the record directory over the measurement window. That is the
# ingest rate for every arm EXCEPT snapshot mode: there the recorder buffers in
# RAM and writes only when the service call fires, so the same arithmetic
# measures the DUMP, not the load — a 20 MB/s experiment would be labelled
# 2.3 MB/s. The snapshot arm's figure is measured further down from the dumped
# file's own message span instead, and until then this stays unset rather than
# holding a number that means something different in this arm than every other.
write_rate_mbs = round(written / 1e6 / elapsed_s, 3) if elapsed_s > 0 else None
is_snapshot = cfg["snapshot_arm"] == "snapshot"
if is_snapshot:
    # Starts as not-measurable and is only upgraded once a real figure exists.
    # If the block below ever fails to run, the field degrades to "unknown"
    # rather than to a None wearing an affirmative-looking label.
    achieved, achieved_source = None, "not measurable: cache dump not yet read"
elif write_rate_mbs is not None:
    achieved, achieved_source = write_rate_mbs, "record_dir_write_rate"
else:
    achieved, achieved_source = None, "not measurable: no measurement window"

reasons = []
if env.get("RJ_REASON"):
    reasons.append(env["RJ_REASON"])

# --- triggers -------------------------------------------------------------
#
# Two line shapes exist. The ordinary one is a trigger; the driver also writes
# a short `{"unexpected_recorded": true, ...}` line if a duplicate `Recorded`
# ever arrives for an already-matched name, which carries none of the trigger
# keys. Those are counted and surfaced — they would mean a clipper
# double-publish — but they are not triggers and must not be read as one.
skipped = bool(env.get("RJ_SKIPPED"))
pruning = bool(env.get("RJ_PRUNE_CLIPS"))
triggers, matched, missing_files, unexpected = [], 0, 0, 0
seen_by_clipper = None
tpath = os.path.join(run_dir, "triggers.jsonl")
if os.path.exists(tpath):
    with open(tpath, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except ValueError:
                reasons.append("triggers.jsonl has an unparseable line")
                continue
            if rec.get("unexpected_recorded"):
                unexpected += 1
            else:
                triggers.append(rec)

# Clip files are unlinked as they are announced when pruning is on, so their
# absence then is the pruner working, not a missing clip. The byte totals come
# from triggers.jsonl either way, never from the files.
pruned_bytes, pruned_count = 0, 0
ppath_pruned = os.path.join(run_dir, "pruned_clips.json")
if os.path.exists(ppath_pruned):
    try:
        pruned = json.load(open(ppath_pruned, encoding="utf-8"))
        pruned_bytes = int(pruned.get("pruned_bytes") or 0)
        pruned_count = int(pruned.get("pruned_count") or 0)
    except (OSError, ValueError):
        reasons.append("pruned_clips.json is unreadable, so pruned bytes are unaccounted")

clip_bytes_announced = sum(int(t["bytes"]) for t in triggers if t.get("bytes"))

for t in triggers:
    if t.get("recorded_ns") is not None:
        matched += 1
    # 0 means clipper's subscription never saw the trigger; >=1 means it saw it
    # and produced nothing. Different failures, and worth telling apart.
    subs = t.get("subscribers_at_publish")
    if subs is not None:
        seen_by_clipper = subs if seen_by_clipper is None else min(seen_by_clipper, subs)
    if not pruning:
        for f in t.get("files") or []:
            if not os.path.exists(f):
                missing_files += 1

latencies = sorted(t["latency_ns"] for t in triggers if t.get("latency_ns") is not None)


def pct(p):
    if not latencies:
        return None
    k = min(len(latencies) - 1, int(round((len(latencies) - 1) * p)))
    return latencies[k]


# --- phases ---------------------------------------------------------------
phase_notes = {}
phases = {"warmup": [t0, t1], "measure": [t1, t2], "teardown": [t2, t3]}

# The overlapping-window pattern splits in two, and the split is subtler than
# it looks. `waiting` is the interval in which EVERY admitted handler is parked
# and NONE can yet copy, so it runs from the LAST trigger sent (before that,
# handlers are still spawning) to the EARLIEST moment any window becomes
# copyable — min over triggers of (anchor + postroll).
#
# The tempting boundary, "last sent -> first Recorded", is wrong and not
# obviously so: with a 1 s stagger and a 60 s postroll, window 00 becomes
# copyable at sent[0]+60 s, which is nine seconds BEFORE window 09's postroll
# elapses. That interval therefore already contains active copying, and
# measuring it reports parked handlers as costing ~27% of a core when they
# actually cost ~0.55% — a fifty-fold error, and a plausible-looking one.
#
# Boundaries come from the per-trigger stamps in triggers.jsonl rather than
# from configuration, because the actual sends jitter.
if triggers:
    sent = [t["sent_ns"] for t in triggers if t.get("sent_ns") is not None]
    done = [t["recorded_ns"] for t in triggers if t.get("recorded_ns") is not None]
    copyable = [t["sent_ns"] + t["postroll_ns"] for t in triggers
                if t.get("sent_ns") is not None and t.get("postroll_ns") is not None]
    if sent and copyable:
        all_parked = max(sent)
        first_copyable = min(copyable)
        clip_end = max(done) if done else t2
        if first_copyable > all_parked:
            phases["waiting"] = [all_parked, first_copyable]
            phases["clipping"] = [first_copyable, clip_end]
        else:
            # stagger x count >= postroll: the first window became copyable
            # before the last trigger was even sent, so no interval exists in
            # which every handler is parked and none is copying. Emitting a
            # zero-length or inverted window would be read as a measurement of
            # that state, so the phase is absent instead.
            phases["clipping"] = [first_copyable, clip_end]
            phase_notes["waiting"] = (
                "absent: no pure-waiting interval exists for this pattern — the "
                f"first window became copyable {(all_parked - first_copyable) / 1e9:.1f} s "
                "before the last trigger was sent")

# --- dropped messages ------------------------------------------------------
#
# rosbag2 emits no parseable drop count, on either distro, even on a clean
# exit — so the count is measured rather than scraped: the recording's own
# MCAP Statistics record gives how many messages actually landed, and the
# source bag's Statistics gives the rate the player was feeding in. The MCAP
# offsets come from calibrate.py, which was reviewed against the spec, so
# there is one parser for this format and not two.
#
# The field is never left as a bare null: a null with no explanation reads as
# "no drops", which would be a fabricated answer.
sys.path.insert(0, os.path.dirname(env["RJ_CALIBRATE"]))
try:
    import calibrate
except Exception:          # noqa: BLE001 - see below
    # Deliberately broad: importing a module runs it, so this can raise
    # anything at all. Losing the drop metric is a missing field with a stated
    # reason; letting the exception out would lose the entire run record for a
    # run whose measurements are otherwise complete.
    calibrate = None


def bag_statistics(path):
    """(message_count, start_ns, end_ns) from an MCAP's summary section.

    calibrate.py owns this parse and exposes `bag_statistics` for it; the
    reader below is only the fallback for a calibrate.py old enough not to
    have it, and uses the same reviewed constants.
    """
    fn = getattr(calibrate, "bag_statistics", None)
    if fn is not None:
        return fn(path)
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        if size < 2 * len(calibrate.MCAP_MAGIC) + calibrate._FOOTER_RECORD_LEN:
            raise ValueError("too small to be a valid MCAP file")
        f.seek(size - len(calibrate.MCAP_MAGIC))
        if f.read(len(calibrate.MCAP_MAGIC)) != calibrate.MCAP_MAGIC:
            raise ValueError("no trailing magic — the writer never closed this file")
        footer_pos = size - len(calibrate.MCAP_MAGIC) - calibrate._FOOTER_RECORD_LEN
        f.seek(footer_pos)
        opcode = f.read(1)[0]
        (record_len,) = struct.unpack("<Q", f.read(8))
        if opcode != calibrate._FOOTER_OPCODE or record_len != calibrate._FOOTER_CONTENT_LEN:
            raise ValueError("no Footer record where expected")
        summary_start, summary_offset_start, _ = struct.unpack("<QQI", f.read(record_len))
        if summary_start == 0:
            raise ValueError("no summary section")
        summary_end = summary_offset_start or footer_pos
        f.seek(summary_start)
        while f.tell() < summary_end:
            op = f.read(1)
            if not op:
                break
            (record_len,) = struct.unpack("<Q", f.read(8))
            content = f.read(record_len)
            if op[0] == calibrate._STATISTICS_OPCODE:
                (count,) = struct.unpack_from("<Q", content, 0)
                start, end = struct.unpack_from("<QQ", content, calibrate._STATISTICS_TIME_OFFSET)
                return count, start, end
    raise ValueError("summary section has no Statistics record")


# rosbag2 reports drops only when there are drops:
#   "Number of messages lost on the transport layer: 9"
# So the absence of that line is not a measurement of zero unless the recorder
# also shut down cleanly enough to have emitted it. Three states, kept
# distinguishable, because a bare null reads downstream as "zero drops" and
# would publish a claim about recorder integrity that was never measured:
#   reported     - the line appeared; the count is rosbag2's own
#   clean_absent - clean shutdown, no such line: a genuine measured zero
#   not_measured - hard-killed, so absence proves nothing
dropped = None
drop_source = "not_measured"
drop_method = "not measurable: reason unrecorded"
messages_recorded = messages_expected = None

transport_lost = None
rpath = os.path.join(run_dir, "rosbag2.log")
if os.path.exists(rpath):
    text = open(rpath, encoding="utf-8", errors="replace").read()
    hits = [int(m) for m in re.findall(
        r"[Nn]umber of messages lost on the transport layer[:\s]+(\d+)", text)]
    if hits:
        transport_lost = max(hits)

# Whether the recording actually got its summary section is evidence, not an
# assumption about which signal is "clean" — and it is also the direct answer
# to whether stopping on SIGTERM rather than SIGINT still closes an mcap
# properly, which nothing else here can tell us.
recording_has_summary = None
_mcaps = sorted(glob.glob(os.path.join(env["RJ_RECORD_DIR"], "*.mcap")))
if calibrate is not None and _mcaps:
    try:
        bag_statistics(_mcaps[-1])
        recording_has_summary = True
    except Exception:
        recording_has_summary = False

# Snapshot mode's real ingest rate. The dumped file holds exactly what the
# cache held, and its Statistics record carries the span those messages cover,
# so bytes-over-span is the rate that was flowing IN while it buffered — the
# figure the sweep's chart must be labelled with.
if is_snapshot and calibrate is not None and _mcaps:
    try:
        dumped = sum(os.path.getsize(p) for p in _mcaps)
        spans = [bag_statistics(p) for p in _mcaps]
        span_ns = max(e for _, _, e in spans) - min(st for _, st, _ in spans)
        if span_ns > 0 and dumped > 0:
            achieved = round(dumped / 1e6 / (span_ns / 1e9), 3)
            achieved_source = "snapshot_cache_dump_over_buffered_span"
        else:
            achieved_source = "not measurable: the cache dump covers no time span"
    except Exception as exc:
        achieved_source = f"not measurable: {exc}"
elif is_snapshot:
    achieved_source = ("not measurable: no readable cache dump — expected when "
                       "the cache was OOM-killed before the snapshot fired")

clean_stop = env.get("RJ_REC_SIGNAL") != "KILL" and recording_has_summary is True
if transport_lost is not None:
    dropped = transport_lost
    drop_source = "reported"
    drop_method = ("rosbag2 reported it: 'Number of messages lost on the "
                   f"transport layer: {transport_lost}'")
elif clean_stop and os.path.exists(rpath):
    dropped = 0
    drop_source = "clean_absent"
    drop_method = ("rosbag2 shut down cleanly and reported no transport-layer "
                   "loss; it emits that line only when messages are lost, so "
                   "this is a measured zero")
else:
    drop_method = ("not measurable: the recording has no summary section "
                   f"(stopped by SIG{env.get('RJ_REC_SIGNAL') or '?'}), so the "
                   "absence of a transport-loss line proves nothing")

# An independent cross-check, never the primary number. rosbag2 counts what
# *it* was told it lost; this counts what actually reached the file against
# what the player should have fed it, so the two fail differently and a
# disagreement is worth seeing. It is deliberately not allowed to overwrite
# `dropped` — its slack is far too wide to source a headline from.
recorded_mcaps = sorted(glob.glob(os.path.join(env["RJ_RECORD_DIR"], "*.mcap")))
crosscheck = "not attempted"
if calibrate is None:
    crosscheck = "calibrate.py not importable, no MCAP parser"
elif env.get("RJ_REC_SIGNAL") == "KILL":
    crosscheck = "recorder was SIGKILLed, so its mcap has no summary section"
elif not recorded_mcaps:
    crosscheck = "no mcap in the record directory"
elif cfg["prune"]:
    # The retention pruner deletes splits mid-run, so the surviving files are
    # not the whole of what was written.
    crosscheck = "the pruner deleted splits, so surviving mcaps are not the whole recording"
else:
    try:
        total, span_start, span_end = 0, None, None
        for path in recorded_mcaps:
            count, start, end = bag_statistics(path)
            total += count
            span_start = start if span_start is None else min(span_start, start)
            span_end = end if span_end is None else max(span_end, end)
        messages_recorded = total
        span_s = (span_end - span_start) / 1e9
        rate = float(env["RJ_RATE"]) if env.get("RJ_RATE") else None
        src_count, src_start, src_end = bag_statistics(env["RJ_BAG"])
        src_span_s = (src_end - src_start) / 1e9
        if rate and span_s > 0 and src_span_s > 0:
            messages_expected = int(round(src_count / src_span_s * rate * span_s))
            crosscheck = (
                f"{messages_recorded} recorded vs ~{messages_expected} expected "
                f"({src_count} msgs / {src_span_s:.3f} s of source bag at --rate "
                f"{rate}, over {span_s:.1f} s of recording). The recording is "
                "--all, so it also holds trigger/rosout traffic the source bag "
                "does not, and the replay loops: treat this as an order-of-"
                "magnitude check, not a count."
            )
        else:
            crosscheck = "no replay rate recorded for this run"
    except (OSError, ValueError, struct.error) as exc:
        crosscheck = f"unavailable: {exc}"

# --- host clock -----------------------------------------------------------
#
# tegrastats emits local wall-clock with no epoch and no offset. Everything
# else in a run directory is epoch ns. The two boards sit in different zones,
# so converting tegrastats stamps with the *analysis* machine's zone aligns one
# host's power series to a window four hours away from its own measurement —
# and then reports energy computed from entirely the wrong samples.
host_tz = time.tzname[time.daylight and time.localtime().tm_isdst > 0]
try:
    host_tz_name = os.environ.get("TZ") or open("/etc/timezone").read().strip()
except OSError:
    host_tz_name = None
host_utc_offset_s = -(time.altzone if time.localtime().tm_isdst > 0 else time.timezone)
if host_tz_name:
    host_tz = host_tz_name
# One instant expressed both ways, so the offset can be recovered from the data
# itself without trusting a tz database — or the board's clock being right.
_anchor = time.time()
clock_anchor = {
    "epoch_ns": int(_anchor * 1e9),
    "local_wallclock": time.strftime("%m-%d-%Y %H:%M:%S", time.localtime(_anchor)),
    "utc_wallclock": time.strftime("%m-%d-%Y %H:%M:%S", time.gmtime(_anchor)),
    "note": "tegrastats stamps match local_wallclock's format and zone",
}

# --- throttling, from CPU frequency ---------------------------------------
#
# The boards run pinned with jetson_clocks, so a sustained drop below the
# pinned maximum is the throttle. Temperature is logged but nothing is inferred
# from it: a Jetson's thermal zones mix fan trips in with throttle trips, and
# the 35 C fan trip on tj-thermal is exceeded permanently while the board sits
# 40 C below its real limit.
throttled = None
throttle_method = "not measurable: throttle.log absent"
freq_min = freq_limit = None
ppath = os.path.join(run_dir, "throttle.log")
if os.path.exists(ppath):
    with open(ppath, encoding="utf-8") as fh:
        next(fh, None)
        for line in fh:
            parts = line.rstrip("\n").split(",")
            if len(parts) != 5 or parts[1] != "cpufreq":
                continue
            try:
                cur, limit = int(parts[3]), int(parts[4])
            except ValueError:
                continue
            freq_min = cur if freq_min is None else min(freq_min, cur)
            freq_limit = limit if freq_limit is None else max(freq_limit, limit)
    tol = int(env["RJ_FREQ_TOL_PCT"]) / 100.0
    if freq_min is None:
        throttle_method = "not measurable: no cpufreq samples (no scaling_cur_freq?)"
    elif env["RJ_JETSON_CLOCKS"] != "true":
        # Without pinned clocks a low frequency is the governor idling, not a
        # throttle, and calling it one would be a fabricated result.
        throttle_method = "not measurable: clocks not pinned, frequency reflects the governor"
    else:
        throttled = freq_min < freq_limit * tol
        throttle_method = f"cpufreq: min {freq_min} kHz vs pinned {freq_limit} kHz, tolerance {tol:.2f}"

# --- completeness ---------------------------------------------------------
# A run counts as data only when it produced what its scenario promised.
oom = bool(env.get("RJ_OOM"))
expect_triggers = cfg["expect_triggers"]
if skipped:
    # A run that never started has nothing to check; the skip reason is
    # already the only reason, and it must not be buried under a pile of
    # consequential ones.
    pass
elif cfg["snapshot_arm"] == "snapshot":
    # An over-large cache being OOM-killed is the sweep's result, not its
    # failure — the run is complete and says so, and nothing below is expected
    # to have been produced.
    if not oom:
        if written <= 0:
            reasons.append("snapshot produced no bytes")
        # Now that the arm has a real ingest measurement, it gets the same
        # load sanity check as every other arm: this catches a snapshot run
        # whose replay never started, which "the cache dumped some bytes"
        # alone would not.
        elif achieved is not None and achieved < 0.3 * cfg["bitrate_target_mbs"]:
            reasons.append(
                f"cache filled at {achieved} MB/s against a "
                f"{cfg['bitrate_target_mbs']} MB/s target")
else:
    if achieved is None:
        reasons.append("no measurement window")
    elif achieved < 0.3 * cfg["bitrate_target_mbs"]:
        # Not a bitrate the board failed to sustain — a load that never ran.
        reasons.append(
            f"achieved {achieved} MB/s against a {cfg['bitrate_target_mbs']} MB/s target")
    if len(triggers) != expect_triggers:
        reasons.append(f"{len(triggers)} triggers recorded, expected {expect_triggers}")
    if matched != expect_triggers:
        reasons.append(f"{matched} of {expect_triggers} triggers matched a Recorded")
    if cfg["expect_clips"]:
        empty = [t.get("name") for t in triggers if not (t.get("files") or [])]
        if empty:
            reasons.append(f"triggers with no clip: {empty}")
        if missing_files:
            reasons.append(f"{missing_files} announced clip files are absent")
        if pruning and pruned_count < len(triggers):
            reasons.append(
                f"clip pruning accounted for {pruned_count} of {len(triggers)} clips")
    if unexpected:
        reasons.append(f"{unexpected} duplicate Recorded announcements")

if env.get("RJ_MEASURE_TIMEOUT"):
    reasons.append("measurement window hit its hard cap")

reasons = list(dict.fromkeys(reasons))
complete = not reasons

out = {
    "run_id": cfg["run_id"],
    "host": cfg["host"],
    "distro": cfg["distro"],
    "clipper_version": env["RJ_CLIPPER_VERSION"],
    "scenario": cfg["scenario"],
    "variant": cfg["variant"],
    "rep": cfg["rep"],
    "order_seed": cfg["order_seed"],
    "order_index": cfg["order_index"],
    "band": cfg["band"],
    "band_name": cfg["band_name"],
    "bitrate_target_mbs": cfg["bitrate_target_mbs"],
    "bitrate_achieved_mbs": achieved,
    # What that number actually measures. It is NOT the same quantity in every
    # arm, and a chart that labels points by it needs to know which.
    # CONTRACT with analysis, which prefix-matches on this rather than parsing
    # it. Two values mean "the number beside me is real":
    #     "record_dir_write_rate"
    #     "snapshot_cache_dump_over_buffered_span"
    # EVERY other value MUST begin with "not measurable", and whenever it does,
    # `bitrate_achieved_mbs` is null. Reword the tails freely; never reword
    # those two literals or that prefix. The same rule governs
    # `throttle_method` and `rosbag2_dropped_method`.
    "bitrate_achieved_source": achieved_source,
    # The raw record-directory write rate, always. For a snapshot arm this is
    # the dump rate, which is a real thing to know and not the load.
    "record_dir_write_rate_mbs": write_rate_mbs,
    "recorder_profile": cfg["recorder_profile"],
    # What the recorder actually ran with. For the snapshot arm the cache is
    # sized at run time from a measured write rate, and the memory bound from
    # the board's real MemTotal, so neither is the planned figure.
    "max_cache_size": n("RJ_MAX_CACHE"),
    "max_cache_source": env["RJ_MAX_CACHE_SOURCE"],
    "memory_max_bytes": n("RJ_MEMORY_MAX"),
    "mem_total_bytes": n("RJ_MEM_TOTAL"),
    "max_bag_duration": cfg["max_bag_duration"],
    "snapshot_arm": cfg["snapshot_arm"],
    # What clipper actually ran with; `config` keeps the axis value, which is
    # null for the scenarios that do not vary over it.
    "clip_compression": cfg["effective_clip_compression"],
    "extract_parallelism": cfg["effective_extract_parallelism"],
    "nvpmodel": env["RJ_NVPMODEL"],
    "jetson_clocks": env["RJ_JETSON_CLOCKS"] == "true",
    # tegrastats stamps local wall-clock with no date arithmetic attached,
    # while every other series here is epoch ns. Without the offset the two
    # boards' power series align to windows hours apart from their own
    # measurements. Captured per run, not per suite, so an 11-hour suite that
    # crosses a DST boundary still resolves each run correctly.
    "host_tz": host_tz,
    "host_utc_offset_s": host_utc_offset_s,
    # An anchor is what makes this robust even if a board's clock or tz
    # database is simply wrong: one instant, expressed both ways.
    "clock_anchor": clock_anchor,
    "warmup_s": cfg["warmup_s"],
    "measure_s": cfg["measure_s"],
    "phases": phases,
    "phase_notes": phase_notes,
    "record_bytes_written": written,
    # Measured from the directory when nothing is pruned. Under pruning the
    # directory is deliberately incomplete, and adding the pruner's total to it
    # would double-count every clip still present when the window closed — so
    # the figure comes from the sizes the driver recorded at announcement time,
    # which is the same number the pruner itself accounts with.
    "clip_bytes_written": clip_bytes_announced if pruning else n("RJ_CLIP"),
    "clip_bytes_source": "announced_sizes" if pruning else "clipped_directory",
    "clip_bytes_announced": clip_bytes_announced,
    "clip_bytes_on_disk_at_end": n("RJ_CLIP"),
    "clip_bytes_pruned": pruned_bytes,
    "clip_pruning": bool(env.get("RJ_PRUNE_CLIPS")),
    "free_bytes_at_start": n("RJ_FREE_AT_START"),
    "triggers_sent": len(triggers),
    "triggers_matched": matched,
    "triggers_seen_by_clipper": seen_by_clipper,
    "unexpected_recorded": unexpected,
    "latency_ns": {"min": latencies[0] if latencies else None,
                   "p50": pct(0.5), "p95": pct(0.95),
                   "max": latencies[-1] if latencies else None},
    "rosbag2_dropped": dropped,
    "rosbag2_dropped_source": drop_source,
    "rosbag2_dropped_method": drop_method,
    "messages_recorded": messages_recorded,
    "messages_expected": messages_expected,
    "message_count_crosscheck": crosscheck,
    "recording_has_summary": recording_has_summary,
    "throttled": throttled,
    "throttle_method": throttle_method,
    "cpu_freq_min_khz": freq_min,
    "cpu_freq_pinned_khz": freq_limit,
    "oom_killed": oom,
    "recorder_stop_signal": env.get("RJ_REC_SIGNAL") or None,
    "recorder_stop_wait_s": n("RJ_REC_WAIT_S"),
    "stragglers": env.get("RJ_STRAGGLERS", "").split() or [],
    "killed_hard": bool(env.get("RJ_KILLED_HARD")),
    "skipped": skipped,
    "skipped_reason": reasons[0] if (skipped and reasons) else None,
    "incomplete_reasons": reasons,
    "complete": complete,
    "config": cfg,
}

tmp = os.path.join(run_dir, "run.json.tmp")
with open(tmp, "w", encoding="utf-8") as fh:
    json.dump(out, fh, indent=2)
    fh.write("\n")
final = os.path.join(run_dir, "run.json")
os.replace(tmp, final)
# Read it back. run.json is the artefact that makes a run count, so "written"
# is not good enough — it has to be parseable by whatever reads it next.
with open(final, encoding="utf-8") as fh:
    json.load(fh)
print("OK" if complete else "INCOMPLETE: " + "; ".join(reasons))
PY
  then
    say "  !! run.json writer failed for $RUN_ID — recording the run as incomplete"
    python3 - "$PLAN_FILE" "$index" "$run_dir" <<'RJFALLBACK'
import json, os, sys
plan_path, idx, run_dir = sys.argv[1], int(sys.argv[2]), sys.argv[3]
try:
    cfg = json.load(open(plan_path, encoding="utf-8"))["runs"][idx]
except Exception:
    cfg = {}
out = {"run_id": cfg.get("run_id", os.path.basename(run_dir)),
       "host": cfg.get("host"), "scenario": cfg.get("scenario"),
       "complete": False,
       "incomplete_reasons": ["run.json writer raised; see suite.log"],
       "config": cfg}
tmp = os.path.join(run_dir, "run.json.tmp")
with open(tmp, "w", encoding="utf-8") as fh:
    json.dump(out, fh, indent=2)
    fh.write("\n")
os.replace(tmp, os.path.join(run_dir, "run.json"))
RJFALLBACK
  fi
}

# --------------------------------------------------------------------------
# Suite
# --------------------------------------------------------------------------

say "SUITE host=$HOST reps=$REPS runs=$RUN_COUNT only='${ONLY:-*}' resume=${RESUME:-0} dry_run=${DRY_RUN:-0}"
if [[ -z $DRY_RUN ]]; then
  say "      clipper=$CLIPPER_VERSION distro=$ROS_DISTRO nvpmodel=${ORIGINAL_POWER_MODE:-unknown} jetson_clocks=$(jetson_clocks_on)"
  say "      bench_root=$BENCH_ROOT bag=$BAG free=$(free_bytes "$BENCH_ROOT")B"
  for f in "$SAMPLER" "$REPLAY" "$CALIBRATE" "$DRIVER"; do
    [[ -e $f ]] || say "WARNING: missing harness component: $f"
  done
  [[ -e $BAG ]] || say "WARNING: replay bag not found: $BAG"
fi

FAILED_RUNS=0
for (( i = 0; i < RUN_COUNT; i++ )); do
  ( trap contain_cleanup EXIT; run_one "$i" ) &
  CURRENT_RUN_PID=$!
  status=0
  wait "$CURRENT_RUN_PID" || status=$?
  CURRENT_RUN_PID=""
  if (( status != 0 )); then
    FAILED_RUNS=$((FAILED_RUNS + 1))
    say "  !! run $i exited $status — contained; continuing with the next run"
    ensure_run_json "$i" "the run aborted with exit status $status; see suite.log"
  fi
done

say "SUITE done: $RUN_COUNT runs considered, $FAILED_RUNS aborted"
