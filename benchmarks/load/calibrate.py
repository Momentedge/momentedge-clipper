#!/usr/bin/env python3
"""Map a target write rate to a `ros2 bag play --rate` value.

The bag's own byte size and message time span set the baseline: at
`--rate 1.0`, `ros2 bag play` reproduces the bag's original write rate,
`size_bytes / duration_s`. The rate for any target is just that target
divided by the baseline. "MB/s" here means decimal megabytes (1e6 bytes/s),
matching how storage and network throughput is usually quoted.

Byte size comes from the file itself; duration comes from the bag's own MCAP
Statistics record (`message_end_time - message_start_time`, in the summary
section every indexed mcap carries) rather than from a hardcoded bitrate —
swapping in a different bag needs no code change here. This is arithmetic
only: it does not invoke `ros2` or need a ROS2 environment. The rate it prints
is an estimate; the actual achieved write rate is a separate measurement the
orchestrator takes while replay is running.

The same footer/summary parsing is exposed as `bag_statistics(path) ->
(message_count, message_start_ns, message_end_ns)`, for anything else that
needs a bag's own recorded statistics (e.g. a dropped-message check deriving
an expected count from the source bag) without re-parsing MCAP itself;
`bag_duration_seconds(path)` is a thin wrapper over it.

`--correction` (default 1.0, no behaviour change) scales the target before
dividing by the bag's baseline, to pre-correct for a systematic gap between
the bag's own stored byte rate and what the recorder under test actually
writes: the bag's chunks are lz4-compressed, but the benchmark's recorder
writes `fastwrite` — uncompressed and unchunked — so bytes reaching disk
exceed the bag's stored rate by a decompression-plus-framing factor. Measured
on an Orin Nano gate run (MAXN_SUPER, `--storage-preset-profile fastwrite
--max-cache-size 0`, mid tier, target 20.0 MB/s): achieved 20.342 MB/s, a
+1.7% expansion (factor 1.017). That single measurement is the only on-target
data point so far — it should not be assumed constant across tiers without
checking; a same-day cross-check on a workstation (different hardware and
ROS build, so the absolute rates do not transfer, but the expansion ratio is
a property of the data and the storage plugin and should) saw a similar
~1-2% expansion at the mid and heavy tiers but a larger ~6-7% one at the
light tier, suggesting the ratio may not be a single rate-independent
scalar. A measured factor is never applied silently by default — pass it
explicitly via `--correction` once it is trusted for the tier in question.

Prints one line of JSON to stdout: {"rate": <float>, "est_mbs": <float>}.
"""
import argparse
import json
import os
import struct
import sys

MCAP_MAGIC = b"\x89MCAP0\r\n"

_FOOTER_OPCODE = 0x02
_STATISTICS_OPCODE = 0x0B
# Footer content: summary_start(8) + summary_offset_start(8) + summary_crc(4).
_FOOTER_CONTENT_LEN = 20
_FOOTER_RECORD_LEN = 1 + 8 + _FOOTER_CONTENT_LEN  # opcode + length prefix + content
# Statistics content: message_count(8), then schema_count(2) +
# channel_count(4) + attachment_count(4) + metadata_count(4) + chunk_count(4)
# (skipped over — not needed here), then message_start_time(8) and
# message_end_time(8).
_STATISTICS_MESSAGE_COUNT_OFFSET = 0
_STATISTICS_TIME_OFFSET = 8 + 2 + 4 + 4 + 4 + 4


def bag_statistics(path):
    """Return (message_count, message_start_ns, message_end_ns) from the
    bag's own MCAP Statistics record.

    Reads only the MCAP footer and summary section (never the data chunks):
    the footer at the end of the file points to the summary section, which
    holds a Statistics record carrying the total message count and
    `message_start_time`/`message_end_time` (nanoseconds since epoch) over
    every message in the file. Raises ValueError with a specific reason if
    the file isn't a valid, indexed MCAP — `ros2 bag record`'s default
    profile always writes one; a bag missing it would need `mcap recover`
    first.

    This is the one seam other harness components should use to read a
    bag's own recorded statistics (e.g. run_suite.sh's dropped-message
    check, which derives an expected count from the source bag's own
    message_count) rather than re-parsing the footer/summary layout
    themselves — the leading-underscore module constants above are this
    function's implementation detail, not a stable API; only this function
    and bag_duration_seconds are.
    """
    with open(path, "rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        if size < 2 * len(MCAP_MAGIC) + _FOOTER_RECORD_LEN:
            raise ValueError(f"{path}: too small to be a valid MCAP file")

        f.seek(size - len(MCAP_MAGIC))
        if f.read(len(MCAP_MAGIC)) != MCAP_MAGIC:
            raise ValueError(f"{path}: missing trailing MCAP magic bytes — not a complete mcap file")

        footer_pos = size - len(MCAP_MAGIC) - _FOOTER_RECORD_LEN
        f.seek(footer_pos)
        opcode = f.read(1)[0]
        (record_len,) = struct.unpack("<Q", f.read(8))
        if opcode != _FOOTER_OPCODE or record_len != _FOOTER_CONTENT_LEN:
            raise ValueError(f"{path}: no Footer record where expected — truncated or unindexed mcap?")
        summary_start, summary_offset_start, _summary_crc = struct.unpack("<QQI", f.read(_FOOTER_CONTENT_LEN))
        if summary_start == 0:
            raise ValueError(
                f"{path}: has no summary section, so its Statistics record "
                "is unavailable — reindex with `mcap recover`"
            )

        summary_end = summary_offset_start if summary_offset_start != 0 else footer_pos
        f.seek(summary_start)
        while f.tell() < summary_end:
            opcode = f.read(1)
            if not opcode:
                break
            (record_len,) = struct.unpack("<Q", f.read(8))
            content = f.read(record_len)
            if opcode[0] == _STATISTICS_OPCODE:
                (message_count,) = struct.unpack_from("<Q", content, _STATISTICS_MESSAGE_COUNT_OFFSET)
                start_time, end_time = struct.unpack_from("<QQ", content, _STATISTICS_TIME_OFFSET)
                return message_count, start_time, end_time

    raise ValueError(f"{path}: summary section has no Statistics record")


def bag_duration_seconds(path):
    """Return the bag's message time span, in seconds — `bag_statistics`'s
    `message_end_ns - message_start_ns`, converted and validated positive.
    Raises ValueError under the same conditions as `bag_statistics`, plus a
    non-positive span.
    """
    _message_count, start_ns, end_ns = bag_statistics(path)
    duration = (end_ns - start_ns) / 1e9
    if duration <= 0:
        raise ValueError(f"{path}: Statistics record reports a non-positive duration")
    return duration


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--bag", required=True, help="path to the mcap file to calibrate against")
    parser.add_argument(
        "--target-mbs", required=True, type=float,
        help="target write rate in MB/s (decimal, 1e6 bytes/s)",
    )
    parser.add_argument(
        "--correction", type=float, default=1.0,
        help="empirical expansion factor (achieved/target) to pre-correct for; "
        "see the module docstring for the measured value and its provenance "
        "(default: 1.0, no correction)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if not os.path.isfile(args.bag):
        print(f"calibrate.py: --bag {args.bag}: no such file", file=sys.stderr)
        return 1
    if args.target_mbs <= 0:
        print("calibrate.py: --target-mbs must be positive", file=sys.stderr)
        return 1
    if args.correction <= 0:
        print("calibrate.py: --correction must be positive", file=sys.stderr)
        return 1

    try:
        duration_s = bag_duration_seconds(args.bag)
    except ValueError as exc:
        print(f"calibrate.py: {exc}", file=sys.stderr)
        return 1

    size_bytes = os.path.getsize(args.bag)
    base_mbs = size_bytes / duration_s / 1e6

    # Dividing by --correction (>1.0 for a recorder that writes more bytes
    # than the bag's own stored rate) lowers --rate just enough that the
    # *actual* achieved rate, not the bag's raw replay rate, lands on target.
    rate = round(args.target_mbs / (base_mbs * args.correction), 4)
    est_mbs = round(rate * base_mbs * args.correction, 1)

    print(json.dumps({"rate": rate, "est_mbs": est_mbs}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
