# trigger-pub

A development stand-in that publishes `edgestream_msgs/Trigger` on
`/events/edgestream/trigger` at a fixed interval, so `edgestream-rec` has
something to react to without a real trigger source. Built on
[r2r](https://github.com/sequenceplanner/r2r); no async runtime.

## Behaviour

A plain loop: every `--period` seconds (default 5) it stamps `trigger_time` with
the current RosTime via `r2r::Clock` (`ClockType::RosTime` →
`Clock::to_builtin_time`), fills in an incrementing `name` (`<prefix>-<n>`), the
`--description`, and the `--preroll`/`--postroll` windows (u64 nanoseconds), and
publishes. It `spin_once`s briefly each iteration to keep the node healthy, then
sleeps out the period.

`trigger_time` being the publish-time stamp is the point: it is the "original
timestamp" the recorder centres its `[t-preroll, t+postroll]` window on, the same
idea as the other recorders keying off a message's own stamp.

Flags: `--period <secs>`, `--preroll <ns>`, `--postroll <ns>`, `--name <prefix>`,
`--description <text>` (all optional; see the module doc for defaults).

## Build

Pure r2r: the build generates bindings for `edgestream_msgs` (and the rest of the
`IDL_PACKAGE_FILTER` set) at `cargo build`, exactly like `r2r-sub`. Needs the
flake's `LIBCLANG_PATH`; does not need `ROS_DISTRO`.

## Run

```bash
nix develop --command cargo run -p trigger-pub -- --period 5 --preroll 2000000000 --postroll 2000000000
```

Pair with a running `edgestream-rec` and `scripts/record.sh`. Logging uses the
`log` facade with a `pretty_env_logger` backend; `RUST_LOG` controls verbosity.
