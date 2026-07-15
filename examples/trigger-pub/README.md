# trigger-pub — example trigger source

An example node that publishes `momentedge_msgs/Trigger` on
`/events/momentedge/trigger` at a fixed interval, so the triggered recorder
(`clipper`) has something to react to without a real trigger source. A real
deployment supplies its own trigger publisher; `clipper` never depends on this
one. Built on [r2r](https://github.com/sequenceplanner/r2r); no async runtime.

## Behaviour

A plain loop: every `--period` seconds (default 1) it fills in an incrementing
`name` (`<prefix>-<n>`), the `--description`, the `--preroll`/`--postroll`
windows (nanoseconds), and a zero `trigger_time`, and publishes. With
`--preroll`/`--postroll` omitted, each iteration draws that side a fresh random
whole-second window in `[1, 10]` s, pre and post independently, so the recorder
sees varied clip lengths; pass either flag to pin that side to a fixed nanosecond
value.

`trigger_time` is zero by default because clipper reads it only under
`--interface ros --time-source publish`; every other cell anchors the window on
the recorder's own receipt instant and rejects a trigger that sets
`trigger_time`. Pass `--stamp-trigger-time` to stamp it with the current RosTime
at publish — the publish-domain anchor for a `--time-source publish` clipper.

Flags (all optional): `--period <secs>`, `--preroll <ns>`, `--postroll <ns>`,
`--name <prefix>`, `--description <text>`, `--stamp-trigger-time`. `RUST_LOG`
controls verbosity (default `info`).

## Run

From the dev shell, paired with a running `clipper` and `scripts/record.sh`:

```bash
nix develop --command cargo run -p trigger-pub -- --period 5 --preroll 2000000000 --postroll 2000000000
```

On a deployment target, build it with `scripts/build-on-target.sh` and run the
demo launcher (foreground, Ctrl-C to stop; source a ROS2 environment first):

```bash
./examples/trigger-pub/start_demo_trigger_pub.sh --preroll 2000000000 --postroll 3000000000
```

Design rationale and build mechanics are in [CLAUDE.md](CLAUDE.md).
