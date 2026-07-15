# trigger-pub — contributor notes

[README.md](README.md) covers what this example is, its flags, and how to run
it. This file is the rationale beyond it.

## Why `trigger_time` is zero by default

clipper reads `trigger_time` in exactly one cell of its interface × time-source
matrix — `--interface ros --time-source publish`, where the field *is* the window
anchor. Every other cell (including the default `--time-source log`) anchors the
window on the recorder's own receipt instant and rejects a trigger that sets a
non-zero `trigger_time`, so a dev-loop trigger must send zero to be accepted at
all. Hence the default is zero; `--stamp-trigger-time` samples the current
RosTime (`r2r::Clock`, `ClockType::RosTime` → `Clock::to_builtin_time`) into
`trigger_time` for a `--time-source publish` run, where it is the publish-domain
anchor. The node has no subscriptions, but `spin_once`s briefly each iteration to
keep its graph/publisher machinery healthy, then sleeps out the period. The
random window bounds are the inclusive `RANDOM_ROLL_SECS` range, drawn with
`fastrand`.

## Build

Pure r2r: the build generates r2r bindings for `momentedge_msgs` (and the rest
of the `IDL_PACKAGE_FILTER` set) at `cargo build`, gated by `IDL_PACKAGE_FILTER`.
Needs the flake's `LIBCLANG_PATH`; does not need `ROS_DISTRO`.

As an example it is `publish = false`, but it is still a workspace member, so it
inherits the shared version/edition and `[workspace.dependencies]` versions like
the recorder crate — see the root [`CLAUDE.md`](../../CLAUDE.md) "Workspace
layout". `scripts/build-on-target.sh` builds it (`-p trigger-pub`) alongside
`clipper` for the on-target demo.
