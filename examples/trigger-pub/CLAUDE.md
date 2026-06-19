# trigger-pub — contributor notes

[README.md](README.md) covers what this example is, its flags, and how to run
it. This file is the rationale beyond it.

## Why the stamp is publish time

`trigger_time` is stamped with the current RosTime at publish (`r2r::Clock`,
`ClockType::RosTime` → `Clock::to_builtin_time`). That is the point of the
stand-in: `trigger_time` is the "original timestamp" the recorder centres its
`[t-preroll, t+postroll]` window on, rather than the trigger's arrival time. The
node has no subscriptions, but `spin_once`s briefly each iteration to keep its
graph/publisher machinery healthy, then sleeps out the period. The random window
bounds are the inclusive `RANDOM_ROLL_SECS` range, drawn with `fastrand`.

## Build

Pure r2r: the build generates r2r bindings for `momentedge_msgs` (and the rest
of the `IDL_PACKAGE_FILTER` set) at `cargo build`, gated by `IDL_PACKAGE_FILTER`.
Needs the flake's `LIBCLANG_PATH`; does not need `ROS_DISTRO`.

As an example it is `publish = false`, but it is still a workspace member, so it
inherits the shared version/edition and `[workspace.dependencies]` versions like
the recorder crate — see the root [`CLAUDE.md`](../../CLAUDE.md) "Workspace
layout". `scripts/build-on-target.sh` builds it (`-p trigger-pub`) alongside
`clipper` for the on-target demo.
