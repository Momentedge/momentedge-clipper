# clipper — Architecture

The architecture overview lives at the repository root:
**[ARCHITECTURE.md](../../ARCHITECTURE.md)**.

The recorder is three crates — [`clip`](../clip) and [`tail`](../tail), the two
ROS-free libraries, and this binary over them — and the root document covers all
three as one system: the module map, the thread model, tailing mechanics, the
recording collection, atomic clip publication, restart/rollover recovery, the
`ros`/`mcap` interface seam, and the deployment build model.

For deep implementation rationale and concurrency invariants, see
[CLAUDE.md](CLAUDE.md) in this directory; it covers the libraries' internals too.
