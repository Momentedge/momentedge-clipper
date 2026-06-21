# clipper — Architecture

The architecture overview lives at the repository root:
**[ARCHITECTURE.md](../../ARCHITECTURE.md)**.

clipper is the only crate in this workspace, so its design *is* the system
architecture: the thread model, tailing mechanics, recording collection, atomic
clip publication, restart/rollover recovery, the `ros`/`mcap` interface seam,
and the deployment build model are all covered there.

For deep implementation rationale and concurrency invariants, see
[CLAUDE.md](CLAUDE.md) in this directory.
