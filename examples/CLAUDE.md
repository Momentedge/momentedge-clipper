# examples/ — contributor notes

Each README here is the user-facing half: what the example is, its flags, and
how to run it. These files are the rationale beyond them, and one rule they all
share.

**Every example crate is deliberately minimal, and staying that way is the
point.** They are read as teaching material, so a single `src/main.rs` with its
tests beside it is the target shape — not a module tree, not a config layer, not
a trait to swap implementations behind. Growing one costs the thing it exists
for. **Confirm before adding structure to any of them**, however reasonable the
structure looks in isolation.

**They are workspace members for dependency inheritance, not for shipping.**
Membership is what gives them `[workspace.package]` and the shared
`[workspace.dependencies]` versions; every one is `publish = false`. The heavy
exception is [`cu-mcap-record`](cu-mcap-record/CLAUDE.md), whose cu29
dependencies stay declared in its own `Cargo.toml` so that tree never enters
`[workspace.dependencies]`.

**Three of them are live e2e fixtures, and the coupling is invisible from
here.** `crates/clipper/tests/e2e.rs` builds and drives
[`custom-mcap-writer`](custom-mcap-writer/CLAUDE.md) and
[`cu-mcap-record`](cu-mcap-record/CLAUDE.md) as real producers, and
[`trigger-pub`](trigger-pub/CLAUDE.md) ships alongside `clipper` in the
on-target demo build. The assertions key on what those binaries *emit* — topic
names, trigger names, the stamp domain — so changing an output shape breaks a
test in another crate that names no example in its failure message. Check
`crates/clipper/tests/` before editing what one of them writes.

**Each writer example demonstrates one compliant producer path** for clipper's
tailability contract — append complete records, never seek back to rewrite one.
`custom-mcap-writer` is the unchunked path, `chunked-mcap-writer` the buffered-
chunk path, `cu-mcap-record` the same contract reached from copper-rs. The
contract itself is stated in
[ARCHITECTURE.md](../ARCHITECTURE.md#tailing-a-live-mcap) and its consequences
for the scan in [`crates/clip/CLAUDE.md`](../crates/clip/CLAUDE.md).
