# chunked-mcap-writer — contributor notes

[README.md](README.md) covers the buffered-chunk configuration, chunk-close
latency, and how to run it. [`examples/CLAUDE.md`](../CLAUDE.md) covers the rule
every example here shares — minimal on purpose, confirm before adding structure.
This file is what neither says.

**Chunking is not the violation; seeking back is.** `use_chunks(true)` paired
with `disable_seeking(true)` selects the mcap crate's buffered chunk mode, which
assembles each zstd chunk in memory and appends it as one complete `Chunk`
record with a real length and CRC. Nothing is back-patched, so nothing a scan
reads is ever a placeholder. Both options together are what makes this
compliant — dropping either one silently produces an untailable file, which is
why the tests mutation-check both.

**`CHUNK_SIZE` is 4 KiB and closes a chunk roughly every 0.8 s** at this
example's own rate (~100 B/record at 50 Hz). That number is what `--grace-secs`
guidance is derived from: at least twice the fill interval, so 4 s is
comfortable. Treat any figure around 2 s as stale.

**Everything lives on log time here.** `publish_time == log_time` on every
record, on purpose: the capture-time story belongs to
[`custom-mcap-writer`](../custom-mcap-writer/CLAUDE.md), and splitting it across
both examples would make neither legible.

**Not an e2e fixture.** Unlike its two siblings this example is not driven by
`crates/clipper/tests/e2e.rs`; its own inline tests carry the proof — chunked
output, clean framing, decoding against a local `Trigger` mirror, a
truncated-prefix scan, and a sink that panics on rewrite (`AppendOnly`) to pin
the append-only claim at the writer's own boundary.
