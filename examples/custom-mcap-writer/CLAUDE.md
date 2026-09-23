# custom-mcap-writer — contributor notes

[README.md](README.md) covers what it writes, the `publish_time` contract, and
how to run it. [`examples/CLAUDE.md`](../CLAUDE.md) covers the rule every
example here shares — minimal on purpose, confirm before adding structure. This
file is what neither says.

**`use_chunks(false)` is mandatory, not a default worth revisiting.** The mcap
crate's chunked writer back-patches a `Chunk` record's length after the fact, so
a scan meeting one mid-write reads the `u64::MAX` placeholder and faults. This
example is the *unchunked* compliant path; the buffered-chunk path that is also
tailable is [`chunked-mcap-writer`](../chunked-mcap-writer/CLAUDE.md), and
neither is a fallback for the other.

**This is the capture-time example, and that is its whole reason to exist.**
Every data message's `publish_time` is `log_time − --publish-offset-ms`, an
absolute Unix-epoch stamp the `EPOCH_FLOOR_NS` assertion enforces — a relative or
monotonic clock trips it deliberately. The log-time-only story stays in
`chunked-mcap-writer`; keeping the two separate is what makes either readable.

**It is a live e2e fixture.** `live_writer_capture_time_windowing` in
`crates/clipper/tests/e2e.rs` runs this binary against a real `clipper tail` and
asserts a clip named `_custom-mcap-writer-example.mcap` — so the trigger name,
the topic names, and the offset between the two stamps are load-bearing outside
this directory. The harness resolves the binary beside `CARGO_BIN_EXE_clipper`
and builds it on demand if absent (`crates/clipper/tests/harness/mod.rs`), so a
rename of the binary breaks the test, not just the build.

**Deliberately absent:** CDR encoding, file rotation, and `--compression` (which
is meaningless unchunked). The payload's `trigger_time` is `{0,0}` unless
`--stamp-payload-trigger-time` opts in, because the `mcap` trigger source's
cells *reject* a non-zero payload stamp — the record's own stamp is the anchor
there.
