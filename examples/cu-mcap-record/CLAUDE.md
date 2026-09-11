# cu-mcap-record — contributor notes

[README.md](README.md) covers the copper task, its two divergences from
copper's own MCAP exporter, and how to run it.
[`examples/CLAUDE.md`](../CLAUDE.md) covers the rule every example here shares —
minimal on purpose, confirm before adding structure. This file is what neither
says.

**The cu29 dependencies stay in this crate's own `Cargo.toml`.** They are not in
the workspace's `[workspace.dependencies]`, deliberately: the copper tree is
heavy and scoping it to one member is what keeps a ROS-free `cargo build -p
clip` cheap. It still resolves through the shared root lockfile, and a lean CI
job builds and tests it with `-p cu-mcap-record`. Hoisting a cu29 dependency to
the workspace undoes that in one line.

**Two divergences from copper's exporter are the point of the example**, so
neither is a bug to tidy away: the writer is unchunked and append-only
(`use_chunks(false)`), and every stamp is translated into the absolute
Unix-epoch domain rather than copper's monotonic one. `EPOCH_FLOOR_NS`
(2020-01-01) asserts on every `log_time` and `publish_time` so a relative clock
fails loudly at the writer instead of producing a recording clipper windows
against nothing.

**It is a live e2e fixture.** `copper_sink_recording_produces_clip` in
`crates/clipper/tests/e2e.rs` runs this binary as a real producer against
`clipper tail --interface mcap`, with the trigger this example writes into its
own recording as the only input. The harness resolves the binary via
`CU_MCAP_RECORD_BIN`, else builds it on demand with `-p cu-mcap-record`; CI's
matrix `Build` step prebuilds it so the on-demand path stays inside the test
timeout. Changing the trigger topic, its `json` encoding, or the stamp domain
breaks that test.
