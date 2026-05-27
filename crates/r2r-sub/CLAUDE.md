# r2r-sub

A ROS2 recorder built on [r2r](https://github.com/sequenceplanner/r2r) 0.9.5.
It attaches to every live topic via `subscribe_raw`, copies each serialized CDR
message into a timestamp-indexed map, and decodes only the header stamp. See the
[workspace `CLAUDE.md`](../../CLAUDE.md) for the shared recorder model and the
`cdr_header_stamp_ns` stamp logic, which this crate and `rclrs-sub` share.

## Why r2r here

`subscribe_raw(topic, type, qos)` yields a `Stream<Vec<u8>>` of serialized CDR —
one copy out of the RMW, no field decode — which is exactly the recorder's input.
This is r2r's off-the-shelf raw path; `rclrs-sub` reaches the same capability
through an unmerged upstream API.

## Concurrency model

The `r2r::Node` has a **single owner**: one `tokio::task::spawn_blocking` thread
does everything that touches the node — `spin_once`, topic discovery, and
`subscribe_raw`. Per-topic stream consumers are spawned onto the tokio runtime
via a `Handle` and never touch the node. A single writer task owns the index
(`HashMap<Arc<str>, BTreeMap<u64, Vec<u8>>>`) and receives `(topic, stamp_ns,
bytes)` tuples over an mpsc unbounded channel; the `Vec<u8>` moves by ownership,
and the `Arc<str>` topic is a refcount bump.

Single ownership is deliberate. A shared `Arc<Mutex<Node>>` deadlocks here: the
tight `spin_once` loop reacquires the lock faster than a discovery task can get
it (`std::sync::Mutex` is not fair), so discovery starves and only one topic ever
subscribes. Giving the node one owner removes the lock entirely.

The spin loop runs `node.spin_once(Duration::from_millis(10))` continuously
(spinning is what feeds the streams) and rescans topics every 500 ms. QoS is
`best_effort` — the permissive matcher that pairs with both best-effort and
reliable publishers.

## Build

r2r's build script runs `bindgen` (so `LIBCLANG_PATH` must point at libclang —
the flake sets it) and generates Rust bindings at `cargo build` time for every
message package on `AMENT_PREFIX_PATH`. `IDL_PACKAGE_FILTER` (a semicolon-
separated list in `flake.nix`) restricts that codegen to the packages the bag
uses, since r2r does no dependency resolution and would otherwise try to generate
everything. r2r does not need `ROS_DISTRO`.

## Run

```bash
nix develop --command cargo run -p r2r-sub
```

Needs a publisher on the graph (see the workspace doc for the bag replay) and the
same RMW + `ROS_DOMAIN_ID`. Logging uses the `log` facade with a
`pretty_env_logger` backend; `RUST_LOG` controls verbosity (`info` covers startup,
per-topic subscriptions, and periodic index stats; `debug` logs every message).
