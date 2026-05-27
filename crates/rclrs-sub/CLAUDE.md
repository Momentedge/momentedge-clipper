# rclrs-sub

A ROS2 recorder built on [rclrs](https://github.com/ros2-rust/ros2_rust), the
counterpart to `r2r-sub` with the same behaviour but a different runtime model.
See the [workspace `CLAUDE.md`](../../CLAUDE.md) for the shared recorder model and
the `cdr_header_stamp_ns` stamp logic.

## Runtime model: no async, one thread per topic

There is no async runtime. The main thread polls `get_topic_names_and_types()` (a
synchronous rcl graph query) every 500 ms and spawns **one plain OS thread per
topic**. Each thread:

- creates its own `SerializedSubscription` (best-effort QoS),
- owns a **thread-local `BTreeMap<u64, Vec<u8>>`** — no shared index, no lock,
- loops `sub.take(&mut buf)` into one reused `SerializedMessage`, copying the
  bytes into the map only when a message arrives.

Nothing spins an rclrs executor. `take()` polls the RMW reader cache directly
(the Fast DDS receive threads fill it), and the graph query is synchronous, so
the `Node` is just shared across threads as an `Arc` and never driven by an
executor. `take()` takes `&mut self`: a serialized subscription has a single
consumer and `rcl_take_serialized_message` is not safe to call concurrently on
one subscription, so requiring `&mut self` keeps the `Send`/`Sync` impls sound
without an internal lock. It returns `Ok(None)` only for
`RCL_RET_SUBSCRIPTION_TAKE_FAILED` (the empty-queue signal) and `Err` for any
real failure; on `None` the thread sleeps ~1 ms to avoid busy-spinning a core.

This is a deliberate contrast with `r2r-sub`, which funnels every topic through a
single writer task over a channel. Here the maps are genuinely independent.

## Dependency: a forked rclrs

rclrs has no raw-CDR subscription in any released version. The
`SerializedSubscription` / `SerializedMessage` / `create_serialized_subscription`
API comes from the unmerged [PR #592](https://github.com/ros2-rust/ros2_rust/pull/592)
(tracking [issue #326](https://github.com/ros2-rust/ros2_rust/issues/326)).

`Cargo.toml` therefore depends on a fork branch,
`stfl/ros2_rust@serialized-transport-typesupport-fix`, which carries that API plus
fixes the PR needs to actually work:

- It passes rmw the **`rosidl_typesupport_c`** type support (the handle typed and
  dynamic subscriptions use), not the introspection type support — rmw_fastrtps
  rejects the latter with "Type support not from this implementation", failing
  every subscription.
- The type-support library is kept loaded for the subscription's lifetime, the
  publisher path gets the same fix, lock acquisition returns errors instead of
  panicking, and `take` has the `&mut self` / error-handling shape described above.

The sibling checkout `../ros2_rust` is that fork's working tree. Editing the API
means committing there, pushing the branch, and `cargo update -p rclrs`; the
normal build just fetches the branch over git.

## Build

rclrs's build script reads `ROS_DISTRO` (=`jazzy`) and `AMENT_PREFIX_PATH` and
links `rcl`/`rcl_action`/`rcl_yaml_param_parser`/`rcutils`/`rmw`/
`rmw_implementation`. It ships pre-generated rcl bindings (one committed file per
distro), so it needs **no bindgen and no libclang**, unlike r2r. The serialized
path resolves type support at runtime via `ament_rs` + `libloading`, so no
generated message crates and no colcon/cargo-ament-build are involved.

The flake must export `ROS_DISTRO` (the build script aborts without it) and must
provide `example-interfaces` + `test-msgs`, whose type-support libraries rclrs's
vendored interfaces link unconditionally
([#557](https://github.com/ros2-rust/ros2_rust/issues/557)).

## Run

```bash
nix develop --command cargo run -p rclrs-sub
```

Same RMW + `ROS_DOMAIN_ID` as the publisher. Logging uses the `log` facade with a
`pretty_env_logger` backend; `RUST_LOG` controls verbosity (`info` for startup,
subscriptions, and per-topic stats; `debug` for every message).
