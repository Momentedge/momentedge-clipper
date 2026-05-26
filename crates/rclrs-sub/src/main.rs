//! rclrs counterpart to `r2r-sub`: attach to *every* live topic, take each
//! message as raw serialized CDR (one copy out of the RMW, no field decode),
//! and store it in a per-topic, timestamp-indexed `BTreeMap`.
//!
//! Same behaviour as the r2r recorder, different runtime model:
//!   - **No async runtime.** Where r2r-sub drives subscription *streams* on a
//!     tokio runtime, this spawns one plain OS thread per topic. Each thread
//!     owns its own `BTreeMap<u64, Vec<u8>>` — thread-local, so there is no
//!     shared index and no lock — and polls its own subscription in a loop.
//!   - **Raw CDR via rclrs serialized subscriptions** (ros2-rust/ros2_rust#592):
//!     `Node::create_serialized_subscription` + `SerializedSubscription::take`
//!     call `rcl_take_serialized_message`, handing us the serialized bytes with
//!     no per-type Rust codegen. The typesupport is resolved at runtime from the
//!     introspection library, so it works for any installed message type — the
//!     same property r2r's `subscribe_raw` relies on.
//!   - **No executor spin.** `take()` polls the RMW reader cache directly (the
//!     Fast DDS receive threads fill it asynchronously), and
//!     `get_topic_names_and_types()` is a synchronous graph query. Nothing here
//!     needs the rclrs executor to spin, so the `Node` is simply shared (`Arc`)
//!     across the per-topic threads, each of which creates and polls its own
//!     subscription.
//!
//! Index: one `BTreeMap<u64, Vec<u8>>` per topic (owned by that topic's thread),
//! keyed by the header stamp in nanoseconds since the Unix epoch
//! (`sec * 1e9 + nanosec`) — the flat u64 ns clock MCAP uses for `log_time`.
//! Only the leading `builtin_interfaces/Time` stamp is decoded (8 bytes straight
//! out of the CDR buffer); the message body is never deserialized. Per-thread
//! maps mean topics never collide with each other; a true intra-topic duplicate
//! (the same stamp resent, or a looped bag replay) overwrites and is counted.
//! Header-less messages (e.g. `tf2_msgs/TFMessage`, whose first field is a
//! sequence) fail the stamp sanity gate and are counted but not indexed.
//!
//! Pair it with ../ros2_sources: `ros2 bag play bags/example-011-ugv-ds.mcap`.
//! Both must share RMW_IMPLEMENTATION and ROS_DOMAIN_ID — the flake's shellHook
//! sets the same values as ../ros2_sources.
//!
//! Logging uses the `log` facade with a pretty_env_logger backend. Control
//! verbosity with `RUST_LOG` (defaults to `info`):
//!   - `info`  — startup, per-topic subscriptions, periodic per-topic stats
//!   - `debug` — additionally logs every received message (topic, size, stamp)
//!   - `warn`  — skipped topics and take failures

use std::collections::{BTreeMap, HashSet};
use std::thread;
use std::time::Duration;

use log::{debug, info, warn};
use rclrs::{
    Context, CreateBasicExecutor, MessageTypeName, QoSProfile, SerializedMessage,
    SubscriptionOptions,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Default to `info` when RUST_LOG is unset; RUST_LOG still overrides it.
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    // `default_from_env` reads ROS args/env (ROS_DOMAIN_ID, RMW_IMPLEMENTATION),
    // matching how r2r's Context initialises — so both subscribers line up on the
    // same graph. The executor is created only because it is the constructor for
    // nodes; we never spin it (see the module docs).
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("rclrs_all_topics_sub")?;

    info!("Discovering topics… replay with `ros2 bag play` in ../ros2_sources. Ctrl-C to stop.");

    // Topics we've already spawned a recorder thread for. A topic with no
    // advertised type yet is left out so it is retried on the next pass.
    let mut known: HashSet<String> = HashSet::new();
    loop {
        match node.get_topic_names_and_types() {
            Ok(tnat) => {
                for (topic, types) in tnat {
                    if known.contains(&topic) {
                        continue;
                    }
                    let Some(ty) = types.into_iter().next() else {
                        continue; // no advertised type yet; retry next pass
                    };
                    known.insert(topic.clone());

                    // Hand this topic its own thread + node handle (an Arc clone).
                    let node = node.clone();
                    thread::Builder::new()
                        .name(format!("rec:{topic}"))
                        .spawn(move || record_topic(node, topic, ty))
                        .expect("spawn topic recorder thread");
                }
            }
            Err(e) => warn!("topic discovery failed: {e}"),
        }
        thread::sleep(Duration::from_millis(500));
    }
}

/// Record one topic for the life of the process. Runs on its own OS thread and
/// owns everything it touches: a serialized subscription and a thread-local
/// `BTreeMap` index. Polls `take()` in a tight loop, sleeping only when the RMW
/// queue is empty so we don't busy-spin a core.
fn record_topic(node: rclrs::Node, topic: String, ty: String) {
    // "<package>/msg/<Type>" → MessageTypeName. Anything else (a malformed or
    // action/service type) is skipped.
    let msg_type = match MessageTypeName::try_from(ty.as_str()) {
        Ok(t) => t,
        Err(e) => {
            warn!("skip  {topic}  [{ty}]: {e}");
            return;
        }
    };

    // Best-effort QoS is the most permissive matcher: it pairs with both
    // best-effort and reliable publishers.
    let mut options = SubscriptionOptions::new(topic.as_str());
    options.qos = QoSProfile::topics_default().best_effort();

    let mut sub = match node.create_serialized_subscription(msg_type, options) {
        Ok(sub) => sub,
        Err(e) => {
            // Typesupport library missing, or QoS rejected.
            warn!("skip  {topic}  [{ty}]: {e}");
            return;
        }
    };
    info!("subscribed  {topic}  [{ty}]");

    // Thread-local index: this map lives on this thread and is owned solely by
    // it — no `Arc`, no `Mutex`.
    let mut index: BTreeMap<u64, Vec<u8>> = BTreeMap::new();
    // One reusable take buffer. rcl overwrites it (and resizes it if needed) on
    // every take, so the only per-message allocation is the `Vec` we keep.
    let mut buf = SerializedMessage::new(4096).expect("allocate serialized buffer");

    let mut received = 0u64;
    let mut collisions = 0u64;
    let mut total_bytes = 0u64;

    loop {
        match sub.take(&mut buf) {
            Ok(Some(_info)) => {
                let bytes = buf.as_bytes();
                received += 1;
                total_bytes += bytes.len() as u64;
                match cdr_header_stamp_ns(bytes) {
                    Some(ns) => {
                        debug!(
                            "{topic:<52} {ty:<34} #{received:<6} {:>7} B  ns={ns}",
                            bytes.len()
                        );
                        // The only copy out of the reused buffer into the index.
                        if index.insert(ns, bytes.to_vec()).is_some() {
                            collisions += 1; // same topic, same stamp (looped replay)
                        }
                    }
                    None => {
                        debug!("{topic}: no leading stamp, not indexed ({} B)", bytes.len())
                    }
                }
                if received.is_multiple_of(500) {
                    let span = match (index.keys().next(), index.keys().next_back()) {
                        (Some(&first), Some(&last)) => (last - first) as f64 / 1e9,
                        _ => 0.0,
                    };
                    info!(
                        "{topic}: {received} msgs, {} keys, {collisions} collisions, {:.1} MiB, span {span:.2}s",
                        index.len(),
                        total_bytes as f64 / 1_048_576.0,
                    );
                }
            }
            // No message queued right now: yield briefly rather than burning a core.
            Ok(None) => thread::sleep(Duration::from_millis(1)),
            Err(e) => {
                warn!("{topic}: take failed: {e}");
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

/// Read the leading `builtin_interfaces/Time` stamp from a serialized CDR
/// message and return nanoseconds since the Unix epoch (`sec * 1e9 + nanosec`).
///
/// This decodes *only* the timestamp — the message body is never touched. It
/// assumes the message's first field is a `std_msgs/Header` (or a bare `Time`),
/// which holds for nearly all stamped sensor/geometry/nav messages. CDR layout:
/// a 4-byte encapsulation header precedes the body, so `sec` (`int32`) sits at
/// offset 4 and `nanosec` (`uint32`) at offset 8; byte 1 of the encapsulation
/// header selects little-endian (low bit set) vs big-endian.
///
/// Returns `None` when the buffer is too short or the parsed stamp is
/// implausible — which is how header-less messages (e.g. `tf2_msgs/TFMessage`,
/// whose first field is a sequence) are rejected rather than mis-indexed.
fn cdr_header_stamp_ns(buf: &[u8]) -> Option<u64> {
    if buf.len() < 12 {
        return None;
    }
    let little_endian = buf[1] & 1 == 1;
    let bytes_at = |off: usize| -> [u8; 4] { buf[off..off + 4].try_into().unwrap() };
    let (sec, nanosec) = if little_endian {
        (i32::from_le_bytes(bytes_at(4)), u32::from_le_bytes(bytes_at(8)))
    } else {
        (i32::from_be_bytes(bytes_at(4)), u32::from_be_bytes(bytes_at(8)))
    };

    // Sanity gate: a real stamp has sec >= 0 and nanosec < 1e9. This rejects
    // buffers whose offset 4/8 aren't actually a leading Time field.
    if sec < 0 || nanosec >= 1_000_000_000 {
        return None;
    }
    Some(sec as u64 * 1_000_000_000 + nanosec as u64)
}
