//! Generic ROS2 recorder front-end: attach to *every* live topic, take each
//! message as raw serialized CDR (one copy out of the RMW, no field decode),
//! and store it in a timestamp-indexed `BTreeMap`.
//!
//! This follows r2r's `tokio_raw_subscriber` model: `subscribe_raw` yields the
//! serialized message bytes (`Vec<u8>`) rather than a decoded struct. The only
//! thing we decode is the leading `builtin_interfaces/Time` stamp, read
//! directly out of the CDR buffer (8 bytes), so the message body is never
//! deserialized. The raw `Vec<u8>` is then *moved* — not copied — over an mpsc
//! channel into a single writer task that owns the index.
//!
//! Index: one `BTreeMap<u64, Vec<u8>>` **per topic** (held in a
//! `HashMap<topic, BTreeMap<…>>`), keyed by the header stamp in **nanoseconds
//! since the Unix epoch** (`sec * 1e9 + nanosec`) — the same flat u64 ns clock
//! MCAP uses for `log_time`. Per-topic maps mean two different topics that share
//! a stamp (e.g. an image and its camera_info) no longer collide; only a true
//! intra-topic duplicate (the same message resent, or a looped bag replaying)
//! does, and a plain map overwrites it. A production recorder would use
//! `BTreeMap<u64, Vec<Vec<u8>>>`; we count overwrites and report them in the
//! periodic stats instead.
//!
//! Concurrency: the `Node` has a single owner — one blocking thread that does
//! everything touching it (spin, topic discovery, `subscribe_raw`). Spinning is
//! what feeds the streams, so it must run continuously; interleaving discovery
//! on the same thread avoids sharing the node under a mutex (a shared
//! `Arc<Mutex<Node>>` starves: a tight spin loop re-acquires the lock faster
//! than a discovery task can, since `std::sync::Mutex` is not fair). Per-topic
//! stream consumers are spawned onto the tokio runtime via a `Handle`; they
//! never touch the node. One writer task owns the `BTreeMap`, so it needs no
//! lock either.
//!
//! Pair it with ../ros2_sources:  `ros2 bag play bags/example-011-ugv-ds.mcap`.
//! Both must share RMW_IMPLEMENTATION and ROS_DOMAIN_ID — the flake's shellHook
//! sets the same values as ../ros2_sources.
//!
//! Logging uses the `log` facade with a pretty_env_logger backend. Control
//! verbosity with `RUST_LOG` (defaults to `info`):
//!   - `info`  — startup, per-topic subscriptions, periodic index stats
//!   - `debug` — additionally logs every received message (topic, size, stamp)
//!   - `warn`  — skipped topics

use std::collections::{BTreeMap, HashMap, HashSet};
use std::sync::Arc;
use std::time::{Duration, Instant};

use futures::stream::StreamExt;
use log::{debug, info, warn};
use r2r::QosProfile;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Default to `info` when RUST_LOG is unset; RUST_LOG still overrides it.
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_all_topics_sub", "")?;

    // Single writer owning one BTreeMap per topic. Per-topic tasks send
    // `(topic, stamp_ns, bytes)`; the `Arc<str>` topic is a cheap refcount bump
    // and the `Vec<u8>` moves through the channel by ownership (no byte copy).
    let (tx, mut rx) =
        tokio::sync::mpsc::unbounded_channel::<(Arc<str>, u64, Vec<u8>)>();
    tokio::spawn(async move {
        let mut index: HashMap<Arc<str>, BTreeMap<u64, Vec<u8>>> = HashMap::new();
        let mut received = 0u64;
        let mut collisions = 0u64;
        let mut total_bytes = 0u64;
        while let Some((topic, ns, bytes)) = rx.recv().await {
            received += 1;
            total_bytes += bytes.len() as u64;
            if index.entry(topic).or_default().insert(ns, bytes).is_some() {
                collisions += 1; // same topic, same stamp (e.g. a looped replay)
            }
            if received.is_multiple_of(500) {
                let keys: usize = index.values().map(BTreeMap::len).sum();
                info!(
                    "index: {received} msgs received, {} topics, {keys} keys, {collisions} collisions, {:.1} MiB",
                    index.len(),
                    total_bytes as f64 / 1_048_576.0,
                );
            }
        }
    });

    // Best-effort QoS is the most permissive matcher: it pairs with both
    // best-effort and reliable publishers.
    let qos = QosProfile::default().best_effort();

    // Handle used to spawn stream consumers from the (non-async) blocking thread.
    let runtime = tokio::runtime::Handle::current();

    info!("Discovering topics… replay with `ros2 bag play` in ../ros2_sources. Ctrl-C to stop.");

    // The node's single owner: spin it continuously (which feeds the streams)
    // and rescan for new topics every 500 ms on the same thread. No node lock
    // is taken anywhere, so the spin loop can run as tight as it likes.
    let worker = tokio::task::spawn_blocking(move || {
        let mut known: HashSet<String> = HashSet::new();
        let mut next_scan = Instant::now();
        loop {
            if Instant::now() >= next_scan {
                next_scan = Instant::now() + Duration::from_millis(500);
                match node.get_topic_names_and_types() {
                    Ok(tnat) => subscribe_new_topics(&mut node, &qos, &mut known, &runtime, &tx, tnat),
                    Err(e) => warn!("topic discovery failed: {e}"),
                }
            }
            node.spin_once(Duration::from_millis(10));
        }
    });

    worker.await?;
    Ok(())
}

/// Open a raw subscription for every topic in `tnat` not already in `known`,
/// and spawn a consumer task per new topic that forwards `(topic, stamp_ns,
/// bytes)` to the writer. Runs on the node-owning blocking thread.
fn subscribe_new_topics(
    node: &mut r2r::Node,
    qos: &QosProfile,
    known: &mut HashSet<String>,
    runtime: &tokio::runtime::Handle,
    tx: &tokio::sync::mpsc::UnboundedSender<(Arc<str>, u64, Vec<u8>)>,
    tnat: HashMap<String, Vec<String>>,
) {
    for (topic, types) in tnat {
        if known.contains(&topic) {
            continue;
        }
        let Some(ty) = types.into_iter().next() else {
            continue; // no advertised type yet; retry next pass
        };

        match node.subscribe_raw(&topic, &ty, qos.clone()) {
            Ok(mut sub) => {
                known.insert(topic.clone());
                info!("subscribed  {topic}  [{ty}]");
                // One shared topic key per subscription; consumers clone the
                // Arc (refcount bump) rather than the string on every message.
                let topic: Arc<str> = Arc::from(topic);
                let tx = tx.clone();
                runtime.spawn(async move {
                    let mut count = 0u64;
                    while let Some(bytes) = sub.next().await {
                        count += 1;
                        match cdr_header_stamp_ns(&bytes) {
                            Some(ns) => {
                                debug!(
                                    "{topic:<52} {ty:<34} #{count:<6} {:>7} B  ns={ns}",
                                    bytes.len()
                                );
                                // Route to this topic's map — bytes move, no copy.
                                let _ = tx.send((topic.clone(), ns, bytes));
                            }
                            None => {
                                debug!("{topic}: no leading stamp, not indexed ({} B)", bytes.len())
                            }
                        }
                    }
                });
            }
            Err(e) => {
                // Type not in the generated bindings, or QoS rejected.
                // Remember it so we warn once, not every pass.
                known.insert(topic.clone());
                warn!("skip  {topic}  [{ty}]: {e}");
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
