//! Generic ROS2 subscriber: attach to *every* live topic and print a one-line
//! summary for each message that arrives.
//!
//! It does not know any message type at compile time. Instead it periodically
//! asks the ROS graph for all topics and their types
//! (`get_topic_names_and_types`), then opens an *untyped* subscription per
//! topic (`subscribe_untyped`), which decodes each message to a
//! `serde_json::Value`. New topics that appear later (e.g. when a looping
//! `ros2 bag play` restarts) are picked up on the next discovery pass.
//!
//! Pair it with ../ros2_sources:  `ros2 bag play bags/example-011-ugv-ds.mcap`.
//! Both must share RMW_IMPLEMENTATION and ROS_DOMAIN_ID — the flake's shellHook
//! sets the same values as ../ros2_sources.

use std::cell::RefCell;
use std::collections::HashSet;
use std::rc::Rc;
use std::time::{Duration, Instant};

use futures::executor::LocalPool;
use futures::stream::StreamExt;
use futures::task::LocalSpawnExt;
use r2r::QosProfile;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "rust_all_topics_sub", "")?;

    let mut pool = LocalPool::new();
    let spawner = pool.spawner();
    let start = Instant::now();

    // Topics we've already subscribed to (or deliberately skipped), so each
    // discovery pass only acts on what's new.
    let known: Rc<RefCell<HashSet<String>>> = Rc::new(RefCell::new(HashSet::new()));

    // Best-effort QoS is the most permissive matcher: it pairs with both
    // best-effort and reliable publishers, so we don't silently miss sensor
    // topics that offer best-effort.
    let qos = QosProfile::default().best_effort();

    println!("Discovering topics… replay with `ros2 bag play` in ../ros2_sources. Ctrl-C to stop.\n");

    loop {
        // --- Discovery: subscribe to any topic we haven't seen yet ----------
        for (topic, types) in node.get_topic_names_and_types()? {
            if known.borrow().contains(&topic) {
                continue;
            }
            let Some(ty) = types.into_iter().next() else {
                continue; // no advertised type yet; retry next pass
            };

            match node.subscribe_untyped(&topic, &ty, qos.clone()) {
                Ok(mut sub) => {
                    known.borrow_mut().insert(topic.clone());
                    println!("+ subscribed  {topic}  [{ty}]");

                    let count = Rc::new(RefCell::new(0u64));
                    spawner.spawn_local(async move {
                        while let Some(msg) = sub.next().await {
                            match msg {
                                Ok(value) => {
                                    *count.borrow_mut() += 1;
                                    let n = *count.borrow();
                                    let bytes =
                                        serde_json::to_vec(&value).map(|b| b.len()).unwrap_or(0);
                                    let stamp = header_stamp(&value);
                                    let t = start.elapsed().as_secs_f64();
                                    println!(
                                        "[{t:8.3}] {topic:<52} {ty:<34} #{n:<6} {bytes:>7} B{stamp}"
                                    );
                                }
                                Err(e) => eprintln!("! {topic}: decode error: {e}"),
                            }
                        }
                    })?;
                }
                Err(e) => {
                    // Type not in the generated bindings, or QoS rejected.
                    // Remember it so we warn once, not every pass.
                    known.borrow_mut().insert(topic.clone());
                    eprintln!("! skip  {topic}  [{ty}]: {e}");
                }
            }
        }

        // --- Pump ROS callbacks, then drive the async subscription tasks ----
        node.spin_once(Duration::from_millis(100));
        pool.run_until_stalled();
    }
}

/// If the message carries a `std_msgs/Header`, format its stamp as
/// `  stamp=<sec>.<nanosec>` for the log line; otherwise return "".
fn header_stamp(value: &serde_json::Value) -> String {
    let stamp = value.get("header").and_then(|h| h.get("stamp"));
    match stamp {
        Some(s) => {
            let sec = s.get("sec").and_then(|v| v.as_i64()).unwrap_or(0);
            let nsec = s.get("nanosec").and_then(|v| v.as_u64()).unwrap_or(0);
            format!("  stamp={sec}.{nsec:09}")
        }
        None => String::new(),
    }
}
