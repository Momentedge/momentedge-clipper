//! The ROS arm of the interface seam, compiled in only under the `ros` feature.
//!
//! [`RosInterface`] subscribes to the trigger topic on a ROS node and publishes
//! `momentedge_msgs/Recorded` on completion through [`RosAnnouncer`]. It is the
//! device build's interface: the only part of the recorder that creates a
//! `Context`, a `Node`, or a subscription, and the only reason the binary links
//! r2r at all.
//!
//! Without the feature this module does not exist, `--interface` offers `mcap`
//! alone, and the recorder links no ROS — see the parent module.

use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context as _;
use clip::TimeSource;
use clip::trigger::{Announce, Completion, Trigger, now_ns};
use crossbeam_channel::select;
use futures::executor::block_on;
use futures::stream::{Stream, StreamExt};
use log::{error, info};
use r2r::{Publisher, QosProfile};

use super::{Anchor, Interface};
use crate::supervision::{harvest_panic, spawn_supervised};

/// The window anchor the ROS interface resolves for one trigger. A live ROS
/// trigger has no MCAP record — no `log_time`, no `publish_time`, and r2r
/// surfaces no wire timestamp — so the interface anchors on what it has. Under
/// `--time-source log` that is `now` (the subscription instant), faithful to
/// MCAP's `log_time` definition; the trigger's own `trigger_time` is ignored, so
/// the anchor is not `from_trigger_time`. Under `--time-source publish` the
/// publisher's `trigger_time` *is* the anchor — the one cell that reads it,
/// standing in for the `publish_time` a publisher cannot set on the wire.
fn resolve_ros_anchor(trigger: &Trigger, source: TimeSource, now: u64) -> Anchor {
    match source {
        TimeSource::Log => Anchor {
            ns: now,
            from_trigger_time: false,
        },
        TimeSource::Publish => Anchor {
            ns: trigger.trigger_time.ns(),
            from_trigger_time: true,
        },
    }
}

/// The ROS interface: a typed subscription to the trigger topic feeding the
/// driver, and a `Recorded` publisher behind [`RosAnnouncer`].
pub(crate) struct RosInterface {
    node: r2r::Node,
    sub: Pin<Box<dyn Stream<Item = r2r::momentedge_msgs::msg::Trigger> + Send>>,
    announcer: RosAnnouncer,
    /// The clock domain the anchor is resolved on: `log` anchors on the
    /// subscription instant, `publish` on the trigger's `trigger_time`.
    time_source: TimeSource,
}

impl RosInterface {
    /// Create the node, the trigger subscription, and the `Recorded` publisher.
    /// `time_source` selects how each trigger's window anchor is resolved.
    pub(crate) fn new(
        trigger_topic: &str,
        recorded_topic: &str,
        time_source: TimeSource,
    ) -> anyhow::Result<Self> {
        let ctx = r2r::Context::create()?;
        let mut node = r2r::Node::create(ctx, "clipper", "")?;
        let sub = node.subscribe::<r2r::momentedge_msgs::msg::Trigger>(
            trigger_topic,
            QosProfile::default(),
        )?;
        let recorded_pub = node.create_publisher::<r2r::momentedge_msgs::msg::Recorded>(
            recorded_topic,
            QosProfile::default(),
        )?;
        Ok(RosInterface {
            node,
            sub: Box::pin(sub),
            announcer: RosAnnouncer {
                recorded_pub,
                recorded_topic: recorded_topic.into(),
            },
            time_source,
        })
    }
}

impl Interface for RosInterface {
    type Announcer = RosAnnouncer;

    fn name(&self) -> &'static str {
        "ros"
    }

    fn announcer(&self) -> RosAnnouncer {
        self.announcer.clone()
    }

    /// Own the node spin and the subscription drain as two internal supervised
    /// threads, returning when either resolves. A live subscription needs the
    /// node spun, so the two run concurrently; encapsulating both here keeps the
    /// driver's supervision uniform across interfaces, and a dead spin thread
    /// (which would otherwise silently stall trigger delivery) still surfaces.
    fn run<F>(self, fire: F) -> anyhow::Result<()>
    where
        F: Fn(Trigger, Anchor) + Send + 'static,
    {
        let RosInterface {
            mut node,
            mut sub,
            time_source,
            ..
        } = self;

        let spin = spawn_supervised("node-spin", move || {
            loop {
                node.spin_once(Duration::from_millis(10));
            }
        });
        let drain = spawn_supervised("trigger-drain", move || -> anyhow::Result<()> {
            while let Some(t) = block_on(sub.next()) {
                // `t.into()` is the shared r2r-Trigger -> domain-Trigger conversion
                // (the `From` impl in `clip::decode`). The anchor is resolved on
                // the active `--time-source`: `now` at this subscription instant
                // under `log`, the trigger's `trigger_time` under `publish`.
                let trigger: Trigger = t.into();
                let anchor = resolve_ros_anchor(&trigger, time_source, now_ns());
                fire(trigger, anchor);
            }
            anyhow::bail!("the trigger subscription stream ended")
        });

        let (spin_rx, spin_handle) = spin;
        let (drain_rx, drain_handle) = drain;
        select! {
            recv(spin_rx) -> res => match res {
                Ok(()) => anyhow::bail!("node spin thread exited unexpectedly"),
                Err(_) => Err(harvest_panic(spin_handle)
                    .context("node spin thread exited unexpectedly")),
            },
            recv(drain_rx) -> res => match res {
                Ok(r) => r.context("trigger drain ended"),
                Err(_) => Err(harvest_panic(drain_handle)
                    .context("trigger drain thread exited unexpectedly")),
            },
        }
    }
}

/// The ROS completion sink: publishes a `momentedge_msgs/Recorded` per finished
/// clip. `Clone` (the r2r `Publisher` is) so every handler thread holds its own;
/// the topic it publishes on rides along as an `Arc<str>` so the logs name the
/// topic this announcer was actually built with.
#[derive(Clone)]
pub(crate) struct RosAnnouncer {
    recorded_pub: Publisher<r2r::momentedge_msgs::msg::Recorded>,
    recorded_topic: Arc<str>,
}

impl Announce for RosAnnouncer {
    fn announce(&self, completion: &Completion) {
        let recorded = r2r::momentedge_msgs::msg::Recorded::from(completion);
        match self.recorded_pub.publish(&recorded) {
            Ok(()) => info!(
                "emitted {} name={:?} filenames={:?}",
                self.recorded_topic, completion.name, completion.filenames,
            ),
            Err(e) => error!(
                "publishing {} for name={:?} failed: {e}",
                self.recorded_topic, completion.name,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use clip::trigger::Stamp;

    use super::*;

    /// A domain [`Trigger`] with a given `trigger_time`, for the ROS anchor
    /// resolvers (which read the domain trigger, not a raw record).
    fn ros_trigger(trigger_time_ns: u64) -> Trigger {
        Trigger {
            name: "t".to_string(),
            description: String::new(),
            trigger_time: Stamp {
                sec: (trigger_time_ns / 1_000_000_000) as i32,
                nanosec: (trigger_time_ns % 1_000_000_000) as u32,
            },
            preroll: 1,
            postroll: 2,
        }
    }

    /// ROS + `log`: the anchor is `now` (the subscription instant), and the
    /// trigger's own `trigger_time` is ignored — the resolved anchor is not
    /// `from_trigger_time`.
    #[test]
    fn ros_anchor_under_log_is_now_and_ignores_trigger_time() {
        let anchor = resolve_ros_anchor(&ros_trigger(7_000_000_250), TimeSource::Log, 42);
        assert_eq!(
            anchor,
            Anchor {
                ns: 42,
                from_trigger_time: false
            }
        );
    }

    /// ROS + `publish`: the anchor is the trigger's own `trigger_time` — the one
    /// cell that reads it — so the resolved anchor is `from_trigger_time`.
    #[test]
    fn ros_anchor_under_publish_reads_trigger_time() {
        let anchor = resolve_ros_anchor(&ros_trigger(7_000_000_250), TimeSource::Publish, 42);
        assert_eq!(
            anchor,
            Anchor {
                ns: 7_000_000_250,
                from_trigger_time: true
            }
        );
    }

    /// A `Completion` maps field-for-field onto the r2r `Recorded` the ROS
    /// interface publishes, its `Stamp` onto the nested `builtin_interfaces/Time`.
    #[test]
    fn completion_maps_onto_recorded() {
        let completion = Completion {
            name: "evt".to_string(),
            filenames: vec!["/out/a.mcap".to_string(), "/out/b.mcap".to_string()],
            description: "two segments".to_string(),
            trigger_time: Stamp {
                sec: 7,
                nanosec: 250,
            },
            preroll: 1_000,
        };
        let recorded = r2r::momentedge_msgs::msg::Recorded::from(&completion);
        assert_eq!(recorded.name, "evt");
        assert_eq!(recorded.filenames, completion.filenames);
        assert_eq!(recorded.description, "two segments");
        assert_eq!(recorded.trigger_time.sec, 7);
        assert_eq!(recorded.trigger_time.nanosec, 250);
        assert_eq!(recorded.preroll, 1_000);
    }
}
