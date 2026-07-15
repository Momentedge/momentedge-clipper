//! The outside-facing interface seam: trigger input and completion output as
//! one unit, in exactly one of two forms selected by `--interface`.
//!
//! An [`Interface`] is the only layer that knows ROS from MCAP or one wire
//! encoding from another. It produces decoded [`Trigger`]s (calling the driver's
//! `fire` callback) and owns the completion half through its [`Announce`]r:
//!
//! - [`RosInterface`] subscribes to the trigger topic on a ROS node and
//!   publishes `Recorded` on completion. Its `run` owns the node and spawns its
//!   own spin thread, so the driver supervises one uniform interface thread in
//!   either mode.
//! - [`McapInterface`] drains [`TriggerRecord`]s the tail lifts out of the recorded
//!   MCAP, decoding each by `message_encoding` ([`decode_trigger`]). It touches
//!   no ROS node, executor, or subscription — the recorder runs ROS-free in this
//!   mode. Completion is implicit: the clip's atomic move into `out_dir` is the
//!   signal, so its [`Announce`]r is a no-op.
//!
//! The trait is generic (not `dyn`), so the driver dispatches statically over
//! whichever interface is active.

use std::pin::Pin;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context;
use crossbeam_channel::{Receiver, select};
use futures::executor::block_on;
use futures::stream::{Stream, StreamExt};
use log::{error, info, warn};
use r2r::{Publisher, QosProfile};

use crate::TimeSource;
use crate::decode::decode_trigger;
use crate::supervision::{harvest_panic, spawn_supervised};
use crate::trigger::{Announce, Completion, Trigger, TriggerRecord, now_ns};

/// The window anchor an interface resolved for one trigger, plus whether it came
/// from the trigger's own `trigger_time` field. Exactly one cell of the
/// interface × `--time-source` matrix reads `trigger_time` (`ros` + `publish`);
/// every other cell anchors on a transport stamp and ignores it. The driver uses
/// `from_trigger_time` to reject a trigger that set `trigger_time` in a cell that
/// ignores it — the field would otherwise be silently dropped and the window
/// mis-anchored (see [`crate::validate_trigger`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Anchor {
    pub ns: u64,
    pub from_trigger_time: bool,
}

/// The window anchor the MCAP interface resolves for one trigger record: its own
/// `log_time` or `publish_time`, per the active `--time-source`. The record's
/// stamp — not the decoded `trigger_time` — anchors the window, because a
/// publisher cannot set the recording's clock on the wire, so this anchor is
/// never `from_trigger_time`.
fn resolve_mcap_anchor(record: &TriggerRecord, source: TimeSource) -> Anchor {
    let ns = match source {
        TimeSource::Log => record.log_time,
        TimeSource::Publish => record.publish_time,
    };
    Anchor {
        ns,
        from_trigger_time: false,
    }
}

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

/// One active interface to the outside world: a trigger source paired with its
/// completion sink. Exactly one is active per run (`--interface`). Generic so the
/// driver dispatches statically — no `Box<dyn>`.
pub(crate) trait Interface: Send + Sized + 'static {
    /// The completion sink, cloned once per trigger handler thread.
    type Announcer: Announce;

    /// A short label for logs (`"ros"` / `"mcap"`).
    fn name(&self) -> &'static str;

    /// A fresh announcer handle for a trigger handler.
    fn announcer(&self) -> Self::Announcer;

    /// Drive the interface for the process's lifetime, calling `fire` with each
    /// decoded [`Trigger`] and the [`Anchor`] this interface resolved for it —
    /// the instant the clip window centres on, and whether it came from
    /// `trigger_time`. The ROS interface resolves it from `now` or the trigger's
    /// `trigger_time` per `--time-source`; the MCAP interface from the trigger
    /// record's own stamp. Returns only on an end or fault that the driver treats
    /// as a reason to exit the process. `fire` is called from a single thread, so
    /// it need not be `Sync`.
    fn run<F>(self, fire: F) -> anyhow::Result<()>
    where
        F: Fn(Trigger, Anchor) + Send + 'static;
}

// ── ROS interface ───────────────────────────────────────────────────────────

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
                // (the `From` impl in `crate::decode`). The anchor is resolved on
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

/// A finished clip's [`Completion`] maps field-for-field onto the r2r-generated
/// `momentedge_msgs/Recorded` the ROS interface publishes (its [`Stamp`] onto
/// the nested `builtin_interfaces/Time`). By reference — the announcer keeps the
/// `Completion` to log from after the publish. The orphan rule permits this
/// foreign-target impl because [`Completion`] is local.
///
/// [`Stamp`]: crate::trigger::Stamp
impl From<&Completion> for r2r::momentedge_msgs::msg::Recorded {
    fn from(completion: &Completion) -> Self {
        r2r::momentedge_msgs::msg::Recorded {
            name: completion.name.clone(),
            filenames: completion.filenames.clone(),
            description: completion.description.clone(),
            trigger_time: r2r::builtin_interfaces::msg::Time {
                sec: completion.trigger_time.sec,
                nanosec: completion.trigger_time.nanosec,
            },
            preroll: completion.preroll,
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

// ── MCAP interface ────────────────────────────────────────────────────────────

/// The MCAP interface: drains [`TriggerRecord`]s the tail lifts out of the
/// recorded file and decodes each by its `message_encoding`. ROS-free at runtime
/// — no node, executor, or subscription.
pub(crate) struct McapInterface {
    triggers: Receiver<TriggerRecord>,
    /// The topic the tap was wired to — carried only so a skipped trigger names
    /// the topic it actually came off.
    trigger_topic: Arc<str>,
    /// The clock domain the anchor is read from: each trigger record's own
    /// `log_time` or `publish_time`.
    time_source: crate::TimeSource,
}

impl McapInterface {
    /// Drive from the tail's decode-free trigger tap (`Tailer::with_trigger_tap`),
    /// which taps `trigger_topic`. `time_source` selects which of each trigger
    /// record's two stamps anchors its window.
    pub(crate) fn new(
        trigger_topic: &str,
        triggers: Receiver<TriggerRecord>,
        time_source: crate::TimeSource,
    ) -> Self {
        McapInterface {
            triggers,
            trigger_topic: trigger_topic.into(),
            time_source,
        }
    }
}

impl Interface for McapInterface {
    type Announcer = NullAnnouncer;

    fn name(&self) -> &'static str {
        "mcap"
    }

    fn announcer(&self) -> NullAnnouncer {
        NullAnnouncer
    }

    /// Drain the tap until it closes (the tail thread is gone). Each raw trigger
    /// is decoded by `message_encoding`; an undecodable one (unknown encoding or
    /// malformed body) is logged and skipped, never fatal — one bad trigger must
    /// not stop the recorder.
    fn run<F>(self, fire: F) -> anyhow::Result<()>
    where
        F: Fn(Trigger, Anchor) + Send + 'static,
    {
        for raw in self.triggers.iter() {
            match decode_trigger(&raw.message_encoding, &raw.body) {
                Ok(trigger) => {
                    // The window anchors on the trigger record's own stamp — its
                    // `log_time` or `publish_time` per `--time-source` — not on
                    // the decoded `trigger_time` (which the publisher cannot align
                    // to the recording's clock on the wire).
                    let anchor = resolve_mcap_anchor(&raw, self.time_source);
                    info!(
                        "MCAP trigger name={:?} encoding={} source={} anchor={} \
                         (log_time={} publish_time={})",
                        trigger.name,
                        raw.message_encoding,
                        self.time_source,
                        anchor.ns,
                        raw.log_time,
                        raw.publish_time,
                    );
                    fire(trigger, anchor);
                }
                Err(e) => warn!(
                    "skipping an MCAP trigger on {} (encoding={}): {e:#}",
                    self.trigger_topic, raw.message_encoding,
                ),
            }
        }
        anyhow::bail!("the MCAP trigger tap closed")
    }
}

/// The MCAP completion sink: a no-op. The clip's atomic move into `out_dir`
/// (`clip::publish_clip`) is the announcement; the handler's per-clip `info!`
/// lines are the log.
#[derive(Clone)]
pub(crate) struct NullAnnouncer;

impl Announce for NullAnnouncer {
    fn announce(&self, _completion: &Completion) {}
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use crossbeam_channel::unbounded;

    use super::*;
    use crate::trigger::Stamp;

    /// CDR-serialize a `momentedge_msgs/Trigger` the way rosbag2 writes it, so a
    /// [`TriggerRecord`] with `message_encoding = "cdr"` decodes back to it.
    fn cdr_trigger_bytes(name: &str) -> Vec<u8> {
        use r2r::WrappedTypesupport;
        r2r::momentedge_msgs::msg::Trigger {
            name: name.to_string(),
            description: "e2e".to_string(),
            trigger_time: r2r::builtin_interfaces::msg::Time { sec: 1, nanosec: 2 },
            preroll: 10,
            postroll: 20,
        }
        .to_serialized_bytes()
        .expect("serialize")
    }

    fn raw(encoding: &str, body: Vec<u8>, log_time: u64) -> TriggerRecord {
        raw_stamped(encoding, body, log_time, log_time)
    }

    fn raw_stamped(
        encoding: &str,
        body: Vec<u8>,
        log_time: u64,
        publish_time: u64,
    ) -> TriggerRecord {
        TriggerRecord {
            message_encoding: encoding.to_string(),
            body,
            log_time,
            publish_time,
        }
    }

    /// The McapInterface decodes each tapped record by its `message_encoding` and
    /// fires the callback once per decoded trigger, in order.
    #[test]
    fn mcap_interface_decodes_and_fires_each_trigger() {
        let (tx, rx) = unbounded();
        tx.send(raw("cdr", cdr_trigger_bytes("first"), 100))
            .unwrap();
        tx.send(raw("cdr", cdr_trigger_bytes("second"), 200))
            .unwrap();
        // Drop the sender so `run` drains the buffered records and returns.
        drop(tx);

        let fired: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = fired.clone();
        let iface = McapInterface::new(crate::TRIGGER_TOPIC, rx, TimeSource::Log);
        // `run` bails when the tap closes — expected, the records are drained first.
        let _ = iface.run(move |t, _anchor| sink.lock().unwrap().push(t.name));

        assert_eq!(*fired.lock().unwrap(), vec!["first", "second"]);
    }

    /// The failure path: an undecodable record (unknown encoding or malformed
    /// body) is logged and skipped, never fatal — the good triggers around it
    /// still fire.
    #[test]
    fn mcap_interface_skips_undecodable_triggers_and_keeps_going() {
        let (tx, rx) = unbounded();
        // Unknown encoding — no decoder.
        tx.send(raw("ros1", b"whatever".to_vec(), 1)).unwrap();
        // Known encoding, malformed body — decode fails.
        tx.send(raw("cdr", b"not-a-valid-cdr-trigger".to_vec(), 2))
            .unwrap();
        // Malformed JSON — decode fails.
        tx.send(raw("json", b"{ this is not json".to_vec(), 3))
            .unwrap();
        // One good record survives the bad ones on either side.
        tx.send(raw("cdr", cdr_trigger_bytes("survivor"), 4))
            .unwrap();
        drop(tx);

        let fired: Arc<Mutex<Vec<String>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = fired.clone();
        let iface = McapInterface::new(crate::TRIGGER_TOPIC, rx, TimeSource::Log);
        let _ = iface.run(move |t, _anchor| sink.lock().unwrap().push(t.name));

        assert_eq!(
            *fired.lock().unwrap(),
            vec!["survivor"],
            "only the one decodable trigger fires; the three bad ones are skipped"
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

    /// A `json` trigger whose payload `trigger_time` (7_000_000_250 ns) differs
    /// from both record stamps, so the resolved anchor can only be coming from
    /// the record. The MCAP interface anchors on the record's `log_time` under
    /// `--time-source log` and on its `publish_time` under `publish` — never on
    /// the decoded `trigger_time`.
    const JSON_TRIGGER: &[u8] =
        br#"{"name":"t","trigger_time":{"sec":7,"nanosec":250},"preroll":1,"postroll":2}"#;

    /// Drain one `json` trigger record (log_time 100, publish_time 900) through
    /// the MCAP interface on `source` and return the anchors it fired. Every MCAP
    /// anchor is resolved from the record, so none is `from_trigger_time`.
    fn anchors_for(source: TimeSource) -> Vec<Anchor> {
        let (tx, rx) = unbounded();
        tx.send(raw_stamped("json", JSON_TRIGGER.to_vec(), 100, 900))
            .unwrap();
        drop(tx);
        let anchors: Arc<Mutex<Vec<Anchor>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = anchors.clone();
        let iface = McapInterface::new(crate::TRIGGER_TOPIC, rx, source);
        let _ = iface.run(move |_t, anchor| sink.lock().unwrap().push(anchor));
        let anchors = anchors.lock().unwrap();
        anchors.clone()
    }

    #[test]
    fn mcap_interface_anchors_on_the_record_log_time_under_log() {
        assert_eq!(
            anchors_for(TimeSource::Log),
            vec![Anchor {
                ns: 100,
                from_trigger_time: false
            }],
            "log anchors on the trigger record's log_time, not its trigger_time"
        );
    }

    #[test]
    fn mcap_interface_anchors_on_the_record_publish_time_under_publish() {
        assert_eq!(
            anchors_for(TimeSource::Publish),
            vec![Anchor {
                ns: 900,
                from_trigger_time: false
            }],
            "publish anchors on the trigger record's publish_time"
        );
    }

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
}
