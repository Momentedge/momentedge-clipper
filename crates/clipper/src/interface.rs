//! The outside-facing interface seam: trigger input and completion output as
//! one unit, in exactly one of two forms. Which one a run drives is
//! `clipper tail --trigger-source` — the flag names where the triggers come
//! from, and the completion half follows from it, because these two pairings are
//! the only ones there are ([`Interface::SOURCE`]).
//!
//! An [`Interface`] is the only layer that knows ROS from MCAP or one wire
//! encoding from another. It produces decoded [`Trigger`]s (calling the driver's
//! `fire` callback) and owns the completion half through its [`Announce`]r:
//!
//! - [`McapInterface`] drains [`TriggerRecord`]s the tail lifts out of the recorded
//!   MCAP, decoding each by `message_encoding` ([`decode_trigger`]). It touches
//!   no ROS node, executor, or subscription. Completion is implicit: the clip's
//!   atomic move into `out_dir` is the signal, so its [`Announce`]r is a no-op.
//!   Every build offers it.
//! - [`ros::RosInterface`] subscribes to the trigger topic on a ROS node and
//!   publishes `Recorded` on completion. Its `run` owns the node and spawns its
//!   own spin thread, so the driver supervises one uniform interface thread in
//!   either mode. The `ros` cargo feature compiles it in; without the feature
//!   the [`ros`] module does not exist and the recorder links no ROS at all.
//!
//! The trait is generic (not `dyn`), so the driver dispatches statically over
//! whichever interface is active.

use std::sync::Arc;

use clip::TimeSource;
use clip::decode::decode_trigger;
use clip::trigger::{Announce, Completion, Trigger, TriggerRecord};
use crossbeam_channel::Receiver;
use log::{info, warn};

use crate::TriggerSource;

#[cfg(feature = "ros")]
pub(crate) mod ros;

/// The window anchor an interface resolved for one trigger, plus whether it came
/// from the trigger's own `trigger_time` field. At most one cell of the
/// trigger-source × `--time-source` matrix reads `trigger_time` — `ros` +
/// `publish`,
/// so a build without the `ros` feature has no such cell at all; every other
/// cell anchors on a transport stamp and ignores it. The driver uses
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

/// One active interface to the outside world: a trigger source paired with its
/// completion sink. Exactly one is active per run, and the one that is active is
/// the trigger source's: `clipper tail --trigger-source` names where the
/// triggers come from, and [`SOURCE`](Interface::SOURCE) is each implementation
/// saying which value names it. There is no separate setting for the completion
/// half — the two cells here are the only pairings, and splitting them would
/// offer a third (in-recording triggers answered by a `Recorded` publish) that
/// no build without the `ros` feature could even provide.
///
/// Generic so the driver dispatches statically — no `Box<dyn>`.
pub(crate) trait Interface: Send + Sized + 'static {
    /// The completion sink, cloned once per trigger handler thread.
    type Announcer: Announce;

    /// The `--trigger-source` value that selects this interface. It is also the
    /// label the recorder logs itself up with, so the word an operator types and
    /// the word the log prints are one fact rather than two strings to keep in
    /// step.
    const SOURCE: TriggerSource;

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
    time_source: TimeSource,
}

impl McapInterface {
    /// Drive from the tail's decode-free trigger tap (`Tailer::with_trigger_tap`),
    /// which taps `trigger_topic`. `time_source` selects which of each trigger
    /// record's two stamps anchors its window.
    pub(crate) fn new(
        trigger_topic: &str,
        triggers: Receiver<TriggerRecord>,
        time_source: TimeSource,
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

    const SOURCE: TriggerSource = TriggerSource::Mcap;

    fn announcer(&self) -> NullAnnouncer {
        NullAnnouncer
    }

    /// Drain the tap until it closes (the tail thread is gone). Each raw trigger
    /// is decoded by `message_encoding`; an undecodable one (unknown encoding or
    /// malformed body) is logged and skipped, never fatal — one bad trigger must
    /// not stop the recorder. A `cdr` payload is undecodable in a build without
    /// the `ros` feature, which links no CDR typesupport; `json` decodes in every
    /// build.
    fn run<F>(self, fire: F) -> anyhow::Result<()>
    where
        F: Fn(Trigger, Anchor) + Send + 'static,
    {
        for raw in &self.triggers {
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
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        reason = "a failed unwrap or a panicking index is a failing test"
    )]

    use std::sync::{Arc, Mutex};

    use crossbeam_channel::unbounded;

    use super::*;

    /// A `json` trigger payload, the encoding every build decodes. Its
    /// `trigger_time` (7_000_000_250 ns) differs from both record stamps the
    /// tests below give a record, so a resolved anchor can only have come from
    /// the record.
    fn json_trigger_bytes(name: &str) -> Vec<u8> {
        format!(
            r#"{{"name":"{name}","trigger_time":{{"sec":7,"nanosec":250}},"preroll":1,"postroll":2}}"#
        )
        .into_bytes()
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

    /// Drain `records` through the MCAP interface on `source`, returning the
    /// `(name, anchor)` pair of every trigger it fired, in order. `run` bails
    /// when the tap closes — expected, since the buffered records are drained
    /// first.
    fn drain(records: Vec<TriggerRecord>, source: TimeSource) -> Vec<(String, Anchor)> {
        let (tx, rx) = unbounded();
        for record in records {
            tx.send(record).expect("the interface holds the receiver");
        }
        // Drop the sender so `run` drains the buffered records and returns.
        drop(tx);

        let fired: Arc<Mutex<Vec<(String, Anchor)>>> = Arc::new(Mutex::new(Vec::new()));
        let sink = fired.clone();
        let iface = McapInterface::new(crate::TRIGGER_TOPIC, rx, source);
        let _ = iface.run(move |t, anchor| sink.lock().unwrap().push((t.name, anchor)));

        let fired = fired.lock().unwrap();
        fired.clone()
    }

    /// The names the interface fired, dropping the anchors.
    fn fired_names(records: Vec<TriggerRecord>) -> Vec<String> {
        drain(records, TimeSource::Log)
            .into_iter()
            .map(|(name, _)| name)
            .collect()
    }

    /// The McapInterface decodes each tapped record by its `message_encoding` and
    /// fires the callback once per decoded trigger, in order.
    #[test]
    fn mcap_interface_decodes_and_fires_each_trigger() {
        assert_eq!(
            fired_names(vec![
                raw("json", json_trigger_bytes("first"), 100),
                raw("json", json_trigger_bytes("second"), 200),
            ]),
            vec!["first", "second"]
        );
    }

    /// The failure path: an undecodable record (unknown encoding or malformed
    /// body) is logged and skipped, never fatal — the good triggers around it
    /// still fire.
    #[test]
    fn mcap_interface_skips_undecodable_triggers_and_keeps_going() {
        assert_eq!(
            fired_names(vec![
                // Unknown encoding — no decoder.
                raw("ros1", b"whatever".to_vec(), 1),
                // Known encoding, malformed body — decode fails. (In a build
                // without the `ros` feature there is no CDR decoder at all, and
                // the record is skipped for that reason instead.)
                raw("cdr", b"not-a-valid-cdr-trigger".to_vec(), 2),
                // Malformed JSON — decode fails.
                raw("json", b"{ this is not json".to_vec(), 3),
                // One good record survives the bad ones on either side.
                raw("json", json_trigger_bytes("survivor"), 4),
            ]),
            vec!["survivor"],
            "only the one decodable trigger fires; the three bad ones are skipped"
        );
    }

    /// CDR is the encoding rosbag2 writes, so the feature build decodes a real
    /// serialized `momentedge_msgs/Trigger` off the tap — the same rmw
    /// typesupport path a recorded ROS trigger takes.
    #[cfg(feature = "ros")]
    #[test]
    fn mcap_interface_decodes_a_cdr_trigger_under_the_ros_feature() {
        use r2r::WrappedTypesupport;

        let body = r2r::momentedge_msgs::msg::Trigger {
            name: "recorded".to_string(),
            description: "e2e".to_string(),
            trigger_time: r2r::builtin_interfaces::msg::Time { sec: 1, nanosec: 2 },
            preroll: 10,
            postroll: 20,
        }
        .to_serialized_bytes()
        .expect("serialize");

        assert_eq!(fired_names(vec![raw("cdr", body, 100)]), vec!["recorded"]);
    }

    /// The anchors one `json` record (log_time 100, publish_time 900) resolves to
    /// on `source`. Every MCAP anchor is resolved from the record, so none is
    /// `from_trigger_time`.
    fn anchors_for(source: TimeSource) -> Vec<Anchor> {
        drain(
            vec![raw_stamped("json", json_trigger_bytes("t"), 100, 900)],
            source,
        )
        .into_iter()
        .map(|(_, anchor)| anchor)
        .collect()
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
}
