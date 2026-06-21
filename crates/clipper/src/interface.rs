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
//!   MCAP, decoding each by `message_encoding` ([`DecoderFactory`]). It touches
//!   no ROS node, executor, or subscription — the recorder runs ROS-free in this
//!   mode. Completion is implicit: the clip's atomic move into `out_dir` is the
//!   signal, so its [`Announce`]r is a no-op.
//!
//! The trait is generic (not `dyn`), so the driver dispatches statically over
//! whichever interface is active.

use std::pin::Pin;
use std::time::Duration;

use anyhow::Context;
use crossbeam_channel::{Receiver, select};
use futures::executor::block_on;
use futures::stream::{Stream, StreamExt};
use log::{error, info, warn};
use r2r::{Publisher, QosProfile};

use crate::decode::DecoderFactory;
use crate::trigger::{Announce, Completion, Trigger, TriggerRecord};

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
    /// decoded [`Trigger`] as it arrives. Returns only on an end or fault that
    /// the driver treats as a reason to exit the process. `fire` is called from a
    /// single thread, so it need not be `Sync`.
    fn run<F>(self, fire: F) -> anyhow::Result<()>
    where
        F: Fn(Trigger) + Send + 'static;
}

// ── ROS interface ───────────────────────────────────────────────────────────

/// The ROS interface: a typed subscription to the trigger topic feeding the
/// driver, and a `Recorded` publisher behind [`RosAnnouncer`].
pub(crate) struct RosInterface {
    node: r2r::Node,
    sub: Pin<Box<dyn Stream<Item = r2r::momentedge_msgs::msg::Trigger> + Send>>,
    announcer: RosAnnouncer,
}

impl RosInterface {
    /// Create the node, the trigger subscription, and the `Recorded` publisher.
    pub(crate) fn new(trigger_topic: &str, recorded_topic: &str) -> anyhow::Result<Self> {
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
            announcer: RosAnnouncer { recorded_pub },
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
        F: Fn(Trigger) + Send + 'static,
    {
        let RosInterface {
            mut node, mut sub, ..
        } = self;

        let spin = crate::spawn_supervised("node-spin", move || {
            loop {
                node.spin_once(Duration::from_millis(10));
            }
        });
        let drain = crate::spawn_supervised("trigger-drain", move || -> anyhow::Result<()> {
            while let Some(t) = block_on(sub.next()) {
                // `t.into()` is the shared r2r-Trigger -> domain-Trigger conversion
                // (the `From` impl in `crate::decode`).
                fire(t.into());
            }
            anyhow::bail!("the trigger subscription stream ended")
        });

        let (spin_rx, spin_handle) = spin;
        let (drain_rx, drain_handle) = drain;
        select! {
            recv(spin_rx) -> res => match res {
                Ok(()) => anyhow::bail!("node spin thread exited unexpectedly"),
                Err(_) => Err(crate::harvest_panic(spin_handle)
                    .context("node spin thread exited unexpectedly")),
            },
            recv(drain_rx) -> res => match res {
                Ok(r) => r.context("trigger drain ended"),
                Err(_) => Err(crate::harvest_panic(drain_handle)
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
/// clip. `Clone` (the r2r `Publisher` is) so every handler thread holds its own.
#[derive(Clone)]
pub(crate) struct RosAnnouncer {
    recorded_pub: Publisher<r2r::momentedge_msgs::msg::Recorded>,
}

impl Announce for RosAnnouncer {
    fn announce(&self, completion: &Completion) {
        let recorded = r2r::momentedge_msgs::msg::Recorded::from(completion);
        match self.recorded_pub.publish(&recorded) {
            Ok(()) => info!(
                "emitted {} name={:?} filenames={:?}",
                crate::RECORDED_TOPIC,
                completion.name,
                completion.filenames,
            ),
            Err(e) => error!(
                "publishing {} for name={:?} failed: {e}",
                crate::RECORDED_TOPIC,
                completion.name,
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
}

impl McapInterface {
    /// Drive from the tail's decode-free trigger tap (`Tailer::with_trigger_tap`).
    pub(crate) fn new(triggers: Receiver<TriggerRecord>) -> Self {
        McapInterface { triggers }
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
        F: Fn(Trigger) + Send + 'static,
    {
        for raw in self.triggers.iter() {
            match DecoderFactory::for_encoding(&raw.message_encoding)
                .and_then(|decoder| decoder.decode(&raw.body))
            {
                Ok(trigger) => {
                    info!(
                        "MCAP trigger name={:?} encoding={} log_time={}",
                        trigger.name, raw.message_encoding, raw.log_time,
                    );
                    fire(trigger);
                }
                Err(e) => warn!(
                    "skipping an MCAP trigger on {} (encoding={}): {e:#}",
                    crate::TRIGGER_TOPIC,
                    raw.message_encoding,
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
