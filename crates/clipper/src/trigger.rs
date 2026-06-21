//! The neutral trigger/completion contract shared by the interfaces and the
//! handler.
//!
//! This module is the dependency-light boundary both sides of the recorder
//! depend on while staying independent of each other: the [`interface`] layer
//! (which knows ROS vs MCAP and the wire encodings) produces a [`Trigger`] and
//! consumes the handler's [`Completion`] through [`Announce`]; the [`handler`]
//! layer cuts clips from a [`Trigger`] and announces a [`Completion`], knowing
//! nothing of ROS or any encoding. Keeping these types free of `r2r` and `mcap`
//! is what lets either side change without dragging the other along. [`Trigger`]
//! and [`Stamp`] do derive `serde::Deserialize` — their fields mirror the
//! `momentedge_msgs/Trigger` JSON shape, so the JSON decoder reads a payload
//! straight into them with no parallel wire type; the CDR path maps r2r's own
//! generated `Trigger` onto these through a `From` impl.
//!
//! [`interface`]: crate::interface
//! [`handler`]: crate::handler

use serde::Deserialize;

/// A `builtin_interfaces/Time` flattened to its two fields, free of `r2r`. The
/// trigger's window is cut around this stamp (`clipper-535`); a follow-up
/// (`clipper-qo3`) migrates the anchor to a transport timestamp.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize)]
pub struct Stamp {
    pub sec: i32,
    pub nanosec: u32,
}

impl Stamp {
    /// Flatten to nanoseconds since the epoch on the system clock (no
    /// `use_sim_time`); negative seconds clamp to 0. The same arithmetic the
    /// ROS path applies to a `builtin_interfaces/Time`, so a CDR-decoded trigger
    /// and a live ROS trigger anchor their windows identically.
    pub fn ns(&self) -> u64 {
        (self.sec.max(0) as u64) * 1_000_000_000 + self.nanosec as u64
    }
}

/// A decoded trigger: the clip-window request the handler acts on, independent
/// of how it arrived (a live ROS subscription or a record lifted out of the
/// tailed MCAP). Mirrors the fields of `momentedge_msgs/Trigger`.
///
/// `Deserialize` is the `json` wire shape: `description` is optional (defaulting
/// empty), the rest required, unknown fields ignored. The CDR path populates the
/// same fields from r2r's generated type (a `From` impl) without serde.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize)]
pub struct Trigger {
    pub name: String,
    #[serde(default)]
    pub description: String,
    pub trigger_time: Stamp,
    pub preroll: u64,
    pub postroll: u64,
}

/// One trigger-topic MCAP message record, lifted out of the tailed recording
/// undecoded: the channel's wire encoding (`message_encoding` — `cdr`/`json`/…),
/// the message payload bytes, and the record's `log_time`. The name marks it as
/// the MCAP record type — the raw input the MCAP interface turns into a
/// [`Trigger`] by dispatching on `message_encoding`
/// ([`crate::decode::DecoderFactory`]); the tail emits them without decoding
/// anything but the framing.
#[derive(Clone, Debug)]
pub struct TriggerRecord {
    pub message_encoding: String,
    pub body: Vec<u8>,
    pub log_time: u64,
}

/// What the handler emits once a clip is durable: the trigger echo plus the
/// staged segment paths. The ROS interface turns this into a
/// `momentedge_msgs/Recorded`; the MCAP interface treats the clip's atomic move
/// into `out_dir` as the signal and does nothing further.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Completion {
    pub name: String,
    pub filenames: Vec<String>,
    pub description: String,
    pub trigger_time: Stamp,
    pub preroll: u64,
}

/// The output half of an [`crate::interface::Interface`]: announce a finished
/// clip. The ROS interface publishes a `Recorded`; the MCAP interface is a
/// no-op (the file move is the announcement). `Clone + Send` so each trigger
/// handler thread carries its own announcer moved in — not `Sync`, since an
/// announcer is never shared across threads by reference (the r2r `Publisher`
/// behind [`crate::interface::RosAnnouncer`] is `Send` but not `Sync`).
pub trait Announce: Clone + Send + 'static {
    fn announce(&self, completion: &Completion);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamp_ns_flattens_and_clamps() {
        assert_eq!(
            Stamp {
                sec: 2,
                nanosec: 500
            }
            .ns(),
            2_000_000_500
        );
        // Negative seconds clamp to 0, matching the ROS path's time_to_ns.
        assert_eq!(
            Stamp {
                sec: -5,
                nanosec: 250
            }
            .ns(),
            250
        );
    }
}
