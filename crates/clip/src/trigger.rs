//! The neutral trigger and completion contract every trigger source targets.
//!
//! This is the dependency-light boundary between the two halves of cutting a
//! clip, and it exists so neither half has to know the other. Above it sits
//! whatever knows the outside world — a ROS subscription, records lifted out of
//! a recording, a JSON list — and it produces a [`Trigger`] and consumes a
//! [`Completion`] through [`Announce`]. Below it sits the cut, which acts on a
//! [`Trigger`] and reports a [`Completion`] knowing nothing of ROS or any wire
//! encoding. Keeping these types free of `r2r` and `mcap` is what lets either
//! side change without dragging the other along. [`Trigger`]
//! and [`Stamp`] do derive `serde::Deserialize` — their fields mirror the
//! `momentedge_msgs/Trigger` JSON shape, so the JSON decoder reads a payload
//! straight into them with no parallel wire type; the CDR path maps r2r's own
//! generated `Trigger` onto these through a `From` impl.
//!
//! That `r2r`-freedom is deliberate and structural, not incidental: the domain
//! types below never name `r2r` in a default build. Two conversions bridge them
//! to the ROS messages, both behind the `ros` feature and both in this crate
//! because the orphan rule leaves nowhere else — a downstream crate may not
//! implement `From` between two types it does not own. The inbound one, filling
//! a [`Trigger`] from a `momentedge_msgs/Trigger`, sits with the decoder that
//! shares it ([`crate::decode`]); the outbound one, rendering a [`Completion`]
//! as a `momentedge_msgs/Recorded`, sits at the bottom of this module. A
//! consumer that never enables the feature links no ROS at all and still speaks
//! the whole contract.

use serde::Deserialize;

/// A `builtin_interfaces/Time` flattened to its two fields, free of `r2r`. It is
/// the publisher's own publish-domain timestamp (`trigger_time`), one possible
/// source of a window's anchor: read only by the ros interface under the
/// `publish` time source. Every other interface × time-source cell resolves the
/// anchor from a transport stamp and rejects a non-zero `trigger_time` (see the
/// recorder's interface layer).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
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

    /// The stamp naming `ns` nanoseconds since the epoch — what a trigger source
    /// that states its instant as a plain nanosecond count (a command line, a
    /// JSON field) fills the message's `builtin_interfaces/Time` from.
    ///
    /// Exact inverse of [`Self::ns`] for every instant the message can hold.
    /// `sec` is an `i32`, so an instant past its range (some time in 2038)
    /// saturates at the largest stamp there is rather than wrapping into the
    /// past — a window is anchored on the nanosecond count itself, never on the
    /// round trip through here.
    pub fn from_ns(ns: u64) -> Self {
        let nanosec = (ns % 1_000_000_000) as u32;
        match i32::try_from(ns / 1_000_000_000) {
            Ok(sec) => Stamp { sec, nanosec },
            Err(_) => Stamp {
                sec: i32::MAX,
                nanosec: 999_999_999,
            },
        }
    }
}

/// Nanoseconds since the Unix epoch on the system clock — the one time base the
/// recorder reads the clock through. The same scale as a message's `log_time`
/// and [`Stamp::ns`], so the postroll wait (`handler`) and the retention floor
/// (`tail`) compare against recorded times without conversion. Saturates at
/// `u64::MAX` and reports 0 for a pre-epoch clock, so neither can wrap into a
/// small value that would silently misplace a window.
pub fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos().min(u64::MAX as u128) as u64)
        .unwrap_or(0)
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
/// the message payload bytes, and the record's two stamps (`log_time`,
/// `publish_time`). The name marks it as the MCAP record type — the raw input
/// the MCAP interface turns into a [`Trigger`] by dispatching on
/// `message_encoding` ([`crate::decode::decode_trigger`]); the tail emits them
/// without decoding anything but the framing. The MCAP interface anchors the
/// window on one of the two stamps per `--time-source`.
#[derive(Clone, Debug)]
pub struct TriggerRecord {
    pub message_encoding: String,
    pub body: Vec<u8>,
    pub log_time: u64,
    pub publish_time: u64,
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

/// The output half of the recorder's interface: announce a finished clip. The
/// ROS interface publishes a `Recorded`; the MCAP interface is a no-op (the
/// file move is the announcement). `Clone + Send` so each trigger handler
/// thread carries its own announcer moved in — not `Sync`, since an announcer
/// is never shared across threads by reference (the r2r `Publisher` behind the
/// recorder's ROS announcer is `Send` but not `Sync`).
pub trait Announce: Clone + Send + 'static {
    fn announce(&self, completion: &Completion);
}

/// A finished clip's [`Completion`] maps field-for-field onto the r2r-generated
/// `momentedge_msgs/Recorded` a ROS interface publishes (its [`Stamp`] onto the
/// nested `builtin_interfaces/Time`). By reference — an announcer keeps the
/// `Completion` to log from after the publish.
#[cfg(feature = "ros")]
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

    /// A nanosecond count becomes the stamp that names the same instant, and
    /// comes back unchanged — the property a caller anchoring a window on one
    /// and reporting the other depends on.
    #[test]
    fn from_ns_round_trips_every_representable_instant() {
        for ns in [
            0,
            1,
            999_999_999,
            1_000_000_000,
            1_738_000_000_123_456_789,
            i32::MAX as u64 * 1_000_000_000 + 999_999_999,
        ] {
            assert_eq!(Stamp::from_ns(ns).ns(), ns, "{ns} must round-trip");
        }
    }

    /// An instant past the `i32` seconds field saturates at the largest stamp
    /// there is; it must never wrap into a negative second, which `ns()` would
    /// then clamp to the epoch.
    #[test]
    fn from_ns_saturates_past_the_seconds_field() {
        let stamp = Stamp::from_ns(u64::MAX);
        assert_eq!(
            stamp,
            Stamp {
                sec: i32::MAX,
                nanosec: 999_999_999
            }
        );
        assert!(stamp.ns() > 0, "saturation must not land back at the epoch");
    }
}
