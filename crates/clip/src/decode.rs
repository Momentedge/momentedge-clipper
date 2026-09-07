//! Decoding a [`TriggerRecord`](crate::trigger::TriggerRecord)'s payload into a
//! domain [`Trigger`] by dispatching on its MCAP `message_encoding`.
//!
//! [`decode_trigger`] maps an encoding string plus the payload bytes to a
//! [`Trigger`], so a writer interleaving a trigger into an MCAP is never forced
//! to serialize as CDR: `json` is a first-class peer of `cdr`. Only the payload
//! bytes the tail captured are read — no schema, no node, no ROS runtime.
//!
//! The two covered encodings split along the crate's one ROS seam. `json` is
//! always decodable, through `serde_json`. `cdr` sits behind the `ros` feature,
//! decoded through `r2r`'s rmw typesupport — the linked rmw library only, never
//! a `Context`, `Node`, or executor. That feature is off by default, so a
//! ROS-free consumer of a recording never pulls `r2r` into its build; there a
//! `cdr` payload is an error naming the absent feature rather than a decode
//! failure. `cbor`/`protobuf`/`flatbuffer` and any unknown encoding return an
//! error the caller logs and skips.

use anyhow::{Context, Result, bail};

// `Stamp` is named only by the r2r conversion below, which the `ros` feature
// gates, so a default build must not import it; the tests name it themselves.
#[cfg(feature = "ros")]
use crate::trigger::Stamp;
use crate::trigger::Trigger;

/// Decode one trigger payload — `body`, the bytes after an MCAP Message
/// record's fixed fields — according to its channel's `encoding`.
///
/// - **`cdr`**, the ROS2 default, goes through `r2r`'s rmw typesupport, which
///   needs only the linked rmw library — no `Context`, `Node`, or executor — so
///   it works in the fully ROS-free MCAP interface. The payload is the rmw
///   serialized form rosbag2 writes (CDR with its encapsulation header), which
///   is exactly what `from_serialized_bytes` expects. It yields r2r's generated
///   type, which the [`From`] impl below maps onto the neutral domain
///   [`Trigger`]. That arm is compiled in only under the `ros` feature; a build
///   without it answers a `cdr` payload with an error naming the missing
///   feature, so no consumer silently loses triggers it cannot read.
/// - **`json`** is parsed by `serde_json` straight into the domain [`Trigger`],
///   which derives `Deserialize`; its docs give the accepted shape
///   (`description` optional, unknown fields ignored, the rest required, the
///   nested `trigger_time` a `{sec, nanosec}` object).
/// - Anything else — `cbor`, schema-bound `protobuf`/`flatbuffer`, or an
///   unknown encoding — is an error, as is a body that does not parse.
///
/// The sole caller is the recorder's MCAP interface (`McapInterface`, in the
/// `clipper` crate): the ROS interface reads typed triggers off its subscription
/// and never decodes the file, and its tail runs with the trigger tap unwired,
/// so this path — and this error — is reachable only when clipper is reading
/// triggers out of the tailed recording. There the caller logs the error and
/// skips that one trigger rather than failing: an undecodable message on
/// clipper's own trigger topic must not stop the recorder.
pub fn decode_trigger(encoding: &str, body: &[u8]) -> Result<Trigger> {
    match encoding {
        #[cfg(feature = "ros")]
        "cdr" => {
            use r2r::WrappedTypesupport;
            Ok(
                r2r::momentedge_msgs::msg::Trigger::from_serialized_bytes(body)
                    .context("deserializing a CDR momentedge_msgs/Trigger")?
                    .into(),
            )
        }
        #[cfg(not(feature = "ros"))]
        "cdr" => bail!(
            "no CDR trigger decoder in this build: `cdr` needs r2r's rmw typesupport, which only \
             the `ros` feature links, and this build was made without it. The payload was never \
             read, so this says nothing about the recording — encode the trigger as `json`, which \
             needs no decoder here, or rebuild this consumer with the `ros` feature"
        ),
        "json" => serde_json::from_slice(body).context("parsing a JSON momentedge_msgs/Trigger"),
        other => bail!("no trigger decoder for message_encoding {other:?}"),
    }
}

/// The r2r-generated `momentedge_msgs/Trigger` maps field-for-field onto the
/// neutral domain [`Trigger`] (its nested `builtin_interfaces/Time` onto
/// [`Stamp`]). The CDR decoder and the live ROS interface share this one
/// conversion, so the domain type itself stays free of `r2r`.
///
/// It sits in this crate rather than in the recorder that also uses it because
/// the orphan rule leaves nowhere else: to a downstream crate both r2r's
/// generated type and the domain [`Trigger`] are foreign, and a crate may not
/// implement `From` between two foreign types. The conversion has to live with
/// the domain type — which is what the `ros` feature buys, at the cost of `r2r`
/// only for the builds that ask for it.
#[cfg(feature = "ros")]
impl From<r2r::momentedge_msgs::msg::Trigger> for Trigger {
    fn from(t: r2r::momentedge_msgs::msg::Trigger) -> Self {
        Trigger {
            name: t.name,
            description: t.description,
            trigger_time: Stamp {
                sec: t.trigger_time.sec,
                nanosec: t.trigger_time.nanosec,
            },
            preroll: t.preroll,
            postroll: t.postroll,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trigger::Stamp;

    /// The canonical decoded trigger the encoding-specific test bytes all map to.
    fn expected() -> Trigger {
        Trigger {
            name: "evt".to_string(),
            description: "hi".to_string(),
            trigger_time: Stamp {
                sec: 7,
                nanosec: 250,
            },
            preroll: 1_000,
            postroll: 2_000,
        }
    }

    #[test]
    fn unknown_encoding_has_no_decoder() {
        for enc in ["ros1", "cbor", "protobuf", "flatbuffer", "", "CDR"] {
            assert!(
                decode_trigger(enc, b"").is_err(),
                "{enc:?} must not resolve a decoder"
            );
        }
    }

    #[test]
    fn json_decodes_the_msg_shape() {
        let body = br#"{"name":"evt","description":"hi","trigger_time":{"sec":7,"nanosec":250},"preroll":1000,"postroll":2000}"#;
        let got = decode_trigger("json", body).unwrap();
        assert_eq!(got, expected());
    }

    #[test]
    fn json_description_is_optional_and_unknown_fields_ignored() {
        // description omitted (defaults empty), an unknown field present (ignored).
        let body = br#"{"name":"evt","trigger_time":{"sec":0,"nanosec":0},"preroll":5,"postroll":6,"extra":true}"#;
        let got = decode_trigger("json", body).unwrap();
        assert_eq!(got.name, "evt");
        assert_eq!(got.description, "");
        assert_eq!((got.preroll, got.postroll), (5, 6));
    }

    #[test]
    fn json_missing_required_field_is_an_error() {
        // preroll missing — a required field, so the decode fails (logged skip).
        let body = br#"{"name":"evt","trigger_time":{"sec":0,"nanosec":0},"postroll":6}"#;
        assert!(decode_trigger("json", body).is_err());
    }

    /// The ROS-free half of the feature split, and the property it exists for: a
    /// `cdr` payload has no decoder here, and the error says why — this build
    /// lacks the `ros` feature, and `json` needs no decoder — so a reader of the
    /// log knows whether to re-encode the trigger or rebuild the consumer,
    /// rather than suspecting the recording.
    #[cfg(not(feature = "ros"))]
    #[test]
    fn cdr_without_the_ros_feature_names_the_missing_feature() {
        let err = decode_trigger("cdr", b"").expect_err("a build without `ros` has no CDR decoder");
        let msg = err.to_string();
        assert!(
            msg.contains("`ros` feature"),
            "the error must name the missing feature: {msg}"
        );
        assert!(
            msg.contains("`json`"),
            "the error must name the encoding that needs no decoder: {msg}"
        );
    }

    /// CDR round-trip through r2r's rmw typesupport, node-free: serialize a real
    /// `momentedge_msgs/Trigger` with `to_serialized_bytes` (the rmw form
    /// rosbag2 writes) and decode it back. Exercises the path the MCAP interface
    /// takes for a `cdr` channel. Runs inside the dev shell, where the rmw
    /// library and the momentedge_msgs typesupport are on the load path.
    #[cfg(feature = "ros")]
    #[test]
    fn cdr_round_trips_through_r2r() {
        use r2r::WrappedTypesupport;
        let msg = r2r::momentedge_msgs::msg::Trigger {
            name: "evt".to_string(),
            description: "hi".to_string(),
            trigger_time: r2r::builtin_interfaces::msg::Time {
                sec: 7,
                nanosec: 250,
            },
            preroll: 1_000,
            postroll: 2_000,
        };
        let bytes = msg.to_serialized_bytes().expect("serialize");
        let got = decode_trigger("cdr", &bytes).unwrap();
        assert_eq!(got, expected());
    }
}
