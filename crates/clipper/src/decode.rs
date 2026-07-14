//! Decoding a [`TriggerRecord`](crate::trigger::TriggerRecord)'s payload into a
//! domain [`Trigger`] by dispatching on its MCAP `message_encoding`.
//!
//! [`decode_trigger`] maps an encoding string plus the payload bytes to a
//! [`Trigger`], so a writer interleaving a trigger into an MCAP is never forced
//! to serialize as CDR: `json` is a first-class peer of `cdr`. Only the payload
//! bytes the tail captured are read — no schema, no node, no ROS runtime.
//!
//! The two covered encodings are decodable with dependencies already in the
//! closure: `cdr` through `r2r`'s rmw deserialization (the rmw library only,
//! never a node), `json` through `serde_json`. `cbor`/`protobuf`/`flatbuffer`
//! and any unknown encoding return an error the caller logs and skips.

use anyhow::{Context, Result, bail};

use crate::trigger::{Stamp, Trigger};

/// Decode one trigger payload — `body`, the bytes after an MCAP Message
/// record's fixed fields — according to its channel's `encoding`.
///
/// - **`cdr`**, the ROS2 default, goes through `r2r`'s rmw typesupport, which
///   needs only the linked rmw library — no `Context`, `Node`, or executor — so
///   it works in the fully ROS-free MCAP interface. The payload is the rmw
///   serialized form rosbag2 writes (CDR with its encapsulation header), which
///   is exactly what `from_serialized_bytes` expects. It yields r2r's generated
///   type, which the [`From`] impl below maps onto the neutral domain
///   [`Trigger`].
/// - **`json`** is parsed by `serde_json` straight into the domain [`Trigger`],
///   which derives `Deserialize`; its docs give the accepted shape
///   (`description` optional, unknown fields ignored, the rest required, the
///   nested `trigger_time` a `{sec, nanosec}` object).
/// - Anything else — `cbor`, schema-bound `protobuf`/`flatbuffer`, or an
///   unknown encoding — is an error, as is a body that does not parse.
///
/// The sole caller is the MCAP interface ([`crate::interface::McapInterface`]):
/// the ROS interface reads typed triggers off its subscription and never decodes
/// the file, and its tail runs with the trigger tap unwired, so this path — and
/// this error — is reachable only when clipper is reading triggers out of the
/// tailed recording. There the caller logs the error and skips that one trigger
/// rather than failing: an undecodable message on clipper's own trigger topic
/// must not stop the recorder.
pub fn decode_trigger(encoding: &str, body: &[u8]) -> Result<Trigger> {
    match encoding {
        "cdr" => {
            use r2r::WrappedTypesupport;
            Ok(
                r2r::momentedge_msgs::msg::Trigger::from_serialized_bytes(body)
                    .context("deserializing a CDR momentedge_msgs/Trigger")?
                    .into(),
            )
        }
        "json" => serde_json::from_slice(body).context("parsing a JSON momentedge_msgs/Trigger"),
        other => bail!("no trigger decoder for message_encoding {other:?}"),
    }
}

/// The r2r-generated `momentedge_msgs/Trigger` maps field-for-field onto the
/// neutral domain [`Trigger`] (its nested `builtin_interfaces/Time` onto
/// [`Stamp`]). The CDR decoder and the live ROS interface share this one
/// conversion, so the domain type itself stays free of `r2r`.
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

    /// CDR round-trip through r2r's rmw typesupport, node-free: serialize a real
    /// `momentedge_msgs/Trigger` with `to_serialized_bytes` (the rmw form
    /// rosbag2 writes) and decode it back. Exercises the path the MCAP interface
    /// takes for a `cdr` channel. Runs inside the dev shell, where the rmw
    /// library and the momentedge_msgs typesupport are on the load path.
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
