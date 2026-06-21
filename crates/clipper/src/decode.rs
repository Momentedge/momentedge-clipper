//! Decoding a [`TriggerRecord`](crate::trigger::TriggerRecord)'s payload into a
//! domain [`Trigger`] by dispatching on its MCAP `message_encoding`.
//!
//! [`DecoderFactory::for_encoding`] maps an encoding string to a
//! [`TriggerDecoder`], so a writer interleaving a trigger into an MCAP is never
//! forced to serialize as CDR: `json` is a first-class peer of `cdr`. Each
//! decoder reads only the payload bytes the tail captured — no schema, no node,
//! no ROS runtime — and produces a domain [`Trigger`].
//!
//! The two covered encodings are decodable with dependencies already in the
//! closure: `cdr` through `r2r`'s rmw deserialization (the rmw library only,
//! never a node), `json` through `serde_json`. `cbor`/`protobuf`/`flatbuffer`
//! and any unknown encoding return an error the caller logs and skips.

use anyhow::{Context, Result, bail};

use crate::trigger::{Stamp, Trigger};

/// Decodes one trigger payload of a single wire encoding into a [`Trigger`].
pub trait TriggerDecoder {
    /// Decode the message payload `body` (the bytes after an MCAP Message
    /// record's fixed fields) into a [`Trigger`], or an error if the bytes do
    /// not parse as a `momentedge_msgs/Trigger` in this encoding.
    fn decode(&self, body: &[u8]) -> Result<Trigger>;
}

/// Produces a [`TriggerDecoder`] for an MCAP channel's `message_encoding`.
pub struct DecoderFactory;

impl DecoderFactory {
    /// The decoder for `encoding`, or an error for an encoding clipper cannot
    /// decode (`cbor`, schema-bound `protobuf`/`flatbuffer`, or anything
    /// unknown).
    ///
    /// The sole caller is the MCAP interface ([`crate::interface::McapInterface`]):
    /// the ROS interface reads typed triggers off its subscription and never
    /// decodes the file, and its tail runs with the trigger tap unwired, so this
    /// path — and this error — is reachable only when clipper is reading triggers
    /// out of the tailed recording. There the caller logs the error and skips
    /// that one trigger rather than failing: an undecodable message on clipper's
    /// own trigger topic must not stop the recorder.
    pub fn for_encoding(encoding: &str) -> Result<Box<dyn TriggerDecoder>> {
        match encoding {
            "cdr" => Ok(Box::new(CdrDecoder)),
            "json" => Ok(Box::new(JsonDecoder)),
            other => bail!("no trigger decoder for message_encoding {other:?}"),
        }
    }
}

/// `cdr`: the ROS2 default. Deserializes through `r2r`'s rmw typesupport, which
/// needs only the linked rmw library — no `Context`, `Node`, or executor — so
/// it works in the fully ROS-free MCAP interface. The payload is the rmw
/// serialized form rosbag2 writes (CDR with its encapsulation header), which is
/// exactly what `from_serialized_bytes` expects.
struct CdrDecoder;

impl TriggerDecoder for CdrDecoder {
    fn decode(&self, body: &[u8]) -> Result<Trigger> {
        use r2r::WrappedTypesupport;
        // `from_serialized_bytes` yields r2r's own generated Trigger type, not
        // the neutral domain Trigger; the `From` impl below maps it across. (The
        // json path is a one-liner because the domain Trigger derives Deserialize
        // and serde builds it directly.)
        Ok(
            r2r::momentedge_msgs::msg::Trigger::from_serialized_bytes(body)
                .context("deserializing a CDR momentedge_msgs/Trigger")?
                .into(),
        )
    }
}

/// `json`: a first-class peer of `cdr`, for writers that emit JSON. Parsed with
/// `serde_json` straight into the domain [`Trigger`], which derives
/// `Deserialize`; its docs give the accepted shape (`description` optional,
/// unknown fields ignored, the rest required, the nested `trigger_time` a
/// `{sec, nanosec}` object).
struct JsonDecoder;

impl TriggerDecoder for JsonDecoder {
    fn decode(&self, body: &[u8]) -> Result<Trigger> {
        serde_json::from_slice(body).context("parsing a JSON momentedge_msgs/Trigger")
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
                DecoderFactory::for_encoding(enc).is_err(),
                "{enc:?} must not resolve a decoder"
            );
        }
    }

    #[test]
    fn json_decodes_the_msg_shape() {
        let body = br#"{"name":"evt","description":"hi","trigger_time":{"sec":7,"nanosec":250},"preroll":1000,"postroll":2000}"#;
        let got = DecoderFactory::for_encoding("json")
            .unwrap()
            .decode(body)
            .unwrap();
        assert_eq!(got, expected());
    }

    #[test]
    fn json_description_is_optional_and_unknown_fields_ignored() {
        // description omitted (defaults empty), an unknown field present (ignored).
        let body = br#"{"name":"evt","trigger_time":{"sec":0,"nanosec":0},"preroll":5,"postroll":6,"extra":true}"#;
        let got = DecoderFactory::for_encoding("json")
            .unwrap()
            .decode(body)
            .unwrap();
        assert_eq!(got.name, "evt");
        assert_eq!(got.description, "");
        assert_eq!((got.preroll, got.postroll), (5, 6));
    }

    #[test]
    fn json_missing_required_field_is_an_error() {
        // preroll missing — a required field, so the decode fails (logged skip).
        let body = br#"{"name":"evt","trigger_time":{"sec":0,"nanosec":0},"postroll":6}"#;
        assert!(
            DecoderFactory::for_encoding("json")
                .unwrap()
                .decode(body)
                .is_err()
        );
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
        let got = DecoderFactory::for_encoding("cdr")
            .unwrap()
            .decode(&bytes)
            .unwrap();
        assert_eq!(got, expected());
    }
}
