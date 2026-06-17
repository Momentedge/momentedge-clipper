//! Capture-time decoding: the ROS2 message **header stamp** carried inside the
//! CDR message payload, and the schema test that gates when to trust it.
//!
//! The clip window is a *capture-time* window — `[trigger_time - preroll,
//! trigger_time + postroll]` with `trigger_time` in the capture-time clock
//! domain. A recorded message's place on that timeline is its header stamp
//! when its channel carries one, and the MCAP `log_time` (receive time)
//! otherwise. [`effective_time`] folds the two into one merged timeline that
//! the tail (coverage, extent bounds) and the extraction (message selection)
//! both clip against; `log_time` itself is always copied through to the output
//! unchanged.
//!
//! Two layers decide whether a message has a capture stamp:
//!
//! * **Schema gate** ([`schema_is_stamped`]) — a channel carries a stamp iff
//!   its message's first field is a `std_msgs/Header` or a bare
//!   `builtin_interfaces/Time`, which in CDR puts a `Time` at a fixed offset.
//!   This is read from the channel's registry schema, so a headerless type
//!   (e.g. `tf2_msgs/TFMessage`, sequence-first) is never mis-decoded — its
//!   messages fall back to `log_time`. A recording whose channels are *all*
//!   unstamped therefore clips exactly as it did on `log_time`.
//! * **Value sanity + zero guard** ([`cdr_header_stamp_ns`]) — even a
//!   schema-gated stamp is rejected when implausible (`sec < 0`,
//!   `nanosec >= 1e9`) or the unset `(0,0)` sentinel many publishers ship on a
//!   header they never filled; such a message also falls back to `log_time`.
//!
//! The merged timeline is sound because all channels are assumed to share
//! roughly one transport delay (capture → receive), so a single window cuts
//! every channel at approximately one instant — the same near-monotonic
//! ordering assumption [`crate::tail::Coverage`] already rests on, moved from
//! `log_time` to capture time.

use crate::tail::SchemaDef;

/// The fixed CDR offsets of a leading `builtin_interfaces/Time`: 4 bytes of
/// encapsulation header, then `sec` (`int32`) at 4 and `nanosec` (`uint32`) at
/// 8. Both are 4-byte aligned, so no padding intrudes. Also the number of
/// leading payload bytes the tail reads to decode a stamped message's time.
pub(crate) const STAMP_SPAN: usize = 12;

/// Decode the leading `builtin_interfaces/Time` of a CDR message `payload` as
/// nanoseconds since the epoch, or `None` when there is no usable stamp.
///
/// The encapsulation header's second byte selects byte order (low bit set →
/// little-endian, the ROS2 default on every supported target). `sec` sits at
/// offset 4, `nanosec` at offset 8. `None` is returned when the payload is too
/// short to hold a `Time`, the value is implausible (`sec < 0` or
/// `nanosec >= 1e9` — bytes that are not actually a leading `Time`), or the
/// stamp is the unset `(0,0)` sentinel a header-bearing publisher leaves when
/// it never set the stamp. Every `None` path is a fall-back to `log_time` at
/// the call site; this never fabricates a capture time.
pub(crate) fn cdr_header_stamp_ns(payload: &[u8]) -> Option<u64> {
    if payload.len() < STAMP_SPAN {
        return None;
    }
    let little_endian = payload[1] & 1 == 1;
    let word = |off: usize| -> [u8; 4] { payload[off..off + 4].try_into().unwrap() };
    let (sec, nanosec) = if little_endian {
        (i32::from_le_bytes(word(4)), u32::from_le_bytes(word(8)))
    } else {
        (i32::from_be_bytes(word(4)), u32::from_be_bytes(word(8)))
    };
    if sec < 0 || nanosec >= 1_000_000_000 {
        return None;
    }
    let ns = sec as u64 * 1_000_000_000 + nanosec as u64;
    // The (0,0) unset stamp is "no capture time", not "captured at the epoch".
    (ns != 0).then_some(ns)
}

/// A message's position on the clip timeline: its capture stamp when its
/// channel is `stamped` (per [`schema_is_stamped`]) and the stamp decodes to a
/// usable value, else the MCAP `log_time`. The single function the tail and
/// the extraction both window against, so they agree on every message.
pub(crate) fn effective_time(stamped: bool, payload: &[u8], log_time: u64) -> u64 {
    if stamped {
        cdr_header_stamp_ns(payload).unwrap_or(log_time)
    } else {
        log_time
    }
}

/// Whether a channel's messages begin with a `builtin_interfaces/Time` stamp —
/// the schema gate for capture-time decoding. True iff the schema is `ros2msg`
/// and its **top-level** message's first field is a `std_msgs/Header` or a
/// `builtin_interfaces/Time`; every other channel (schemaless, non-`ros2msg`,
/// or headerless like `tf2_msgs/TFMessage`) is treated as unstamped and its
/// messages fall back to `log_time`.
///
/// The `ros2msg` schema is the concatenated `.msg` text: the top-level message
/// is the section before the first `MSG:` / `====` separator. Within it,
/// comments and constants (a `NAME=VALUE` line) are skipped to reach the first
/// real field, whose type token is matched against the stamped leading types.
pub(crate) fn schema_is_stamped(schema: Option<&SchemaDef>) -> bool {
    schema.is_some_and(|s| ros2msg_is_stamped(&s.encoding, &s.data))
}

/// [`schema_is_stamped`] over a raw schema record's `encoding` and `data`, for
/// the tail's scan — where the stamp gate is built before the registry's
/// [`SchemaDef`]s are assembled.
pub(crate) fn ros2msg_is_stamped(encoding: &str, data: &[u8]) -> bool {
    if !encoding.eq_ignore_ascii_case("ros2msg") {
        return false;
    }
    let Ok(text) = std::str::from_utf8(data) else {
        return false;
    };
    first_field_type(text).is_some_and(is_stamp_type)
}

/// The type token of the first field of the top-level message in a `ros2msg`
/// schema, or `None` if it has no field before the first dependent-type
/// separator. Comments (`#…`) and constants (`TYPE NAME=VALUE`) are skipped;
/// field defaults (`TYPE name value`) are not mistaken for constants.
fn first_field_type(text: &str) -> Option<&str> {
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        // The top-level message ends at the first dependent-type block.
        if line.starts_with("MSG:") || line.starts_with("===") {
            return None;
        }
        let mut tokens = line.split_whitespace();
        let ty = tokens.next().unwrap_or("");
        let name = tokens.next().unwrap_or("");
        // A constant is `TYPE NAME=VALUE` or `TYPE NAME = VALUE`. A bounded
        // type carries `<=`/`>=` in the *type* token, not the name, so it is
        // not mistaken for one. A field default (`TYPE name value`) has no `=`.
        let is_constant = name.contains('=') || tokens.next() == Some("=");
        if is_constant {
            continue;
        }
        return Some(ty);
    }
    None
}

/// Whether a field type serializes with a leading `builtin_interfaces/Time` —
/// a `std_msgs/Header` (whose own first field is the stamp) or a `Time`
/// itself. Accepts the `pkg/Type` and `pkg/msg/Type` spellings.
fn is_stamp_type(ty: &str) -> bool {
    matches!(
        ty,
        "std_msgs/Header"
            | "std_msgs/msg/Header"
            | "builtin_interfaces/Time"
            | "builtin_interfaces/msg/Time"
    )
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A CDR payload whose leading field is a `builtin_interfaces/Time`
    /// (little-endian encapsulation), as a header-first ROS2 message lays it
    /// down. Trailing bytes stand in for the rest of the message body.
    pub(crate) fn stamped_payload(sec: i32, nanosec: u32) -> Vec<u8> {
        let mut buf = vec![0x00, 0x01, 0x00, 0x00]; // CDR_LE encapsulation header
        buf.extend_from_slice(&sec.to_le_bytes());
        buf.extend_from_slice(&nanosec.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes()); // empty frame_id (string len 0)
        buf
    }

    /// Split a nanosecond capture time into the `(sec, nanosec)` a payload
    /// carries, the inverse of `cdr_header_stamp_ns`'s recomposition.
    pub(crate) fn payload_for_ns(ns: u64) -> Vec<u8> {
        stamped_payload((ns / 1_000_000_000) as i32, (ns % 1_000_000_000) as u32)
    }

    /// A minimal `ros2msg` schema whose top-level first field is `first_field`.
    fn schema(encoding: &str, body: &str) -> SchemaDef {
        SchemaDef {
            name: "test/Msg".to_string(),
            encoding: encoding.to_string(),
            data: body.as_bytes().to_vec(),
        }
    }

    #[test]
    fn decodes_a_little_endian_stamp_at_the_fixed_offsets() {
        let ns = 1_700_000_000 * 1_000_000_000 + 250_000_000;
        assert_eq!(cdr_header_stamp_ns(&payload_for_ns(ns)), Some(ns));
    }

    #[test]
    fn decodes_a_big_endian_stamp() {
        // Big-endian encapsulation (byte 1 low bit clear): sec/nanosec in BE.
        let mut buf = vec![0x00, 0x00, 0x00, 0x00];
        buf.extend_from_slice(&7i32.to_be_bytes());
        buf.extend_from_slice(&5u32.to_be_bytes());
        assert_eq!(cdr_header_stamp_ns(&buf), Some(7 * 1_000_000_000 + 5));
    }

    #[test]
    fn rejects_a_payload_too_short_for_a_stamp() {
        assert_eq!(cdr_header_stamp_ns(b"payload"), None);
        assert_eq!(cdr_header_stamp_ns(&[]), None);
    }

    #[test]
    fn rejects_the_unset_zero_stamp() {
        // A header whose stamp was never set: (0, 0). Not "captured at the
        // epoch" — no usable capture time, so the caller falls back.
        assert_eq!(cdr_header_stamp_ns(&stamped_payload(0, 0)), None);
    }

    #[test]
    fn rejects_implausible_values() {
        assert_eq!(cdr_header_stamp_ns(&stamped_payload(-1, 0)), None);
        // nanosec must be < 1e9; a larger value is not a real Time field.
        let mut buf = vec![0x00, 0x01, 0x00, 0x00];
        buf.extend_from_slice(&5i32.to_le_bytes());
        buf.extend_from_slice(&1_000_000_000u32.to_le_bytes());
        assert_eq!(cdr_header_stamp_ns(&buf), None);
    }

    #[test]
    fn effective_time_uses_the_stamp_only_for_a_stamped_channel() {
        let payload = payload_for_ns(500);
        assert_eq!(
            effective_time(true, &payload, 9_000),
            500,
            "stamped: capture"
        );
        assert_eq!(
            effective_time(false, &payload, 9_000),
            9_000,
            "unstamped: log_time even when the bytes would decode"
        );
    }

    #[test]
    fn effective_time_falls_back_when_a_stamped_channel_has_no_usable_stamp() {
        // Stamped channel, but the stamp is the unset sentinel / payload short.
        assert_eq!(effective_time(true, &stamped_payload(0, 0), 9_000), 9_000);
        assert_eq!(effective_time(true, b"hi", 9_000), 9_000);
    }

    #[test]
    fn header_first_schema_is_stamped() {
        assert!(schema_is_stamped(Some(&schema(
            "ros2msg",
            "std_msgs/Header header\nuint32 value\n"
        ))));
    }

    #[test]
    fn time_first_schema_is_stamped() {
        // e.g. rcl_interfaces/Log: constants precede the first real field.
        let body = "byte DEBUG=10\nbyte INFO=20\nbuiltin_interfaces/Time stamp\nstring msg\n";
        assert!(schema_is_stamped(Some(&schema("ros2msg", body))));
    }

    #[test]
    fn comments_and_blank_lines_before_the_header_are_skipped() {
        let body = "# a comment\n\n   # indented comment\nstd_msgs/Header header\n";
        assert!(schema_is_stamped(Some(&schema("ros2msg", body))));
    }

    #[test]
    fn msg_form_with_slash_msg_is_recognised() {
        assert!(schema_is_stamped(Some(&schema(
            "ros2msg",
            "builtin_interfaces/msg/Time stamp\n"
        ))));
    }

    #[test]
    fn headerless_schema_is_not_stamped() {
        // A sequence-first type (TFMessage shape) and a scalar-first type.
        assert!(!schema_is_stamped(Some(&schema(
            "ros2msg",
            "geometry_msgs/TransformStamped[] transforms\n"
        ))));
        assert!(!schema_is_stamped(Some(&schema(
            "ros2msg",
            "string data\n"
        ))));
    }

    #[test]
    fn only_the_top_level_message_decides() {
        // The dependent Header block must not make a String message look
        // stamped: the separator ends the top-level scan first.
        let body = "string data\n\
                    ================================================================================\n\
                    MSG: std_msgs/Header\n\
                    builtin_interfaces/Time stamp\n\
                    string frame_id\n";
        assert!(!schema_is_stamped(Some(&schema("ros2msg", body))));
    }

    #[test]
    fn a_field_default_is_not_mistaken_for_a_constant() {
        // `int32 x 5` is a field with a default, not a constant: it is the
        // first field, and its type is not a stamp type, so: unstamped.
        assert!(!schema_is_stamped(Some(&schema(
            "ros2msg",
            "int32 x 5\nstd_msgs/Header header\n"
        ))));
    }

    #[test]
    fn schemaless_or_non_ros2msg_is_not_stamped() {
        assert!(!schema_is_stamped(None));
        assert!(!schema_is_stamped(Some(&schema(
            "jsonschema",
            "std_msgs/Header header\n"
        ))));
    }
}
