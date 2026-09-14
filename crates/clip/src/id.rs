//! What a clip is called: its **id**, a function of the request that asked for
//! it and of nothing else.
//!
//! An id is `<anchor_ns>_<hash>` — the resolved anchor in nanoseconds on the
//! run's [time source](crate::TimeSource), then sixteen lower-case hex
//! characters of SHA-256 written as four groups of four:
//!
//! ```text
//! 1726300000000000000_fc43-6475-ade8-4730
//! ```
//!
//! **The anchor leads so ids sort in time order**, and the hash follows so two
//! triggers at one instant are two clips rather than one. The six fields the
//! hash covers are the whole of what a caller asked for — the resolved anchor,
//! `name`, `description`, `preroll`, `postroll`, and the time source — so a
//! change to any of them, the description included, yields a different id, and
//! the same request yields the same id on every machine and every version.
//!
//! **No trigger text reaches the id.** A name holding `/`, `..`, unicode or
//! nothing at all is hashed like any other and cannot shape a path, which is why
//! nothing sanitizes a name and nothing needs to.
//!
//! ## The canonical encoding
//!
//! It is a **published contract** (the reference page is
//! [`docs/clip-manifest.md`](https://github.com/Momentedge/momentedge-clipper/blob/main/docs/clip-manifest.md),
//! which carries a worked vector `sha256sum` reproduces) and must not change:
//! an id printed in an incident report has to stay valid. [`ENCODING_TAG`] is
//! the first line for that reason — a future encoding is a new tag and a new id
//! space, never the same bytes quietly meaning something else.
//!
//! Nine lines, each terminated by `\n`:
//!
//! ```text
//! momentedge.clip.id/1
//! <anchor_ns>
//! <byte length of name>
//! <name>
//! <byte length of description>
//! <description>
//! <preroll_ns>
//! <postroll_ns>
//! log | publish
//! ```
//!
//! The integers are rendered decimal without separators or padding, and the time
//! source as the name its command line takes. **The two free-text fields are
//! length-prefixed**, so the newline after each is decoration rather than a
//! delimiter: a name that itself holds newlines, or ends in one, still encodes
//! to exactly one byte string, which is what makes two distinct triggers unable
//! to collide by construction rather than by luck.

use std::fmt;

use sha2::{Digest as _, Sha256};

use crate::manifest::CutRequest;

/// The first line of every canonical encoding: this encoding's own name and
/// version. It is inside the hash, so an id under a later encoding cannot equal
/// one under this encoding even where every field agrees.
pub const ENCODING_TAG: &str = "momentedge.clip.id/1";

/// How many bytes of the SHA-256 digest an id's hash is rendered from. Eight
/// bytes are sixteen hex characters: short enough to read back off a filename,
/// and 2^64 wide, so the triggers one vehicle produces will not collide.
const DIGEST_BYTES: usize = 8;

/// A clip's id: the resolved anchor and the digest of the request behind it.
///
/// Rendered by [`Display`](fmt::Display) as `<anchor_ns>_aaaa-bbbb-cccc-dddd`,
/// which is the name the clip is written under and the value a clip's manifest
/// carries as `clip.id`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClipId {
    anchor_ns: u64,
    digest: [u8; DIGEST_BYTES],
}

impl ClipId {
    /// The id of the clip `request` asks for.
    #[must_use]
    pub fn of(request: &CutRequest) -> Self {
        let mut digest = [0u8; DIGEST_BYTES];
        for (slot, byte) in digest
            .iter_mut()
            .zip(Sha256::digest(canonical_encoding(request).as_bytes()))
        {
            *slot = byte;
        }
        ClipId {
            anchor_ns: request.anchor_ns(),
            digest,
        }
    }

    /// The instant the clip's window centres on, which is also what the id sorts
    /// on.
    #[must_use]
    pub fn anchor_ns(self) -> u64 {
        self.anchor_ns
    }
}

impl fmt::Display for ClipId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}_", self.anchor_ns)?;
        for (i, group) in self.digest.chunks(2).enumerate() {
            if i > 0 {
                f.write_str("-")?;
            }
            for byte in group {
                write!(f, "{byte:02x}")?;
            }
        }
        Ok(())
    }
}

/// The bytes an id's digest is taken over: the module header's nine lines, in
/// that order, for this one request.
///
/// A `String` rather than a `Vec<u8>` because every field already is one — the
/// trigger's name and description are Rust strings, so the encoding is valid
/// UTF-8 by construction and a reader can print it. The lengths are `str::len`,
/// which is bytes, so they mean the same thing to a shell script counting with
/// `wc -c` as they do here.
fn canonical_encoding(request: &CutRequest) -> String {
    let trigger = request.trigger();
    let mut encoding = String::new();
    let mut line = |field: &str| {
        encoding.push_str(field);
        encoding.push('\n');
    };

    line(ENCODING_TAG);
    line(&request.anchor_ns().to_string());
    line(&trigger.name.len().to_string());
    line(&trigger.name);
    line(&trigger.description.len().to_string());
    line(&trigger.description);
    line(&trigger.preroll.to_string());
    line(&trigger.postroll.to_string());
    line(&request.time_source().to_string());

    encoding
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        reason = "a failed unwrap is a failing test"
    )]

    use super::*;
    use crate::TimeSource;
    use crate::testing::TEST_PRODUCER;
    use crate::trigger::{Stamp, Trigger};

    /// The request the six fields of one trigger resolve to. `trigger_time` is
    /// deliberately not among them: the anchor is what a trigger source resolved
    /// *from* it, and two requests agreeing on the anchor describe one clip
    /// whatever stamp got them there.
    #[expect(
        clippy::too_many_arguments,
        reason = "the six fields an id is a function of, which is the point: a \
                  test states all of them so it can move exactly one"
    )]
    fn request(
        anchor_ns: u64,
        name: &str,
        description: &str,
        preroll: u64,
        postroll: u64,
        source: TimeSource,
    ) -> CutRequest {
        CutRequest::new(
            TEST_PRODUCER,
            Trigger {
                name: name.to_string(),
                description: description.to_string(),
                trigger_time: Stamp { sec: 0, nanosec: 0 },
                preroll,
                postroll,
            },
            anchor_ns,
            source,
        )
    }

    /// The vector `docs/clip-manifest.md` publishes, hashed here byte for byte.
    ///
    /// This is the contract test: the encoding is a promise to everyone holding
    /// an id, so a change to the field order, the separators, the tag, or how a
    /// value is rendered has to break this test before it can break someone's
    /// records. Reproduce it by hand with
    ///
    /// ```text
    /// printf 'momentedge.clip.id/1\n1726300000000000000\n11\nbrake-event\n21\nhard brake over 0.8 g\n5000000000\n5000000000\nlog\n' | sha256sum
    /// ```
    #[test]
    fn the_published_vector_encodes_and_hashes_to_its_documented_id() {
        let published = request(
            1_726_300_000_000_000_000,
            "brake-event",
            "hard brake over 0.8 g",
            5_000_000_000,
            5_000_000_000,
            TimeSource::Log,
        );

        assert_eq!(
            canonical_encoding(&published),
            "momentedge.clip.id/1\n\
             1726300000000000000\n\
             11\n\
             brake-event\n\
             21\n\
             hard brake over 0.8 g\n\
             5000000000\n\
             5000000000\n\
             log\n",
            "the documented encoding, line for line"
        );
        assert_eq!(
            ClipId::of(&published).to_string(),
            "1726300000000000000_fc43-6475-ade8-4730",
            "the documented id: the leading 8 bytes of the digest \
             fc436475ade84730780c870fb500412fe9d0ee985bbebdbff8ce54ce6f2222dd, \
             in four groups of four hex characters"
        );
    }

    /// The id is a function of the request: the same one twice is the same clip,
    /// and a change to any of the six fields — the description included, which
    /// changes no byte of the clip — is a different one.
    #[test]
    fn every_field_of_the_request_moves_the_id() {
        let base = request(3_000, "brake", "hard brake", 400, 600, TimeSource::Log);
        let id = ClipId::of(&base);
        assert_eq!(
            id,
            ClipId::of(&request(
                3_000,
                "brake",
                "hard brake",
                400,
                600,
                TimeSource::Log
            )),
            "the same trigger asked for the same clip"
        );

        for (what, other) in [
            (
                "the anchor",
                request(3_001, "brake", "hard brake", 400, 600, TimeSource::Log),
            ),
            (
                "the name",
                request(3_000, "braked", "hard brake", 400, 600, TimeSource::Log),
            ),
            (
                "the description",
                request(3_000, "brake", "hard braked", 400, 600, TimeSource::Log),
            ),
            (
                "the preroll",
                request(3_000, "brake", "hard brake", 401, 600, TimeSource::Log),
            ),
            (
                "the postroll",
                request(3_000, "brake", "hard brake", 400, 601, TimeSource::Log),
            ),
            (
                "the time source",
                request(3_000, "brake", "hard brake", 400, 600, TimeSource::Publish),
            ),
        ] {
            assert_ne!(id, ClipId::of(&other), "{what} moves the id");
        }
    }

    /// Two triggers whose name and description are the same text split
    /// differently encode differently, so they are two clips.
    ///
    /// This is what the length prefixes buy, and the reason a bare separator
    /// would not do: `"ab" + ""` and `"a" + "b"` concatenate to the same
    /// characters, and an encoding that let them hash alike would drop one of
    /// the two clips on the floor.
    #[test]
    fn a_field_boundary_cannot_be_moved_without_moving_the_id() {
        let split = request(3_000, "a", "b", 400, 600, TimeSource::Log);
        let joined = request(3_000, "ab", "", 400, 600, TimeSource::Log);
        assert_ne!(ClipId::of(&split), ClipId::of(&joined));
    }

    /// A name that would be hostile in a path, or absent, is hashed like any
    /// other text: the id it produces is the same shape, carries none of it, and
    /// is a single path component.
    #[test]
    fn a_name_that_could_break_a_path_never_reaches_one() {
        for name in ["../../etc/passwd", "a/b", "", ".hidden", "ブレーキ", "a\nb"] {
            let id = ClipId::of(&request(3_000, name, "", 400, 600, TimeSource::Log)).to_string();
            assert_eq!(
                id.len(),
                "3000".len() + 1 + 19,
                "an id is the anchor and four hex groups, whatever the name: {id}"
            );
            assert!(
                !id.contains(['/', '\\', '.', '\n']),
                "an id is one plain path component: {id}"
            );
            assert!(
                name.is_empty() || !id.contains(name),
                "no trigger text reaches the id: {id}"
            );
        }
    }
}
