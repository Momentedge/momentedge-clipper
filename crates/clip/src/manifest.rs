//! What a clip says about itself: the MCAP metadata record every cut writes.
//!
//! A clip is a standalone file that leaves its output directory and is read
//! somewhere else, by someone who has neither the recorder's logs nor the
//! recording it was cut from. The manifest is the answer it carries with it —
//! who cut it, what asked for the window, which bytes of which recording it was
//! copied from, and what came out per channel — written as one
//! [`mcap::records::Metadata`] record under [`MANIFEST_NAME`].
//!
//! **Flat dotted keys, string values.** MCAP metadata is a `string -> string`
//! map, so structure lives in the key (`window.start_ns`,
//! `channel.3.messages`) and every value is rendered text. That keeps the record
//! readable by any MCAP tool — `mcap get metadata --name momentedge.clip` prints
//! it as it stands — without a second encoding to agree on.
//!
//! **Where the halves come from.** The copy knows what it read and wrote; it
//! does not know who asked, or what the planner offered it. So a manifest is
//! assembled from two sides: a [`CutRequest`] the caller builds once per window
//! (the producer, the trigger, the window that trigger resolves to) plus the
//! [`Planned`] facts of that window, and the per-segment counters the copy
//! accumulates as it runs. [`ClipManifest`] is the two brought together, one per
//! segment: a window straddling a rollover writes one record per segment, each
//! naming its own source file and its own counts, and the two sides agreeing on
//! everything else.
//!
//! **Why an empty clip still says something.** Every trigger produces a clip,
//! including one holding nothing. Three keys are what make the empty ones
//! distinguishable, since their message sections are byte-identical:
//! `source.files_planned` (zero when no recording held any byte of the window),
//! `clip.messages`, and `clip.short` (the recording had not covered the window
//! end when the cut was taken). A clip empty because nothing was recorded is
//! `files_planned=0 short=true`; one empty because the window fell in a gap
//! between splits is `files_planned=0 short=false` — the recording ran past the
//! window, it simply held no bytes inside it; one empty because no message
//! matched is `files_planned>=1 short=false`.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result};

use crate::TimeSource;
use crate::index::op;
use crate::trigger::Trigger;

/// The MCAP metadata record name a clip's manifest is written under.
///
/// Vendor-namespaced so it never collides with a metadata record the *recording*
/// carried: `ros2 bag record` writes its own under the bare name `rosbag2`, and
/// a clip cut from such a recording may sit beside tooling that reads it. The
/// dotted vendor prefix keeps the two apart by construction, and leaves room for
/// further Momentedge records under the same prefix.
pub const MANIFEST_NAME: &str = "momentedge.clip";

/// The manifest schema's own version, written under `manifest.version`.
///
/// A reader dispatches on it: keys may be added within a version, but a key that
/// changes meaning takes a new version. It is the first key so that a reader can
/// decide whether it understands the rest.
pub const MANIFEST_VERSION: &str = "1";

/// The program that cut a clip and which of its modes ran.
///
/// Both are compile-time names — `clipper` and the subcommand clap parsed — so a
/// clip states which binary and which mode produced it without the cut path
/// learning anything about either. The crate version and project URL are not
/// carried here: they are build facts of the cut path itself, taken from this
/// crate's own manifest ([`ClipManifest::entries`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Producer {
    /// The binary's name, as an operator invokes it (`clipper`).
    pub program: &'static str,
    /// The subcommand that cut the clip (`tail`).
    pub mode: &'static str,
}

/// Whether the recording had covered the window's end when the cut was taken.
///
/// This is the one thing a clip cannot show from its own contents: a clip whose
/// last message sits well before the window end looks the same whether the
/// recorded topics simply went quiet or the recorder never got there. A caller
/// that waits for coverage before cutting (the device recorder waits out its
/// `--grace-secs`) knows which it was, and says so here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindowCoverage {
    /// The recording reached the window end before the cut ran, so the clip
    /// holds everything the window asked for that was ever recorded.
    Covered,
    /// The cut ran without the recording ever reaching the window end — a wait
    /// that timed out, or a recording that stops inside the window — so the clip
    /// may end short of what was asked for.
    Short,
}

impl WindowCoverage {
    /// The `clip.short` value: `true` for [`Self::Short`].
    fn short(self) -> bool {
        match self {
            WindowCoverage::Covered => false,
            WindowCoverage::Short => true,
        }
    }
}

/// What a caller asked one cut for: who is cutting, the trigger that named the
/// window, and the window that trigger resolves to.
///
/// The bounds are **derived here**, not passed in, so the window the manifest
/// reports and the window every message's membership is tested against are one
/// value that cannot drift apart. `start_ns` and `end_ns` saturate, so a preroll
/// past the epoch or a postroll past `u64::MAX` clamps rather than wrapping into
/// a window somewhere else entirely.
#[derive(Clone, Debug)]
pub struct CutRequest {
    producer: Producer,
    trigger: Trigger,
    anchor_ns: u64,
    time_source: TimeSource,
    start_ns: u64,
    end_ns: u64,
}

impl CutRequest {
    /// The window `[anchor - preroll, anchor + postroll]` on `time_source`, as
    /// `trigger` asked for it and `producer` is about to cut it.
    pub fn new(
        producer: Producer,
        trigger: Trigger,
        anchor_ns: u64,
        time_source: TimeSource,
    ) -> Self {
        let start_ns = anchor_ns.saturating_sub(trigger.preroll);
        let end_ns = anchor_ns.saturating_add(trigger.postroll);
        CutRequest {
            producer,
            trigger,
            anchor_ns,
            time_source,
            start_ns,
            end_ns,
        }
    }

    /// The inclusive window start.
    pub fn start_ns(&self) -> u64 {
        self.start_ns
    }

    /// The inclusive window end.
    pub fn end_ns(&self) -> u64 {
        self.end_ns
    }

    /// The clock domain the whole window lives in.
    pub fn time_source(&self) -> TimeSource {
        self.time_source
    }

    /// The instant the window centres on.
    pub fn anchor_ns(&self) -> u64 {
        self.anchor_ns
    }

    /// The trigger that named the window.
    pub fn trigger(&self) -> &Trigger {
        &self.trigger
    }
}

/// What the cut found around one window, repeated in every segment's manifest.
///
/// Both facts are properties of the *window*, not of any one segment: the
/// segments of a straddling window agree on them and differ only in their own
/// source file and their own counters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Planned {
    /// How many source recordings the window was planned over. Zero means no
    /// recording the planner knows held a single byte of the window.
    pub files: usize,
    /// Whether the recording had covered the window end when the cut was taken.
    pub coverage: WindowCoverage,
}

/// What one channel contributed to a clip: how many of its messages were copied
/// and the earliest and latest stamp among them, on the window's clock.
///
/// Accumulated as messages are copied, so a channel appears in a manifest only
/// once it has contributed a message. A channel a cut declined to copy — nothing
/// of it inside the window, or a selection that left it out — therefore has no
/// keys at all rather than a row of zeroes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ChannelTally {
    pub messages: u64,
    pub first_ns: u64,
    pub last_ns: u64,
}

impl ChannelTally {
    /// The tally a channel's first copied message opens.
    pub fn opened(stamp_ns: u64) -> Self {
        ChannelTally {
            messages: 1,
            first_ns: stamp_ns,
            last_ns: stamp_ns,
        }
    }

    /// Fold one more copied message in. Min/max rather than first/last seen:
    /// `publish_time` may arrive out of order, so file order is not stamp order
    /// and only the extremes are a fact about the clip's contents.
    pub fn absorb(&mut self, stamp_ns: u64) {
        self.messages = self.messages.saturating_add(1);
        self.first_ns = self.first_ns.min(stamp_ns);
        self.last_ns = self.last_ns.max(stamp_ns);
    }
}

/// One clip's manifest: everything its record says about the cut that wrote it.
///
/// Borrowed rather than owned — it is built at the end of a copy from state the
/// copy already holds, rendered once by [`Self::record`], and dropped.
#[derive(Debug)]
pub struct ClipManifest<'a> {
    /// Who asked for the window, and which window.
    pub request: &'a CutRequest,
    /// What the planner offered for that window.
    pub planned: Planned,
    /// The recording this segment was copied from; `None` for a segment with no
    /// source at all (a window no recording covered).
    pub source: Option<&'a Path>,
    /// How many planned extents this segment read.
    pub extents_read: usize,
    /// How many bytes those extents spanned.
    pub bytes_read: u64,
    /// How many messages were copied into the clip.
    pub messages: u64,
    /// Per **output** channel id — the id the clip's own `Channel` records use,
    /// so a reader joins on what it can see — what that channel contributed.
    pub channels: &'a BTreeMap<u16, ChannelTally>,
}

impl ClipManifest<'_> {
    /// Render as the MCAP metadata record a clip's writer emits.
    pub fn record(&self) -> mcap::records::Metadata {
        mcap::records::Metadata {
            name: MANIFEST_NAME.to_string(),
            metadata: self.entries(),
        }
    }

    /// The flat dotted key/value map the record carries.
    ///
    /// `producer.version` and `producer.url` come from this crate's own build
    /// metadata: they identify the cut path that wrote the file, which is the
    /// version a reader needs when a clip looks wrong, and they follow the
    /// workspace version without a second place to update.
    fn entries(&self) -> BTreeMap<String, String> {
        let request = self.request;
        let trigger = request.trigger();
        let mut m = BTreeMap::new();
        let mut put = |key: &str, value: String| {
            m.insert(key.to_string(), value);
        };

        put("manifest.version", MANIFEST_VERSION.to_string());

        put("producer.name", request.producer.program.to_string());
        put("producer.mode", request.producer.mode.to_string());
        put("producer.version", env!("CARGO_PKG_VERSION").to_string());
        put("producer.url", env!("CARGO_PKG_REPOSITORY").to_string());

        put("trigger.name", trigger.name.clone());
        put("trigger.description", trigger.description.clone());
        put("trigger.anchor_ns", request.anchor_ns().to_string());
        put("trigger.preroll_ns", trigger.preroll.to_string());
        put("trigger.postroll_ns", trigger.postroll.to_string());

        put("window.time_source", request.time_source().to_string());
        put("window.start_ns", request.start_ns().to_string());
        put("window.end_ns", request.end_ns().to_string());

        put("source.files_planned", self.planned.files.to_string());
        if let Some(path) = self.source {
            put("source.path", path.display().to_string());
        }
        put("source.extents_read", self.extents_read.to_string());
        put("source.bytes_read", self.bytes_read.to_string());

        put("clip.messages", self.messages.to_string());
        put("clip.short", self.planned.coverage.short().to_string());

        for (id, tally) in self.channels {
            put(
                &format!("channel.{id}.messages"),
                tally.messages.to_string(),
            );
            put(
                &format!("channel.{id}.first_ns"),
                tally.first_ns.to_string(),
            );
            put(&format!("channel.{id}.last_ns"), tally.last_ns.to_string());
        }
        m
    }
}

/// The manifest a written clip carries, read back through the summary's metadata
/// index — a bounded seek and one record, never a walk of the message section.
///
/// `Ok(None)` means the file parses but carries no [`MANIFEST_NAME`] record: an
/// MCAP from somewhere else, or one whose summary was lost. Errors are a file
/// that will not parse at all.
pub fn read_manifest(path: &Path) -> Result<Option<BTreeMap<String, String>>> {
    let buf = std::fs::read(path).with_context(|| format!("reading {}", path.display()))?;
    let Some(summary) = mcap::Summary::read(&buf)
        .with_context(|| format!("reading the summary of {}", path.display()))?
    else {
        return Ok(None);
    };
    let Some(index) = summary
        .metadata_indexes
        .iter()
        .find(|i| i.name == MANIFEST_NAME)
    else {
        return Ok(None);
    };
    // The index addresses the whole record; its body starts past the opcode and
    // the u64 length prefix the framing puts in front of every record.
    let start = index.offset as usize + 9;
    let end = index
        .offset
        .checked_add(index.length)
        .map(|end| end as usize)
        .filter(|end| *end <= buf.len() && *end >= start)
        .with_context(|| format!("{} indexes its manifest out of bounds", path.display()))?;
    let record = mcap::parse_record(op::METADATA, &buf[start..end])
        .with_context(|| format!("parsing the manifest of {}", path.display()))?;
    #[expect(
        clippy::wildcard_enum_match_arm,
        reason = "mcap::records::Record is a foreign enum; only the Metadata arm is reachable here"
    )]
    match record {
        mcap::records::Record::Metadata(metadata) => Ok(Some(metadata.metadata)),
        _ => unreachable!("a METADATA opcode parses to Record::Metadata"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trigger::Stamp;

    fn producer() -> Producer {
        Producer {
            program: "clipper",
            mode: "tail",
        }
    }

    fn trigger(preroll: u64, postroll: u64) -> Trigger {
        Trigger {
            name: "evt".to_string(),
            description: "a description".to_string(),
            trigger_time: Stamp { sec: 0, nanosec: 0 },
            preroll,
            postroll,
        }
    }

    /// The window is the trigger's, derived once: a manifest cannot report
    /// bounds the copy did not use, because there is only one pair of bounds.
    #[test]
    fn a_request_derives_its_window_from_the_trigger() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Log);
        assert_eq!((request.start_ns(), request.end_ns()), (600, 1_600));
    }

    /// A preroll reaching past the epoch and a postroll past `u64::MAX` clamp
    /// instead of wrapping into an unrelated window.
    #[test]
    fn a_request_saturates_rather_than_wrapping() {
        let early = CutRequest::new(producer(), trigger(10, 0), 5, TimeSource::Log);
        assert_eq!(early.start_ns(), 0);
        let late = CutRequest::new(producer(), trigger(0, 10), u64::MAX, TimeSource::Log);
        assert_eq!(late.end_ns(), u64::MAX);
    }

    #[test]
    fn a_tally_folds_stamps_in_either_order() {
        let mut tally = ChannelTally::opened(500);
        tally.absorb(100);
        tally.absorb(900);
        assert_eq!(
            tally,
            ChannelTally {
                messages: 3,
                first_ns: 100,
                last_ns: 900,
            }
        );
    }

    /// The whole key set of a populated manifest, spelled out: the record is a
    /// wire contract read by tools outside this repo, so a renamed or dropped
    /// key has to be a deliberate edit here rather than a silent change.
    #[test]
    fn a_manifest_renders_every_key_group() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Publish);
        let channels = BTreeMap::from([(
            3,
            ChannelTally {
                messages: 7,
                first_ns: 610,
                last_ns: 1_590,
            },
        )]);
        let manifest = ClipManifest {
            request: &request,
            planned: Planned {
                files: 2,
                coverage: WindowCoverage::Covered,
            },
            source: Some(Path::new("/rec/bag_0.mcap")),
            extents_read: 4,
            bytes_read: 8_192,
            messages: 7,
            channels: &channels,
        };

        let entries = manifest.entries();
        let expected = [
            ("manifest.version", "1"),
            ("producer.name", "clipper"),
            ("producer.mode", "tail"),
            ("producer.version", env!("CARGO_PKG_VERSION")),
            ("producer.url", env!("CARGO_PKG_REPOSITORY")),
            ("trigger.name", "evt"),
            ("trigger.description", "a description"),
            ("trigger.anchor_ns", "1000"),
            ("trigger.preroll_ns", "400"),
            ("trigger.postroll_ns", "600"),
            ("window.time_source", "publish"),
            ("window.start_ns", "600"),
            ("window.end_ns", "1600"),
            ("source.files_planned", "2"),
            ("source.path", "/rec/bag_0.mcap"),
            ("source.extents_read", "4"),
            ("source.bytes_read", "8192"),
            ("clip.messages", "7"),
            ("clip.short", "false"),
            ("channel.3.messages", "7"),
            ("channel.3.first_ns", "610"),
            ("channel.3.last_ns", "1590"),
        ];
        let expected: BTreeMap<String, String> = expected
            .into_iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect();
        assert_eq!(entries, expected);
        assert_eq!(manifest.record().name, MANIFEST_NAME);
        assert!(
            !env!("CARGO_PKG_REPOSITORY").is_empty(),
            "the project URL must be a real value, not an empty package field"
        );
    }

    /// A segment with no source recording omits `source.path` outright rather
    /// than writing an empty one — the key's absence *is* "no recording covered
    /// this window", alongside `source.files_planned = 0`.
    #[test]
    fn a_sourceless_manifest_omits_the_path_and_reports_no_files() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Log);
        let channels = BTreeMap::new();
        let entries = ClipManifest {
            request: &request,
            planned: Planned {
                files: 0,
                coverage: WindowCoverage::Short,
            },
            source: None,
            extents_read: 0,
            bytes_read: 0,
            messages: 0,
            channels: &channels,
        }
        .entries();

        assert!(!entries.contains_key("source.path"));
        assert_eq!(entries["source.files_planned"], "0");
        assert_eq!(entries["clip.messages"], "0");
        assert_eq!(entries["clip.short"], "true");
        assert!(
            !entries.keys().any(|k| k.starts_with("channel.")),
            "a clip that copied nothing carries no per-channel keys"
        );
    }
}
