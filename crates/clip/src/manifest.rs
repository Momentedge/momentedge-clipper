//! What a clip says about itself: the `clip_metadata.yaml` document beside its
//! files, and the one key each of those files carries inside it.
//!
//! A clip leaves its output directory and is read somewhere else, by someone who
//! has neither the recorder's logs nor the recording it was cut from. Two
//! artefacts answer for it, and the split between them is the whole design:
//!
//! - **One [`ClipMetadata`] document per clip**, written as
//!   [`clip_metadata.yaml`](crate::layout::METADATA_FILE) into the clip's own
//!   directory once every file in it is durable. It holds the whole account —
//!   who cut it, what asked for the window, which bytes of which recordings were
//!   copied, and what came out per channel — stated once for the clip rather
//!   than repeated per file. Its presence is also what "complete" means, which
//!   is [`crate::layout`]'s half.
//! - **One key inside every MCAP file** ([`id_record`]): an
//!   [`mcap::records::Metadata`] record under [`MANIFEST_NAME`] carrying
//!   `clip.id` and nothing else. A file separated from its directory can still
//!   be grouped; everything else it might have said is in the document beside
//!   it, where one copy cannot disagree with another.
//!
//! **The id is derived, never carried.** [`ClipId::of`] runs over the same
//! [`CutRequest`] the `trigger` and `window` groups are written from, and over
//! the same request [`crate::layout`] names the directory and its files from.
//! One computation over one value is what makes the id, the record and the paths
//! unable to disagree.
//!
//! **Where the halves come from.** The copy knows what it read and wrote; it
//! does not know who asked, or what the planner offered it. So the document is
//! assembled from two sides: a [`CutRequest`] the caller builds once per window
//! (the producer, the trigger, the window that trigger resolves to) plus the
//! [`Planned`] facts of that window, and the per-file counters each copy
//! accumulated as it ran ([`crate::cut::ClipStats`]). [`ClipMetadata::of`] is
//! that join, and it is pure.
//!
//! **Why an empty clip still says something.** Every trigger produces a clip,
//! including one holding nothing. Three fields are what make the empty ones
//! distinguishable, since their message sections are byte-identical:
//! `window.files_planned` (zero when no recording held any byte of the window),
//! `clip.messages`, and `clip.short` (the recording had not covered the window
//! end when the cut was taken). A clip empty because nothing was recorded is
//! `files_planned=0 short=true`; one empty because the window fell in a gap
//! between splits is `files_planned=0 short=false` — the recording ran past the
//! window, it simply held no bytes inside it; one empty because no message
//! matched is `files_planned>=1 short=false`.

use std::collections::BTreeMap;
use std::ops::Range;
use std::path::Path;

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};

use crate::TimeSource;
use crate::cut::ClipStats;
use crate::id::ClipId;
use crate::index::op;
use crate::trigger::Trigger;

/// The MCAP metadata record name every clip file carries its `clip.id` under.
///
/// Vendor-namespaced so it never collides with a metadata record the *recording*
/// carried: `ros2 bag record` writes its own under the bare name `rosbag2`, and
/// a clip cut from such a recording may sit beside tooling that reads it. The
/// dotted vendor prefix keeps the two apart by construction, and leaves room for
/// further Momentedge records under the same prefix.
pub const MANIFEST_NAME: &str = "momentedge.clip";

/// The key [`id_record`] writes, and the only one an MCAP file of a clip
/// carries. Everything else a clip says about itself is in the
/// [`ClipMetadata`] document beside it.
pub const CLIP_ID_KEY: &str = "clip.id";

/// The [`ClipMetadata`] document's own schema version, written as its first
/// field.
///
/// A reader dispatches on it: fields may be added within a version, but a field
/// that changes meaning takes a new version. It leads the document so that a
/// reader can decide whether it understands the rest.
pub const METADATA_VERSION: &str = "1";

/// The program that cut a clip and which of its modes ran.
///
/// Both are compile-time names — `clipper` and the subcommand clap parsed — so a
/// clip states which binary and which mode produced it without the cut path
/// learning anything about either. The crate version and project URL are not
/// carried here: they are build facts of the cut path itself, taken from this
/// crate's own package manifest ([`ClipMetadata::of`]).
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
    ///
    /// ```
    /// use clip::{CutRequest, Producer, Stamp, TimeSource, Trigger};
    ///
    /// let trigger = Trigger {
    ///     name: "brake".to_string(),
    ///     description: String::new(),
    ///     trigger_time: Stamp { sec: 0, nanosec: 0 },
    ///     preroll: 400,
    ///     postroll: 600,
    /// };
    /// let producer = Producer {
    ///     program: "clipper",
    ///     mode: "tail",
    /// };
    ///
    /// let request = CutRequest::new(producer, trigger.clone(), 1_000, TimeSource::Log);
    /// assert_eq!((request.start_ns(), request.end_ns()), (600, 1_600));
    ///
    /// // The bounds saturate: an anchor closer to the epoch than the preroll
    /// // clamps to 0 rather than wrapping into a window somewhere else.
    /// let early = CutRequest::new(producer, trigger, 100, TimeSource::Log);
    /// assert_eq!((early.start_ns(), early.end_ns()), (0, 700));
    /// ```
    #[must_use]
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
    #[must_use]
    pub fn start_ns(&self) -> u64 {
        self.start_ns
    }

    /// The inclusive window end.
    #[must_use]
    pub fn end_ns(&self) -> u64 {
        self.end_ns
    }

    /// The clock domain the whole window lives in.
    #[must_use]
    pub fn time_source(&self) -> TimeSource {
        self.time_source
    }

    /// The instant the window centres on.
    #[must_use]
    pub fn anchor_ns(&self) -> u64 {
        self.anchor_ns
    }

    /// The trigger that named the window.
    #[must_use]
    pub fn trigger(&self) -> &Trigger {
        &self.trigger
    }
}

/// What the cut found around one window: over how many recordings it was
/// planned, and whether the recording had reached the window end.
///
/// Both are facts about the *window* rather than about any one of the files it
/// was cut into, which is why they reach the document's clip-wide groups
/// ([`WindowMeta::files_planned`] and [`ClipMeta::short`]) and appear in no
/// [`SourceMeta`] entry — a window straddling a split would otherwise state each
/// of them once per file, and the copies could disagree.
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
/// Accumulated as messages are copied, so a channel appears in the document
/// only once it has contributed a message. A channel a cut declined to copy —
/// nothing of it inside the window, or a selection that left it out — therefore
/// has no entry at all rather than a row of zeroes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ChannelTally {
    pub messages: u64,
    pub first_ns: u64,
    pub last_ns: u64,
}

impl ChannelTally {
    /// The tally a channel's first copied message opens.
    #[must_use]
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

/// The one-key metadata record every MCAP file of a clip carries: the
/// [`ClipId`] of the window it belongs to, under [`MANIFEST_NAME`].
///
/// One key and no more, deliberately. A clip's account of itself is the
/// [`ClipMetadata`] document beside its files, stated once; repeating any of it
/// per file would be several copies that a partial rewrite could leave
/// disagreeing. What a *file* has to answer on its own is only "which clip is
/// this?", so that one carried away from its directory can still be grouped —
/// and that is what this is.
#[must_use]
pub fn id_record(request: &CutRequest) -> mcap::records::Metadata {
    mcap::records::Metadata {
        name: MANIFEST_NAME.to_string(),
        metadata: BTreeMap::from([(CLIP_ID_KEY.to_string(), ClipId::of(request).to_string())]),
    }
}

/// Everything one clip says about itself: the document written as
/// [`clip_metadata.yaml`](crate::layout::METADATA_FILE) into the clip's
/// directory, last of all, once every file in it is durable.
///
/// The field groups are the clip's several halves kept apart on purpose — what
/// the clip *is*, who cut it, what asked for it, the window it covers, and one
/// entry per file it holds. A reader that wants one of them never parses the
/// others, and a field that moves between groups is a schema change the
/// [`version`](Self::version) names.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClipMetadata {
    /// This document's own schema version ([`METADATA_VERSION`]), first so a
    /// reader decides whether it understands the rest before reading it.
    pub version: String,
    /// What the clip is: its id, what it holds, and whether it is short.
    pub clip: ClipMeta,
    /// Which program and subcommand cut it.
    pub producer: ProducerMeta,
    /// The trigger that asked for it.
    pub trigger: TriggerMeta,
    /// The window it covers, on the clock it lives on.
    pub window: WindowMeta,
    /// One entry per MCAP file in the directory, in file-number order: which
    /// recording it was copied from and what came out of it.
    pub sources: Vec<SourceMeta>,
}

/// What the clip is, independent of how many files it took to hold it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ClipMeta {
    /// The clip's [`ClipId`] — also the name of its directory, and the stem
    /// every file in it opens with.
    pub id: String,
    /// How many messages the whole clip holds, across every file.
    pub messages: u64,
    /// Whether the cut ran without the recording ever reaching the window end,
    /// so the clip may stop short of what was asked for.
    pub short: bool,
}

/// Which build of which program cut the clip.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProducerMeta {
    /// The binary, as an operator invokes it (`clipper`).
    pub name: String,
    /// The subcommand that cut the clip (`tail` or `clip`).
    pub mode: String,
    /// The cut path's own crate version — the version a reader needs when a
    /// clip looks wrong.
    pub version: String,
    /// The project the cut path came from.
    pub url: String,
}

/// The trigger that asked for the clip, echoed verbatim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TriggerMeta {
    pub name: String,
    pub description: String,
    /// The instant the window centres on, as the trigger source resolved it.
    pub anchor_ns: u64,
    pub preroll_ns: u64,
    pub postroll_ns: u64,
}

/// The window the clip was cut for, and what the planner found for it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct WindowMeta {
    /// The clock domain the whole window lives in (`log` or `publish`).
    pub time_source: String,
    /// The inclusive window start.
    pub start_ns: u64,
    /// The inclusive window end.
    pub end_ns: u64,
    /// How many source recordings the window was planned over. Zero means no
    /// recording the planner knows held a single byte of it — which is why this
    /// is here and not derivable from [`ClipMetadata::sources`], whose entries
    /// count only the files that contributed.
    pub files_planned: usize,
}

/// One MCAP file of the clip and the recording it was copied from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SourceMeta {
    /// The clip's own file this entry describes — a name inside the clip
    /// directory, `<id>_N.mcap`.
    pub file: String,
    /// The recording it was copied from. Absent for the one file a window no
    /// recording covered still produces: there was no source to name.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// How many planned extents this file's copy read.
    pub extents_read: usize,
    /// How many bytes those extents spanned.
    pub bytes_read: u64,
    /// How many messages were copied into this file.
    pub messages: u64,
    /// Per **output** channel id — the id this file's own `Channel` records
    /// use, so a reader joins on what it can see — what that channel
    /// contributed.
    pub channels: BTreeMap<u16, ChannelTally>,
}

impl ClipMetadata {
    /// The document for a finished clip: the caller's [`CutRequest`], what the
    /// planner offered it, and one [`ClipStats`] per file the cut wrote, in
    /// file-number order.
    ///
    /// Pure — it reads no disk and writes none. Handing the written files in
    /// rather than the staged ones is what lets every `sources` entry name the
    /// file it describes: the numbering is settled by then.
    #[must_use]
    pub fn of(request: &CutRequest, planned: Planned, files: &[ClipStats]) -> Self {
        let trigger = request.trigger();
        ClipMetadata {
            version: METADATA_VERSION.to_string(),
            clip: ClipMeta {
                id: ClipId::of(request).to_string(),
                messages: files.iter().map(|f| f.messages_copied).sum(),
                short: planned.coverage.short(),
            },
            producer: ProducerMeta {
                name: request.producer.program.to_string(),
                mode: request.producer.mode.to_string(),
                version: env!("CARGO_PKG_VERSION").to_string(),
                url: env!("CARGO_PKG_REPOSITORY").to_string(),
            },
            trigger: TriggerMeta {
                name: trigger.name.clone(),
                description: trigger.description.clone(),
                anchor_ns: request.anchor_ns(),
                preroll_ns: trigger.preroll,
                postroll_ns: trigger.postroll,
            },
            window: WindowMeta {
                time_source: request.time_source().to_string(),
                start_ns: request.start_ns(),
                end_ns: request.end_ns(),
                files_planned: planned.files,
            },
            sources: files.iter().map(SourceMeta::of).collect(),
        }
    }
}

impl SourceMeta {
    /// One file's entry, read off what its copy did.
    fn of(stats: &ClipStats) -> Self {
        SourceMeta {
            file: stats
                .out_path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
            path: stats.source.as_ref().map(|p| p.display().to_string()),
            extents_read: stats.extents_read,
            bytes_read: stats.bytes_read,
            messages: stats.messages_copied,
            channels: stats.channels.clone(),
        }
    }
}

/// The byte range inside a recording's buffer that a metadata index addresses,
/// or `None` when the index points outside it.
///
/// Both numbers come out of the file, so both the addition and the narrowing to
/// `usize` are checked rather than trusted: a metadata index is as forgeable as
/// any other record, and a clip read far from where it was cut may simply be
/// damaged. The range starts past the frame header — the opcode and the u64
/// length prefix every record carries, which the index's offset addresses the
/// front of — and ends where the framed record does, so a `length` shorter than
/// that header addresses nothing and is refused with the rest.
fn manifest_range(offset: u64, length: u64, buf_len: usize) -> Option<Range<usize>> {
    /// The opcode and the u64 length prefix the framing puts in front of every
    /// record, which a metadata index's offset addresses the front of.
    const FRAME_HEADER_LEN: u64 = 1 + size_of::<u64>() as u64;

    let start = usize::try_from(offset.checked_add(FRAME_HEADER_LEN)?).ok()?;
    let end = usize::try_from(offset.checked_add(length)?).ok()?;
    (end <= buf_len && end >= start).then_some(start..end)
}

/// The [`MANIFEST_NAME`] record an MCAP file carries, read back through the
/// summary's metadata index: the record is addressed directly rather than found
/// by walking the message section.
///
/// A file clipper wrote answers one key, [`CLIP_ID_KEY`]. The map is returned
/// whole rather than just that value so a reader can tell "clipper wrote this
/// and said nothing else" from a record under the same name written by
/// something else.
///
/// The file is read whole into memory first, so this is for a clip — a bounded
/// window — and for tests and tooling, not for a recording of arbitrary size.
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
    let range = manifest_range(index.offset, index.length, buf.len())
        .with_context(|| format!("{} indexes its manifest out of bounds", path.display()))?;
    #[expect(
        clippy::indexing_slicing,
        reason = "`manifest_range` returns a range only when it lies inside a \
                  buffer of the length it was given, which is this one's"
    )]
    let body = &buf[range];
    let record = mcap::parse_record(op::METADATA, body)
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
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::too_many_lines,
        reason = "a failed unwrap or a panicking index is a failing test, and a \
                  test that spells out a whole wire contract field for field is \
                  long by construction — splitting one would scatter the contract"
    )]

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

    /// A clip file's record carries the clip's id and nothing else.
    ///
    /// The "nothing else" is the assertion worth making: everything a clip has
    /// to say is stated once in the document beside its files, so a second copy
    /// of any of it inside a file is a second thing to keep true.
    #[test]
    fn a_clip_file_states_its_id_and_nothing_else() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Log);
        let record = id_record(&request);

        assert_eq!(record.name, MANIFEST_NAME);
        assert_eq!(
            record.metadata,
            BTreeMap::from([(CLIP_ID_KEY.to_string(), ClipId::of(&request).to_string())]),
            "one key, and it is the id `clip::id` derives from the same request"
        );
    }

    /// The record's name and its one key, spelled out as the contract publishes
    /// them.
    ///
    /// Every other assertion in the workspace reaches them through
    /// [`MANIFEST_NAME`] and [`CLIP_ID_KEY`], so renaming either constant would
    /// leave all of them green while breaking what a reader looks the record up
    /// by (`mcap get metadata --name momentedge.clip`) and the key it reads out
    /// of it. The literals are what cannot move with the constants.
    #[test]
    fn a_files_record_is_named_and_keyed_as_the_contract_publishes() {
        assert_eq!(MANIFEST_NAME, "momentedge.clip");
        assert_eq!(CLIP_ID_KEY, "clip.id");
    }

    /// One written file, as the copy reports it.
    fn file(name: &str, source: Option<&str>, messages: u64) -> ClipStats {
        ClipStats {
            out_path: Path::new("/out/clip").join(name),
            source: source.map(std::path::PathBuf::from),
            extents_read: 4,
            bytes_read: 8_192,
            messages_copied: messages,
            bytes_copied: 512,
            records_skipped: 0,
            chunks_dropped: 0,
            channels: BTreeMap::from([(
                3,
                ChannelTally {
                    messages,
                    first_ns: 610,
                    last_ns: 1_590,
                },
            )]),
        }
    }

    /// The whole document of a clip straddling a split, spelled out and then
    /// round-tripped through YAML.
    ///
    /// The document is a wire contract read by an upload pipeline and by an
    /// incident analyst, neither of whom builds from this repo, so a renamed or
    /// dropped field has to be a deliberate edit here rather than a silent
    /// change. Two files is the interesting shape: the clip-level totals are the
    /// sum, and each entry names the file it describes beside the recording it
    /// came from.
    #[test]
    fn a_clips_document_states_every_group_and_one_entry_per_file() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Publish);
        let files = [
            file("clip_0.mcap", Some("/rec/bag_0.mcap"), 7),
            file("clip_1.mcap", Some("/rec/bag_1.mcap"), 4),
        ];

        let metadata = ClipMetadata::of(
            &request,
            Planned {
                files: 3,
                coverage: WindowCoverage::Covered,
            },
            &files,
        );

        // The id's own contract — the encoding and a published vector — is
        // pinned in `clip::id`; what this states is that the document carries
        // it, not what it is.
        assert_eq!(
            metadata,
            ClipMetadata {
                version: "1".to_string(),
                clip: ClipMeta {
                    id: ClipId::of(&request).to_string(),
                    messages: 11,
                    short: false,
                },
                producer: ProducerMeta {
                    name: "clipper".to_string(),
                    mode: "tail".to_string(),
                    version: env!("CARGO_PKG_VERSION").to_string(),
                    url: env!("CARGO_PKG_REPOSITORY").to_string(),
                },
                trigger: TriggerMeta {
                    name: "evt".to_string(),
                    description: "a description".to_string(),
                    anchor_ns: 1_000,
                    preroll_ns: 400,
                    postroll_ns: 600,
                },
                window: WindowMeta {
                    time_source: "publish".to_string(),
                    start_ns: 600,
                    end_ns: 1_600,
                    files_planned: 3,
                },
                sources: vec![
                    SourceMeta {
                        file: "clip_0.mcap".to_string(),
                        path: Some("/rec/bag_0.mcap".to_string()),
                        extents_read: 4,
                        bytes_read: 8_192,
                        messages: 7,
                        channels: BTreeMap::from([(
                            3,
                            ChannelTally {
                                messages: 7,
                                first_ns: 610,
                                last_ns: 1_590,
                            },
                        )]),
                    },
                    SourceMeta {
                        file: "clip_1.mcap".to_string(),
                        path: Some("/rec/bag_1.mcap".to_string()),
                        extents_read: 4,
                        bytes_read: 8_192,
                        messages: 4,
                        channels: BTreeMap::from([(
                            3,
                            ChannelTally {
                                messages: 4,
                                first_ns: 610,
                                last_ns: 1_590,
                            },
                        )]),
                    },
                ],
            }
        );
        assert!(
            !env!("CARGO_PKG_REPOSITORY").is_empty(),
            "the project URL must be a real value, not an empty package field"
        );

        let yaml = serde_norway::to_string(&metadata).expect("the document serializes");
        assert!(
            yaml.starts_with("version: '1'\n"),
            "the schema version leads the document, so a reader dispatches on it \
             before parsing the rest: {yaml}"
        );
        assert_eq!(
            serde_norway::from_str::<ClipMetadata>(&yaml).expect("and parses back"),
            metadata,
            "what a clip states is what a consumer reads"
        );
    }

    /// The one file a window no recording covered still produces names no source
    /// recording: the field's absence *is* "there was nothing to copy from",
    /// alongside `window.files_planned = 0`.
    #[test]
    fn a_clip_no_recording_covered_names_no_source() {
        let request = CutRequest::new(producer(), trigger(400, 600), 1_000, TimeSource::Log);
        let files = [ClipStats {
            out_path: Path::new("/out/clip/clip_0.mcap").to_path_buf(),
            ..ClipStats::default()
        }];

        let metadata = ClipMetadata::of(
            &request,
            Planned {
                files: 0,
                coverage: WindowCoverage::Short,
            },
            &files,
        );

        assert_eq!(metadata.window.files_planned, 0);
        assert_eq!(metadata.clip.messages, 0);
        assert!(metadata.clip.short);
        assert_eq!(metadata.sources.len(), 1, "an empty clip is still one file");
        assert_eq!(metadata.sources[0].path, None);
        assert!(
            metadata.sources[0].channels.is_empty(),
            "a file that copied nothing tallies no channel"
        );

        let yaml = serde_norway::to_string(&metadata).expect("the document serializes");
        assert!(
            !yaml.contains("path:"),
            "an absent source is omitted rather than written empty: {yaml}"
        );
    }

    /// A metadata index is as forgeable as any other record, and a clip read far
    /// from where it was cut may simply be damaged. Every way the addressed
    /// range can fall outside the buffer is refused, so the slice that follows
    /// cannot panic.
    #[test]
    fn a_manifest_index_pointing_outside_the_file_is_refused() {
        // The frame header is 9 bytes, so a record at offset 0 declaring 20
        // bytes carries an 11-byte body at 9..20.
        assert_eq!(
            manifest_range(0, 20, 64),
            Some(9..20),
            "a record wholly inside the buffer addresses its own body"
        );
        assert_eq!(
            manifest_range(0, 9, 64),
            Some(9..9),
            "a record that is all header addresses an empty body, not a fault"
        );

        assert_eq!(
            manifest_range(0, 65, 64),
            None,
            "a record running past the end of the file"
        );
        assert_eq!(
            manifest_range(60, 20, 64),
            None,
            "a record starting inside the file and ending past it"
        );
        assert_eq!(
            manifest_range(0, 8, 64),
            None,
            "a record shorter than its own frame header, which would invert the range"
        );
        assert_eq!(
            manifest_range(u64::MAX, 9, 64),
            None,
            "an offset whose frame header overflows the address space"
        );
        assert_eq!(
            manifest_range(9, u64::MAX, 64),
            None,
            "a length that overflows the address space"
        );
    }

    /// Reading a clip clipper did not write is ordinary: an MCAP from somewhere
    /// else carries no manifest, and saying so is not an error.
    #[test]
    fn a_recording_without_a_manifest_reports_none() -> anyhow::Result<()> {
        let dir = crate::testing::test_dir("manifest-absent")?;
        let path = dir.join("rec.mcap");
        crate::testing::write_recording(&path, false, &[("/a", 1_000)])?;

        assert_eq!(
            read_manifest(&path)?,
            None,
            "a finished recording with no metadata record has no manifest to report"
        );

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }
}
