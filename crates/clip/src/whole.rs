//! The index of a recording nobody is writing, taken from the recording's own
//! summary instead of by walking it.
//!
//! A finalised MCAP ends with a summary section: a chunk index per chunk (its
//! byte range and the `log_time` span of the messages inside it), the whole
//! schema and channel registry, and the file's message statistics. That is
//! everything [`crate::index`]'s incremental scan spends a full pass over the
//! data section rebuilding — so for a recording that is already finished, the
//! index is a footer seek and one read of the summary, whatever the recording's
//! size, and no chunk is decompressed until the cut asks for one.
//!
//! The result is folded into a [`RecordingIndex`], the same artefact a scan
//! produces, and served through the same [`WindowPlanner`] seam: one plan whose
//! extents are the chunks overlapping the window, with the registry to map their
//! channel ids through. [`crate::cut`] therefore takes a window out of a
//! summary-built index and out of a scanned one with the same code and no idea
//! which it was handed.
//!
//! **What the summary does not say.** MCAP states message times on `log_time`
//! alone — a chunk index's span and the statistics' bounds are both log times,
//! and nothing in the summary bounds a chunk's `publish_time`s. An extent built
//! here therefore carries the unbounded publish span: a window on
//! [`TimeSource::Publish`] selects every chunk rather than silently dropping one
//! whose publish times the summary cannot vouch for, and the copy's own
//! per-message membership test still decides what lands in the clip.
//!
//! **What it covers.** Only chunk-indexed bytes are planned, so a message a
//! chunked recording wrote outside a chunk is not in any extent. The log bounds,
//! by contrast, are the recording's own ([`Statistics`](mcap::records::Statistics)
//! where the summary carries them), so [`WholeFileIndex::log_end_ns`] answers
//! whether the *recording* reached a window end — the question a clip's
//! `clip.short` key is about — rather than whether this index can plan it.
//!
//! **What it refuses.** A summary is not something every recording has, and the
//! ones that have one do not all carry an index worth planning from. Every
//! input this cannot index is refused by name — [`IndexRefusal`] is the whole
//! taxonomy, one variant per fault an operator repairs differently — and each
//! refusal names the `mcap` command that repairs it. Every verdict
//! is reached from the footer and the summary alone, so refusing a recording
//! costs the same seek and read that accepting one does, whatever its size, and
//! the input is opened read-only and left exactly as it was found: recovering
//! and re-indexing are the operator's, never this crate's.
//!
//! **A bag directory is one collection.** A recorder that ran for hours left a
//! directory of splits rather than a file, and [`WholeFileIndex::open`] takes
//! either. Each split is indexed on its own and satisfies the contract on its
//! own — a directory holding one that does not is refused naming *that
//! recording*, not the directory — and the splits are then planned in the order
//! [`crate::bag`] put them in, so a window straddling a rollover yields one
//! plan per contributing split and the shared cut path publishes one segment
//! each. The recorder's own per-topic counts, where its metadata file states
//! them, are cross-checked against what the splits' summaries add up to
//! ([`CountDisagreement`]): a collection short a split still cuts, and says
//! how much of it is missing.

use std::collections::{BTreeMap, HashMap};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use log::warn;
use mcap::sans_io::summary_reader::{SummaryReadEvent, SummaryReader, SummaryReaderOptions};

use crate::TimeSource;
use crate::bag::{BagError, METADATA_FILE};
use crate::index::{
    ChannelDef, Extent, MAGIC, RecordingIndex, SchemaDef, Span, Stamps, TimeBounds, WindowPlan,
    WindowPlanner,
};

/// The span a stamp the summary cannot bound gets: every instant, so an extent
/// carrying it is never excluded from a window on that clock.
const UNBOUNDED: Span = Span {
    min: 0,
    max: u64::MAX,
};

/// The fixed frame every MCAP file ends with: the footer record — its opcode,
/// its length prefix and its 20-byte body (`summary_start`,
/// `summary_offset_start`, `summary_crc`) — followed by the closing magic.
const FOOTER_FRAME_LEN: u64 = 1 + 8 + 20 + 8;

/// The shortest byte count an MCAP file can have: the opening magic and the
/// footer frame, with no records between them. Anything smaller cannot carry a
/// footer, so it is not an MCAP file.
const MIN_MCAP_LEN: u64 = MAGIC.len() as u64 + FOOTER_FRAME_LEN;

/// The repair every [`IndexRefusal`] names, spelled once because it is the same
/// three commands whichever fault the recording has.
///
/// `mcap recover` rewrites a recording into the shape this reads — chunked,
/// summarised, message-indexed — and is what turns a refused input into one
/// clipper cuts from; `mcap compress` writes the same shape where the copy
/// should also be smaller. `mcap list chunks` prints the per-chunk
/// `message index length` column that [`IndexRefusal::Unindexed`] reads, so an
/// operator sees the field the refusal is about rather than taking its word.
///
/// clipper runs none of them. It opens a recording read-only and never writes
/// to one, so recovering and re-indexing stay the operator's, with the repaired
/// copy under a name they chose.
const REPAIR: &str = "`mcap recover <in.mcap> -o <out.mcap>` rewrites it into \
     the chunked, summarised, message-indexed shape this reads, and `mcap \
     compress <in.mcap> -o <out.mcap>` writes that shape where the copy should \
     also be smaller; `mcap list chunks <out.mcap>` prints the per-chunk \
     `message index length` this reads. clipper never rewrites, recovers or \
     re-indexes a recording itself";

/// A recording clipper will not build an index out of, named by the fault.
///
/// Each variant is a different thing to do about it, which is why they are
/// separate: a truncated file wants recovering, an unchunked one wants
/// rewriting with chunks, a recording holding no message wants a different
/// recording. Every one of them is decided from the footer and the summary
/// section alone — no chunk is decompressed and no data byte is read — so a
/// refusal costs a seek and one read whatever the recording's size.
///
/// The variant set is the promise, so it is exhaustive: a caller matches every
/// way this can refuse, and a new one is a compile error at the match rather
/// than a message that reads differently.
#[derive(Debug, thiserror::Error)]
pub enum IndexRefusal {
    /// Shorter than the frame an MCAP file cannot be smaller than.
    #[error(
        "{} is {len} bytes: the magic-footer-magic frame an MCAP file cannot be \
         written without takes {MIN_MCAP_LEN} bytes on its own, so this is not \
         an MCAP recording. {REPAIR}",
        path.display()
    )]
    NotMcap { path: PathBuf, len: u64 },
    /// The last eight bytes are not the closing magic.
    #[error(
        "{} does not end with the MCAP magic: it is truncated, or was copied \
         off a device while it was still being written. {REPAIR}",
        path.display()
    )]
    Unfinalised { path: PathBuf },
    /// A well-formed footer whose `summary_start` is zero.
    #[error(
        "{}'s footer points at no summary section: the writer was configured \
         without one, so the recording carries no index to read. {REPAIR}",
        path.display()
    )]
    NoSummary { path: PathBuf },
    /// The summary's statistics report a message count of zero.
    #[error(
        "{} holds no message: its summary's statistics report a message count \
         of zero, and no window can be cut out of an empty recording. {REPAIR}",
        path.display()
    )]
    Empty { path: PathBuf },
    /// A summary with messages behind it and no chunk index at all.
    #[error(
        "{}'s summary indexes no chunk: the writer used an unchunked profile, \
         and an unchunked recording carries nothing to plan a window from. \
         {REPAIR}",
        path.display()
    )]
    Unchunked { path: PathBuf },
    /// Chunk indexes that address bytes but index no message.
    #[error(
        "none of {}'s {chunks} chunk indexes carries a message index: the \
         writer had message indexing disabled, so its summary indexes bytes \
         rather than messages. {REPAIR}",
        path.display()
    )]
    Unindexed { path: PathBuf, chunks: usize },
    /// A chunk index addressing bytes the file does not have, or declaring a
    /// record longer than [`crate::index::MAX_RECORD_LEN`].
    ///
    /// The length is a number the *recording* states, so it is bounded before
    /// the cut allocates a buffer from it: an unbounded one turns a corrupt
    /// summary into an allocation failure, which aborts the process instead of
    /// refusing the file.
    #[error(
        "{}'s summary indexes a chunk at offset {offset} of {len} bytes, which \
         the {file_len}-byte recording cannot hold: its summary does not \
         describe its own bytes. {REPAIR}",
        path.display()
    )]
    ChunkOutOfRange {
        path: PathBuf,
        offset: u64,
        len: u64,
        file_len: u64,
    },
}

/// Everything [`WholeFileIndex::open`] can fail with: the bag directory could
/// not be listed, a file could not be read, its tail is not MCAP, or it is MCAP
/// and clipper refuses to index it.
///
/// The four are separated because they are four different things to do: point
/// at the directory the splits are in, fix the path or the permissions, treat
/// the file as corrupt, or apply the repair the [`IndexRefusal`] names.
#[derive(Debug, thiserror::Error)]
pub enum OpenError {
    /// The input is a directory and is not a bag directory this can list.
    /// Its own message says which of those it is.
    #[error(transparent)]
    Bag(#[from] BagError),
    /// The file could not be opened, stat'd, seeked or read.
    #[error("cannot read {}", path.display())]
    Unreadable {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    /// The bytes are there and are not MCAP: a footer that is not a footer
    /// record, or a summary section that does not parse.
    #[error("the footer or summary section of {} does not parse as MCAP", path.display())]
    Unparsable {
        path: PathBuf,
        #[source]
        source: mcap::McapError,
    },
    /// The recording parses and holds no index clipper will plan a window from.
    #[error(transparent)]
    Refused(#[from] IndexRefusal),
}

/// Every read [`WholeFileIndex::open`] makes is a small one near the footer, and
/// they all fail the same way.
fn unreadable(path: &Path) -> impl Fn(std::io::Error) -> OpenError {
    move |source| OpenError::Unreadable {
        path: path.to_path_buf(),
        source,
    }
}

/// One topic whose message count over a bag directory's splits disagrees with
/// what the recorder's metadata file states for it.
///
/// Both numbers are counted rather than estimated — `stated` is the recorder's
/// own `topics_with_message_count`, `indexed` the sum of the splits' summary
/// statistics — so a disagreement means the collection is not the one the
/// recorder wrote: a split is missing, an extra one was dropped in beside them,
/// or one was rewritten. It is reported and not refused, because a collection
/// short a split still cuts every window its splits do cover, and only the
/// operator knows whether the missing part mattered.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CountDisagreement {
    /// The topic the two counts are about.
    pub topic: String,
    /// What the metadata file says the collection holds on it.
    pub stated: u64,
    /// What the splits actually present add up to.
    pub indexed: u64,
}

impl std::fmt::Display for CountDisagreement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let Self {
            topic,
            stated,
            indexed,
        } = self;
        write!(
            f,
            "{topic}: the recordings present hold {indexed} message(s), the \
             {METADATA_FILE} states {stated}"
        )
    }
}

/// One finished recording — or one bag directory of them — indexed from the
/// summaries and ready to cut windows out of.
///
/// [`Self::open`] is the whole construction; from there it is a
/// [`WindowPlanner`] like any other, so [`crate::segment::cut_window`] takes it
/// exactly as it takes a live tail. Nothing here waits: the recordings have an
/// end, so a window that reaches past it is short and stays short however long
/// a caller stands around.
///
/// The splits are held in recording order and planned in it, which is the order
/// a straddling window's segments are numbered in — so the collection's order
/// is a clip's segment order, and [`crate::bag`] owns the decision that sets it.
#[derive(Debug)]
pub struct WholeFileIndex {
    /// One index per recording, in the order [`crate::bag`] put them in. A
    /// single-file input is a collection of one.
    splits: Vec<RecordingIndex>,
    /// What the collection and the recorder's metadata file disagree about, in
    /// topic order; always empty for a single file and for a directory with no
    /// metadata file.
    disagreements: Vec<CountDisagreement>,
}

impl WholeFileIndex {
    /// Index what `path` names: one finished recording, or a bag directory read
    /// as one time-ordered collection of splits.
    ///
    /// Per recording this reads the footer and the summary section and nothing
    /// else — the data section is not walked and no chunk is decompressed, so
    /// the cost is independent of the recording's size and is not paid twice
    /// when the cut then reads the extents it was given. Files are opened
    /// read-only and never written to: a recording this refuses is left exactly
    /// as it was found, and repairing it is the operator's.
    ///
    /// Every recording this cannot index is refused by name, from the same two
    /// reads: [`IndexRefusal`] is the taxonomy, and the checks run in the order
    /// its variants are declared, since each one is what makes the next
    /// meaningful. The statistics get the first word over the chunk indexes —
    /// a chunked recording that holds no message indexes no chunk either, and
    /// [`IndexRefusal::Empty`] is the honest name for it, not
    /// [`IndexRefusal::Unchunked`].
    ///
    /// **Each split of a directory has to satisfy that contract on its own.**
    /// A collection is only as plannable as the recordings in it, and a window
    /// is cut out of one of them at a time, so a directory holding one this
    /// cannot index is refused naming that recording — the file an operator
    /// repairs or removes — rather than the directory they typed. The usual
    /// offender is the last split of a directory copied off a device while the
    /// recording was still being written, which is also the directory with no
    /// metadata file in it.
    pub fn open(path: &Path) -> Result<Self, OpenError> {
        if path.is_dir() {
            Self::open_dir(path)
        } else {
            Ok(WholeFileIndex {
                splits: vec![index_split(path)?.0],
                disagreements: Vec::new(),
            })
        }
    }

    /// Index every split of the bag directory at `dir`, in recording order, and
    /// cross-check the collection against the recorder's own account of it.
    ///
    /// The cross-check needs both sides whole: per-topic counts from the
    /// metadata file, and a message count from every split. A summary is
    /// allowed to carry no statistics record, and one split without it makes
    /// the sum an undercount rather than a disagreement; a metadata file that
    /// states no counts at all is no account rather than an account of zero. In
    /// either case the check is skipped entirely rather than reporting a
    /// shortfall that belongs to the format instead of to the collection.
    fn open_dir(dir: &Path) -> Result<Self, OpenError> {
        let bag = crate::bag::open(dir)?;
        let mut splits = Vec::with_capacity(bag.splits.len());
        let mut indexed: Option<BTreeMap<String, u64>> = Some(BTreeMap::new());
        for path in &bag.splits {
            let (index, counts) = index_split(path)?;
            splits.push(index);
            indexed = match (indexed, counts) {
                (Some(total), Some(counts)) => Some(sum_counts(total, counts)),
                _ => None,
            };
        }

        let disagreements = match (bag.metadata, indexed) {
            (Some(metadata), Some(indexed)) if !metadata.topic_counts.is_empty() => {
                cross_check(&metadata.topic_counts, &indexed)
            }
            _ => Vec::new(),
        };
        for disagreement in &disagreements {
            warn!(
                "{} does not hold what it says it does — {disagreement}",
                dir.display()
            );
        }

        Ok(WholeFileIndex {
            splits,
            disagreements,
        })
    }

    /// The recordings a window is planned over, in the order their segments come
    /// out — one path for a single file, one per split for a bag directory.
    #[must_use]
    pub fn splits(&self) -> Vec<&Path> {
        self.splits
            .iter()
            .map(|index| index.path.as_path())
            .collect()
    }

    /// What the collection and the recorder's metadata file disagree about, in
    /// topic order. Empty when they agree, when the directory carries no
    /// metadata file, and for a single recording.
    #[must_use]
    pub fn disagreements(&self) -> &[CountDisagreement] {
        &self.disagreements
    }

    /// The highest `log_time` the collection holds, or `None` when no recording
    /// in it holds a message.
    ///
    /// This is what says whether a window's end was ever reached: a finished
    /// recording that stops inside a window cuts a clip that is
    /// [`Short`](crate::manifest::WindowCoverage::Short), and only the recording
    /// itself can say so — the clip cannot show it from its own contents. Over a
    /// bag directory it is the last split's end, so a window reaching past the
    /// whole collection is what makes a clip short, not one reaching past the
    /// split it started in.
    #[must_use]
    pub fn log_end_ns(&self) -> Option<u64> {
        self.splits
            .iter()
            .filter(|index| index.bounds.has_messages)
            .map(|index| index.bounds.log.max)
            .max()
    }
}

/// The per-topic message counts one recording's summary states, or `None` for a
/// summary carrying no statistics record.
///
/// The statistics count every message in the file, chunked or not, per channel;
/// two channels sharing a topic — a schema that changed mid-recording — add up
/// under the one topic, which is how the recorder counted them too. A count
/// against a channel the summary's own registry does not carry leaves the
/// account short by that channel's messages, and an account that cannot be
/// completed is no account: `None` again, rather than a total that would
/// disagree with the recorder for a reason that is the file's.
fn topic_counts(summary: &mcap::Summary) -> Option<BTreeMap<String, u64>> {
    let stats = summary.stats.as_ref()?;
    let mut counts = BTreeMap::new();
    for (id, count) in &stats.channel_message_counts {
        *counts
            .entry(summary.channels.get(id)?.topic.clone())
            .or_default() += count;
    }
    Some(counts)
}

/// Every topic the two accounts of a collection disagree about, in topic order.
///
/// A topic only one side names is a disagreement too, counted as zero on the
/// side that does not name it: a topic the recorder counted messages on and the
/// recordings present hold none of is exactly the shape of a missing split.
fn cross_check(
    stated: &BTreeMap<String, u64>,
    indexed: &BTreeMap<String, u64>,
) -> Vec<CountDisagreement> {
    stated
        .keys()
        .chain(indexed.keys())
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .filter_map(|topic| {
            let stated = stated.get(topic).copied().unwrap_or(0);
            let indexed = indexed.get(topic).copied().unwrap_or(0);
            (stated != indexed).then(|| CountDisagreement {
                topic: topic.clone(),
                stated,
                indexed,
            })
        })
        .collect()
}

/// Fold one split's per-topic message counts into the collection's running
/// total. Topics absent from `total` start at zero, so a topic that appears
/// only in a later split still sums correctly.
fn sum_counts(
    mut total: BTreeMap<String, u64>,
    counts: BTreeMap<String, u64>,
) -> BTreeMap<String, u64> {
    for (topic, count) in counts {
        *total.entry(topic).or_default() += count;
    }
    total
}

/// Index one recording from its summary: the [`RecordingIndex`] a window is
/// planned over, and the per-topic counts that cross-check a collection.
///
/// The whole of what opening a recording costs, and the only place a refusal is
/// decided — a single file and one split of a directory go through exactly this,
/// so the contract is one contract and a refusal names the recording that
/// failed it either way.
fn index_split(path: &Path) -> Result<(RecordingIndex, Option<BTreeMap<String, u64>>), OpenError> {
    let file = File::open(path).map_err(unreadable(path))?;
    let end_of_scan = file.metadata().map_err(unreadable(path))?.len();

    if end_of_scan < MIN_MCAP_LEN {
        return Err(IndexRefusal::NotMcap {
            path: path.to_path_buf(),
            len: end_of_scan,
        }
        .into());
    }
    // The closing magic is what separates a finalised recording from one
    // still being written or truncated in transit, and it is eight bytes at
    // a known offset — cheaper and more specific than letting the summary
    // reader discover it as a parse failure.
    if !ends_with_magic(&file, end_of_scan).map_err(unreadable(path))? {
        return Err(IndexRefusal::Unfinalised {
            path: path.to_path_buf(),
        }
        .into());
    }
    let Some(summary) = read_summary(&file, end_of_scan, path)? else {
        return Err(IndexRefusal::NoSummary {
            path: path.to_path_buf(),
        }
        .into());
    };
    refuse_unplannable(path, &summary, end_of_scan)?;

    let counts = topic_counts(&summary);
    let mut index = RecordingIndex::new(path.to_path_buf(), Arc::new(file));
    // The magic is behind the summary this file just parsed, and there is
    // nothing left to scan: the cursor sits at the end of the file.
    index.magic_ok = true;
    index.offset = end_of_scan;
    index.schemas = schemas(&summary);
    index.channels = channels(&summary);
    index.extents = extents(&summary);
    index.bounds = bounds(&summary);
    Ok((index, counts))
}

impl WindowPlanner for WholeFileIndex {
    /// One plan per recording whose chunks overlap the window, in recording
    /// order; none at all when no chunk of any of them does.
    ///
    /// A single file therefore yields at most one plan, and a bag directory one
    /// per contributing split — the same shape the live tail serves out of its
    /// own collection, which is why the cut path needs to know neither which of
    /// the two it was handed nor how many recordings are behind it.
    fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
        self.splits
            .iter()
            .filter_map(|index| index.plan(start_ns, end_ns, source))
            .collect()
    }
}

/// Whether the file's last eight bytes are the MCAP closing magic — the mark a
/// writer lays down only once it has written the footer.
///
/// Reads those eight bytes and nothing else. The subtraction is sound because
/// the caller has already refused anything shorter than [`MIN_MCAP_LEN`].
fn ends_with_magic(mut file: &File, len: u64) -> std::io::Result<bool> {
    let mut tail = [0u8; MAGIC.len()];
    file.seek(SeekFrom::Start(len - MAGIC.len() as u64))?;
    file.read_exact(&mut tail)?;
    Ok(tail == MAGIC)
}

/// Drive the sans-io summary reader over `file`: it asks for a seek to the
/// footer and reads the summary section back, so the bytes this touches are the
/// tail of the file and nothing else. `Ok(None)` is a well-formed MCAP whose
/// footer points at no summary section.
///
/// The reader is told the file length, so it seeks straight to the footer and
/// bounds every record it is willing to read by what is left of the file.
///
/// Shared with [`crate::embedded`], which reads a finished recording's own
/// triggers out of the same summary: the two ask it different questions, and
/// neither should own a second copy of the read.
pub(crate) fn read_summary(
    mut file: &File,
    len: u64,
    path: &Path,
) -> Result<Option<mcap::Summary>, OpenError> {
    let mut reader =
        SummaryReader::new_with_options(SummaryReaderOptions::default().with_file_size(len));
    while let Some(event) = reader.next_event() {
        let event = event.map_err(|source| OpenError::Unparsable {
            path: path.to_path_buf(),
            source,
        })?;
        match event {
            SummaryReadEvent::ReadRequest(need) => {
                let read = file.read(reader.insert(need)).map_err(unreadable(path))?;
                reader.notify_read(read);
            }
            SummaryReadEvent::SeekRequest(to) => {
                reader.notify_seeked(file.seek(to).map_err(unreadable(path))?);
            }
        }
    }
    Ok(reader.finish())
}

/// The refusals the summary itself decides, in the order that makes each one
/// mean what it says.
///
/// The statistics are asked first: a recording holding no message has no chunk
/// to index, so testing the chunk indexes first would call every empty
/// recording unchunked and send an operator after a writer profile that is not
/// the problem. Only where the statistics vouch for messages — or say nothing,
/// the record being optional — does an absent chunk index mean an unchunked
/// writer, and only then does an indexed chunk that indexes no message mean a
/// writer with message indexing turned off.
fn refuse_unplannable(
    path: &Path,
    summary: &mcap::Summary,
    file_len: u64,
) -> Result<(), IndexRefusal> {
    if summary
        .stats
        .as_ref()
        .is_some_and(|stats| stats.message_count == 0)
    {
        return Err(IndexRefusal::Empty {
            path: path.to_path_buf(),
        });
    }
    if summary.chunk_indexes.is_empty() {
        return Err(IndexRefusal::Unchunked {
            path: path.to_path_buf(),
        });
    }
    // Every chunk, not any: a chunk carrying only schema and channel records
    // legitimately indexes no message, and it is a summary where *no* chunk
    // does that names a writer with message indexing disabled.
    if summary
        .chunk_indexes
        .iter()
        .all(|chunk| chunk.message_index_length == 0)
    {
        return Err(IndexRefusal::Unindexed {
            path: path.to_path_buf(),
            chunks: summary.chunk_indexes.len(),
        });
    }
    // Every extent's length reaches `vec![0u8; len]` in the cut, and both it
    // and the offset are numbers the recording states about itself. The scan
    // path bounds the same length as it walks (`index::MAX_RECORD_LEN`); this
    // is where the summary-built path bounds it, before anything allocates.
    for chunk in &summary.chunk_indexes {
        let out_of_range = chunk.chunk_length > crate::index::MAX_RECORD_LEN
            || chunk
                .chunk_start_offset
                .checked_add(chunk.chunk_length)
                .is_none_or(|end| end > file_len);
        if out_of_range {
            return Err(IndexRefusal::ChunkOutOfRange {
                path: path.to_path_buf(),
                offset: chunk.chunk_start_offset,
                len: chunk.chunk_length,
                file_len,
            });
        }
    }
    Ok(())
}

/// The summary's schema registry, owned.
fn schemas(summary: &mcap::Summary) -> HashMap<u16, SchemaDef> {
    summary
        .schemas
        .iter()
        .map(|(id, schema)| (*id, schema_def(schema)))
        .collect()
}

/// The summary's channel registry, owned, each channel's schema resolved — the
/// summary carries the resolution, so nothing has to look one up.
fn channels(summary: &mcap::Summary) -> HashMap<u16, ChannelDef> {
    summary
        .channels
        .iter()
        .map(|(id, channel)| {
            (
                *id,
                ChannelDef {
                    topic: channel.topic.clone(),
                    message_encoding: channel.message_encoding.clone(),
                    metadata: channel.metadata.clone(),
                    schema: channel.schema.as_ref().map(|s| schema_def(s)),
                },
            )
        })
        .collect()
}

fn schema_def(schema: &mcap::Schema<'_>) -> SchemaDef {
    SchemaDef {
        name: schema.name.clone(),
        encoding: schema.encoding.clone(),
        data: schema.data.to_vec(),
    }
}

/// One extent per chunk index, in file order.
///
/// A chunk index addresses the whole framed record — `chunk_start_offset` is the
/// opcode and `chunk_length` counts the opcode and length prefix in — which is
/// exactly the byte range [`crate::cut`] walks with its own framing, so an
/// extent here is one `Chunk` record and the copy needs no special case for
/// where it came from.
fn extents(summary: &mcap::Summary) -> Vec<Extent> {
    let mut extents: Vec<Extent> = summary
        .chunk_indexes
        .iter()
        .map(|chunk| Extent {
            offset: chunk.chunk_start_offset,
            len: chunk.chunk_length,
            time: Some(Stamps {
                log: Span {
                    min: chunk.message_start_time,
                    max: chunk.message_end_time,
                },
                publish: UNBOUNDED,
            }),
        })
        .collect();
    // A conformant writer emits the indexes in file order already; sorting makes
    // the plan's file order a property of this function rather than a hope about
    // the producer.
    extents.sort_by_key(|extent| extent.offset);
    extents
}

/// The recording's time bounds. `has_messages` is false for a recording that
/// holds none, which is what tells "no data" from "data stamped 0".
fn bounds(summary: &mcap::Summary) -> TimeBounds {
    let Some(log) = log_span(summary) else {
        return TimeBounds::default();
    };
    TimeBounds {
        log,
        // The summary states no publish time anywhere, so the recording's
        // publish span is unknown rather than empty.
        publish: UNBOUNDED,
        has_messages: true,
    }
}

/// The `log_time` span of the recording's messages, or `None` for a recording
/// that holds none.
///
/// The statistics record is the recording's own answer and covers every message
/// in the file, chunked or not; it is optional in the format, and a summary
/// without one is described by the union of its chunk index spans instead.
fn log_span(summary: &mcap::Summary) -> Option<Span> {
    match &summary.stats {
        Some(stats) if stats.message_count > 0 => Some(Span {
            min: stats.message_start_time,
            max: stats.message_end_time,
        }),
        Some(_) => None,
        None => summary
            .chunk_indexes
            .iter()
            .map(|chunk| Span {
                min: chunk.message_start_time,
                max: chunk.message_end_time,
            })
            .reduce(|mut acc, span| {
                acc.min = acc.min.min(span.min);
                acc.max = acc.max.max(span.max);
                acc
            }),
    }
}

#[cfg(test)]
impl WholeFileIndex {
    /// The one index behind a single-file input, for a test asserting on what
    /// the summary put in it. A collection has no "the" split, so this panics
    /// on one — the tests that hold a directory read [`Self::splits`] instead.
    fn only_split(&self) -> &RecordingIndex {
        match self.splits.as_slice() {
            [index] => index,
            splits => panic!("{} recordings, not one", splits.len()),
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::single_match_else,
        clippy::cast_possible_truncation,
        clippy::too_many_lines,
        reason = "a failed unwrap or a panicking index is a failing test, \
                  and a test that builds a fixture, drives it and asserts on the \
                  whole result is long, nested and argument-heavy by \
                  construction — splitting one would scatter the case it states"
    )]

    use std::collections::BTreeMap;
    use std::io::BufWriter;
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use anyhow::{Context, Result};

    use super::*;
    use crate::cut;
    use crate::manifest::{WindowCoverage, read_manifest};
    use crate::segment::{Publication, cut_window, spawn_stage_workers};
    use crate::select::{ChannelSelection, Spec};
    use crate::testing::{index_file, scan_to_end, test_dir, window_request, write_bag_metadata};
    use crate::trigger::now_ns;

    /// The clip compression the recorder defaults to, so these cuts write the
    /// clips an operator actually gets.
    const TEST_COMPRESSION: Option<mcap::Compression> = Some(mcap::Compression::Zstd);

    /// One message of a fixture recording, distinct from its neighbours in
    /// every field a copied clip is compared on.
    struct Msg {
        topic: &'static str,
        log_time: u64,
        publish_time: u64,
        sequence: u32,
        payload_len: usize,
    }

    /// What a clip holds, per message: the five fields a copy must carry
    /// through unchanged.
    type Copied = (String, u64, u64, u32, usize);

    /// The writer options a chunked fixture is written with. The chunk size is
    /// tiny, so the messages spread over several chunks and a window selects a
    /// subset of them — which is the whole point of indexing per chunk.
    ///
    /// A fixture that exists to exercise one writer setting starts here and
    /// changes that one setting, so the rest of its shape is the shape every
    /// other fixture has.
    fn chunked_opts() -> mcap::WriteOptions {
        mcap::WriteOptions::new()
            .use_chunks(true)
            .compression(TEST_COMPRESSION)
            .chunk_size(Some(128))
    }

    /// A finished recording holding `msgs`, written with `opts`.
    fn write_with(path: &Path, opts: mcap::WriteOptions, msgs: &[Msg]) -> Result<()> {
        let mut writer = opts.create(BufWriter::new(File::create(path)?))?;
        let mut ids: HashMap<&str, u16> = HashMap::new();
        for msg in msgs {
            let id = match ids.get(msg.topic) {
                Some(id) => *id,
                None => {
                    let schema =
                        writer.add_schema("std_msgs/msg/String", "ros2msg", b"string data")?;
                    let id = writer.add_channel(schema, msg.topic, "cdr", &BTreeMap::new())?;
                    ids.insert(msg.topic, id);
                    id
                }
            };
            writer.write_to_known_channel(
                &mcap::records::MessageHeader {
                    channel_id: id,
                    sequence: msg.sequence,
                    log_time: msg.log_time,
                    publish_time: msg.publish_time,
                },
                &vec![0xAB; msg.payload_len],
            )?;
        }
        writer.finish()?;
        Ok(())
    }

    /// A finished, chunked recording holding `msgs` — the shape every accepting
    /// test cuts from.
    fn write_chunked(path: &Path, msgs: &[Msg]) -> Result<()> {
        write_with(path, chunked_opts(), msgs)
    }

    /// Read a finished clip back as one [`Copied`] per message, in file order.
    fn read_messages(path: &Path) -> Result<Vec<Copied>> {
        let buf = std::fs::read(path)?;
        mcap::MessageStream::new(&buf)?
            .map(|msg| {
                let msg = msg?;
                Ok((
                    msg.channel.topic.clone(),
                    msg.log_time,
                    msg.publish_time,
                    msg.sequence,
                    msg.data.len(),
                ))
            })
            .collect()
    }

    /// The topics a finished clip declares, sorted — what the topic selection
    /// decided, read back off the clip rather than off the messages, so a clip
    /// that declared a channel it never wrote is not mistaken for one that
    /// excluded it.
    fn channels(path: &Path) -> Result<Vec<String>> {
        let buf = std::fs::read(path)?;
        let summary = mcap::Summary::read(&buf)?.expect("a finished clip has a summary");
        let mut topics: Vec<String> = summary.channels.values().map(|c| c.topic.clone()).collect();
        topics.sort();
        Ok(topics)
    }

    /// The device's planner over the same recording: the index a full
    /// incremental scan of the data section builds.
    struct Scanned(RecordingIndex);

    impl WindowPlanner for Scanned {
        fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
            self.0.plan(start_ns, end_ns, source).into_iter().collect()
        }
    }

    fn scanned(path: &Path) -> Result<Scanned> {
        let (mut index, file) = index_file(path)?;
        scan_to_end(&mut index, &file)?;
        Ok(Scanned(index))
    }

    /// Overwrite everything between the opening magic and the summary section
    /// with bytes that are neither valid framing nor decompressible, leaving the
    /// summary, the footer and the closing magic untouched.
    fn clobber_data_section(src: &Path, dst: &Path) -> Result<PathBuf> {
        let mut buf = std::fs::read(src)?;
        // The footer's `summary_start` field sits 28 bytes from the end: the
        // closing magic (8) behind `summary_crc` (4) and `summary_offset_start`
        // (8) behind the field itself (8).
        let at = buf.len() - 28;
        let summary_start = u64::from_le_bytes(buf[at..at + 8].try_into().unwrap()) as usize;
        buf[MAGIC.len()..summary_start].fill(0xFF);
        std::fs::write(dst, buf)?;
        Ok(dst.to_path_buf())
    }

    /// A clip cut from a summary-built index and one cut from a scanned index
    /// hold the same messages, field for field.
    ///
    /// This is the whole claim of the module: the summary is a faithful stand-in
    /// for the walk, so a clip cut in the cloud is the clip the device would
    /// have cut from the same recording and window. The comparison runs over
    /// topic, log time, publish time, sequence and payload length — every field
    /// a copy carries through — because agreeing on log time alone would pass
    /// for a copy that dropped a sequence number or mixed two channels up.
    #[test]
    fn a_summary_built_cut_matches_a_scanned_one_message_for_message() -> Result<()> {
        let root = test_dir("whole-agree")?;
        let rec = root.join("rec.mcap");
        write_chunked(
            &rec,
            &[
                Msg {
                    topic: "/a",
                    log_time: 100,
                    publish_time: 90,
                    sequence: 7,
                    payload_len: 40,
                },
                Msg {
                    topic: "/b",
                    log_time: 200,
                    publish_time: 250,
                    sequence: 1,
                    payload_len: 90,
                },
                Msg {
                    topic: "/a",
                    log_time: 300,
                    publish_time: 280,
                    sequence: 8,
                    payload_len: 70,
                },
                Msg {
                    topic: "/b",
                    log_time: 400,
                    publish_time: 410,
                    sequence: 2,
                    payload_len: 50,
                },
                Msg {
                    topic: "/a",
                    log_time: 500,
                    publish_time: 500,
                    sequence: 9,
                    payload_len: 60,
                },
            ],
        )?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = Arc::new(window_request(150, 450, TimeSource::Log));

        let from_summary = cut_window(
            &WholeFileIndex::open(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("summary.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        let from_scan = cut_window(
            &scanned(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("scan.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        let summary_msgs = read_messages(&from_summary[0].out_path)?;
        assert_eq!(
            summary_msgs,
            vec![
                ("/b".to_string(), 200, 250, 1, 90),
                ("/a".to_string(), 300, 280, 8, 70),
                ("/b".to_string(), 400, 410, 2, 50),
            ],
            "the window's three messages, each field carried through"
        );
        assert_eq!(
            summary_msgs,
            read_messages(&from_scan[0].out_path)?,
            "a summary-built cut and a scanned one are the same clip"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// One configuration cuts one channel set out of one recording, whichever
    /// index found the window.
    ///
    /// `clipper clip` plans over the summary-built index and `clipper tail` over
    /// a scanned one, but the selection rides in the staging pool both drive, so
    /// the clip a device cuts and the clip cut from the same recording
    /// afterwards declare the same channels — the property that lets a cloud cut
    /// stand in for a device one.
    #[test]
    fn one_selection_cuts_one_channel_set_from_either_index() -> Result<()> {
        let root = test_dir("whole-select")?;
        let rec = root.join("rec.mcap");
        write_chunked(
            &rec,
            &[
                Msg {
                    topic: "/camera/image_raw",
                    log_time: 100,
                    publish_time: 100,
                    sequence: 1,
                    payload_len: 40,
                },
                Msg {
                    topic: "/imu/data",
                    log_time: 200,
                    publish_time: 200,
                    sequence: 2,
                    payload_len: 50,
                },
                Msg {
                    topic: "/diagnostics",
                    log_time: 300,
                    publish_time: 300,
                    sequence: 3,
                    payload_len: 60,
                },
                Msg {
                    topic: "/camera/depth",
                    log_time: 400,
                    publish_time: 400,
                    sequence: 4,
                    payload_len: 70,
                },
            ],
        )?;

        let selection = ChannelSelection::try_from(Spec {
            include_regex: Some("^/camera/".to_string()),
            exclude: vec!["/camera/depth".to_string()],
            ..Spec::default()
        })?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, selection);
        let request = Arc::new(window_request(0, 1000, TimeSource::Log));

        let from_summary = cut_window(
            &WholeFileIndex::open(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("summary.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        let from_scan = cut_window(
            &scanned(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("scan.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        assert_eq!(
            channels(&from_summary[0].out_path)?,
            vec!["/camera/image_raw"],
            "the configuration's channel set, and only it"
        );
        assert_eq!(
            channels(&from_summary[0].out_path)?,
            channels(&from_scan[0].out_path)?,
            "a cloud cut and a device cut declare the same channels"
        );
        assert_eq!(
            read_messages(&from_summary[0].out_path)?,
            read_messages(&from_scan[0].out_path)?,
            "and hold the same messages"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Accepting a recording touches its summary and nothing else.
    ///
    /// The recording's whole data section is replaced with bytes that are
    /// neither valid record framing nor decompressible as a chunk, so any
    /// implementation that walked it or decompressed one chunk to build the
    /// registry, the extents or the bounds fails outright. Reading the summary
    /// alone yields exactly the index the intact file yields.
    #[test]
    fn accepting_a_recording_reads_only_its_summary() -> Result<()> {
        let root = test_dir("whole-summary-only")?;
        let rec = root.join("rec.mcap");
        write_chunked(
            &rec,
            &[
                Msg {
                    topic: "/a",
                    log_time: 100,
                    publish_time: 100,
                    sequence: 0,
                    payload_len: 64,
                },
                Msg {
                    topic: "/b",
                    log_time: 200,
                    publish_time: 200,
                    sequence: 1,
                    payload_len: 64,
                },
                Msg {
                    topic: "/a",
                    log_time: 300,
                    publish_time: 300,
                    sequence: 2,
                    payload_len: 64,
                },
            ],
        )?;
        let gutted = clobber_data_section(&rec, &root.join("gutted.mcap"))?;

        // The fixture is only worth anything if the data section really is
        // destroyed: a scan of it recovers no channel and reaches no message.
        let walked = scanned(&gutted)?;
        assert!(
            walked.0.channels.is_empty() && !walked.0.bounds.has_messages,
            "the gutted data section must carry nothing a walk can recover"
        );

        let intact = WholeFileIndex::open(&rec)?;
        let from_summary = WholeFileIndex::open(&gutted)?;

        assert_eq!(from_summary.log_end_ns(), Some(300));
        assert_eq!(from_summary.log_end_ns(), intact.log_end_ns());

        let topics = |idx: &WholeFileIndex| {
            let mut t: Vec<String> = idx
                .only_split()
                .channels
                .values()
                .map(|c| c.topic.clone())
                .collect();
            t.sort();
            t
        };
        assert_eq!(
            topics(&from_summary),
            vec!["/a".to_string(), "/b".to_string()]
        );
        assert_eq!(topics(&from_summary), topics(&intact));
        assert!(
            from_summary
                .only_split()
                .channels
                .values()
                .all(|c| c.schema.is_some()),
            "the summary carries each channel's schema resolved"
        );

        let ranges = |idx: &WholeFileIndex| -> Vec<(u64, u64)> {
            idx.only_split()
                .extents
                .iter()
                .map(|e| (e.offset, e.len))
                .collect()
        };
        assert!(
            !ranges(&intact).is_empty(),
            "a chunked recording has chunks"
        );
        assert_eq!(ranges(&from_summary), ranges(&intact));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A cut whose extents no longer hold what the index says publishes nothing:
    /// the copy fails, and the output directory is left without a clip rather
    /// than with a partial one.
    ///
    /// The gutted recording is the reachable way to provoke it here — the
    /// summary indexes chunks the data section no longer contains — and it is
    /// the same all-or-nothing publication every cut goes through, exercised
    /// from this path.
    #[test]
    fn a_failed_cut_publishes_no_clip() -> Result<()> {
        let root = test_dir("whole-fail")?;
        let rec = root.join("rec.mcap");
        write_chunked(
            &rec,
            &[Msg {
                topic: "/a",
                log_time: 100,
                publish_time: 100,
                sequence: 0,
                payload_len: 64,
            }],
        )?;
        let gutted = clobber_data_section(&rec, &root.join("gutted.mcap"))?;

        let out_dir = root.join("clipped");
        crate::cut::reset_capturing_dir(&out_dir)?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let err = cut_window(
            &WholeFileIndex::open(&gutted)?,
            &Arc::new(window_request(0, 1_000, TimeSource::Log)),
            WindowCoverage::Covered,
            &out_dir.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("framing inconsistent"),
            "the copy's own error reaches the caller: {err:#}"
        );

        let published: Vec<PathBuf> = std::fs::read_dir(&out_dir)?
            .map(|e| Ok(e?.path()))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .filter(|p| p.extension().is_some_and(|e| e == "mcap"))
            .collect();
        assert!(
            published.is_empty(),
            "a failed run leaves no clip in the output directory: {published:?}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A cut over a finished recording never waits.
    ///
    /// The window ends a minute in the wall-clock future, which is the wait the
    /// live path owes a window that may still be filling: it sleeps until the
    /// clock passes the window end and then until the tail's coverage reaches
    /// it. A finished recording has an end, so there is nothing to wait for and
    /// the cut runs at once — this returns in milliseconds where a wait-shaped
    /// path would return in a minute.
    #[test]
    fn a_cut_over_a_finished_recording_does_not_wait() -> Result<()> {
        let root = test_dir("whole-nowait")?;
        let rec = root.join("rec.mcap");
        let base = now_ns();
        write_chunked(
            &rec,
            &[
                Msg {
                    topic: "/a",
                    log_time: base,
                    publish_time: base,
                    sequence: 0,
                    payload_len: 64,
                },
                Msg {
                    topic: "/a",
                    log_time: base + 1_000,
                    publish_time: base + 1_000,
                    sequence: 1,
                    payload_len: 64,
                },
            ],
        )?;

        let index = WholeFileIndex::open(&rec)?;
        // A window ending a minute past the last recorded message — and a
        // minute past the wall clock. The live path parks for it; this one
        // cannot.
        let end_ns = base + 60_000_000_000;
        let request = Arc::new(window_request(base, end_ns, TimeSource::Log));
        assert_eq!(request.end_ns(), end_ns);
        assert!(
            index.log_end_ns().is_some_and(|end| end < end_ns),
            "the recording stops well inside the window"
        );

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let began = Instant::now();
        let stats = cut_window(
            &index,
            &request,
            WindowCoverage::Short,
            &root.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        let elapsed = began.elapsed();

        assert_eq!(
            stats[0].messages_copied, 2,
            "both messages are in the window"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "a cut over a finished recording must not sleep out the window: took {elapsed:?}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── the refusal taxonomy ───────────────────────────────────────────────

    /// One message, so a fixture that needs *some* data says so in one line.
    fn one_message() -> [Msg; 1] {
        [Msg {
            topic: "/a",
            log_time: 100,
            publish_time: 100,
            sequence: 0,
            payload_len: 64,
        }]
    }

    /// A chunked recording whose writer had message indexing turned off: its
    /// chunk indexes address bytes and index no message, which is what the
    /// `message-index-length` column of `mcap list chunks` shows as zero.
    fn write_unindexed(path: &Path, msgs: &[Msg]) -> Result<()> {
        write_with(path, chunked_opts().emit_message_indexes(false), msgs)
    }

    /// Copy `src` to `dst` with the footer's `summary_start` zeroed — a
    /// well-formed footer that points at no summary section, which is what a
    /// writer configured without one lays down.
    fn blank_summary_pointer(src: &Path, dst: &Path) -> Result<PathBuf> {
        let mut buf = std::fs::read(src)?;
        // The footer's `summary_start` field sits 28 bytes from the end: the
        // closing magic (8) behind `summary_crc` (4) and `summary_offset_start`
        // (8) behind the field itself (8).
        let at = buf.len() - 28;
        buf[at..at + 8].fill(0);
        std::fs::write(dst, &buf)?;
        Ok(dst.to_path_buf())
    }

    /// Open `path` and take the refusal it has to fail with, so a test asserts
    /// on the variant rather than on a message.
    fn refusal(path: &Path) -> IndexRefusal {
        match WholeFileIndex::open(path) {
            Err(OpenError::Refused(refusal)) => refusal,
            Err(other) => panic!("{} must be refused, not {other:?}", path.display()),
            Ok(_) => panic!("{} must not be indexable", path.display()),
        }
    }

    /// One instance of every [`IndexRefusal`] variant, each paired with the
    /// phrase its message has to carry.
    ///
    /// The `match` is exhaustive with no catch-all, so a variant added to the
    /// taxonomy is a compile error here until it states what it names.
    fn every_refusal(path: &Path) -> Vec<(IndexRefusal, &'static str)> {
        let at = || path.to_path_buf();
        [
            IndexRefusal::NotMcap { path: at(), len: 3 },
            IndexRefusal::Unfinalised { path: at() },
            IndexRefusal::NoSummary { path: at() },
            IndexRefusal::Empty { path: at() },
            IndexRefusal::Unchunked { path: at() },
            IndexRefusal::Unindexed {
                path: at(),
                chunks: 4,
            },
            IndexRefusal::ChunkOutOfRange {
                path: at(),
                offset: 41,
                len: 1 << 40,
                file_len: 4_096,
            },
        ]
        .into_iter()
        .map(|refusal| {
            let phrase = match &refusal {
                IndexRefusal::NotMcap { .. } => "not an MCAP recording",
                IndexRefusal::Unfinalised { .. } => "does not end with the MCAP magic",
                IndexRefusal::NoSummary { .. } => "points at no summary section",
                IndexRefusal::Empty { .. } => "holds no message",
                IndexRefusal::Unchunked { .. } => "indexes no chunk",
                IndexRefusal::Unindexed { .. } => "carries a message index",
                IndexRefusal::ChunkOutOfRange { .. } => "cannot hold",
            };
            (refusal, phrase)
        })
        .collect()
    }

    /// Every refusal names three things: the recording, the fault, and the
    /// commands that repair it.
    ///
    /// The messages are also all different from one another — the point of a
    /// taxonomy is that an operator reading one knows which of the six they
    /// have, and two variants sharing a sentence would hide that.
    #[test]
    fn every_refusal_names_the_recording_the_fault_and_the_repair() {
        let path = Path::new("/data/record/rosbag2_0.mcap");
        let refusals = every_refusal(path);
        for (refusal, phrase) in &refusals {
            let text = refusal.to_string();
            assert!(
                text.contains("/data/record/rosbag2_0.mcap"),
                "the refusal names the recording: {text}"
            );
            assert!(text.contains(phrase), "the refusal names the fault: {text}");
            for command in ["mcap recover", "mcap compress", "mcap list chunks"] {
                assert!(
                    text.contains(command),
                    "the refusal names `{command}`: {text}"
                );
            }
        }

        // The two variants carrying a number render it: a count no message
        // shows is a payload nobody can act on.
        assert!(
            IndexRefusal::NotMcap {
                path: path.to_path_buf(),
                len: 3,
            }
            .to_string()
            .contains("is 3 bytes"),
            "the size a too-small file has is in its message"
        );
        assert!(
            IndexRefusal::Unindexed {
                path: path.to_path_buf(),
                chunks: 4,
            }
            .to_string()
            .contains("4 chunk indexes"),
            "the number of unindexed chunks is in its message"
        );

        let mut messages: Vec<String> = refusals.iter().map(|(r, _)| r.to_string()).collect();
        messages.sort();
        messages.dedup();
        assert_eq!(
            messages.len(),
            refusals.len(),
            "each refusal says its own thing"
        );
    }

    /// The failures that carry a low-level cause say their own layer and let the
    /// cause say the rest, rather than printing the cause twice.
    ///
    /// `Refused` and `Bag` are transparent instead: their own message already
    /// names the input and the fault, so a layer above it would only repeat the
    /// path.
    #[test]
    fn a_failure_that_is_not_a_refusal_states_its_own_layer() {
        let source = std::io::Error::new(std::io::ErrorKind::PermissionDenied, "denied");
        let denied = source.to_string();
        let err = OpenError::Unreadable {
            path: PathBuf::from("/data/rec.mcap"),
            source,
        };
        let text = err.to_string();
        assert!(text.contains("/data/rec.mcap"), "{text}");
        assert!(
            !text.contains(&denied),
            "the cause prints itself through the source chain: {text}"
        );

        let err = OpenError::Unparsable {
            path: PathBuf::from("/data/rec.mcap"),
            source: mcap::McapError::BadFooter,
        };
        let text = err.to_string();
        assert!(text.contains("/data/rec.mcap"), "{text}");
        assert!(
            !text.contains(&mcap::McapError::BadFooter.to_string()),
            "the cause prints itself through the source chain: {text}"
        );
    }

    /// A file too short to hold a footer is not an MCAP recording at all.
    #[test]
    fn a_file_smaller_than_a_footer_is_not_an_mcap_recording() -> Result<()> {
        let root = test_dir("whole-tiny")?;
        let rec = root.join("rec.mcap");
        // The opening magic and nothing else: the eight bytes that make a file
        // look like an MCAP to anything that only checks the front.
        std::fs::write(&rec, MAGIC)?;

        let refused = refusal(&rec);
        let IndexRefusal::NotMcap { path, len } = &refused else {
            panic!("a {}-byte file is not an MCAP: {refused}", MAGIC.len())
        };
        assert_eq!(path, &rec);
        assert_eq!(*len, MAGIC.len() as u64);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording that never got its footer written — one still being written,
    /// or copied off a device mid-write — has no closing magic, and that is what
    /// it is told.
    #[test]
    fn a_recording_with_no_closing_magic_is_refused_as_unfinalised() -> Result<()> {
        let root = test_dir("whole-unfinished")?;
        let rec = root.join("rec.mcap");
        crate::testing::write_unfinished_recording(&rec, "/a", &[100, 200])?;

        let refused = refusal(&rec);
        let IndexRefusal::Unfinalised { path } = &refused else {
            panic!("a recording with no footer is unfinalised, not: {refused}")
        };
        assert_eq!(path, &rec);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A finalised recording whose footer names no summary section carries no
    /// index, and the writer that omitted it is what the refusal names.
    #[test]
    fn a_footer_pointing_at_no_summary_is_refused() -> Result<()> {
        let root = test_dir("whole-nosummary")?;
        let chunked = root.join("chunked.mcap");
        write_chunked(&chunked, &one_message())?;
        let rec = blank_summary_pointer(&chunked, &root.join("rec.mcap"))?;

        // The fixture is only worth anything if the file is otherwise intact:
        // the one it was copied from indexes fine.
        WholeFileIndex::open(&chunked)?;
        let refused = refusal(&rec);
        let IndexRefusal::NoSummary { path } = &refused else {
            panic!("a footer with no summary pointer is refused as such, not: {refused}")
        };
        assert_eq!(path, &rec);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording written without chunks has a summary that indexes nothing,
    /// and the unchunked writer profile is what the refusal names.
    #[test]
    fn an_unchunked_recording_is_refused_as_unchunked() -> Result<()> {
        let root = test_dir("whole-unchunked")?;
        let rec = root.join("rec.mcap");
        crate::testing::write_recording(&rec, false, &[("/a", 100), ("/a", 200)])?;

        let refused = refusal(&rec);
        let IndexRefusal::Unchunked { path } = &refused else {
            panic!("an unchunked recording is refused as unchunked, not: {refused}")
        };
        assert_eq!(path, &rec);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A chunked recording that holds no message is refused as empty, not as
    /// unchunked.
    ///
    /// It has no chunk index either — a writer emits a chunk when there is
    /// something to put in one — so the chunk indexes alone cannot tell the two
    /// apart, and reading them first would send an operator after a writer
    /// profile that is not the problem. The statistics record is what
    /// distinguishes them, and it gets the first word.
    #[test]
    fn a_chunked_recording_holding_no_messages_is_refused_as_empty() -> Result<()> {
        let root = test_dir("whole-empty")?;
        let rec = root.join("rec.mcap");
        write_chunked(&rec, &[])?;

        // The fixture has to be the confusable one: written chunked, and with
        // no chunk index for the taxonomy to read.
        let summary = read_summary(&File::open(&rec)?, std::fs::metadata(&rec)?.len(), &rec)?
            .expect("a finished recording carries a summary");
        assert!(
            summary.chunk_indexes.is_empty(),
            "a recording holding no message indexes no chunk either"
        );
        assert_eq!(
            summary.stats.as_ref().map(|stats| stats.message_count),
            Some(0),
            "the statistics are what say it is empty rather than unchunked"
        );

        let refused = refusal(&rec);
        let IndexRefusal::Empty { path } = &refused else {
            panic!("a recording holding no message is refused as empty, not: {refused}")
        };
        assert_eq!(path, &rec);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A chunked recording whose writer had message indexing disabled is
    /// refused as unindexed: its chunk indexes address bytes and index no
    /// message.
    #[test]
    fn chunk_indexes_with_no_message_index_are_refused() -> Result<()> {
        let root = test_dir("whole-unindexed")?;
        let rec = root.join("rec.mcap");
        write_unindexed(
            &rec,
            &[
                Msg {
                    topic: "/a",
                    log_time: 100,
                    publish_time: 100,
                    sequence: 0,
                    payload_len: 64,
                },
                Msg {
                    topic: "/a",
                    log_time: 200,
                    publish_time: 200,
                    sequence: 1,
                    payload_len: 64,
                },
            ],
        )?;

        // The fixture differs from an accepted recording in exactly one field —
        // the one `mcap list chunks` prints as `message-index-length`.
        let summary = read_summary(&File::open(&rec)?, std::fs::metadata(&rec)?.len(), &rec)?
            .expect("a finished recording carries a summary");
        assert!(
            !summary.chunk_indexes.is_empty(),
            "the recording is chunked, and its chunks are indexed by byte range"
        );
        assert!(
            summary
                .chunk_indexes
                .iter()
                .all(|chunk| chunk.message_index_length == 0),
            "no chunk carries a message index"
        );

        let refused = refusal(&rec);
        let IndexRefusal::Unindexed { path, chunks } = &refused else {
            panic!("a recording with no message indexes is refused as such, not: {refused}")
        };
        assert_eq!(path, &rec);
        assert_eq!(*chunks, summary.chunk_indexes.len());

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// One chunk index, with `message_index_length` the only field a test cares
    /// about.
    fn chunk_index(message_index_length: u64) -> mcap::records::ChunkIndex {
        mcap::records::ChunkIndex {
            message_start_time: 100,
            message_end_time: 200,
            chunk_start_offset: 41,
            chunk_length: 170,
            message_index_offsets: BTreeMap::new(),
            message_index_length,
            compression: "zstd".to_string(),
            compressed_size: 117,
            uncompressed_size: 166,
        }
    }

    /// A chunk index the file cannot back is refused rather than allocated
    /// from.
    ///
    /// `chunk_length` is a number the recording states about itself and the cut
    /// allocates a buffer of exactly that size, so an unbounded one turns a
    /// corrupt summary into an allocation failure — which aborts the process,
    /// and an abort is not a refusal anyone can act on. Both halves of the
    /// bound are checked: a length past `MAX_RECORD_LEN`, and one that fits the
    /// bound but not the file.
    #[test]
    fn a_chunk_index_the_file_cannot_back_is_refused() {
        let path = Path::new("/data/rec.mcap");
        let indexed = |offset: u64, len: u64| mcap::records::ChunkIndex {
            chunk_start_offset: offset,
            chunk_length: len,
            ..chunk_index(47)
        };

        let huge = mcap::Summary {
            chunk_indexes: vec![indexed(41, crate::index::MAX_RECORD_LEN + 1)],
            ..mcap::Summary::default()
        };
        assert!(
            matches!(
                refuse_unplannable(path, &huge, u64::MAX),
                Err(IndexRefusal::ChunkOutOfRange { .. })
            ),
            "a chunk longer than MAX_RECORD_LEN is refused even in a huge file"
        );

        let past_the_end = mcap::Summary {
            chunk_indexes: vec![indexed(41, 170)],
            ..mcap::Summary::default()
        };
        assert!(
            matches!(
                refuse_unplannable(path, &past_the_end, 200),
                Err(IndexRefusal::ChunkOutOfRange {
                    offset: 41,
                    len: 170,
                    file_len: 200,
                    ..
                })
            ),
            "a chunk running past the end of the file is refused"
        );
        assert!(
            refuse_unplannable(path, &past_the_end, 211).is_ok(),
            "the same chunk in a file that holds it is plannable"
        );

        let overflowing = mcap::Summary {
            chunk_indexes: vec![indexed(u64::MAX, 1)],
            ..mcap::Summary::default()
        };
        assert!(
            matches!(
                refuse_unplannable(path, &overflowing, u64::MAX),
                Err(IndexRefusal::ChunkOutOfRange { .. })
            ),
            "an offset+length that overflows is refused, not wrapped"
        );
    }

    /// A recording where one chunk indexes no message is not a recording whose
    /// writer had message indexing disabled.
    ///
    /// A chunk carrying only schema and channel records legitimately indexes no
    /// message, so it is a summary where *no* chunk does that names the writer
    /// setting. Refusing on the first unindexed chunk would refuse a recording
    /// that is perfectly plannable.
    #[test]
    fn one_unindexed_chunk_among_indexed_ones_is_not_a_refusal() {
        let path = Path::new("/data/rec.mcap");
        let mixed = mcap::Summary {
            chunk_indexes: vec![chunk_index(0), chunk_index(47)],
            ..mcap::Summary::default()
        };
        assert!(
            refuse_unplannable(path, &mixed, 4_096).is_ok(),
            "one unindexed chunk beside an indexed one is plannable"
        );

        let none = mcap::Summary {
            chunk_indexes: vec![chunk_index(0), chunk_index(0)],
            ..mcap::Summary::default()
        };
        assert!(
            matches!(
                refuse_unplannable(path, &none, 4_096),
                Err(IndexRefusal::Unindexed { chunks: 2, .. })
            ),
            "a summary where no chunk indexes a message is refused"
        );
    }

    /// A refusal is reached from the footer and the summary alone — no chunk is
    /// decompressed.
    ///
    /// Each fixture's whole data section is replaced with bytes that are neither
    /// valid record framing nor decompressible as a chunk, so an implementation
    /// that walked it — or opened one chunk to count what is inside — fails
    /// outright instead of naming the fault. The verdict is the intact
    /// recording's.
    #[test]
    fn a_refusal_reads_no_chunk() -> Result<()> {
        let root = test_dir("whole-refuse-summary")?;

        let unindexed = root.join("unindexed.mcap");
        write_unindexed(&unindexed, &one_message())?;
        let gutted_unindexed =
            clobber_data_section(&unindexed, &root.join("gutted-unindexed.mcap"))?;

        let unchunked = root.join("unchunked.mcap");
        crate::testing::write_recording(&unchunked, false, &[("/a", 100)])?;
        let gutted_unchunked =
            clobber_data_section(&unchunked, &root.join("gutted-unchunked.mcap"))?;

        assert!(
            matches!(refusal(&gutted_unindexed), IndexRefusal::Unindexed { .. }),
            "the message-index verdict comes from the summary"
        );
        assert!(
            matches!(refusal(&gutted_unchunked), IndexRefusal::Unchunked { .. }),
            "the chunk-index verdict comes from the summary"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A refused recording is left exactly as it was found, and nothing is
    /// written anywhere: clipper opens an input read-only and never rewrites,
    /// recovers or re-indexes one — [`REPAIR`] names the commands that do, and
    /// the operator runs them.
    #[test]
    fn a_refused_recording_is_left_untouched() -> Result<()> {
        let root = test_dir("whole-untouched")?;

        let tiny = root.join("tiny.mcap");
        std::fs::write(&tiny, MAGIC)?;
        let unfinalised = root.join("unfinalised.mcap");
        crate::testing::write_unfinished_recording(&unfinalised, "/a", &[100])?;
        let chunked = root.join("chunked.mcap");
        write_chunked(&chunked, &one_message())?;
        let no_summary = blank_summary_pointer(&chunked, &root.join("no-summary.mcap"))?;
        let empty = root.join("empty.mcap");
        write_chunked(&empty, &[])?;
        let unchunked = root.join("unchunked.mcap");
        crate::testing::write_recording(&unchunked, false, &[("/a", 100)])?;
        let unindexed = root.join("unindexed.mcap");
        write_unindexed(&unindexed, &one_message())?;

        let refused = [
            &tiny,
            &unfinalised,
            &no_summary,
            &empty,
            &unchunked,
            &unindexed,
        ];
        let before: Vec<(Vec<u8>, std::time::SystemTime)> = refused
            .iter()
            .map(|path| Ok((std::fs::read(path)?, std::fs::metadata(path)?.modified()?)))
            .collect::<Result<Vec<_>>>()?;

        let contents = |dir: &Path| -> Result<Vec<PathBuf>> {
            let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)?
                .map(|entry| Ok(entry?.path()))
                .collect::<Result<Vec<_>>>()?;
            paths.sort();
            Ok(paths)
        };
        let listing = contents(&root)?;

        for path in refused {
            let _ = refusal(path);
        }

        for (path, (bytes, modified)) in refused.iter().zip(before) {
            assert_eq!(
                std::fs::read(path)?,
                bytes,
                "{} was rewritten by a refusal",
                path.display()
            );
            assert_eq!(
                std::fs::metadata(path)?.modified()?,
                modified,
                "{} was touched by a refusal",
                path.display()
            );
        }
        assert_eq!(
            contents(&root)?,
            listing,
            "a refusal writes nothing, not even beside the recording"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The summary bounds no publish time, so a window on that clock selects
    /// every chunk rather than dropping one the summary cannot vouch for — the
    /// copy's own per-message test then decides what lands in the clip.
    #[test]
    fn a_publish_window_selects_every_chunk_and_the_copy_decides() -> Result<()> {
        let root = test_dir("whole-publish")?;
        let rec = root.join("rec.mcap");
        // Log times and publish times are far apart, so a publish window that
        // holds one message falls entirely outside every chunk's log span.
        write_chunked(
            &rec,
            &[
                Msg {
                    topic: "/a",
                    log_time: 100,
                    publish_time: 5_000,
                    sequence: 0,
                    payload_len: 64,
                },
                Msg {
                    topic: "/a",
                    log_time: 200,
                    publish_time: 9_000,
                    sequence: 1,
                    payload_len: 64,
                },
            ],
        )?;

        let index = WholeFileIndex::open(&rec)?;
        let plans = index.plan_window(4_500, 5_500, TimeSource::Publish);
        assert_eq!(plans.len(), 1);
        assert_eq!(
            plans[0].extents.len(),
            index.only_split().extents.len(),
            "every chunk is planned on a clock the summary does not bound"
        );

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let stats = cut_window(
            &index,
            &Arc::new(window_request(4_500, 5_500, TimeSource::Publish)),
            WindowCoverage::Covered,
            &root.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        assert_eq!(
            read_messages(&stats[0].out_path)?,
            vec![("/a".to_string(), 100, 5_000, 0, 64)],
            "only the message published inside the window is copied"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── a bag directory: one time-ordered collection ───────────────────────

    /// One message on `topic` at `log_time`, with every other field derived
    /// from it: a fixture that is about *when* a message is names only that.
    fn at(topic: &'static str, log_time: u64) -> Msg {
        Msg {
            topic,
            log_time,
            publish_time: log_time,
            sequence: (log_time / 100) as u32,
            payload_len: 40,
        }
    }

    /// One recording of a bag directory, written after a pause long enough that
    /// each call lands a strictly newer modification time — so a fixture that
    /// writes the splits out of order makes the filesystem's order disagree
    /// with the recorder's, which is the only way to see which one was used.
    fn write_split(dir: &Path, name: &str, msgs: &[Msg]) -> Result<PathBuf> {
        std::thread::sleep(Duration::from_millis(10));
        let path = dir.join(name);
        write_chunked(&path, msgs)?;
        Ok(path)
    }

    /// A fresh bag directory under `root`.
    fn bag_dir(root: &Path) -> Result<PathBuf> {
        let dir = root.join("bag");
        std::fs::create_dir_all(&dir)?;
        Ok(dir)
    }

    /// A published clip's manifest.
    fn manifest_of(clip: &Path) -> Result<BTreeMap<String, String>> {
        read_manifest(clip)?.context("every published clip carries a manifest")
    }

    /// The recording a published segment says its bytes came from.
    fn source_of(clip: &Path) -> Result<PathBuf> {
        Ok(PathBuf::from(
            manifest_of(clip)?
                .get("source.path")
                .context("a segment cut from a recording names it")?,
        ))
    }

    /// The published segments' filenames, in the order they were cut.
    fn segment_names(segments: &[cut::ClipStats]) -> Vec<String> {
        segments
            .iter()
            .map(|stats| {
                stats
                    .out_path
                    .file_name()
                    .unwrap_or_default()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect()
    }

    /// A window straddling a split is cut into one segment per contributing
    /// recording, and the segments together are the window.
    ///
    /// The claim is that a collection is not a different clip from the
    /// recording it was split out of: the same window over a single recording
    /// holding all six messages holds the same messages, field for field, and
    /// arrives in one file instead of two.
    #[test]
    fn a_window_straddling_a_split_is_one_segment_per_contributing_recording() -> Result<()> {
        let root = test_dir("whole-straddle")?;
        let bag = bag_dir(&root)?;
        let first = write_split(
            &bag,
            "bag_0.mcap",
            &[at("/a", 100), at("/a", 200), at("/a", 300)],
        )?;
        let second = write_split(
            &bag,
            "bag_1.mcap",
            &[at("/a", 400), at("/a", 500), at("/a", 600)],
        )?;
        write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 6)])?;

        let index = WholeFileIndex::open(&bag)?;
        assert_eq!(
            index.splits(),
            vec![first.as_path(), second.as_path()],
            "both recordings, in the order the recorder wrote them"
        );
        assert_eq!(
            index.log_end_ns(),
            Some(600),
            "the collection ends where its last recording does"
        );

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = Arc::new(window_request(250, 450, TimeSource::Log));
        let segments = cut_window(
            &index,
            &request,
            WindowCoverage::Covered,
            &root.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        assert_eq!(
            segment_names(&segments),
            vec!["clip_00.mcap", "clip_01.mcap"],
            "one segment per contributing recording, numbered in collection order"
        );
        assert_eq!(source_of(&segments[0].out_path)?, first);
        assert_eq!(source_of(&segments[1].out_path)?, second);

        let mut copied = Vec::new();
        for stats in &segments {
            copied.extend(read_messages(&stats.out_path)?);
        }
        assert_eq!(
            copied,
            vec![
                ("/a".to_string(), 300, 300, 3, 40),
                ("/a".to_string(), 400, 400, 4, 40),
            ],
            "every message in the window and no other, across the rollover"
        );

        // The same window out of one recording holding all six messages: the
        // collection is cut into the same clip, only in two files.
        let single_rec = root.join("single.mcap");
        write_chunked(
            &single_rec,
            &[
                at("/a", 100),
                at("/a", 200),
                at("/a", 300),
                at("/a", 400),
                at("/a", 500),
                at("/a", 600),
            ],
        )?;
        let single = cut_window(
            &WholeFileIndex::open(&single_rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("unsplit.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        assert_eq!(single.len(), 1, "one recording, one segment");
        assert_eq!(
            copied,
            read_messages(&single[0].out_path)?,
            "the segments concatenated are the clip the unsplit recording cuts"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording that contributes nothing to a window is dropped, and the
    /// segments left are numbered by where they land — not by which split they
    /// came from.
    ///
    /// The middle recording covers a camera outage: it holds messages, and the
    /// window selects its chunks, but none of what it holds is in the channel
    /// set this clip is cut from. So the collection plans three recordings and
    /// publishes two, and `_01` is the third recording rather than the second.
    #[test]
    fn a_recording_contributing_nothing_is_dropped_and_the_rest_renumber() -> Result<()> {
        let root = test_dir("whole-empty-middle")?;
        let bag = bag_dir(&root)?;
        let first = write_split(
            &bag,
            "bag_0.mcap",
            &[at("/camera/image", 100), at("/camera/image", 200)],
        )?;
        write_split(
            &bag,
            "bag_1.mcap",
            &[at("/diagnostics", 300), at("/diagnostics", 400)],
        )?;
        let third = write_split(
            &bag,
            "bag_2.mcap",
            &[at("/camera/image", 500), at("/camera/image", 600)],
        )?;
        write_bag_metadata(
            &bag,
            &["bag_0.mcap", "bag_1.mcap", "bag_2.mcap"],
            &[("/camera/image", 4), ("/diagnostics", 2)],
        )?;

        let selection = ChannelSelection::try_from(Spec {
            include_regex: Some("^/camera/".to_string()),
            ..Spec::default()
        })?;
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, selection);
        let segments = cut_window(
            &WholeFileIndex::open(&bag)?,
            &Arc::new(window_request(0, 1_000, TimeSource::Log)),
            WindowCoverage::Covered,
            &root.join("clip.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;

        assert_eq!(
            segment_names(&segments),
            vec!["clip_00.mcap", "clip_01.mcap"],
            "two segments out of three recordings, numbered from zero"
        );
        assert_eq!(
            source_of(&segments[0].out_path)?,
            first,
            "_00 is the first recording"
        );
        assert_eq!(
            source_of(&segments[1].out_path)?,
            third,
            "_01 is the third recording, not the second"
        );
        assert_eq!(
            manifest_of(&segments[0].out_path)?["source.files_planned"],
            "3",
            "the clip still states how many recordings the window was planned over"
        );
        assert_eq!(
            read_messages(&segments[0].out_path)?,
            vec![
                ("/camera/image".to_string(), 100, 100, 1, 40),
                ("/camera/image".to_string(), 200, 200, 2, 40),
            ],
        );
        assert_eq!(
            read_messages(&segments[1].out_path)?,
            vec![
                ("/camera/image".to_string(), 500, 500, 5, 40),
                ("/camera/image".to_string(), 600, 600, 6, 40),
            ],
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The recorder's metadata file states the collection's order; a directory
    /// without one is ordered by modification time.
    ///
    /// One fixture proves both, because its two recordings are written
    /// newest-first: the filesystem's order is the reverse of the recorder's,
    /// so the segments come out in one order with the metadata file present and
    /// the other with it gone.
    #[test]
    fn the_metadata_file_orders_a_collection_and_modification_time_orders_one_without_it()
    -> Result<()> {
        let root = test_dir("whole-order")?;
        let bag = bag_dir(&root)?;
        let second = write_split(&bag, "bag_1.mcap", &[at("/a", 400), at("/a", 500)])?;
        let first = write_split(&bag, "bag_0.mcap", &[at("/a", 100), at("/a", 200)])?;
        let metadata = write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 4)])?;

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION, ChannelSelection::default());
        let request = Arc::new(window_request(0, 1_000, TimeSource::Log));

        let stated = cut_window(
            &WholeFileIndex::open(&bag)?,
            &request,
            WindowCoverage::Covered,
            &root.join("stated.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        assert_eq!(
            [
                source_of(&stated[0].out_path)?,
                source_of(&stated[1].out_path)?
            ],
            [first.clone(), second.clone()],
            "the order the metadata file states, against the modification times"
        );

        std::fs::remove_file(&metadata)?;
        let by_mtime = cut_window(
            &WholeFileIndex::open(&bag)?,
            &request,
            WindowCoverage::Covered,
            &root.join("mtime.mcap"),
            Publication::Suffix,
            &stage_tx,
        )?;
        assert_eq!(
            [
                source_of(&by_mtime[0].out_path)?,
                source_of(&by_mtime[1].out_path)?
            ],
            [second, first],
            "with no metadata file, oldest first — the same two recordings, the \
             other way round"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The metadata file's per-topic counts are checked against what the
    /// recordings present actually hold, and every topic they disagree about is
    /// reported.
    ///
    /// The recorder counted two messages on `/b`; the directory holds one, which
    /// is what a collection short a split looks like from the inside. It is
    /// reported and not refused: the windows the recordings present do cover
    /// still cut.
    #[test]
    fn a_collections_topic_counts_are_cross_checked_against_the_metadata_file() -> Result<()> {
        let root = test_dir("whole-counts")?;
        let bag = bag_dir(&root)?;
        write_split(&bag, "bag_0.mcap", &[at("/a", 100), at("/b", 150)])?;
        write_split(&bag, "bag_1.mcap", &[at("/a", 200)])?;

        write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 2), ("/b", 2)])?;
        let short = WholeFileIndex::open(&bag)?;
        assert_eq!(
            short.disagreements().to_vec(),
            vec![CountDisagreement {
                topic: "/b".to_string(),
                stated: 2,
                indexed: 1,
            }],
            "the one topic the two accounts differ on, and only it"
        );
        let reported = short.disagreements()[0].to_string();
        for phrase in ["/b", "1 message(s)", "states 2", METADATA_FILE] {
            assert!(
                reported.contains(phrase),
                "the report names {phrase}: {reported}"
            );
        }

        write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 2), ("/b", 1)])?;
        assert!(
            WholeFileIndex::open(&bag)?.disagreements().is_empty(),
            "an account that adds up says nothing"
        );

        write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[])?;
        assert!(
            WholeFileIndex::open(&bag)?.disagreements().is_empty(),
            "a metadata file stating no counts is no account, not an account of zero"
        );

        std::fs::remove_file(bag.join(METADATA_FILE))?;
        assert!(
            WholeFileIndex::open(&bag)?.disagreements().is_empty(),
            "a directory with no metadata file has no account to disagree with"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Every recording of a collection satisfies the index contract on its own,
    /// and a directory holding one that does not is refused naming *that
    /// recording* — with nothing written anywhere.
    ///
    /// The fixture is the case this exists for: a bag directory copied off a
    /// device while it was still recording. The last split never got its
    /// footer, and the recorder never got as far as writing the metadata file,
    /// so its absence and the truncation arrive together.
    #[test]
    fn a_collection_holding_one_unindexable_recording_is_refused_naming_it() -> Result<()> {
        let root = test_dir("whole-bad-split")?;
        let bag = bag_dir(&root)?;
        let good = write_split(&bag, "bag_0.mcap", &one_message())?;
        std::thread::sleep(Duration::from_millis(10));
        let truncated = bag.join("bag_1.mcap");
        crate::testing::write_unfinished_recording(&truncated, "/a", &[200, 300])?;

        // The fixture is only worth anything if the other recording is fine:
        // the directory is refused for the one that is not.
        WholeFileIndex::open(&good)?;

        let listing = |dir: &Path| -> Result<Vec<PathBuf>> {
            let mut paths: Vec<PathBuf> = std::fs::read_dir(dir)?
                .map(|entry| Ok(entry?.path()))
                .collect::<Result<Vec<_>>>()?;
            paths.sort();
            Ok(paths)
        };
        let before = listing(&bag)?;

        let refused = refusal(&bag);
        let IndexRefusal::Unfinalised { path } = &refused else {
            panic!("a truncated split is refused as unfinalised, not: {refused}")
        };
        assert_eq!(
            path, &truncated,
            "the refusal names the recording that failed the contract"
        );
        let text = refused.to_string();
        assert!(
            text.contains("bag_1.mcap") && !text.contains("bag_0.mcap"),
            "the operator is told which file to repair: {text}"
        );
        assert_eq!(
            listing(&bag)?,
            before,
            "a refusal writes nothing, not even beside the recordings"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording's `log_time` span comes from its own statistics record where
    /// it has one — the record covers every message in the file, chunked or not,
    /// and outranks whatever the chunk indexes say. The format makes that record
    /// optional, so a producer writing none is described by the union of its
    /// chunk spans instead, and a statistics record stating zero messages means
    /// "holds none" rather than "holds one stamped 0".
    #[test]
    fn a_log_span_prefers_the_statistics_record_and_falls_back_to_the_chunks() {
        fn stats(message_count: u64, start: u64, end: u64) -> mcap::records::Statistics {
            mcap::records::Statistics {
                message_count,
                message_start_time: start,
                message_end_time: end,
                ..Default::default()
            }
        }
        fn chunk(start: u64, end: u64) -> mcap::records::ChunkIndex {
            mcap::records::ChunkIndex {
                message_start_time: start,
                message_end_time: end,
                chunk_start_offset: 0,
                chunk_length: 0,
                message_index_offsets: BTreeMap::new(),
                message_index_length: 0,
                compression: String::new(),
                compressed_size: 0,
                uncompressed_size: 0,
            }
        }

        let counted = mcap::Summary {
            stats: Some(stats(3, 100, 400)),
            chunk_indexes: vec![chunk(1, 2)],
            ..Default::default()
        };
        assert_eq!(
            log_span(&counted),
            Some(Span { min: 100, max: 400 }),
            "the statistics record is the recording's own answer about every \
             message in it, so the chunk spans do not get a vote"
        );

        let counted_empty = mcap::Summary {
            stats: Some(stats(0, 0, 0)),
            chunk_indexes: vec![chunk(1, 2)],
            ..Default::default()
        };
        assert_eq!(
            log_span(&counted_empty),
            None,
            "a recording stating zero messages holds none, not one stamped 0"
        );

        let uncounted = mcap::Summary {
            stats: None,
            chunk_indexes: vec![chunk(300, 400), chunk(100, 250)],
            ..Default::default()
        };
        assert_eq!(
            log_span(&uncounted),
            Some(Span { min: 100, max: 400 }),
            "with no statistics record the span is the union of the chunk spans, \
             whatever order the summary lists them in"
        );

        assert_eq!(
            log_span(&mcap::Summary::default()),
            None,
            "no statistics record and no chunks describe no messages at all"
        );
    }
}
