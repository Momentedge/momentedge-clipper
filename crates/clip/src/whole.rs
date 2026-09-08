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
//! refusal names the `mcap` command that repairs it ([`REPAIR`]). Every verdict
//! is reached from the footer and the summary alone, so refusing a recording
//! costs the same seek and read that accepting one does, whatever its size, and
//! the input is opened read-only and left exactly as it was found: recovering
//! and re-indexing are the operator's, never this crate's.

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use mcap::sans_io::summary_reader::{SummaryReadEvent, SummaryReader, SummaryReaderOptions};

use crate::TimeSource;
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
}

/// Everything [`WholeFileIndex::open`] can fail with: the file could not be
/// read, its tail is not MCAP, or it is MCAP and clipper refuses to index it.
///
/// The three are separated because they are three different things to do: fix
/// the path or the permissions, treat the file as corrupt, or apply the repair
/// the [`IndexRefusal`] names.
#[derive(Debug, thiserror::Error)]
pub enum OpenError {
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
fn unreadable(path: &Path) -> impl Fn(std::io::Error) -> OpenError + use<'_> {
    move |source| OpenError::Unreadable {
        path: path.to_path_buf(),
        source,
    }
}

/// One finished recording, indexed from its own summary and ready to cut
/// windows out of.
///
/// [`Self::open`] is the whole construction; from there it is a
/// [`WindowPlanner`] like any other, so [`crate::segment::cut_window`] takes it
/// exactly as it takes a live tail. Nothing here waits: the recording has an
/// end, so a window that reaches past it is short and stays short however long
/// a caller stands around.
#[derive(Debug)]
pub struct WholeFileIndex {
    index: RecordingIndex,
}

impl WholeFileIndex {
    /// Index the finished recording at `path` from its summary.
    ///
    /// Reads the footer and the summary section and nothing else — the data
    /// section is not walked and no chunk is decompressed, so the cost is
    /// independent of the recording's size and is not paid twice when the cut
    /// then reads the extents it was given. The file is opened read-only and
    /// never written to: a recording this refuses is left exactly as it was
    /// found, and repairing it is the operator's ([`REPAIR`]).
    ///
    /// Every recording this cannot index is refused by name, from the same two
    /// reads: [`IndexRefusal`] is the taxonomy, and the checks run in the order
    /// its variants are declared, since each one is what makes the next
    /// meaningful. The statistics get the first word over the chunk indexes —
    /// a chunked recording that holds no message indexes no chunk either, and
    /// [`IndexRefusal::Empty`] is the honest name for it, not
    /// [`IndexRefusal::Unchunked`].
    pub fn open(path: &Path) -> Result<Self, OpenError> {
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
        refuse_unplannable(path, &summary)?;

        let mut index = RecordingIndex::new(path.to_path_buf(), Arc::new(file));
        // The magic is behind the summary this file just parsed, and there is
        // nothing left to scan: the cursor sits at the end of the file.
        index.magic_ok = true;
        index.offset = end_of_scan;
        index.schemas = schemas(&summary);
        index.channels = channels(&summary);
        index.extents = extents(&summary);
        index.bounds = bounds(&summary);
        Ok(WholeFileIndex { index })
    }

    /// The highest `log_time` the recording holds, or `None` for one that holds
    /// no message.
    ///
    /// This is what says whether a window's end was ever reached: a finished
    /// recording that stops inside a window cuts a clip that is
    /// [`Short`](crate::manifest::WindowCoverage::Short), and only the recording
    /// itself can say so — the clip cannot show it from its own contents.
    pub fn log_end_ns(&self) -> Option<u64> {
        self.index
            .bounds
            .has_messages
            .then_some(self.index.bounds.log.max)
    }
}

impl WindowPlanner for WholeFileIndex {
    /// The single plan over this recording, or none at all when no chunk's span
    /// overlaps the window. One file means at most one plan — the multi-plan
    /// shape of the seam belongs to a caller holding several recordings.
    fn plan_window(&self, start_ns: u64, end_ns: u64, source: TimeSource) -> Vec<WindowPlan> {
        self.index
            .plan(start_ns, end_ns, source)
            .into_iter()
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
fn read_summary(
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
fn refuse_unplannable(path: &Path, summary: &mcap::Summary) -> Result<(), IndexRefusal> {
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
mod tests {
    use std::collections::BTreeMap;
    use std::io::BufWriter;
    use std::path::PathBuf;
    use std::time::{Duration, Instant};

    use anyhow::Result;

    use super::*;
    use crate::manifest::WindowCoverage;
    use crate::segment::{cut_window, spawn_stage_workers};
    use crate::testing::{index_file, scan_to_end, test_dir, window_request};
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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let request = Arc::new(window_request(150, 450, TimeSource::Log));

        let from_summary = cut_window(
            &WholeFileIndex::open(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("summary.mcap"),
            &stage_tx,
        )?;
        let from_scan = cut_window(
            &scanned(&rec)?,
            &request,
            WindowCoverage::Covered,
            &root.join("scan.mcap"),
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
                .index
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
                .index
                .channels
                .values()
                .all(|c| c.schema.is_some()),
            "the summary carries each channel's schema resolved"
        );

        let ranges = |idx: &WholeFileIndex| -> Vec<(u64, u64)> {
            idx.index
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
        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let err = cut_window(
            &WholeFileIndex::open(&gutted)?,
            &Arc::new(window_request(0, 1_000, TimeSource::Log)),
            WindowCoverage::Covered,
            &out_dir.join("clip.mcap"),
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

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let began = Instant::now();
        let stats = cut_window(
            &index,
            &request,
            WindowCoverage::Short,
            &root.join("clip.mcap"),
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

    /// The two failures that are not refusals say their own layer and let the
    /// cause say the rest, rather than printing the cause twice.
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
            index.index.extents.len(),
            "every chunk is planned on a clock the summary does not bound"
        );

        let stage_tx = spawn_stage_workers(1, TEST_COMPRESSION);
        let stats = cut_window(
            &index,
            &Arc::new(window_request(4_500, 5_500, TimeSource::Publish)),
            WindowCoverage::Covered,
            &root.join("clip.mcap"),
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
}
