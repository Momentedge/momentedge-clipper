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

use std::collections::HashMap;
use std::fs::File;
use std::io::{Read, Seek};
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, bail};
use mcap::sans_io::summary_reader::{SummaryReadEvent, SummaryReader};

use crate::TimeSource;
use crate::index::{
    ChannelDef, Extent, RecordingIndex, SchemaDef, Span, Stamps, TimeBounds, WindowPlan,
    WindowPlanner,
};

/// The span a stamp the summary cannot bound gets: every instant, so an extent
/// carrying it is never excluded from a window on that clock.
const UNBOUNDED: Span = Span {
    min: 0,
    max: u64::MAX,
};

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
    /// then reads the extents it was given.
    ///
    /// Fails on a recording this cannot index at all: one with no summary
    /// section (never finalised, or its tail lost), and one whose summary
    /// reports messages but indexes no chunk — a plan built from it would be
    /// empty, and an empty clip is the wrong answer to "this recording holds
    /// your window".
    pub fn open(path: &Path) -> Result<Self> {
        let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
        let end_of_scan = file
            .metadata()
            .with_context(|| format!("stat of {}", path.display()))?
            .len();
        // Two shapes reach the same verdict, and the operator is told which:
        // a file whose tail is not a footer at all (one still being written, or
        // one truncated) fails the parse, and a finalised file whose footer
        // points at no summary parses to nothing.
        let summary = read_summary(&file)
            .and_then(|summary| summary.context("its footer names no summary section"))
            .with_context(|| {
                format!(
                    "{} carries no summary to index from; a recording is \
                     indexable this way only once it has been finalised",
                    path.display()
                )
            })?;

        if summary.chunk_indexes.is_empty()
            && let Some(messages) = summary
                .stats
                .as_ref()
                .map(|stats| stats.message_count)
                .filter(|count| *count > 0)
        {
            bail!(
                "{} indexes no chunk but its summary reports {messages} messages; \
                 nothing of it can be planned",
                path.display(),
            );
        }

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

/// Drive the sans-io summary reader over `file`: it asks for a seek to the
/// footer and reads the summary section back, so the bytes this touches are the
/// tail of the file and nothing else. `Ok(None)` is a well-formed MCAP with no
/// summary section.
fn read_summary(mut file: &File) -> Result<Option<mcap::Summary>> {
    let mut reader = SummaryReader::new();
    while let Some(event) = reader.next_event() {
        match event.context("parsing the summary section")? {
            SummaryReadEvent::ReadRequest(need) => {
                let read = file.read(reader.insert(need)).context("reading")?;
                reader.notify_read(read);
            }
            SummaryReadEvent::SeekRequest(to) => {
                reader.notify_seeked(file.seek(to).context("seeking")?);
            }
        }
    }
    Ok(reader.finish())
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

    use super::*;
    use crate::index::MAGIC;
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

    /// A finished, chunked recording holding `msgs`. The chunk size is tiny, so
    /// the messages spread over several chunks and a window selects a subset of
    /// them — which is the whole point of indexing per chunk.
    fn write_chunked(path: &Path, msgs: &[Msg]) -> Result<()> {
        let mut writer = mcap::WriteOptions::new()
            .use_chunks(true)
            .compression(TEST_COMPRESSION)
            .chunk_size(Some(128))
            .create(BufWriter::new(File::create(path)?))?;
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

    /// A recording that was never finalised carries no summary, and there is
    /// nothing to index it from.
    #[test]
    fn a_recording_with_no_summary_is_refused() -> Result<()> {
        let root = test_dir("whole-unfinished")?;
        let rec = root.join("rec.mcap");
        crate::testing::write_unfinished_recording(&rec, "/a", &[100, 200])?;

        let err = WholeFileIndex::open(&rec).unwrap_err();
        assert!(
            format!("{err:#}").contains("carries no summary to index from"),
            "the refusal names what is missing: {err:#}"
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
