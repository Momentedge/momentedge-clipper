//! The triggers a finished recording carries, lifted out by the recording's own
//! summary.
//!
//! A recording made with `ros2 bag record --all` captures the trigger topic
//! along with everything else, so the file states the windows that were asked
//! for while it was being written. [`read_triggers`] hands them back — undecoded
//! [`TriggerRecord`]s, exactly the shape the tail's trigger tap emits on the
//! live path — so a caller cutting from a finished recording gets its trigger
//! list before the first cut and decodes each record the same way
//! ([`crate::decode::decode_trigger`]).
//!
//! **Only the chunks that hold the trigger channel are read.** A finalised MCAP
//! carries, per chunk, the offset of a message index for every channel with a
//! message in it. Naming the trigger channel therefore names its chunks, and the
//! read seeks to those and decompresses nothing else: the cost of finding the
//! triggers in a recording is set by how many of them there are, not by how
//! large the recording is. A recording that never carried the topic is answered
//! from the summary alone, with no chunk read at all.
//!
//! **What this does not see.** Chunk-indexed messages only, the same bytes
//! [`crate::whole`] plans a window over: a trigger a chunked recording wrote
//! outside a chunk is in no chunk index and is not found here. A writer that
//! emits chunks but no message indexes leaves the summary unable to say which
//! chunk holds what; the read then falls back to every chunk, still yielding the
//! trigger channel's messages and nothing else.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::path::Path;

use anyhow::{Context, Result, bail};
use mcap::sans_io::indexed_reader::{IndexedReadEvent, IndexedReader, IndexedReaderOptions};

use crate::index::MAX_RECORD_LEN;
use crate::trigger::TriggerRecord;
use crate::whole::read_summary;

/// The largest number of triggers one recording is read for.
///
/// Every trigger found is a clip a caller is about to cut, and every record is
/// held in memory until it is. A recording whose trigger topic runs into the
/// millions — a stuck publisher, or a topic that is not the trigger topic at all
/// — is a mistake worth refusing outright rather than answering with an
/// unbounded list and an afternoon of clips. The bound is far above any real
/// trigger stream: a legitimate trigger arrives when something happened.
pub const MAX_EMBEDDED_TRIGGERS: usize = 10_000;

/// Every message on `topic` the finished recording at `path` carries, in
/// `log_time` order, lifted out undecoded.
///
/// The records are raw: their `message_encoding` is the channel's, and their
/// bodies are the payload bytes as recorded. Turning one into a
/// [`Trigger`](crate::trigger::Trigger) is [`crate::decode::decode_trigger`]'s
/// job, and a caller that survives one undecodable trigger keeps that decision
/// where it can — this returns everything the recording holds on the topic and
/// judges none of it.
///
/// `Ok(vec![])` is the answer for a recording that carried no trigger at all:
/// finding nothing is not a fault, and a caller writes no clip and says so.
///
/// Fails on a recording this cannot read triggers out of at all: one with no
/// summary section (never finalised, or its tail lost), one whose bytes no
/// longer match what its summary says, and one holding more than
/// [`MAX_EMBEDDED_TRIGGERS`] of them.
pub fn read_triggers(path: &Path, topic: &str) -> Result<Vec<TriggerRecord>> {
    let mut file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let len = file
        .metadata()
        .with_context(|| format!("sizing {}", path.display()))?
        .len();
    let summary = read_summary(&file, len, path)
        .map_err(anyhow::Error::from)
        .and_then(|summary| summary.context("its footer names no summary section"))
        .with_context(|| {
            format!(
                "{} carries no summary to find its triggers in; a recording \
                 states what it holds only once it has been finalised",
                path.display()
            )
        })?;

    // A recording that never carried the topic holds no trigger, and answering
    // that here is what keeps the read selective: an empty channel set is no
    // filter at all to the indexed reader, which would then stream every chunk
    // in the file looking for messages that cannot be in it.
    if !summary
        .channels
        .values()
        .any(|channel| channel.topic == topic)
    {
        return Ok(Vec::new());
    }

    collect_triggers(&mut file, &summary, path, topic)
}

/// Drive the indexed reader over the trigger channel alone: service each chunk
/// it asks for out of `file`, and collect every message it yields.
///
/// Selectivity lives in the reader's options — naming the topic names the chunks
/// that hold it, so this touches the recording's data section only where the
/// triggers are. The [`MAX_EMBEDDED_TRIGGERS`] ceiling is checked as the messages
/// arrive rather than after, so a stuck publisher's recording is refused without
/// first being materialised.
fn collect_triggers(
    file: &mut File,
    summary: &mcap::Summary,
    path: &Path,
    topic: &str,
) -> Result<Vec<TriggerRecord>> {
    let mut reader = IndexedReader::new_with_options(
        summary,
        IndexedReaderOptions::new()
            .include_topics([topic])
            // The same record-length ceiling the scan applies, so a summary
            // claiming an absurd chunk is refused rather than allocated for.
            .with_record_length_limit(usize::try_from(MAX_RECORD_LEN).unwrap_or(usize::MAX)),
    )
    .with_context(|| format!("indexing the triggers in {}", path.display()))?;

    let mut chunk = Vec::new();
    let mut records: Vec<TriggerRecord> = Vec::new();
    while let Some(event) = reader.next_event() {
        match event.with_context(|| format!("reading the triggers in {}", path.display()))? {
            // The reader names one chunk it needs — by construction one holding
            // the trigger channel — and this is the only place the recording's
            // data section is touched.
            IndexedReadEvent::ReadChunkRequest { offset, length } => {
                file.seek(SeekFrom::Start(offset))
                    .with_context(|| format!("seeking to {offset} in {}", path.display()))?;
                chunk.resize(length, 0);
                file.read_exact(&mut chunk).with_context(|| {
                    format!("reading {length} bytes at {offset} of {}", path.display())
                })?;
                reader
                    .insert_chunk_record_data(offset, &chunk)
                    .with_context(|| {
                        format!("decompressing the chunk at {offset} of {}", path.display())
                    })?;
            }
            IndexedReadEvent::Message { header, data } => {
                if records.len() >= MAX_EMBEDDED_TRIGGERS {
                    bail!(
                        "{} holds more than {MAX_EMBEDDED_TRIGGERS} messages on {topic}; \
                         that is a stuck publisher or the wrong topic, not a trigger list",
                        path.display(),
                    );
                }
                let channel = summary.channels.get(&header.channel_id).with_context(|| {
                    format!(
                        "{} holds a message on channel {}, which its summary does not register",
                        path.display(),
                        header.channel_id,
                    )
                })?;
                records.push(TriggerRecord {
                    message_encoding: channel.message_encoding.clone(),
                    body: data.to_vec(),
                    log_time: header.log_time,
                    publish_time: header.publish_time,
                });
            }
        }
    }
    Ok(records)
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        reason = "a failed unwrap or a panicking index is a failing test"
    )]

    use super::*;
    use crate::decode::decode_trigger;
    use crate::testing::{FixtureMsg, clobber_chunks, test_dir, write_recording_with_triggers};
    use crate::trigger::{Stamp, Trigger};

    /// The topic the fixtures below carry their triggers on.
    const TRIGGER_TOPIC: &str = "/events/momentedge/trigger";

    /// A trigger fixture message: `name` at `log_time`, with a preroll and
    /// postroll distinct enough to tell one decoded trigger from another.
    fn trigger_at(log_time: u64, name: &str) -> FixtureMsg<'static> {
        FixtureMsg::Trigger {
            log_time,
            trigger: Trigger {
                name: name.to_string(),
                description: format!("{name} happened"),
                trigger_time: Stamp { sec: 0, nanosec: 0 },
                preroll: 100,
                postroll: 200,
            },
        }
    }

    /// A data fixture message on `/data` at `log_time`.
    fn data_at(log_time: u64) -> FixtureMsg<'static> {
        FixtureMsg::Data {
            topic: "/data",
            log_time,
        }
    }

    /// The `(name, log_time)` of each trigger a read lifted, decoded.
    fn decoded(records: &[TriggerRecord]) -> Vec<(String, u64)> {
        records
            .iter()
            .map(|rec| {
                let trigger = decode_trigger(&rec.message_encoding, &rec.body)
                    .expect("a fixture trigger decodes");
                (trigger.name, rec.log_time)
            })
            .collect()
    }

    /// A recording's own triggers come back whole, in log-time order, whatever
    /// chunk each of them sits in and whatever else shares that chunk.
    #[test]
    fn every_trigger_the_recording_carries_is_lifted_in_order() -> Result<()> {
        let root = test_dir("embedded-order")?;
        let rec = root.join("rec.mcap");
        write_recording_with_triggers(
            &rec,
            TRIGGER_TOPIC,
            &[
                &[data_at(100), trigger_at(150, "first"), data_at(200)],
                &[data_at(300), data_at(400)],
                &[trigger_at(500, "second"), trigger_at(600, "third")],
            ],
        )?;

        let records = read_triggers(&rec, TRIGGER_TOPIC)?;
        assert_eq!(
            decoded(&records),
            vec![
                ("first".to_string(), 150),
                ("second".to_string(), 500),
                ("third".to_string(), 600),
            ],
            "every trigger, in the order the recording stamped them"
        );
        assert!(
            records.iter().all(|rec| rec.message_encoding == "json"),
            "each record carries its channel's own wire encoding"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The claim this module exists for: the summary names the chunks holding
    /// the trigger channel, and no other chunk is read.
    ///
    /// The fixture confines the trigger channel to two of five chunks and then
    /// destroys the other three — their bytes are neither valid framing nor
    /// decompressible — so an implementation that streamed the file, or
    /// decompressed a chunk to find out what was in it, fails outright. Reading
    /// only the two named chunks yields exactly what the intact recording
    /// yields.
    #[test]
    fn only_the_chunks_the_summary_names_are_read() -> Result<()> {
        let root = test_dir("embedded-selective")?;
        let rec = root.join("rec.mcap");
        write_recording_with_triggers(
            &rec,
            TRIGGER_TOPIC,
            &[
                &[data_at(100)],
                &[trigger_at(200, "first"), data_at(250)],
                &[data_at(300)],
                &[data_at(400), trigger_at(450, "second")],
                &[data_at(500)],
            ],
        )?;
        let gutted = clobber_chunks(&rec, &root.join("gutted.mcap"), &[0, 2, 4])?;

        // The fixture is only worth anything if those chunks really are
        // destroyed. `/data` sits in all five, so asking for it is the same read
        // over the chunks the trigger topic is not in — and it fails.
        let err = read_triggers(&gutted, "/data").unwrap_err();
        assert!(
            format!("{err:#}").contains("decompressing the chunk"),
            "a read that must touch a gutted chunk fails: {err:#}"
        );

        assert_eq!(
            decoded(&read_triggers(&gutted, TRIGGER_TOPIC)?),
            vec![("first".to_string(), 200), ("second".to_string(), 450)],
            "the trigger channel's chunks are read, and only those"
        );
        assert_eq!(
            decoded(&read_triggers(&gutted, TRIGGER_TOPIC)?),
            decoded(&read_triggers(&rec, TRIGGER_TOPIC)?),
            "and the answer is the intact recording's"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording that never carried the trigger topic holds no trigger, and
    /// saying so costs no chunk: every chunk of the fixture is destroyed, and
    /// the answer still comes back from the summary.
    #[test]
    fn a_recording_without_the_topic_holds_no_trigger() -> Result<()> {
        let root = test_dir("embedded-none")?;
        let rec = root.join("rec.mcap");
        write_recording_with_triggers(
            &rec,
            TRIGGER_TOPIC,
            &[&[data_at(100)], &[data_at(200)], &[data_at(300)]],
        )?;
        let gutted = clobber_chunks(&rec, &root.join("gutted.mcap"), &[0, 1, 2])?;

        assert!(
            read_triggers(&gutted, TRIGGER_TOPIC)?.is_empty(),
            "no trigger topic, no triggers — and no chunk touched to find out"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording that was never finalised carries no summary, so nothing can
    /// say where its triggers are; the refusal names what is missing.
    #[test]
    fn a_recording_with_no_summary_is_refused() -> Result<()> {
        let root = test_dir("embedded-unfinished")?;
        let rec = root.join("rec.mcap");
        crate::testing::write_unfinished_recording(&rec, TRIGGER_TOPIC, &[100, 200])?;

        let err = read_triggers(&rec, TRIGGER_TOPIC).unwrap_err();
        assert!(
            format!("{err:#}").contains("carries no summary"),
            "the refusal names what is missing: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Each record carries both stamps the recording gave the trigger message,
    /// distinctly, since which of them anchors a clip window is the caller's
    /// choice and it can only make it if the two arrive apart.
    #[test]
    fn a_record_carries_the_recordings_own_stamps() -> Result<()> {
        let root = test_dir("embedded-stamps")?;
        let rec = root.join("rec.mcap");
        write_recording_with_triggers(&rec, TRIGGER_TOPIC, &[&[trigger_at(7_000, "evt")]])?;

        let records = read_triggers(&rec, TRIGGER_TOPIC)?;
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].log_time, 7_000);
        assert_eq!(
            records[0].publish_time,
            7_000 + crate::testing::FIXTURE_PUBLISH_SKEW_NS
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The ceiling is a refusal, not a truncation. A recording carrying more
    /// messages on the trigger topic than any real trigger list has is a stuck
    /// publisher or the wrong topic, and answering with the first
    /// [`MAX_EMBEDDED_TRIGGERS`] of them would turn that into a plausible-looking
    /// cut list instead of an error.
    #[test]
    fn a_recording_past_the_trigger_ceiling_is_refused() -> Result<()> {
        let root = test_dir("embedded-ceiling")?;
        let rec = root.join("rec.mcap");
        // One past the ceiling: the check runs before each push, so the
        // `MAX_EMBEDDED_TRIGGERS`-th message is still accepted and the next is
        // what refuses the recording.
        let flood: Vec<FixtureMsg<'static>> = (0..=MAX_EMBEDDED_TRIGGERS)
            .map(|i| trigger_at(i as u64 + 1, "flood"))
            .collect();
        write_recording_with_triggers(&rec, TRIGGER_TOPIC, &[&flood])?;

        let err = read_triggers(&rec, TRIGGER_TOPIC)
            .expect_err("a recording past the ceiling is refused, not truncated");
        let rendered = format!("{err:#}");
        assert!(
            rendered.contains(&MAX_EMBEDDED_TRIGGERS.to_string())
                && rendered.contains(TRIGGER_TOPIC),
            "the refusal names the ceiling it hit and the topic it counted on: {rendered}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
