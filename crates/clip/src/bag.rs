//! A bag directory: the recordings one `ros2 bag record` run left behind, read
//! as one time-ordered collection.
//!
//! A recorder that runs for hours does not write one file. It rolls its output
//! over on a size or duration limit, so what an operator hands a cutter is a
//! directory — the splits side by side, plus the sidecar the recorder writes
//! when it shuts down. This module is the answer to "which recordings is that,
//! and in what order", and nothing more: it opens no MCAP, reads no summary and
//! decides nothing about whether a recording can be indexed. [`crate::whole`]
//! takes the ordered list from here and indexes each entry on its own.
//!
//! **Order comes from the recorder where the recorder said so.** The metadata
//! file's [`relative_file_paths`](BagMetadata::files) is the writer's own
//! account of which split it wrote first, and it is authoritative: a window
//! straddling a rollover is cut into one segment per contributing split, in
//! that order, so getting the order wrong reorders a clip's segments rather
//! than merely renaming them. Where the metadata file is absent, modification
//! time is the order — and its absence is itself a signal, since the recorder
//! writes it at shutdown: a directory without it was copied off a device while
//! the recording was still growing, which is also why its last split is the one
//! that usually fails the index contract.
//!
//! **What it states is checked, not trusted.** A file the metadata names that
//! is not in the directory, and a recording in the directory the metadata does
//! not name, are both said out loud; neither is fatal, because a collection
//! missing a split still cuts every window the splits present do cover. The
//! per-topic counts are the same kind of claim — [`crate::whole`] cross-checks
//! them against what the splits' own summaries add up to.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::SystemTime;

use log::warn;
use serde::Deserialize;

/// The sidecar `ros2 bag record` writes into the bag directory when it stops.
///
/// Public because its absence is a fact a caller reasons about: a directory
/// without it was taken while the recording was still being written.
pub const METADATA_FILE: &str = "metadata.yaml";

/// The extension a recording in a bag directory carries — the same rule the
/// live tail's directory watch applies, so both halves of the system agree on
/// what counts as a recording.
const MCAP_EXT: &str = "mcap";

/// A bag directory this cannot turn into an ordered list of recordings.
///
/// Each variant is a different thing to do about it: fix the path or the
/// permissions, point at the directory the splits are actually in, or repair
/// (or delete) a metadata file that no longer parses.
#[derive(Debug, thiserror::Error)]
pub enum BagError {
    /// The directory, or the metadata file in it, could not be read.
    #[error("cannot read {}", path.display())]
    Unreadable {
        path: PathBuf,
        #[source]
        source: std::io::Error,
    },
    /// A directory holding no `*.mcap` at all.
    #[error(
        "{} holds no *.{MCAP_EXT} recording: a bag directory is the directory \
         the splits themselves are in, not the directory above it",
        dir.display()
    )]
    NoRecordings { dir: PathBuf },
    /// A metadata file that is there and does not parse.
    #[error(
        "{} does not parse as a rosbag2 {METADATA_FILE}: it states the order \
         the splits were recorded in, which is the order a window straddling a \
         rollover is cut in, so a file that cannot be read is not quietly \
         replaced by the recordings' modification times — repair it, or delete \
         it to fall back to those",
        path.display()
    )]
    UnparsableMetadata {
        path: PathBuf,
        #[source]
        source: serde_norway::Error,
    },
}

/// The recorder's own account of a bag directory: which recordings it wrote, in
/// which order, and how many messages each topic got across all of them.
///
/// Both fields are claims about a collection, and both are checked rather than
/// believed — see the module docs.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BagMetadata {
    /// The recordings the run wrote, relative to the bag directory, in the
    /// order it wrote them.
    pub files: Vec<PathBuf>,
    /// Messages per topic over the whole collection, as the recorder counted
    /// them.
    pub topic_counts: BTreeMap<String, u64>,
}

/// One bag directory read: its recordings in the order they were written, and
/// the recorder's account of them where it left one.
#[derive(Debug)]
pub struct BagDir {
    /// Every `*.mcap` in the directory, in recording order, each a path under
    /// the directory it was found in.
    pub splits: Vec<PathBuf>,
    /// What [`METADATA_FILE`] states, or `None` for a directory without one.
    pub metadata: Option<BagMetadata>,
}

/// The document `metadata.yaml` holds, with only the two facts this reads
/// modelled.
///
/// Everything else in the file — the storage identifier, the durations, the
/// per-file starting times, the QoS profiles offered per topic — is ignored by
/// serde rather than described here, so a ROS distro that adds a key, or drops
/// one this does not name, still parses.
#[derive(Deserialize)]
struct MetadataFile {
    rosbag2_bagfile_information: BagfileInformation,
}

/// The one mapping under the document's single top-level key.
#[derive(Deserialize)]
struct BagfileInformation {
    /// The split order. Defaulted rather than required: a file that states no
    /// recordings is a metadata file that names nothing, which the ordering
    /// treats exactly as it treats a recording it did not name — not a parse
    /// failure.
    #[serde(default)]
    relative_file_paths: Vec<PathBuf>,
    /// The collection-wide per-topic counts, defaulted for the same reason: the
    /// counts are a cross-check, and losing them must not cost the split order
    /// that shares the file.
    #[serde(default)]
    topics_with_message_count: Vec<TopicEntry>,
}

/// One topic's entry in `topics_with_message_count`.
#[derive(Deserialize)]
struct TopicEntry {
    #[serde(default)]
    topic_metadata: TopicMetadata,
    #[serde(default)]
    message_count: u64,
}

/// The nested mapping naming the topic. Only its name is read; the type, the
/// serialization format and the QoS profiles are the recorder's business.
#[derive(Deserialize, Default)]
struct TopicMetadata {
    #[serde(default)]
    name: String,
}

/// What [`METADATA_FILE`] in `dir` states, or `None` for a directory without
/// one.
///
/// A directory with no metadata file is the ordinary copied-mid-recording case
/// and not a fault; one whose metadata file is unreadable or unparsable is.
pub fn read_metadata(dir: &Path) -> Result<Option<BagMetadata>, BagError> {
    let path = dir.join(METADATA_FILE);
    let text = match std::fs::read_to_string(&path) {
        Ok(text) => text,
        Err(source) if source.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(source) => return Err(BagError::Unreadable { path, source }),
    };
    parse_metadata(&text)
        .map(Some)
        .map_err(|source| BagError::UnparsableMetadata { path, source })
}

/// The pure half of [`read_metadata`]: YAML text to the two facts read out of
/// it.
///
/// A topic entry with no name is dropped rather than counted under the empty
/// string — an unnameable topic cross-checks against nothing.
fn parse_metadata(text: &str) -> Result<BagMetadata, serde_norway::Error> {
    let info = serde_norway::from_str::<MetadataFile>(text)?.rosbag2_bagfile_information;
    Ok(BagMetadata {
        files: info.relative_file_paths,
        topic_counts: info
            .topics_with_message_count
            .into_iter()
            .filter(|entry| !entry.topic_metadata.name.is_empty())
            .map(|entry| (entry.topic_metadata.name, entry.message_count))
            .collect(),
    })
}

/// One `*.mcap` found in the directory, with the modification time that orders
/// it when the metadata file does not.
#[derive(Debug)]
struct Split {
    path: PathBuf,
    modified: SystemTime,
}

/// Read `dir` as one bag: its recordings in the order they were written, and
/// the metadata that decided that order where there was one.
///
/// Touches the directory listing and the metadata file, and opens no recording:
/// whether each one can be indexed is [`crate::whole`]'s question, asked of each
/// split on its own.
pub fn open(dir: &Path) -> Result<BagDir, BagError> {
    let metadata = read_metadata(dir)?;
    let present = present_recordings(dir)?;
    if present.is_empty() {
        return Err(BagError::NoRecordings {
            dir: dir.to_path_buf(),
        });
    }
    let splits = recording_order(dir, metadata.as_ref().map(|m| m.files.as_slice()), present);
    Ok(BagDir { splits, metadata })
}

/// The recordings `path` names, in recording order: the single file it is, or
/// the splits of the bag directory it is.
///
/// The one place that answers "what did the operator point at", so a caller
/// reading a collection's triggers walks exactly the files a cut plans over,
/// in the same order.
pub fn splits(path: &Path) -> Result<Vec<PathBuf>, BagError> {
    if path.is_dir() {
        Ok(open(path)?.splits)
    } else {
        Ok(vec![path.to_path_buf()])
    }
}

/// Every `*.mcap` file directly in `dir`, with its modification time.
///
/// Subdirectories are skipped whatever they are named, and the listing does not
/// recurse: a bag directory holds its splits beside each other.
fn present_recordings(dir: &Path) -> Result<Vec<Split>, BagError> {
    let unreadable = |path: &Path| {
        let path = path.to_path_buf();
        move |source| BagError::Unreadable { path, source }
    };
    let mut splits = Vec::new();
    for entry in std::fs::read_dir(dir).map_err(unreadable(dir))? {
        let path = entry.map_err(unreadable(dir))?.path();
        if path.extension().is_none_or(|ext| ext != MCAP_EXT) {
            continue;
        }
        // Follows symlinks, as the live tail's watch does: a bag directory of
        // links to recordings elsewhere is still a bag directory.
        let meta = std::fs::metadata(&path).map_err(unreadable(&path))?;
        if !meta.is_file() {
            continue;
        }
        let modified = meta.modified().map_err(unreadable(&path))?;
        splits.push(Split { path, modified });
    }
    Ok(splits)
}

/// The order the recordings are read in: the ones the metadata names, in the
/// order it names them, then whatever it did not name, oldest first.
///
/// `named` is `None` for a directory with no metadata file — the
/// copied-mid-recording case — and modification time is then the whole order,
/// silently, because there is no claim to disagree with. It is `Some` for a
/// directory that has one, and every disagreement between what it names and
/// what is on disk is said out loud: a named recording that is missing, and a
/// recording present that it never named. Neither is fatal. A collection short
/// a split still cuts every window the splits present cover, and the per-topic
/// cross-check in [`crate::whole`] is what says how much is missing.
fn recording_order(dir: &Path, named: Option<&[PathBuf]>, mut present: Vec<Split>) -> Vec<PathBuf> {
    let mut ordered = Vec::with_capacity(present.len());
    for relative in named.unwrap_or_default() {
        let wanted = dir.join(relative);
        match present.iter().position(|split| split.path == wanted) {
            Some(at) => ordered.push(present.remove(at).path),
            None => warn!(
                "{}'s {METADATA_FILE} names {}, which is not in the directory; \
                 the collection is cut without it",
                dir.display(),
                wanted.display(),
            ),
        }
    }
    present.sort_by(|a, b| (a.modified, &a.path).cmp(&(b.modified, &b.path)));
    for split in present {
        if named.is_some() {
            warn!(
                "{}'s {METADATA_FILE} does not name {}; it is cut after every \
                 recording that file does name, ordered by modification time",
                dir.display(),
                split.path.display(),
            );
        }
        ordered.push(split.path);
    }
    ordered
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::assert_is_empty,
        reason = "a failed unwrap or a panicking index is a failing test, and \
                  `assert!(x.is_empty())` names the claim better than the \
                  empty-array `assert_eq!` the lint asks for"
    )]

    use std::fs::File;
    use std::io::Write;
    use std::time::Duration;

    use anyhow::Result;

    use super::*;
    use crate::testing::{bag_metadata, test_dir, write_bag_metadata};

    /// Write `<dir>/<name>` with one byte in it, after a pause long enough that
    /// each call lands a strictly newer modification time — the ordering the
    /// fallback reads.
    fn write_after(dir: &Path, name: &str) -> Result<PathBuf> {
        std::thread::sleep(Duration::from_millis(10));
        let path = dir.join(name);
        File::create(&path)?.write_all(b"x")?;
        Ok(path)
    }

    /// The two fields read out of a real-shaped metadata file, and nothing the
    /// rest of it can do about them.
    #[test]
    fn a_metadata_file_states_its_split_order_and_its_topic_counts() -> Result<()> {
        let parsed = parse_metadata(&bag_metadata(
            &["bag_0.mcap", "bag_1.mcap"],
            &[("/camera/image", 4), ("/imu/data", 2)],
        ))?;
        assert_eq!(
            parsed.files,
            vec![PathBuf::from("bag_0.mcap"), PathBuf::from("bag_1.mcap")],
            "the order the recorder wrote them in"
        );
        assert_eq!(
            parsed.topic_counts,
            BTreeMap::from([
                ("/camera/image".to_string(), 4),
                ("/imu/data".to_string(), 2),
            ]),
            "the collection-wide count per topic"
        );
        Ok(())
    }

    /// A metadata file stating neither of the two facts is still a metadata
    /// file: the fields default, and the ordering treats it as naming nothing.
    #[test]
    fn a_metadata_file_naming_nothing_parses_to_nothing() -> Result<()> {
        let parsed = parse_metadata("rosbag2_bagfile_information:\n  version: 9\n")?;
        assert!(parsed.files.is_empty());
        assert!(parsed.topic_counts.is_empty());
        Ok(())
    }

    /// The metadata file states the split order, whatever the modification
    /// times say — the whole reason it is read.
    #[test]
    fn the_metadata_file_orders_the_splits_against_their_mtimes() -> Result<()> {
        let root = test_dir("bag-metadata-order")?;
        // Written second-file-first, so modification time and the recorder's
        // account disagree and only one of them can be the answer.
        write_after(&root, "bag_1.mcap")?;
        write_after(&root, "bag_0.mcap")?;
        write_bag_metadata(&root, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 3)])?;

        let bag = open(&root)?;
        assert_eq!(
            bag.splits,
            vec![root.join("bag_0.mcap"), root.join("bag_1.mcap")],
            "the order the metadata file states"
        );
        assert_eq!(
            bag.metadata.expect("the directory has a metadata file"),
            BagMetadata {
                files: vec![PathBuf::from("bag_0.mcap"), PathBuf::from("bag_1.mcap")],
                topic_counts: BTreeMap::from([("/a".to_string(), 3)]),
            }
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// With no metadata file — a directory copied off a device mid-recording —
    /// modification time is the order.
    #[test]
    fn without_a_metadata_file_modification_time_is_the_order() -> Result<()> {
        let root = test_dir("bag-mtime-order")?;
        write_after(&root, "bag_1.mcap")?;
        write_after(&root, "bag_0.mcap")?;

        let bag = open(&root)?;
        assert!(
            bag.metadata.is_none(),
            "the recorder never got to write one"
        );
        assert_eq!(
            bag.splits,
            vec![root.join("bag_1.mcap"), root.join("bag_0.mcap")],
            "oldest first, whatever the names suggest"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording the metadata file does not name is cut last rather than
    /// dropped, and one it names that is not there costs nothing but a warning:
    /// a collection short a split still cuts the windows its splits cover.
    #[test]
    fn what_the_metadata_and_the_directory_disagree_on_is_kept() -> Result<()> {
        let root = test_dir("bag-disagree")?;
        write_after(&root, "bag_0.mcap")?;
        let stray = write_after(&root, "stray.mcap")?;
        write_bag_metadata(&root, &["bag_0.mcap", "bag_1.mcap"], &[("/a", 3)])?;

        let bag = open(&root)?;
        assert_eq!(
            bag.splits,
            vec![root.join("bag_0.mcap"), stray],
            "the named recording first, then the one it never named"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A directory with nothing to cut from is refused by name, and the message
    /// names the mistake that produces it.
    #[test]
    fn a_directory_holding_no_recording_is_refused() -> Result<()> {
        let root = test_dir("bag-empty")?;
        std::fs::create_dir_all(root.join("bag_0"))?;
        std::fs::write(root.join("notes.txt"), b"nothing here")?;

        let err = open(&root).expect_err("a directory with no recording cuts nothing");
        let BagError::NoRecordings { dir } = &err else {
            panic!("a directory holding no recording is refused as such, not: {err}")
        };
        assert_eq!(dir, &root);
        assert!(
            err.to_string().contains("not the directory above it"),
            "the message names the mistake: {err}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A directory whose splits are only in a subdirectory holds no recording:
    /// the listing does not recurse, and a directory named `*.mcap` is not one.
    #[test]
    fn the_listing_takes_the_directorys_own_files_only() -> Result<()> {
        let root = test_dir("bag-nonrecursive")?;
        std::fs::create_dir_all(root.join("inner"))?;
        write_after(&root.join("inner"), "bag_0.mcap")?;
        std::fs::create_dir_all(root.join("looks_like.mcap"))?;

        let err = open(&root).expect_err("nothing at the top level is nothing to cut");
        assert!(matches!(err, BagError::NoRecordings { .. }), "{err}");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A metadata file that is there and does not parse is a fault, not a
    /// silent fall back to modification time: it is the split order, and a
    /// collection cut in the wrong order is a wrong clip rather than a
    /// misnamed one.
    #[test]
    fn an_unparsable_metadata_file_is_refused_rather_than_ignored() -> Result<()> {
        let root = test_dir("bag-bad-metadata")?;
        write_after(&root, "bag_0.mcap")?;
        std::fs::write(root.join(METADATA_FILE), "rosbag2_bagfile_information: [\n")?;

        let err = open(&root).expect_err("a metadata file that lies is not read past");
        let BagError::UnparsableMetadata { path, .. } = &err else {
            panic!("an unparsable metadata file is refused as such, not: {err}")
        };
        assert_eq!(path, &root.join(METADATA_FILE));
        assert!(
            err.to_string().contains("delete it to fall back"),
            "the message names the repair: {err}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A path that is not a directory is the one recording it is, so a caller
    /// walking a collection walks a single file the same way.
    #[test]
    fn a_file_path_is_a_collection_of_one() -> Result<()> {
        let root = test_dir("bag-single")?;
        let rec = write_after(&root, "rec.mcap")?;
        assert_eq!(splits(&rec)?, vec![rec.clone()]);
        assert_eq!(splits(&root)?, vec![rec]);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
