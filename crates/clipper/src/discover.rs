//! Lazy directory watch that yields newly appeared `*.mcap` files in mtime
//! order, one per [`Iterator::next`].
//!
//! [`NewFileWatchIterator`] owns the "which recordings have appeared" concern
//! for the tail. On each `next()` it reads the directory and returns the
//! **oldest** (by mtime) `*.mcap` whose inode it has not yielded before. The
//! filesystem is touched only when `next()` is called; there is no background
//! thread or inotify watch.
//!
//! Files are tracked by `(dev, ino)` identity, not by a mtime cursor: a file
//! being written grows and its mtime advances, so a cursor would re-yield the
//! file under tail every poll. Inode identity yields each recording exactly
//! once however much it grows afterwards. Inodes no longer present in the
//! directory are forgotten on each pass — so the memory of seen files stays
//! bounded to what is on disk, and a filesystem that reuses a freed inode for a
//! later recording yields that recording rather than mistaking it for the old
//! one. mtime orders the unseen files (oldest first) so several appearing
//! between calls drain in creation order.
//!
//! The iterator is **not fused**: a `None` means "nothing newer right now", and
//! a later call yields a file as soon as it appears.

use std::collections::HashSet;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};

/// Yields `*.mcap` files under a directory in mtime order, one per `next()`,
/// each inode exactly once.
///
/// See the module docs for the identity/ordering contract.
pub struct NewFileWatchIterator {
    dir: PathBuf,
    /// `(dev, ino)` of files already yielded (and, for [`Self::seeded`], the
    /// files that existed at construction). Pruned each pass to the inodes still
    /// on disk.
    seen: HashSet<(u64, u64)>,
}

impl NewFileWatchIterator {
    /// Watch `dir`, yielding every `*.mcap` from the oldest onward — including
    /// any already present when the iterator is built.
    pub fn new(dir: impl Into<PathBuf>) -> Self {
        Self {
            dir: dir.into(),
            seen: HashSet::new(),
        }
    }

    /// Watch `dir`, treating every `*.mcap` present right now as already seen, so
    /// only files that appear later are yielded — the startup case, after the
    /// newest existing recording has been adopted directly and the backlog
    /// behind it is deliberately not indexed.
    pub fn seeded(dir: impl Into<PathBuf>) -> Self {
        let dir = dir.into();
        let mut seen = HashSet::new();
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for entry in entries.flatten() {
                if let Some(id) = mcap_id(&entry.path()) {
                    seen.insert(id);
                }
            }
        }
        Self { dir, seen }
    }
}

/// An unseen `*.mcap` candidate for one pass: its `(mtime, mtime_nsec)` (for
/// oldest-first ordering), its `(dev, ino)` identity, and its path.
type Candidate = ((i64, i64), (u64, u64), PathBuf);

/// The `(dev, ino)` of an `*.mcap` entry, or `None` if it is not an mcap file or
/// its metadata cannot be read this pass.
fn mcap_id(path: &Path) -> Option<(u64, u64)> {
    if path.extension().is_none_or(|e| e != "mcap") {
        return None;
    }
    let m = std::fs::metadata(path).ok()?;
    Some((m.dev(), m.ino()))
}

impl Iterator for NewFileWatchIterator {
    type Item = PathBuf;

    fn next(&mut self) -> Option<PathBuf> {
        // A missing/unreadable directory yields None and leaves `seen` untouched,
        // so a transient read error retries on a later call.
        let entries = std::fs::read_dir(&self.dir).ok()?;

        let mut present = HashSet::new();
        let mut candidates: Vec<Candidate> = Vec::new();
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_none_or(|e| e != "mcap") {
                continue;
            }
            let Ok(m) = std::fs::metadata(&path) else {
                continue;
            };
            let id = (m.dev(), m.ino());
            present.insert(id);
            if !self.seen.contains(&id) {
                candidates.push(((m.mtime(), m.mtime_nsec()), id, path));
            }
        }

        // Forget inodes no longer on disk: bounds `seen` to the files present and
        // lets a reused inode yield its new file rather than be mistaken for the
        // deleted one.
        self.seen.retain(|id| present.contains(id));

        // Oldest unseen file by mtime; record it seen so it is never re-yielded.
        let (_, id, path) = candidates.into_iter().min_by_key(|(mtime, _, _)| *mtime)?;
        self.seen.insert(id);
        Some(path)
    }
}

#[cfg(test)]
mod tests {
    use std::fs::File;
    use std::time::{Duration, SystemTime, UNIX_EPOCH};

    use anyhow::Result;

    use super::*;

    /// A fresh temp directory, mirroring `tail::tests::test_dir`.
    fn test_dir(name: &str) -> Result<PathBuf> {
        let nanos = SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos();
        let path = std::env::temp_dir().join(format!(
            "clipper-discover-{name}-{}-{nanos}",
            std::process::id()
        ));
        std::fs::create_dir_all(&path)?;
        Ok(path)
    }

    /// Create `<dir>/<name>` and return its path, sleeping first so each file
    /// lands a distinct, strictly-newer mtime (the ordering the iterator uses).
    /// The same 10 ms-sleep pattern the tail's `newest_mcap` tests use.
    fn touch_after(dir: &Path, name: &str) -> Result<PathBuf> {
        std::thread::sleep(Duration::from_millis(10));
        let path = dir.join(name);
        File::create(&path)?;
        Ok(path)
    }

    #[test]
    fn empty_or_missing_dir_yields_none() -> Result<()> {
        let root = test_dir("empty")?;
        assert_eq!(NewFileWatchIterator::new(root.join("missing")).next(), None);
        assert_eq!(NewFileWatchIterator::new(&root).next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn one_file_then_none() -> Result<()> {
        let root = test_dir("one")?;
        let a = touch_after(&root, "a.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn three_files_in_mtime_order_one_per_next() -> Result<()> {
        let root = test_dir("three")?;
        let a = touch_after(&root, "a.mcap")?;
        let b = touch_after(&root, "b.mcap")?;
        let c = touch_after(&root, "c.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));
        assert_eq!(it.next(), Some(b));
        assert_eq!(it.next(), Some(c));
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn non_mcap_is_ignored() -> Result<()> {
        let root = test_dir("ext")?;
        let _note = touch_after(&root, "note.txt")?;
        let a = touch_after(&root, "a.mcap")?;
        let _yaml = touch_after(&root, "metadata.yaml")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn files_added_between_calls_are_not_lost() -> Result<()> {
        let root = test_dir("between")?;
        let a = touch_after(&root, "a.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));

        // Two splits appear after the first drain.
        let b = touch_after(&root, "b.mcap")?;
        let c = touch_after(&root, "c.mcap")?;
        assert_eq!(it.next(), Some(b));
        assert_eq!(it.next(), Some(c));
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn several_at_once_all_drain() -> Result<()> {
        let root = test_dir("burst")?;
        let a = touch_after(&root, "a.mcap")?;
        let b = touch_after(&root, "b.mcap")?;
        let c = touch_after(&root, "c.mcap")?;
        // All three already on disk before the first next(): drained in order.
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));
        assert_eq!(it.next(), Some(b));
        assert_eq!(it.next(), Some(c));
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn not_fused_resumes_after_none() -> Result<()> {
        let root = test_dir("notfused")?;
        let a = touch_after(&root, "a.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a));
        assert_eq!(it.next(), None, "nothing newer right now");
        let b = touch_after(&root, "b.mcap")?;
        assert_eq!(it.next(), Some(b), "a later file is yielded after a None");
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn a_growing_file_is_not_re_yielded() -> Result<()> {
        // The regression this iterator exists to prevent: a file under tail
        // grows, advancing its mtime, but must be yielded only once — never
        // re-indexed as a phantom duplicate recording.
        let root = test_dir("grow")?;
        let a = touch_after(&root, "a.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a.clone()));

        // Append to `a`, bumping its mtime well past when it was yielded.
        for _ in 0..3 {
            std::thread::sleep(Duration::from_millis(10));
            use std::io::Write;
            let mut f = File::options().append(true).open(&a)?;
            f.write_all(b"more bytes")?;
            f.sync_all()?;
            assert_eq!(it.next(), None, "the growing file must not be re-yielded");
        }

        // A genuinely new file beside it is still yielded.
        let b = touch_after(&root, "b.mcap")?;
        assert_eq!(it.next(), Some(b));
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn seeded_skips_preexisting_and_yields_only_later_files() -> Result<()> {
        let root = test_dir("seeded")?;
        let _a = touch_after(&root, "a.mcap")?;
        let _b = touch_after(&root, "b.mcap")?;
        // Seeded past everything present: the backlog is skipped.
        let mut it = NewFileWatchIterator::seeded(&root);
        assert_eq!(it.next(), None, "pre-existing files are not yielded");

        let c = touch_after(&root, "c.mcap")?;
        assert_eq!(it.next(), Some(c), "a file appearing later is yielded");
        assert_eq!(it.next(), None);
        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    #[test]
    fn a_deleted_file_is_forgotten_so_a_reused_path_yields_again() -> Result<()> {
        // A new file at the same path (a fresh inode) after the first is gone
        // must be yielded — `seen` forgets inodes no longer on disk.
        let root = test_dir("reuse")?;
        let a = touch_after(&root, "rec.mcap")?;
        let mut it = NewFileWatchIterator::new(&root);
        assert_eq!(it.next(), Some(a.clone()));
        assert_eq!(it.next(), None);

        std::fs::remove_file(&a)?;
        // Drain the deletion (no candidate, prunes `seen`).
        assert_eq!(it.next(), None);
        let a2 = touch_after(&root, "rec.mcap")?;
        assert_eq!(it.next(), Some(a2), "a fresh inode at the same path is new");
        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
