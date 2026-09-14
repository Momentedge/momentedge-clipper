//! What a clip is on disk: a **directory** named by the clip's
//! [id](crate::id), holding one `<id>_N.mcap` per contributing source recording
//! and the [`METADATA_FILE`] that says it is complete.
//!
//! ```text
//! <out-dir>/
//!   1726300000000000000_3fa9-c1b2-77e0-0d4c/
//!     1726300000000000000_3fa9-c1b2-77e0-0d4c_0.mcap
//!     1726300000000000000_3fa9-c1b2-77e0-0d4c_1.mcap   # the window straddled a split
//!     clip_metadata.yaml                                # written last
//! ```
//!
//! Three rules are the whole contract, and each is a single filesystem
//! operation rather than a protocol:
//!
//! - **The directory is the claim.** `ClipDir::claim` is one `mkdir` with no
//!   parents, which the kernel makes atomic: exactly one caller can create a
//!   given path, and everyone else sees `AlreadyExists`. That settles a repeated
//!   trigger, two concurrent windows with one id, and two processes writing into
//!   one output directory, with no lock file and no coordination — the loser
//!   never had the directory and writes nothing.
//! - **The metadata file is the completion signal.** It is written after every
//!   MCAP file in the directory is durable, and the directory is fsynced after
//!   it, so a directory without it is incomplete by definition. An upload
//!   pipeline filters on that one fact; it never has to know the naming scheme
//!   or guess at grouping.
//! - **A failed cut takes its directory with it** (`ClipDir::discard`), so a
//!   directory without the metadata file is crash residue and nothing else. A
//!   crash does leave one, and a later window with that id skips it rather than
//!   repairing it: the residue is the evidence that something died, and
//!   overwriting it would destroy the only trace.
//!
//! **Nothing is ever written into the root of the output directory except clip
//! directories** — no staging area, no lock, no sidecar — so an operator can
//! point a sync tool at it with no exclude list. The output directory itself is
//! created with parents when missing ([`prepare_out_dir`]), never required
//! empty, and never cleared.

use std::fs::File;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, Result};

use crate::id::ClipId;
use crate::manifest::ClipMetadata;

/// The name of the document a complete clip carries beside its MCAP files.
///
/// **Its presence is what "complete" means.** It is written last, after every
/// file in the directory is durable, so a consumer syncing or uploading clips
/// filters on this one name and never sees a half-written clip. The contents are
/// [`ClipMetadata`]; [`read_metadata`] reads one back.
pub const METADATA_FILE: &str = "clip_metadata.yaml";

/// The extension an MCAP file of a clip carries while it is still being written.
///
/// A file's number is the position it ends up at *after* the empty ones are
/// dropped, which is known only once every copy has run — so each copy writes
/// under a name derived from its plan's position and is renamed into place at
/// the end. The suffix keeps those two name spaces apart, and nothing outside
/// the cut ever sees one: the directory has no metadata file while a `.part`
/// exists in it, so it is incomplete either way.
const STAGING_EXT: &str = "part";

/// Where a clip's [`METADATA_FILE`] sits inside its directory.
///
/// The one place the file's location is spelled, so a consumer testing a clip
/// for completeness and the cut that writes it cannot disagree.
#[must_use]
pub fn metadata_path(clip_dir: &Path) -> PathBuf {
    clip_dir.join(METADATA_FILE)
}

/// Make sure the output directory exists, creating it **with parents**.
///
/// Called once at startup by each subcommand — so a run that cuts nothing still
/// leaves the directory it was pointed at — and again before every claim, since
/// a claim is one `mkdir` with no parents and has nothing to create the tree
/// under. It never clears anything and never requires the directory to be empty:
/// foreign files in it are left exactly as they were found.
pub fn prepare_out_dir(out_dir: &Path) -> Result<()> {
    std::fs::create_dir_all(out_dir)
        .with_context(|| format!("creating output directory {}", out_dir.display()))
}

/// What [`ClipDir::claim`] found: the directory was this window's to create, or
/// it was already there.
///
/// Two variants rather than an `Option` because both carry the same path and a
/// caller has to say something about each — the claim's whole job is to make a
/// taken id impossible to mistake for a free one.
#[derive(Debug)]
#[must_use = "a claim decides whether this window writes anything at all"]
pub(crate) enum Claim {
    /// The `mkdir` succeeded, so this window owns the directory outright: no
    /// other window, process or run can be writing into it.
    Ours(ClipDir),
    /// The directory was already there — complete, or residue from a cut that
    /// died — and this window writes nothing. Carries the directory, which is
    /// what the caller's warning names.
    Taken(PathBuf),
}

/// A clip directory this cut created and owns until it completes or discards it.
///
/// Holding one is the proof that the `mkdir` succeeded, so every path it hands
/// out is a path nothing else is writing to. It is consumed by
/// [`discard`](Self::discard) and borrowed by everything else, which is what
/// makes "a failed cut removes its directory" a matter of the one error arm
/// rather than a rule to remember.
#[derive(Debug)]
pub(crate) struct ClipDir {
    path: PathBuf,
    id: ClipId,
}

impl ClipDir {
    /// Claim `<out_dir>/<id>` for this window with **one atomic `mkdir`, no
    /// parents**.
    ///
    /// The atomicity is the whole point: `mkdir` either creates the directory or
    /// fails with `AlreadyExists`, and the kernel picks exactly one winner among
    /// however many callers race for it. No parents, because the tree above is
    /// [`prepare_out_dir`]'s and creating it here would turn a typo'd
    /// `--out-dir` into a silently fresh one.
    ///
    /// Anything other than `AlreadyExists` is the error it is — a missing output
    /// directory, no permission, a full filesystem — and the cut has not started.
    pub(crate) fn claim(out_dir: &Path, id: ClipId) -> Result<Claim> {
        let path = out_dir.join(id.to_string());
        match std::fs::create_dir(&path) {
            Ok(()) => Ok(Claim::Ours(ClipDir { path, id })),
            Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => Ok(Claim::Taken(path)),
            Err(e) => Err(e).with_context(|| format!("claiming clip directory {}", path.display())),
        }
    }

    /// The directory itself — the one path a completion announcement names, and
    /// the unit a consumer syncs.
    pub(crate) fn path(&self) -> &Path {
        &self.path
    }

    /// Where the copy of the plan at position `plan_idx` writes while it runs.
    pub(crate) fn staging(&self, plan_idx: usize) -> PathBuf {
        self.path
            .join(format!("{}_{plan_idx}.{STAGING_EXT}", self.id))
    }

    /// Rename a finished copy to `<id>_<n>.mcap`, its place in the clip.
    ///
    /// A rename inside a directory nothing else may write to, so there is no
    /// collision to resolve and no window in which a reader could see two names
    /// for one file.
    pub(crate) fn place(&self, staged: &Path, n: usize) -> Result<PathBuf> {
        let named = self.path.join(format!("{}_{n}.mcap", self.id));
        std::fs::rename(staged, &named).with_context(|| {
            format!(
                "naming {} as {}",
                staged.display(),
                named.file_name().unwrap_or_default().to_string_lossy()
            )
        })?;
        Ok(named)
    }

    /// Write [`METADATA_FILE`] and make the whole clip durable: the clip is
    /// complete when this returns.
    ///
    /// The order is what the completion rule rests on. Every MCAP file was
    /// fsynced by its own copy, so by the time this runs the only thing missing
    /// is the document; writing and fsyncing it, then the clip directory (which
    /// makes every name in it durable), then the output directory (which makes
    /// the clip directory's own entry durable) means a crash can lose the clip
    /// but can never leave a directory that has the metadata file and is missing
    /// a file it names.
    pub(crate) fn complete(&self, metadata: &ClipMetadata) -> Result<()> {
        let path = metadata_path(&self.path);
        let document = serde_norway::to_string(metadata)
            .with_context(|| format!("rendering {}", path.display()))?;
        let mut file =
            File::create(&path).with_context(|| format!("creating {}", path.display()))?;
        file.write_all(document.as_bytes())
            .with_context(|| format!("writing {}", path.display()))?;
        file.sync_all()
            .with_context(|| format!("syncing {}", path.display()))?;
        sync_dir(&self.path)?;
        match self.path.parent() {
            Some(out_dir) => sync_dir(out_dir),
            None => Ok(()),
        }
    }

    /// Remove the directory this cut claimed, then hand back the error that got
    /// us here.
    ///
    /// **Best effort, and the cause survives either way.** The cut has already
    /// failed; removing the directory is what keeps "a directory without
    /// [`METADATA_FILE`] is crash residue" true, so that an operator finding one
    /// knows the process died rather than that a cut merely erred. A removal
    /// that itself fails cannot make the cut any more failed, so it is folded
    /// into the returned error as context naming the directory left behind — the
    /// path an operator removes by hand, and the id that will be skipped until
    /// they do.
    pub(crate) fn discard(self, cause: anyhow::Error) -> anyhow::Error {
        match std::fs::remove_dir_all(&self.path) {
            Ok(()) => cause,
            Err(e) => cause.context(format!(
                "the failed clip's directory {} could not be removed ({e}); it holds \
                 no {METADATA_FILE}, so it is incomplete, and every later window with \
                 this id is skipped until it is removed",
                self.path.display()
            )),
        }
    }
}

/// fsync a directory so its entries — not just the data in the files they name
/// — survive a crash. Opening it and `sync_all`ing is the POSIX way to flush
/// directory metadata; it works on Linux.
fn sync_dir(dir: &Path) -> Result<()> {
    File::open(dir)
        .and_then(|d| d.sync_all())
        .with_context(|| format!("syncing directory {}", dir.display()))
}

/// The [`ClipMetadata`] a complete clip directory carries.
///
/// An error means the directory is not a complete clip: the file is absent
/// (the clip is incomplete, or the directory is not a clip at all), unreadable,
/// or holds something this version does not understand.
pub fn read_metadata(clip_dir: &Path) -> Result<ClipMetadata> {
    let path = metadata_path(clip_dir);
    let text =
        std::fs::read_to_string(&path).with_context(|| format!("reading {}", path.display()))?;
    serde_norway::from_str(&text).with_context(|| format!("parsing {}", path.display()))
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
    use crate::manifest::{Planned, WindowCoverage};
    use crate::testing::{test_dir, window_request};

    fn an_id() -> ClipId {
        ClipId::of(&window_request(100, 200, TimeSource::Log))
    }

    /// The claim is the exclusion: whoever's `mkdir` lands owns the directory,
    /// and every caller after it is told the id is taken rather than being
    /// handed a second writer's view of one clip.
    #[test]
    fn the_first_claim_of_an_id_wins_and_every_later_one_is_taken() -> Result<()> {
        let root = test_dir("claim")?;
        let out_dir = root.join("clipped");
        prepare_out_dir(&out_dir)?;
        let id = an_id();

        let Claim::Ours(dir) = ClipDir::claim(&out_dir, id)? else {
            panic!("the first claim of a free id owns it");
        };
        assert_eq!(dir.path(), out_dir.join(id.to_string()));
        assert!(dir.path().is_dir(), "the claim is the directory");

        let Claim::Taken(taken) = ClipDir::claim(&out_dir, id)? else {
            panic!("a claimed id is taken");
        };
        assert_eq!(taken, out_dir.join(id.to_string()));

        // A directory holding crash residue — anything at all, or nothing — is
        // taken on exactly the same terms: the claim reads no contents.
        std::fs::write(dir.path().join("leftover.part"), b"x")?;
        assert!(matches!(ClipDir::claim(&out_dir, id)?, Claim::Taken(_)));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A claim never creates the tree above it: an output directory that is not
    /// there is the run's problem, not something to invent one component at a
    /// time under a mistyped path.
    #[test]
    fn a_claim_creates_no_parents() -> Result<()> {
        let root = test_dir("claim-no-parents")?;
        let missing = root.join("not").join("there");

        let err = ClipDir::claim(&missing, an_id()).unwrap_err();
        assert!(
            format!("{err:#}").contains("claiming clip directory"),
            "the error names what it was doing: {err:#}"
        );
        assert!(
            !missing.exists(),
            "nothing was created under a missing tree"
        );

        prepare_out_dir(&missing)?;
        assert!(
            missing.is_dir(),
            "the out-dir itself is created with parents"
        );
        assert!(matches!(ClipDir::claim(&missing, an_id())?, Claim::Ours(_)));

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The document round-trips: what a clip states is what a consumer reads
    /// back, through the same file name both halves spell in one place.
    #[test]
    fn a_completed_clip_carries_the_document_it_was_given() -> Result<()> {
        let root = test_dir("complete")?;
        let out_dir = root.join("clipped");
        prepare_out_dir(&out_dir)?;
        let request = window_request(100, 200, TimeSource::Log);
        let Claim::Ours(dir) = ClipDir::claim(&out_dir, ClipId::of(&request))? else {
            panic!("a free id is claimable");
        };

        let metadata = ClipMetadata::of(
            &request,
            Planned {
                files: 1,
                coverage: WindowCoverage::Covered,
            },
            &[],
        );
        assert!(
            !metadata_path(dir.path()).exists(),
            "a claimed directory is incomplete until the document is written"
        );
        dir.complete(&metadata)?;

        assert_eq!(read_metadata(dir.path())?, metadata);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Reading a directory that is not a complete clip is an ordinary error
    /// naming the file that is missing, which is how a consumer tells crash
    /// residue from a clip.
    #[test]
    fn an_incomplete_directory_has_no_metadata_to_read() -> Result<()> {
        let root = test_dir("incomplete")?;
        let err = read_metadata(&root).unwrap_err();
        assert!(
            format!("{err:#}").contains(METADATA_FILE),
            "the error names the file that says a clip is complete: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
