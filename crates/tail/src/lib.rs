//! Following a recording that is still being written, and cutting clips out of
//! it as the data lands.
//!
//! [`clip`] turns a recording into an index and copies windows out of it; this
//! crate is what that takes when the recording has no end yet. A file under tail
//! grows under the reader, splits into a successor, or vanishes and comes back
//! when its writer restarts — so following one is a collection problem, not a
//! file problem, and the collection is what everything here is about:
//!
//! - [`discover`] yields each new `*.mcap` in a directory exactly once, by
//!   `(dev, ino)` identity rather than a timestamp cursor — a file being
//!   appended to advances its mtime, so a cursor would keep re-yielding it.
//! - [`tailer`] owns the recordings in time order, scans the current one
//!   incrementally through [`clip::index`], retires it when it ends, advances to
//!   its successor, and prunes recordings whose data has aged out. It is also
//!   the [`clip::index::WindowPlanner`] the shared cut path plans through, so a
//!   window straddling a rollover yields one segment per source file.
//! - [`watch`] is the notification primitive coverage rides on.
//! - [`handler`] is the one thing a live recording forces that a finished one
//!   does not: **waiting**. A window may reach past the last byte on disk, so a
//!   cut waits for the wall clock to pass the window end and for the tail's
//!   coverage to catch up, and only then calls [`clip::segment::cut_window`] —
//!   the same code that cuts from a recording nobody is writing.
//!
//! Nothing here links r2r or needs a ROS installation. What arrives as a trigger
//! and who is told about a finished clip are a caller's business; this crate
//! knows only [`clip`]'s neutral contract.

pub mod discover;
pub mod handler;
pub mod tailer;
pub mod watch;

pub use tailer::{Coverage, Tailer};
pub use watch::Watch;
