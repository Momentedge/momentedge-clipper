//! What every consumer of an MCAP recording shares: the format layer that turns
//! a recording into an index, the cut path that copies a window out of it, the
//! neutral trigger contract that names a window, and the segment assembly that
//! publishes the result.
//!
//! The crate is deliberately **ROS-free by default** — nothing here links r2r,
//! opens a node, or needs a ROS installation, so a plain-Linux consumer (the
//! cloud clipper, the compactor) builds and runs against a recording with the
//! same code the device recorder runs. The one ROS-shaped piece, the `cdr`
//! trigger decoder, sits behind the `ros` feature ([`decode`]).
//!
//! ## The modules
//!
//! - [`index`] — the format layer: schema and channel definitions, extents with
//!   their `log_time`/`publish_time` spans, the per-recording index, the
//!   incremental scan and its delta, the window plan and the [`index::WindowPlanner`]
//!   that serves one.
//! - [`whole`] — the same index for a recording that is already finished, taken
//!   from its summary rather than by walking it: a footer seek and one read,
//!   whatever the recording's size.
//! - [`cut`] — the copy: raw message bytes out of the planned extents into a new
//!   MCAP, finished with a manifest record, a summary and a footer.
//! - [`manifest`] — what a clip says about itself: the metadata record every cut
//!   writes, naming the producer, the trigger, the window, the source recording
//!   and what each channel contributed.
//! - [`trigger`] — the neutral trigger and completion contract every trigger
//!   source targets.
//! - [`decode`] — a trigger payload's bytes to a [`trigger::Trigger`], dispatched
//!   on its MCAP `message_encoding`.
//! - [`segment`] — one window to durable clips: plan, stage a segment per source
//!   recording, drop the empties, publish atomically.
//! - [`select`] — which of a recording's topics a clip is cut from, the decision
//!   [`cut`] applies at the two places it can matter: where a channel is
//!   registered in the output, and where a message is copied.
//! - [`config`] — the layered configuration file every setting and that
//!   selection are read from: a system file under a per-run file, both under
//!   the environment and the command line.
//! - `testing` (under the `test-support` feature) — the MCAP fixture writers the
//!   modules above are tested against, for a downstream crate's tests.
//!
//! Time is the seam that runs through all of them: every window lives in one
//! [`TimeSource`], and the same choice selects the extents read, the messages
//! that fall inside, and the coverage a caller waits on.

pub mod config;
pub mod cut;
pub mod decode;
pub mod index;
pub mod manifest;
pub mod segment;
pub mod select;
/// The MCAP fixtures this crate's tests are written against, published for a
/// downstream crate's tests under the `test-support` feature.
#[cfg(any(test, feature = "test-support"))]
pub mod testing;
pub mod trigger;
pub mod whole;

pub use config::Layered;
pub use index::{
    ChannelDef, Extent, PlanSource, RecordingIndex, ScanDelta, ScanProgress, ScanSeed, SchemaDef,
    Span, Stamps, TimeBounds, WindowPlan, WindowPlanner,
};
pub use manifest::{CutRequest, Producer, WindowCoverage};
pub use select::ChannelSelection;
pub use trigger::{Announce, Completion, Stamp, Trigger, TriggerRecord};
pub use whole::WholeFileIndex;

/// The clock domain a clip's whole window lives in.
///
/// It governs which extents are read, which messages fall inside the window, and
/// which coverage a caller waits on — and nothing else (retention ages files on
/// `log_time`, a postroll floor is the wall clock). Nothing here interprets what
/// a producer wrote into `publish_time`; a window lives on whatever is there.
///
/// Under the `clap` feature the two variant doc comments below are also the
/// value help clap renders into a binary's `--help`, which is why they read as
/// operator copy and name `--grace-secs` — the recorder's spelling of the bound
/// every consumer puts on a `publish`-domain wait. A consumer whose flag is
/// spelled differently documents that on its own argument, whose help clap
/// prints directly above these.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "clap", derive(clap::ValueEnum))]
pub enum TimeSource {
    /// Window on each message's `log_time` — when the producer received it.
    /// Approximately non-decreasing in file order, so coverage on it is a
    /// completeness proof. The default.
    #[default]
    Log,
    /// Window on each message's `publish_time` — whatever the producer put
    /// there (a DDS source timestamp, a capture time). Publish times may arrive
    /// out of order, so coverage on it is a liveness signal, not a completeness
    /// proof: a message can land after the cut with an in-window `publish_time`
    /// and be lost. `--grace-secs` bounds the wait.
    Publish,
}

impl std::fmt::Display for TimeSource {
    /// Render as the name a CLI accepts (`log`/`publish`). Under the `clap`
    /// feature this is the `ValueEnum` possible-value name, so a `--help`
    /// default and the accepted flag values read the same — a test below pins
    /// the two together.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self {
            TimeSource::Log => "log",
            TimeSource::Publish => "publish",
        })
    }
}

/// Render a panic payload (from [`std::thread::JoinHandle::join`] or
/// [`std::panic::catch_unwind`]) as text: panics carry a `&str` or `String`
/// message in practice; anything else gets a placeholder.
///
/// Lives here because the staging worker pool ([`segment`]) needs it: a worker
/// catches a panicking stage per job and has to render the payload as the error
/// it replies. Anything else supervising threads around a recording — a
/// `JoinHandle::join` that came back `Err` — wants the same three lines, so it
/// is public rather than private to that module.
pub fn panic_text(payload: &(dyn std::any::Any + Send)) -> String {
    payload
        .downcast_ref::<&str>()
        .map(|s| s.to_string())
        .or_else(|| payload.downcast_ref::<String>().cloned())
        .unwrap_or_else(|| "non-string panic payload".to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_source_renders_the_cli_names() {
        assert_eq!(TimeSource::Log.to_string(), "log");
        assert_eq!(TimeSource::Publish.to_string(), "publish");
        assert_eq!(TimeSource::default(), TimeSource::Log);
    }

    /// The hand-written [`Display`] above and clap's possible-value names are
    /// two spellings of one thing; a binary renders its `--help` default from
    /// `Display` and accepts the clap name, so they must not drift.
    #[cfg(feature = "clap")]
    #[test]
    fn display_matches_the_clap_value_names() {
        use clap::ValueEnum;
        for source in TimeSource::value_variants() {
            assert_eq!(
                source.to_string(),
                source
                    .to_possible_value()
                    .expect("no TimeSource variant is skipped")
                    .get_name(),
            );
        }
    }

    #[test]
    fn panic_text_reads_both_payload_shapes() {
        assert_eq!(panic_text(&"boom"), "boom");
        assert_eq!(panic_text(&"boom".to_string()), "boom");
        assert_eq!(panic_text(&7u32), "non-string panic payload");
    }
}
