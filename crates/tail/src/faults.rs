//! What repeated whole-clip refusals against one recording cost, counted so the
//! recorder can say it out loud.
//!
//! The scan and the cut meet a framing fault on opposite sides of the scan
//! offset, and the two need opposite answers. The scan meets damage *ahead* of
//! it, where the fault blocks every recording behind it: nothing more is
//! indexed, coverage stops rising, and every clip silently degrades to a
//! grace-timeout cut — so the [scan-fault budget](crate::tailer) gives up and
//! the process exits for a supervisor. The cut meets damage *behind* the scan
//! ([`clip::cut::FramingDesync`]), where the recorder is still perfectly
//! functional: the index is intact, coverage keeps rising, windows over other
//! extents still cut, and the next recording is unaffected. What is lost is the
//! extent the damage sits in — and every window whose plan includes it.
//!
//! **So the cut side counts where the scan side exits.** Exiting here would
//! trade a recorder that refuses some clips for one that produces none: a
//! restarted clipper re-indexes the same recording from the start, and its fresh
//! scan then meets that same damage *ahead* of it and exhausts the scan-fault
//! budget in about three seconds. With no successor for the startup adopt to
//! pick up — a recording that is not being split has none — every restart
//! repeats that, so the supervisor gets a restart loop and the vehicle gets
//! nothing. Staying up is what lets the recorder keep serving every window the
//! damage does not touch, and lets a rollover retire the damaged recording on
//! its own.
//!
//! What the recorder owes an operator instead is that the repetition is
//! *visible*: the first refusal against a recording is announced in full, and
//! every refusal after it carries the running count, so a log shows a tally
//! climbing rather than one indistinguishable line per trigger. That is what
//! [`CutFaults`] is.

use std::path::PathBuf;
use std::sync::Mutex;

use clip::cut::FramingDesync;

/// The recorder's tally of clips refused because a recording's bytes changed
/// under the tail, kept per recording and shared by every trigger handler.
///
/// **Only a [`FramingDesync`] counts**, which is why that is the only thing
/// [`CutFaults::refused`] accepts. A cut can fail for reasons that are not file
/// damage — an IO error on the recording, a full disk, an output failure, a
/// staging panic — and those are transient, are fixed elsewhere, and would make
/// this tally mean nothing if they were folded into it. They keep their own
/// per-trigger error and no count.
///
/// **The tally is per recording and nothing resets it but a different one.** A
/// rollover, a bag split, or a recorder restart puts a fresh file under the
/// tail, and its bytes are not the damaged ones, so the next refusal there
/// starts at one and is announced in full again. A *successful* cut against the
/// damaged recording does not reset it: it only proves that window planned
/// another extent, and the damage has not healed — a length past
/// `MAX_RECORD_LEN` is a value no valid record reaches, and the scan is long
/// past those bytes and never re-reads them. That is the deliberate difference
/// from the scan-fault budget, which *does* reset on a clean pass because a
/// scan fault can be a record that was still being appended.
///
/// One recording is tracked at a time: the damaged file is the one being tailed,
/// and a refusal naming another replaces it. Two damaged recordings being cut
/// from in alternation therefore re-announce, which is the loud direction of
/// that trade.
#[derive(Debug, Default)]
pub struct CutFaults {
    seen: Mutex<Option<Desynced>>,
}

/// The recording currently being refused against, and what it has cost.
#[derive(Debug)]
struct Desynced {
    recording: PathBuf,
    refusals: u64,
}

impl CutFaults {
    /// A tally with nothing refused yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Count one clip refused over `desync`, and return how many clips the
    /// recording it names has now cost, this one included.
    ///
    /// A return of `1` is the first refusal against that recording — the
    /// caller announces it in full. Anything higher is a repeat, and the number
    /// is what the caller reports in its place.
    pub fn refused(&self, desync: &FramingDesync) -> u64 {
        #[expect(
            clippy::unwrap_used,
            reason = "a poisoned lock means a handler panicked between the two \
                      lines below, which cannot happen; propagating is the policy \
                      the rest of the crate takes"
        )]
        let mut seen = self.seen.lock().unwrap();
        match seen.as_mut() {
            Some(d) if d.recording == desync.recording() => {
                d.refusals += 1;
                d.refusals
            }
            _ => {
                *seen = Some(Desynced {
                    recording: desync.recording().to_path_buf(),
                    refusals: 1,
                });
                1
            }
        }
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::excessive_nesting,
        reason = "the concurrency case nests a scoped-thread closure inside the \
                  scope inside the test; flattening it would hide which of the \
                  three the tally is being driven from"
    )]

    use std::path::{Path, PathBuf};

    use super::*;

    /// A desync naming `recording`, the only input the tally accepts.
    fn desync(recording: &str) -> FramingDesync {
        FramingDesync::RecordLength {
            recording: PathBuf::from(recording),
            extent_offset: 0,
            offset: 19_773,
            declared: u64::MAX,
        }
    }

    /// The first refusal against a recording is the one worth announcing; every
    /// later one is a number. This is the whole escalation contract: `1` means
    /// "say it in full", anything else means "say how many".
    #[test]
    fn the_first_refusal_announces_and_the_rest_only_count() {
        let faults = CutFaults::new();
        let d = desync("/rec/record_0.mcap");
        assert_eq!(faults.refused(&d), 1, "the first refusal is announced");
        assert_eq!(faults.refused(&d), 2);
        assert_eq!(faults.refused(&d), 3, "the tally climbs, monotonically");
    }

    /// A rollover, a split, or a recorder restart puts fresh bytes under the
    /// tail, so the tally starts over and the new recording is announced in
    /// full — the count belongs to a recording, never to the process.
    #[test]
    fn a_different_recording_starts_its_own_tally() {
        let faults = CutFaults::new();
        let first = desync("/rec/record_0.mcap");
        assert_eq!(faults.refused(&first), 1);
        assert_eq!(faults.refused(&first), 2);

        let next = desync("/rec/record_1.mcap");
        assert_eq!(
            faults.refused(&next),
            1,
            "the successor is announced in its own right"
        );
        assert_eq!(faults.refused(&next), 2);
    }

    /// The identity is the recording, not the fault: the same damaged file
    /// refusing a window through its second extent — a different offset, a
    /// different arm of the enum — is the same damaged file, and counts on.
    ///
    /// Announcing per (recording, extent) instead would put the loud line back
    /// on a per-trigger footing the moment the recording grew past one extent.
    #[test]
    fn the_same_recording_counts_on_whatever_the_fault_looks_like() {
        let faults = CutFaults::new();
        assert_eq!(faults.refused(&desync("/rec/record_0.mcap")), 1);
        assert_eq!(
            faults.refused(&FramingDesync::ShortExtent {
                recording: PathBuf::from("/rec/record_0.mcap"),
                extent_offset: 4 * 1024 * 1024,
                offset: 12,
            }),
            2,
            "a second extent of the same recording is not a new announcement"
        );
    }

    /// The tally is shared state read from one handler thread per trigger, so
    /// concurrent refusals must add up rather than race: sixteen threads (the
    /// recorder's whole admission cap) refusing against one recording leave a
    /// tally of exactly sixteen, and exactly one of them saw the `1` that
    /// announces.
    #[test]
    fn concurrent_refusals_against_one_recording_add_up() {
        let faults = CutFaults::new();
        let announced = std::sync::atomic::AtomicU64::new(0);
        std::thread::scope(|s| {
            for _ in 0..16 {
                s.spawn(|| {
                    if faults.refused(&desync("/rec/record_0.mcap")) == 1 {
                        announced.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                    }
                });
            }
        });
        assert_eq!(faults.refused(&desync("/rec/record_0.mcap")), 17);
        assert_eq!(
            announced.load(std::sync::atomic::Ordering::Relaxed),
            1,
            "exactly one refusal is the first"
        );
    }

    /// The accessor the tally keys on: both arms name their recording, so no
    /// desync can land in the wrong file's count.
    #[test]
    fn every_desync_names_its_recording() {
        assert_eq!(
            desync("/rec/record_0.mcap").recording(),
            Path::new("/rec/record_0.mcap")
        );
        assert_eq!(
            FramingDesync::ShortExtent {
                recording: PathBuf::from("/rec/record_9.mcap"),
                extent_offset: 0,
                offset: 3,
            }
            .recording(),
            Path::new("/rec/record_9.mcap")
        );
    }
}
