//! Triggered clip recorder tailing a continuous MCAP recording.
//!
//! A continuous `ros2 bag record` (started separately — see the README and
//! `scripts/record.sh`) writes a growing MCAP file, rolling over to a
//! successor on a bag split. This binary discovers each recording, keeps it
//! open, and tails it ([`tail`]): an incremental scan over the record framing
//! that maintains a byte-extent index, a schema/channel registry, and a
//! collection-wide coverage watch (the highest `log_time` on disk). Rollovers
//! are recovered from the files themselves — there is no
//! `/events/write_split` dependency.
//!
//! A trigger requests the window `[trigger_time - preroll, trigger_time +
//! postroll]`: the [`handler`] waits until the wall clock passes the window end,
//! waits until the tail's coverage reaches it (the recording provably holds the
//! window), then bulk-copies the in-window messages out of the planned extents
//! into a clip at `./clipped/<trigger_ns>_<name>.mcap` (see [`clip`] — a
//! raw-bytes copy, no CDR decode, finished with a proper summary + footer,
//! assembled in a capturing dir and moved atomically into place so observers
//! never see a footer-less file).
//!
//! Where triggers come from is `--trigger-source`, and how completion is
//! signalled follows from it: the two are one seam, the [`interface`], with one
//! form active per run. The `mcap` source reads triggers out of the tailed
//! recording itself — decoding each by its MCAP `message_encoding`
//! ([`decode`]) — and runs ROS-free, the clip's atomic move into the output
//! directory standing in for a completion announcement. The `ros` source
//! subscribes to `/events/momentedge/trigger` (`momentedge_msgs/Trigger`) on a
//! ROS node and publishes `/events/momentedge/recorded`
//! (`momentedge_msgs/Recorded`) naming every durable segment. The handler
//! cutting the clip is identical either way; it knows only the neutral
//! [`trigger`] contract.
//!
//! **Two builds.** The `ros` cargo feature is what links the ROS client and
//! compiles the `ros` interface in. With it — the device build, which every
//! packaging path selects — `clipper tail --trigger-source` takes `ros` (the
//! default there) or `mcap`. Without it the binary links no ROS, builds and runs
//! on a host with no ROS installation, and offers `mcap` alone. Nothing else
//! differs: the tail, the window plan, the cut, and every other flag are the
//! same code either way.
//!
//! Time base: MCAP `log_time`, the trigger stamp, and the wait clock are all
//! treated as nanoseconds on the system (ROS) clock — this assumes the default
//! (no `use_sim_time`). Each trigger is handled on its own thread, so
//! overlapping windows are cut concurrently against one shared tail — at most
//! [`MAX_ACTIVE_TRIGGERS`] at once; a trigger beyond that limit is rejected,
//! logged, and ignored.
//!
//! Everything runs on plain OS threads — there is no async runtime. The main
//! thread supervises ([`supervise`]) two long-lived companions over crossbeam
//! channels — the tail thread (file scan) and the interface thread (draining the
//! active trigger source; the `ros` interface owns its node spin and
//! subscription drain internally) — plus a signal forwarder (SIGINT/SIGTERM →
//! orderly exit 0). Clip copies run on a fixed pool of `extract_parallelism`
//! worker threads consuming one FIFO job channel.
//!
//! `clipper` is one binary and the mode is a subcommand ([`Mode`]): the device
//! recorder this module describes is `clipper tail`. `clipper --help` lists the
//! modes, `clipper tail --help` lists the recorder's own flags, and a flag
//! offered to `clipper` itself is a parse error pointing at the subcommand.
//!
//! **The other mode is `clipper clip`** ([`clip_mode`]): one clip out of one
//! finished recording, named by a trigger on the command line, then exit. It
//! shares everything below the trigger — the window plan, the copy, the
//! manifest, atomic publication — and differs in the two things a finished input
//! makes meaningless. The recording is indexed from its own summary
//! ([`clip::whole::WholeFileIndex`]) rather than by a scan that keeps resuming,
//! and neither wait above runs: there is no later data to wait for, so a window
//! reaching past the recording's end is simply short and the clip's manifest
//! says so. A run's result is the output directory's contents when the process
//! exits, and the exit status is the verdict.
//!
//! Configuration resolves through four layers over each setting's built-in
//! default: the system configuration file, the per-run file ([`clip::config`]),
//! the `MOMENTEDGE_<KEY>` environment variable, and the flag — strongest last.
//! The files are read before the parser is built and become its defaults
//! ([`with_file_defaults`]), which is what puts the four in that order; the
//! `MOMENTEDGE_*` names are derived from one prefix applied to every field of
//! every mode ([`with_env_prefix`]); and `--print-config` prints what they came
//! to ([`effective_config`]), as the log does at startup. `--version` prints the
//! version; [`Config`]'s field docs are the `clipper tail --help` text and the
//! authoritative per-flag reference (`docs/configuration.md` is the user-facing
//! copy of the same set).
//!
//! Which topics a clip is cut from is the one setting with no flag: the files'
//! `[topics]` table becomes a [`clip::ChannelSelection`] that both modes hand to
//! their staging pool, so the recorder and the one-shot cutter cut the same
//! channel set out of one recording.
//!
//! Logging uses the `log` facade with a pretty_env_logger backend and goes to
//! **stdout**; `RUST_LOG` controls verbosity. Under `--trigger-source ros` the
//! ROS layer's own diagnostics are a separate stream — rcutils writes them to
//! stderr unless `RCUTILS_LOGGING_USE_STDOUT=1`.

mod interface;
mod supervision;

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;

use anyhow::Context;
use clap::builder::TypedValueParser as _;
use clap::{Args, CommandFactory, FromArgMatches, Parser, Subcommand, ValueEnum};
use clip::config::{self, Layer, Layered};
use clip::manifest::Producer;
#[cfg(feature = "ros")]
use clip::trigger::ANNOUNCE_TOPIC;
use clip::trigger::{TRIGGER_TOPIC, Trigger, now_ns};
use clip::{TimeSource, segment};
use crossbeam_channel::{Receiver, Sender, bounded, select, unbounded};
#[cfg(feature = "ros")]
use interface::ros::RosInterface;
use interface::{Anchor, Interface, McapInterface};
use log::{error, info, warn};
use signal_hook::consts::{SIGINT, SIGTERM};
use supervision::{Supervised, harvest_panic, spawn_supervised};
use tail::{Coverage, Tailer, Watch, handler};

/// How many trigger handlers may be active (admitted, waiting, or extracting)
/// at once. Beyond this limit an arriving trigger is rejected at admission:
/// `error!`-logged and ignored — no handler is spawned, no clip is cut, and no
/// `Recorded` message is published.
///
/// An active handler is one parked thread: it sleeps out its postroll window
/// and waits on the coverage watch. The heavy work — the bulk file copy — is
/// already serialized by the extraction worker pool (`extract_parallelism`).
/// This constant is therefore a flood-sanity bound on thread and announcement
/// growth, not a resource budget; 16 comfortably exceeds any legitimate
/// concurrent burst.
///
/// **Failure mode for downstream automation.** A rejected trigger produces no
/// `Recorded` announcement, and there is no negative acknowledgement on the
/// wire: a consumer waiting on `/events/momentedge/recorded` to learn that a
/// clip was written simply never hears back for that trigger and would hang if
/// it blocks on the reply. The `error!` log line is the *only* signal that a
/// trigger was dropped, so alerting on it is how an operator detects a
/// sustained trigger flood that is outrunning the recorder.
const MAX_ACTIVE_TRIGGERS: usize = 16;

/// Compression codec for written clips, the clap surface of the otherwise
/// implicit `mcap::WriteOptions` compression. [`to_mcap`](ClipCompression::to_mcap)
/// maps it to the `Option<mcap::Compression>` the clip writer takes: `None` →
/// uncompressed, `Zstd`/`Lz4` → the matching codec.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum ClipCompression {
    None,
    Zstd,
    Lz4,
}

impl ClipCompression {
    /// The `mcap::WriteOptions::compression` argument this codec selects.
    fn to_mcap(self) -> Option<mcap::Compression> {
        match self {
            ClipCompression::None => None,
            ClipCompression::Zstd => Some(mcap::Compression::Zstd),
            ClipCompression::Lz4 => Some(mcap::Compression::Lz4),
        }
    }
}

impl std::fmt::Display for ClipCompression {
    /// Render as the clap value name (`none`/`zstd`/`lz4`) so the `--help`
    /// default rendered by `default_value_t` and the accepted flag values share
    /// one source — the `ValueEnum` possible-value names.
    #[expect(
        clippy::expect_used,
        reason = "`to_possible_value` is `None` only for a `#[clap(skip)]` variant, \
                  and this enum has none — a skipped one would also break `--help`"
    )]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.to_possible_value()
            .expect("no ClipCompression variant is skipped")
            .get_name()
            .fmt(f)
    }
}

/// Where a run's triggers come from — one value set, named `--trigger-source`
/// under both subcommands, and the seam each variant drives underneath it.
///
/// The variants are mutually exclusive: a run drives exactly one. `mcap` means
/// the same thing under both subcommands — the triggers the recording itself
/// carries on the trigger topic, decoded by each message's MCAP
/// `message_encoding` ([`clip::decode`]) — and only the mechanism differs, since
/// `clipper tail` taps them out of a file as it is written while `clipper clip`
/// reads them out of one that is finished. That shared meaning is why one enum
/// serves both.
///
/// **Each subcommand takes a subset**, and [`TriggerSource::modes`] is where a
/// variant says which subcommands take it. The two arguments narrow to that
/// subset ([`trigger_source_parser`]) rather than restating it, so a source a
/// subcommand does not take is refused by name while the command line is being
/// read and never appears in that subcommand's `--help`.
///
/// The variant set is also the build's: `Ros` exists only under the `ros` cargo
/// feature, so a ROS-free build refuses `--trigger-source ros` as an unknown
/// value wherever the flag is offered. [`TAIL_DEFAULT_TRIGGER_SOURCE`] and
/// [`CLIP_DEFAULT_TRIGGER_SOURCE`] are what each subcommand takes when the flag
/// is absent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, ValueEnum)]
enum TriggerSource {
    /// Subscribe to the trigger topic on a ROS node and answer each clip with a
    /// `Recorded` publish. The deployed, ROS-native path, and the recorder's
    /// default where the feature built it.
    #[cfg(feature = "ros")]
    Ros,
    /// Take the triggers the recording carries on the trigger topic, each clip
    /// anchored on the stamp the recording gave its trigger message. Runs
    /// ROS-free: no node, executor, subscription, or publish.
    Mcap,
    /// Cut the one trigger the `--trigger-*` flags name. One run, one clip.
    Param,
}

impl TriggerSource {
    /// The subcommands that take this source.
    ///
    /// The subsets live here, on the type, rather than as two literals at the
    /// two argument definitions: the match is exhaustive with no catch-all, so
    /// adding a variant is a compile error until it says who takes it, and
    /// [`trigger_source_parser`] derives both `--trigger-source` surfaces from
    /// this one answer.
    fn modes(self) -> &'static [config::Mode] {
        match self {
            // A live subscription needs a topic somebody is still publishing on,
            // which is the recorder's situation and not the cutter's: a finished
            // recording has no live topic.
            #[cfg(feature = "ros")]
            TriggerSource::Ros => &[config::Mode::Tail],
            // The one source both subcommands share, and the reason they share a
            // key at all.
            TriggerSource::Mcap => &[config::Mode::Tail, config::Mode::Clip],
            // `param` is the cutter's alone, and deliberately not the recorder's:
            // `clipper tail` runs until a shutdown signal, so after the single
            // cut a `param` run would have a loop with nothing left to do.
            // "Follow a growing recording, wait for one window, cut it, exit" is
            // an exit condition rather than a trigger source, and it is not a
            // mode clipper has.
            TriggerSource::Param => &[config::Mode::Clip],
        }
    }

    /// Whether the `--trigger-*` flags state this source's trigger.
    ///
    /// Only `param` does. Every other source has a trigger of its own — off a
    /// live topic, or out of the recording, each stating its own name,
    /// description, preroll and postroll — so the flags have nothing left to say
    /// and a command line giving one anyway is refused
    /// ([`ClipConfig::trigger_argument_fault`]).
    fn reads_the_trigger_flags(self) -> bool {
        match self {
            #[cfg(feature = "ros")]
            TriggerSource::Ros => false,
            TriggerSource::Mcap => false,
            TriggerSource::Param => true,
        }
    }

    /// The sources `mode` takes, in this enum's own variant order — which is the
    /// order its `--help` lists them in.
    fn accepted_by(mode: config::Mode) -> impl Iterator<Item = TriggerSource> {
        TriggerSource::value_variants()
            .iter()
            .copied()
            .filter(move |source| source.modes().contains(&mode))
    }
}

/// The `--trigger-source` value parser for `mode`: the sources
/// [`TriggerSource::accepted_by`] lists for that subcommand, and nothing else.
///
/// Narrowing at the argument is what makes the refusal clap's own, raised while
/// the command line is being read: `clipper clip --trigger-source ros` is an
/// invalid value that names the value and the ones that *are* accepted, and
/// `clipper <mode> --help` lists that subset with each variant's own help line.
/// Every spelling comes from [`ValueEnum`], so the accepted values, the `--help`
/// listing and the [`Display`](std::fmt::Display) a default is rendered through
/// stay one fact.
///
/// The map back cannot fail — the value was just checked against the subset —
/// but it is a `try_map` rather than an unwrap, so a variant that ever stopped
/// round-tripping would report itself as a value error instead of a panic.
#[expect(
    clippy::expect_used,
    reason = "`to_possible_value` is `None` only for a `#[clap(skip)]` variant, \
              and this enum has none — a skipped one would also break `--help`"
)]
fn trigger_source_parser(
    mode: config::Mode,
) -> impl clap::builder::TypedValueParser<Value = TriggerSource> {
    clap::builder::PossibleValuesParser::new(TriggerSource::accepted_by(mode).map(|source| {
        source
            .to_possible_value()
            .expect("no TriggerSource variant is skipped")
    }))
    .try_map(|name: String| <TriggerSource as ValueEnum>::from_str(&name, false))
}

/// The source `clipper tail` takes when `--trigger-source` is absent: the live
/// subscription where the `ros` feature built it, and otherwise the only source
/// the recorder has.
#[cfg(feature = "ros")]
const TAIL_DEFAULT_TRIGGER_SOURCE: TriggerSource = TriggerSource::Ros;
#[cfg(not(feature = "ros"))]
const TAIL_DEFAULT_TRIGGER_SOURCE: TriggerSource = TriggerSource::Mcap;

/// The source `clipper clip` takes when `--trigger-source` is absent: the
/// command line, the source that needs nothing of the recording but its
/// messages.
const CLIP_DEFAULT_TRIGGER_SOURCE: TriggerSource = TriggerSource::Param;

/// The trigger name a `param` run takes when `--trigger-name` is absent. A name
/// is not optional — it goes in the clip's filename and its manifest — so the
/// one flag a caller may leave out has a value spelled here rather than at the
/// argument, which carries no clap default (a default would be
/// indistinguishable from a name the caller typed, and `mcap` refuses the flag
/// on exactly that distinction).
const DEFAULT_TRIGGER_NAME: &str = "clip";

/// `clipper tail`'s `--trigger-source` short help. It names the values this
/// build actually accepts, which is the `ros` feature's one visible difference
/// on the command line.
#[cfg(feature = "ros")]
const TAIL_TRIGGER_SOURCE_HELP: &str =
    "Where triggers come from and completions go: `ros` or `mcap`";
#[cfg(not(feature = "ros"))]
const TAIL_TRIGGER_SOURCE_HELP: &str = "Where triggers come from and completions go: `mcap`";

/// `clipper tail`'s `--trigger-source` long help (`--help`, not `-h`), likewise
/// per build: the ROS arm is described only where it can be selected, and the
/// ROS-free build says outright that it was built without it.
#[cfg(feature = "ros")]
const TAIL_TRIGGER_SOURCE_LONG_HELP: &str = "\
Where triggers come from and completions go: `ros` or `mcap`.

`ros` (the default) subscribes to the trigger topic on a ROS node and publishes \
`Recorded` on completion. `mcap` takes the triggers the recording itself carries \
(decoding each by its `message_encoding`) and signals completion by moving the \
clip into `out_dir` — it runs ROS-free, with no node, subscription, or publish. \
The source is also the completion half: exactly one of the two is active per \
run, and there is no third combination to select.";
#[cfg(not(feature = "ros"))]
const TAIL_TRIGGER_SOURCE_LONG_HELP: &str = "\
Where triggers come from and completions go: `mcap`.

`mcap` takes the triggers the recording itself carries (decoding each by its \
`message_encoding`) and signals completion by moving the clip into `out_dir` — \
it runs ROS-free, with no node, subscription, or publish. It is the only source \
this binary has: `ros`, which subscribes to the trigger topic on a ROS node and \
publishes `Recorded`, is compiled in by the `ros` cargo feature, and this build \
was made without it.";

/// `clipper clip`'s `--trigger-source` short help.
const CLIP_TRIGGER_SOURCE_HELP: &str = "Where this run's triggers come from: `param` or `mcap`";

/// `clipper clip`'s `--trigger-source` long help (`--help`, not `-h`): what each
/// source cuts, which flags it takes, and the one thing an operator has to get
/// right about a recorded trigger's encoding.
const CLIP_TRIGGER_SOURCE_LONG_HELP: &str = "\
Where this run's triggers come from: `param` or `mcap`.

`param` (the default) cuts the single trigger the `--trigger-*` flags name, and \
needs `--trigger-time`, `--preroll` and `--postroll`. `mcap` cuts every trigger \
the recording itself carries on /events/momentedge/trigger — one clip each, \
anchored on the log time the recording stamped that trigger message with — and \
takes no `--trigger-*` flag at all, since the recorded trigger states its own \
name, description, preroll and postroll. A recording holding no trigger cuts \
nothing and says so. A recorded trigger encoded as `json` is decoded by every \
build; `cdr` needs the rmw typesupport the `ros` cargo feature links, and a \
build without it skips such a trigger with an error naming the feature. Exactly \
one source is active per run. `ros` is a live subscription and a finished \
recording has no live topic, so this subcommand does not offer it.";

impl std::fmt::Display for TriggerSource {
    /// Render as the clap value name (`ros`/`mcap`/`param`) so the `--help`
    /// default and the accepted flag values share the `ValueEnum`
    /// possible-value names.
    #[expect(
        clippy::expect_used,
        reason = "`to_possible_value` is `None` only for a `#[clap(skip)]` variant, \
                  and this enum has none — a skipped one would also break `--help`"
    )]
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.to_possible_value()
            .expect("no TriggerSource variant is skipped")
            .get_name()
            .fmt(f)
    }
}

/// The error a subcommand raises when it is handed a trigger source it does not
/// take.
///
/// [`trigger_source_parser`] refuses such a value while the command line is
/// being read, so neither dispatch can reach this in a run clap accepted. It
/// exists so each dispatch stays one decision per variant rather than a
/// catch-all that would silently swallow a source added later.
fn unaccepted_source(mode: &str, source: TriggerSource) -> anyhow::Error {
    anyhow::anyhow!("`clipper {mode}` takes no `{source}` trigger source")
}

/// The one way a `clipper clip` command line can state its trigger wrongly:
/// choosing the source that reads the `--trigger-*` flags and then not giving
/// one they need, or choosing the source that reads the recording and giving one
/// anyway.
///
/// Both are argument faults, not run faults: [`parse_cli`] turns either into the
/// clap error that ends the process, so the flag at fault is named while the
/// command line is being read and nothing is written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TriggerArgFault {
    /// `--trigger-source param` cuts the trigger the flags name, and this flag
    /// is not there.
    Missing(&'static str),
    /// `--trigger-source mcap` takes every trigger from the recording, so this
    /// flag has nothing to say.
    Conflicting(&'static str),
}

impl TriggerArgFault {
    /// The clap error kind this reports as, so a trigger fault exits the way
    /// every other bad command line does.
    fn kind(self) -> clap::error::ErrorKind {
        match self {
            TriggerArgFault::Missing(_) => clap::error::ErrorKind::MissingRequiredArgument,
            TriggerArgFault::Conflicting(_) => clap::error::ErrorKind::ArgumentConflict,
        }
    }

    /// The message clap prints above its usage line: the flag at fault, then
    /// which source reads which flags, since the fault is always about that.
    fn message(self) -> String {
        match self {
            TriggerArgFault::Missing(flag) => format!(
                "the following required argument was not provided: {flag}\n\n\
                 `--trigger-source param` cuts the trigger the `--trigger-*` flags name, so it \
                 needs `--trigger-time`, `--preroll` and `--postroll`; `--trigger-source mcap` \
                 cuts the triggers the recording carries and needs none of them"
            ),
            TriggerArgFault::Conflicting(flag) => format!(
                "the argument '{flag}' cannot be used with '--trigger-source mcap'\n\n\
                 `mcap` takes every trigger from the recording itself: the name, description, \
                 preroll, postroll and instant of each clip all come from its own recorded \
                 trigger message"
            ),
        }
    }
}

/// The command line: one binary, one mode per invocation, and the mode is a
/// subcommand. `clipper tail` is the device recorder; every flag belongs to the
/// mode rather than to `clipper` itself, so a bare `clipper --record-dir …` is
/// rejected ([`mode_hint`] says where the flag belongs).
///
/// The `mode` field is what makes naming no mode an error: it is not an
/// `Option`, so clap's derive requires the subcommand and answers a bare
/// `clipper` with the mode listing and a non-zero exit. No
/// `subcommand_required`/`arg_required_else_help` is needed on top of that, and
/// adding them changes nothing.
///
/// `long_about` is written out rather than taken from this doc comment: clap's
/// derive would otherwise put the rationale above — rustdoc links and all — in
/// front of an operator running `clipper --help`.
#[derive(Debug, Parser)]
#[command(
    name = "clipper",
    version,
    about = "Triggered MCAP clip recorder",
    long_about = "Triggered MCAP clip recorder.\n\n\
                  One mode runs per invocation, and the mode is a subcommand: \
                  `clipper tail` follows a continuous recording and cuts a clip \
                  per trigger, while `clipper clip` cuts a clip per trigger out \
                  of one finished recording and exits. Every flag belongs to a mode, so \
                  `clipper <mode> --help` is that mode's own flag reference."
)]
struct Cli {
    #[command(subcommand)]
    mode: Mode,
}

/// The modes clipper runs, one per invocation. Each carries its own flags, so
/// `clipper <mode> --help` lists that mode's surface and nothing else.
#[derive(Debug, Subcommand)]
enum Mode {
    /// Tail a continuous recording and cut a clip per trigger.
    ///
    /// Discovers and follows the growing MCAP files a continuous `ros2 bag
    /// record` writes, and cuts the window each trigger asks for out of them.
    /// Runs until a shutdown signal.
    Tail(Config),
    /// Cut a clip per trigger out of one finished recording and exit.
    ///
    /// Takes a recording nobody is writing any more, and writes the window each
    /// trigger asks for. `--trigger-source` says where the triggers come from:
    /// the command line, or the recording itself. The recording is indexed from
    /// its own summary rather than by walking it, and nothing is waited for —
    /// the input has an end. The run's result is the contents of the output
    /// directory when the process exits; the exit status is the verdict.
    Clip(ClipConfig),
}

/// The program name every clip's manifest carries under `producer.name`: this
/// binary, as an operator invokes it.
const PROGRAM: &str = "clipper";

/// clap's names for [`Mode::Tail`] and [`Mode::Clip`] — the word an operator
/// types after `clipper`, derived by the derive from each variant name. Spelled
/// here so [`scan_mode`] can find the mode in argv before clap parses it and
/// [`parse_cli`] can raise a trigger-argument fault against `clip` and get its
/// usage line; a test pins both to the commands clap actually built.
const TAIL_MODE: &str = "tail";
const CLIP_MODE: &str = "clip";

/// Each mode as the command line spells it, paired with the [`config::Mode`]
/// whose `[settings]` rows that subcommand reads.
///
/// The two enums name the same pair of subcommands from opposite sides: clap's
/// [`Mode`] carries a mode's fully parsed configuration and so exists only after
/// argv is parsed, while [`config::Mode`] is the bare name and so can be known
/// before it — which is what the configuration files need, since their key set
/// is the mode's. This table is where the two are tied together, and
/// `every_mode_argument_is_a_settings_key_and_back` is what keeps the tie
/// honest.
const MODES: [(&str, config::Mode); 2] = [
    (TAIL_MODE, config::Mode::Tail),
    (CLIP_MODE, config::Mode::Clip),
];

impl Mode {
    /// What this mode's clips record as having cut them. The subcommand name is
    /// spelled here rather than recovered from the parser, so a mode added to
    /// the enum is a compile error until it says what its clips are stamped
    /// with — and a clip cut by a later mode is told apart from a recorder's
    /// without opening the file it came from.
    fn producer(&self) -> Producer {
        let mode = match self {
            Mode::Tail(_) => "tail",
            Mode::Clip(_) => "clip",
        };
        Producer {
            program: PROGRAM,
            mode,
        }
    }
}

/// Recorder configuration for [`Mode::Tail`], parsed by clap from CLI flags with
/// a `MOMENTEDGE_*` environment-variable fallback per field (see
/// [`load_cli`]). The field doc comments are the `clipper tail --help` text:
/// the first line is the short help, the rest is shown under `--help`.
#[derive(Debug, Args)]
struct Config {
    /// Bag directory of the continuous recording that is tailed.
    #[arg(long, default_value = "./record")]
    record_dir: PathBuf,
    /// Directory finished clips are written to.
    #[arg(long, default_value = "./clipped")]
    out_dir: PathBuf,
    /// Seconds to wait past the window end for coverage before cutting.
    ///
    /// How long past the window end to keep waiting for the recording to
    /// cover `trigger_time + postroll` before cutting the clip from what is
    /// on disk. Coverage normally lags the wall clock by the recorder's flush
    /// latency only; the timeout fires when the recorded topics go quiet, and
    /// the clip then simply ends at the last data that exists.
    #[arg(long, default_value_t = 30)]
    grace_secs: u64,
    /// Number of concurrent clip extractions (extraction worker-pool size).
    ///
    /// How many clip extractions may run at once — the size of the extraction
    /// worker pool. The default of 1 serializes the bulk copies FIFO:
    /// extraction reads compete with the recorder's writes on the same disk,
    /// and concurrent copies inflate the recorder's flush latency (rosbag2's
    /// cache drops messages when it cannot drain). Waiting — postroll,
    /// coverage — is always concurrent; only the copy is queued. Raise on
    /// storage with IO headroom.
    #[arg(long, default_value_t = 1)]
    extract_parallelism: usize,
    /// Compression codec for written clips (none, lz4, zstd).
    ///
    /// `zstd` (the default) writes the smallest clips; `lz4` trades size for
    /// lower CPU; `none` skips recompression entirely. It is the only property
    /// of a clip's encoding this binary sets; everything else about the file
    /// layout belongs to [`clip::cut`].
    #[arg(long, value_enum, default_value_t = ClipCompression::Zstd)]
    clip_compression: ClipCompression,

    /// Where triggers come from and completions go.
    ///
    /// The one flag whose surface the `ros` cargo feature changes, so its help
    /// text is per build ([`TAIL_TRIGGER_SOURCE_HELP`] /
    /// [`TAIL_TRIGGER_SOURCE_LONG_HELP`]) rather than this doc comment, and its
    /// default is [`TAIL_DEFAULT_TRIGGER_SOURCE`]. The accepted values are the
    /// recorder's subset of [`TriggerSource`] ([`trigger_source_parser`]), so
    /// `--help` lists exactly what this build of this subcommand can select and
    /// nothing else — `param` belongs to `clipper clip` and is refused here.
    ///
    /// The source is the completion half too: `ros` drives the live
    /// subscription and the `Recorded` publish that answers it, `mcap` the
    /// in-recording triggers and the clip's move into `out_dir`
    /// ([`crate::interface`]).
    #[arg(
        long,
        value_parser = trigger_source_parser(config::Mode::Tail),
        default_value_t = TAIL_DEFAULT_TRIGGER_SOURCE,
        help = TAIL_TRIGGER_SOURCE_HELP,
        long_help = TAIL_TRIGGER_SOURCE_LONG_HELP,
    )]
    trigger_source: TriggerSource,

    /// Clock domain the clip window lives in: `log` or `publish`.
    ///
    /// `log` (the default) windows on each message's `log_time` — when the
    /// producer received it — the completeness-proof clock. `publish` windows on
    /// each message's `publish_time` — whatever the producer wrote there (a DDS
    /// source timestamp, a capture time); clipper never interprets it. Publish
    /// times can arrive out of order, so a message may land after the cut with an
    /// in-window `publish_time` and be lost — `--grace-secs` bounds the wait.
    /// The flag governs the anchor, window membership, extent selection, and the
    /// coverage a handler waits on; retention still ages files on `log_time`.
    #[arg(long, value_enum, default_value_t = TimeSource::Log)]
    time_source: TimeSource,

    /// Seconds to keep a finished recording indexed (and its fd open) for clip
    /// preroll.
    ///
    /// A finished (split/restart) recording is watched while it holds data newer
    /// than now minus this duration, so a trigger's preroll can still reach into
    /// it. The tail prunes expired recordings every poll (whole files only,
    /// never the one being recorded), so open fds and index memory stay bounded
    /// even when the recorder stops splitting or goes idle. Pruning forgets a
    /// recording in-memory; it does NOT delete the file unless
    /// --delete-old-files is set. Set this comfortably above the largest preroll
    /// any trigger will request: a preroll reaching past the watch floor may
    /// lose its oldest segment.
    #[arg(long, default_value_t = 600)]
    watch_old_files_duration: u64,

    /// Also delete a recording from disk when it is pruned past the watch floor.
    ///
    /// Off by default: clipper forgets old recordings in-memory but leaves the
    /// .mcap files for `ros2 bag record` / other consumers. When set, a prune
    /// unlinks the expired file too (whole files only, never the current one);
    /// an in-flight extraction's open fd keeps the inode readable until it
    /// finishes.
    #[arg(long, default_value_t = false)]
    delete_old_files: bool,
}

impl Config {
    fn grace(&self) -> Duration {
        Duration::from_secs(self.grace_secs)
    }

    fn watch_old_files(&self) -> Duration {
        Duration::from_secs(self.watch_old_files_duration)
    }
}

/// Configuration for [`Mode::Clip`]: the recording to cut from, where the clips
/// go, and where the triggers that name them come from. As with every mode, each
/// field falls back to its `MOMENTEDGE_*` environment variable ([`load_cli`])
/// and the field doc comments are the `clipper clip --help` text.
///
/// The five `--trigger-*` flags spell out the fields a `momentedge_msgs/Trigger`
/// carries, so a clip cut here states the same trigger a clip cut from a live
/// topic does. They belong to `--trigger-source param` alone and are all
/// `Option`, with no clap default between them: a default is indistinguishable
/// from a value the caller typed, and `--trigger-source mcap` — which takes
/// every trigger from the recording — refuses the flags on exactly that
/// distinction. What each source then needs, and what it refuses, is
/// [`Self::trigger_argument_fault`].
///
/// There is no clock-domain flag. The window lives on `log_time` ([`CLIP_TIME_SOURCE`]),
/// the clock a recording's summary states its message times on and the one a
/// completeness claim can be made about; `--time-source` belongs to the tail,
/// where coverage is something a caller waits for.
#[derive(Debug, Args)]
struct ClipConfig {
    /// The finished recording to cut the clips out of: one `.mcap` file, or a
    /// bag directory whose splits are read as one time-ordered collection.
    ///
    /// A directory is ordered by the recorder's own `metadata.yaml` where it
    /// wrote one, and by modification time where it did not — the
    /// copied-mid-recording case, since the recorder writes that file at
    /// shutdown. Every split is indexed on its own and has to satisfy the same
    /// contract on its own, so a directory holding one that does not is refused
    /// naming that recording. A window straddling a split is cut into one
    /// segment per contributing recording.
    recording: PathBuf,

    /// Directory the finished clips are written to.
    #[arg(long)]
    out_dir: PathBuf,

    /// Where this run's triggers come from.
    ///
    /// Its help text is spelled out ([`CLIP_TRIGGER_SOURCE_HELP`] /
    /// [`CLIP_TRIGGER_SOURCE_LONG_HELP`]) rather than taken from this doc
    /// comment, because it has to say which flags each source reads; its default
    /// is [`CLIP_DEFAULT_TRIGGER_SOURCE`]. The accepted values are the cutter's
    /// subset of [`TriggerSource`] ([`trigger_source_parser`]), so `--help`
    /// lists exactly what can be selected — `ros` is a live subscription and a
    /// finished recording has no live topic, so it is refused here.
    #[arg(
        long,
        value_parser = trigger_source_parser(config::Mode::Clip),
        default_value_t = CLIP_DEFAULT_TRIGGER_SOURCE,
        help = CLIP_TRIGGER_SOURCE_HELP,
        long_help = CLIP_TRIGGER_SOURCE_LONG_HELP,
    )]
    trigger_source: TriggerSource,

    /// The instant the clip window centres on, in nanoseconds since the epoch
    /// (`--trigger-source param` only, and required there).
    ///
    /// The window is `[trigger-time - preroll, trigger-time + postroll]` on the
    /// recording's `log_time`, and the instant also names the clip
    /// (`<trigger-time>_<trigger-name>.mcap`). There is no default: the one
    /// thing only the caller knows is which moment the clip is about. Under
    /// `--trigger-source mcap` each recorded trigger's own log time is that
    /// instant, and passing this flag is a parse error.
    #[arg(long)]
    trigger_time: Option<u64>,

    /// Nanoseconds before the trigger instant to include in the clip
    /// (`--trigger-source param` only, and required there).
    #[arg(long)]
    preroll: Option<u64>,

    /// Nanoseconds after the trigger instant to include in the clip
    /// (`--trigger-source param` only, and required there).
    #[arg(long)]
    postroll: Option<u64>,

    /// The trigger's name, which also names the clip file
    /// (`--trigger-source param` only; defaults to `clip`).
    ///
    /// Carried into the clip's manifest under `trigger.name` and embedded in the
    /// output filename, so it is bounded and kept filename-safe the same way a
    /// name arriving on a topic is.
    #[arg(long)]
    trigger_name: Option<String>,

    /// The trigger's description, carried into the clip's manifest
    /// (`--trigger-source param` only; defaults to empty).
    #[arg(long)]
    trigger_description: Option<String>,
}

/// One trigger this run cuts a clip for, and the instant its window centres on.
///
/// The anchor rides beside the trigger rather than inside it because the two
/// sources resolve it differently: `--trigger-time` names it outright, while a
/// trigger the recording carries is anchored on the `log_time` the recording
/// stamped its trigger message with — the same stamp the recorder's `mcap`
/// `mcap` source anchors on, so a clip cut here and the one the device cut from
/// that trigger centre on the same instant.
#[derive(Debug, Clone)]
struct AnchoredTrigger {
    trigger: Trigger,
    anchor_ns: u64,
}

impl ClipConfig {
    /// Every flag that states part of a command-line trigger, paired with
    /// whether this command line gave it, in `--help` order — so a run that got
    /// several of them wrong is told about the first.
    fn trigger_params(&self) -> [(&'static str, bool); 5] {
        [
            ("--trigger-time", self.trigger_time.is_some()),
            ("--preroll", self.preroll.is_some()),
            ("--postroll", self.postroll.is_some()),
            ("--trigger-name", self.trigger_name.is_some()),
            ("--trigger-description", self.trigger_description.is_some()),
        ]
    }

    /// The trigger the `--trigger-*` flags name, or the first flag that keeps
    /// them from naming one.
    ///
    /// One function, called twice with the same answer: [`parse_cli`] calls it
    /// to end a bad command line before anything is written, and the cut calls
    /// it for the trigger it builds. The flags that have to be there and the
    /// trigger they add up to are one fact, so they are decided in one place.
    fn param_trigger(&self) -> Result<AnchoredTrigger, TriggerArgFault> {
        let anchor_ns = self
            .trigger_time
            .ok_or(TriggerArgFault::Missing("--trigger-time"))?;
        let preroll = self.preroll.ok_or(TriggerArgFault::Missing("--preroll"))?;
        let postroll = self
            .postroll
            .ok_or(TriggerArgFault::Missing("--postroll"))?;
        Ok(AnchoredTrigger {
            trigger: Trigger {
                name: self
                    .trigger_name
                    .clone()
                    .unwrap_or_else(|| DEFAULT_TRIGGER_NAME.to_string()),
                description: self.trigger_description.clone().unwrap_or_default(),
                // The instant is the anchor, and the stamp the trigger carries
                // states the same instant — a clip cut here echoes the trigger a
                // clip cut from a live topic echoes.
                trigger_time: clip::Stamp::from_ns(anchor_ns),
                preroll,
                postroll,
            },
            anchor_ns,
        })
    }

    /// The `--trigger-*` flag this command line got wrong, if any: one
    /// `--trigger-source param` needs and did not get, or one
    /// `--trigger-source mcap` has no use for.
    ///
    /// This is the cross-field check clap's derive cannot state, because the
    /// requirement depends on another flag's *value* rather than its presence:
    /// an absent `--trigger-source` still selects `param`, so a conflict or a
    /// requirement keyed on the flag being given would miss the default run
    /// entirely.
    fn trigger_argument_fault(&self) -> Option<TriggerArgFault> {
        if self.trigger_source.reads_the_trigger_flags() {
            return self.param_trigger().err();
        }
        self.trigger_params()
            .into_iter()
            .find(|(_, given)| *given)
            .map(|(flag, _)| TriggerArgFault::Conflicting(flag))
    }
}

/// Prefix shared by every `MOMENTEDGE_*` environment variable.
/// [`with_env_prefix`] applies it to every argument of every [`Mode`], so the
/// env names track the field names (`grace_secs` → `MOMENTEDGE_GRACE_SECS`)
/// with no per-field wiring.
const ENV_PREFIX: &str = "MOMENTEDGE";

/// Give every mode's arguments an environment-variable fallback named
/// `<ENV_PREFIX>_` + the field name upper-cased (`record_dir` →
/// `MOMENTEDGE_RECORD_DIR`). One place defines the prefix; the auto-generated
/// `--help`/`--version` flags are left without an env binding.
///
/// The flags live on the subcommands, not on `clipper` itself, so the walk goes
/// through every subcommand. The names are collected first because
/// [`clap::Command::mut_subcommand`] consumes the command it mutates.
fn with_env_prefix(cmd: clap::Command) -> clap::Command {
    let modes: Vec<String> = cmd
        .get_subcommands()
        .map(|sub| sub.get_name().to_owned())
        .collect();
    modes.into_iter().fold(cmd, |cmd, mode| {
        cmd.mut_subcommand(mode, |sub| {
            sub.mut_args(|arg| match arg.get_id().as_str() {
                "help" | "version" => arg,
                id => {
                    let env = format!("{ENV_PREFIX}_{}", id.to_uppercase());
                    arg.env(env)
                }
            })
        })
    })
}

/// The line appended to a parse failure that left the mode unnamed, since
/// clap's own text says the argument is unexpected without saying where it
/// belongs. `None` for a failure that named a mode (a bad value for one of its
/// flags), and for `--help`/`--version`, which are not failures at all.
fn mode_hint(kind: clap::error::ErrorKind) -> Option<&'static str> {
    use clap::error::ErrorKind::{
        DisplayHelpOnMissingArgumentOrSubcommand, InvalidSubcommand, MissingSubcommand,
        UnknownArgument,
    };
    matches!(
        kind,
        UnknownArgument
            | InvalidSubcommand
            | MissingSubcommand
            | DisplayHelpOnMissingArgumentOrSubcommand
    )
    .then_some(
        "clipper runs one mode per invocation, named as a subcommand. \
         The recorder is `clipper tail` and the one-shot cutter is \
         `clipper clip` — try `clipper tail --help`.",
    )
}

/// The three flags every mode carries that are *about* the configuration rather
/// than in it: the two file locations and the request to print what they came
/// to. They are injected onto each mode rather than declared as fields of
/// [`Config`] and [`ClipConfig`], so one definition serves every mode and no
/// mode's own struct grows a field it never reads.
///
/// Their ids are what [`with_env_prefix`] turns into `MOMENTEDGE_CONFIG`,
/// `MOMENTEDGE_SYSTEM_CONFIG` and `MOMENTEDGE_PRINT_CONFIG`. None of them is a
/// `[settings]` key, so a configuration file can never name another one.
const CONFIG_ARG: &str = "config";
const SYSTEM_CONFIG_ARG: &str = "system_config";
const PRINT_CONFIG_ARG: &str = "print_config";

/// Add the three configuration flags to every mode.
fn with_config_args(cmd: clap::Command) -> clap::Command {
    let modes: Vec<String> = cmd
        .get_subcommands()
        .map(|sub| sub.get_name().to_owned())
        .collect();
    modes.into_iter().fold(cmd, |cmd, mode| {
        cmd.mut_subcommand(mode, |sub| {
            sub.arg(
                clap::Arg::new(CONFIG_ARG)
                    .long("config")
                    .value_name("PATH")
                    .value_parser(clap::value_parser!(PathBuf))
                    .help("Per-run configuration file (TOML)")
                    .long_help(
                        "Per-run configuration file (TOML). It may set the keys this run \
                         decides — where the clips go, how long to wait, which topics to \
                         cut — and a key reserved to the system file is refused by name. \
                         Optional: there is no per-run file unless one is named.",
                    ),
            )
            .arg(
                clap::Arg::new(SYSTEM_CONFIG_ARG)
                    .long("system-config")
                    .value_name("PATH")
                    .value_parser(clap::value_parser!(PathBuf))
                    .help("System configuration file (TOML)")
                    .long_help(
                        "System configuration file (TOML), read from \
                         /etc/momentedge/clipper.toml unless this moves it. It may set \
                         every key. A file that is not there is not an error.",
                    ),
            )
            .arg(
                clap::Arg::new(PRINT_CONFIG_ARG)
                    .long("print-config")
                    .action(clap::ArgAction::SetTrue)
                    .help("Print the effective configuration and exit")
                    .long_help(
                        "Print every setting this run would use, the value it resolved to \
                         and the layer that decided it, then exit. The same text is logged \
                         at startup.",
                    ),
            )
        })
    })
}

/// The value `flag` carries in `argv`, in either spelling (`--flag VALUE` or
/// `--flag=VALUE`), or `None`.
///
/// The configuration files decide the defaults the parser is *built* with, so
/// they have to be known before the parser exists — which is why this is a scan
/// and not a parse. clap parses the same two flags afterwards, so a bad spelling
/// is still reported the ordinary way. The scan stops at `--`, past which
/// nothing is a flag, and works on bytes so a path that is not UTF-8 is found in
/// both spellings.
#[expect(
    clippy::similar_names,
    reason = "`argv` is the whole command line and `args` the cursor walking it; \
              both names are the ones the scan is about"
)]
fn scan_flag(argv: &[std::ffi::OsString], flag: &str) -> Option<PathBuf> {
    use std::os::unix::ffi::OsStrExt;

    let with_eq = format!("{flag}=");
    let mut args = argv.iter();
    while let Some(arg) = args.next() {
        let bytes = arg.as_bytes();
        if bytes == b"--" {
            return None;
        }
        if bytes == flag.as_bytes() {
            return args.next().map(PathBuf::from);
        }
        if let Some(value) = bytes.strip_prefix(with_eq.as_bytes()) {
            return Some(PathBuf::from(std::ffi::OsStr::from_bytes(value)));
        }
    }
    None
}

/// The mode `argv` names, or `None` where it names none.
///
/// Which keys the configuration files may carry is the mode's
/// ([`config::Mode`]), so the mode has to be known before [`Layered::load`]
/// runs — which is before clap has parsed anything. Like [`scan_flag`], this is
/// therefore a scan and not a parse.
///
/// The scan reads one word, `argv[1]`, and that is sound because nothing can
/// stand between `clipper` and its subcommand: [`Cli`] declares no argument of
/// its own, so the only flags `clipper` itself takes are clap's generated
/// `--help` and `--version`, neither of which takes a value and both of which
/// end the run; and the three configuration flags are injected onto the
/// subcommands by [`with_config_args`], not onto the root. A command line whose
/// second word is not a mode is one clap is about to reject.
///
/// `None` is "this command line names no mode" — a bare `clipper`, `clipper
/// --help`, a misspelt subcommand. There is no key set to read a file against,
/// so no file is read and clap reports the command line the way it always does.
fn scan_mode(argv: &[std::ffi::OsString]) -> Option<config::Mode> {
    let word = argv.get(1)?;
    MODES
        .iter()
        .find_map(|(name, mode)| (word == name).then_some(*mode))
}

/// Where the two configuration files are for this run: the command line first,
/// then the environment variable clap would have read for the same flag, then
/// nothing (the system file falls back to its built-in location inside
/// [`Layered::load`]).
fn config_paths(argv: &[std::ffi::OsString]) -> (Option<PathBuf>, Option<PathBuf>) {
    let located = |flag: &str, id: &str| {
        scan_flag(argv, flag).or_else(|| {
            std::env::var_os(format!("{ENV_PREFIX}_{}", id.to_uppercase())).map(PathBuf::from)
        })
    };
    (
        located("--system-config", SYSTEM_CONFIG_ARG),
        located("--config", CONFIG_ARG),
    )
}

/// Give each argument the value the configuration files resolved for it, as its
/// default.
///
/// This is the whole join between the file layers and the top two: clap already
/// resolves a flag over an environment variable over a default, so handing it a
/// file's value *as* the default puts the four layers in the documented order
/// with no further wiring — and makes a file's value pass exactly the value
/// parser a flag's value passes. An argument no file named keeps the default
/// compiled into it.
fn with_file_defaults(cmd: clap::Command, layered: &Layered) -> clap::Command {
    let modes: Vec<String> = cmd
        .get_subcommands()
        .map(|sub| sub.get_name().to_owned())
        .collect();
    modes.into_iter().fold(cmd, |cmd, mode| {
        cmd.mut_subcommand(mode, |sub| {
            sub.mut_args(|arg| match layered.setting(arg.get_id().as_str()) {
                // `required` is cleared with the same stroke: an argument a file
                // answers has been provided, and clap's required check does not
                // count a default as an answer.
                Some(setting) => arg
                    .default_value(setting.value().to_string())
                    .required(false),
                None => arg,
            })
        })
    })
}

/// The column the `=` and the `<-` of a report line are aligned on: the longest
/// key (`watch_old_files_duration`) and a value column wide enough for the
/// ordinary ones. A longer value pushes its own origin right rather than moving
/// every other line's.
const REPORT_KEY_WIDTH: usize = 24;
const REPORT_VALUE_WIDTH: usize = 22;

/// The effective configuration: every setting this run uses, the value it
/// resolved to, and the layer that decided it.
///
/// It is read back out of the **parsed** command line — the same `ArgMatches`
/// the mode's own struct is built from — rather than re-derived from the layers,
/// so the report cannot claim a value the run does not use. clap knows the top
/// two layers apart (a flag from an environment variable); everything it calls a
/// default is a file's value or the built-in one, which [`Layered`] tells apart.
fn effective_config(
    mode: &str,
    args: &clap::Command,
    matches: &clap::ArgMatches,
    layered: &Layered,
) -> String {
    let mut out = format!("clipper {mode} effective configuration\n  [settings]\n");
    // The mode's own arguments, taken off the command rather than out of the
    // matches: `ArgMatches::ids` also yields the argument *group* clap's derive
    // names after the mode's config struct, which is not a setting.
    let mut keys: Vec<&str> = args
        .get_arguments()
        .map(|arg| arg.get_id().as_str())
        .filter(|id| {
            !matches!(
                *id,
                CONFIG_ARG | SYSTEM_CONFIG_ARG | PRINT_CONFIG_ARG | "help" | "version"
            )
        })
        .collect();
    keys.sort_unstable();
    for key in keys {
        let value = matches
            .get_raw(key)
            .map(|values| {
                values
                    .map(|v| v.to_string_lossy().into_owned())
                    .collect::<Vec<_>>()
                    .join(", ")
            })
            .unwrap_or_default();
        let layer = match matches.value_source(key) {
            Some(clap::parser::ValueSource::CommandLine) => Layer::Flag,
            Some(clap::parser::ValueSource::EnvVariable) => Layer::Env,
            // Everything else clap can report is a default it was built with,
            // which is a file's value where a file named the key.
            _ => layered
                .setting(key)
                .map_or(Layer::Builtin, clip::config::Setting::layer),
        };
        out.push_str(&report_line(key, &value, &layered.origin(layer)));
    }
    out.push_str("  [topics]\n");
    for (key, setting) in layered.topics() {
        out.push_str(&report_line(
            key,
            setting.value(),
            &layered.origin(setting.layer()),
        ));
    }
    if !layered.refusals().is_empty() {
        out.push_str("  refused\n");
        #[expect(
            clippy::format_push_string,
            reason = "a startup report built once; `write!` into a String would \
                      trade a clear line for a discarded `Result` that cannot fail"
        )]
        for refusal in layered.refusals() {
            out.push_str(&format!("    {refusal}\n"));
        }
    }
    out
}

fn report_line(key: &str, value: &str, origin: &str) -> String {
    format!("    {key:<REPORT_KEY_WIDTH$} = {value:<REPORT_VALUE_WIDTH$} <- {origin}\n")
}

/// A fully-resolved command line: the mode, the topic selection its clips are
/// cut with, and the effective-configuration report.
struct Loaded {
    cli: Cli,
    selection: clip::ChannelSelection,
    report: String,
    print_config: bool,
}

/// What can go wrong before the run begins.
enum StartupError {
    /// clap's own: a parse error, or `--help`/`--version`, which clap reports
    /// through the same channel.
    Cli(clap::Error),
    /// A configuration file that exists and cannot be used.
    Config(anyhow::Error),
}

/// Parse `argv` into a fully-resolved [`Loaded`], four layers deep.
///
/// The mode and then the two configuration files are located in `argv` (and the
/// environment) first, since the mode decides which keys the files may carry and
/// what they say becomes the parser's defaults; clap then resolves the flag and
/// the environment on top of them, so a setting present in all four layers comes
/// out of the strongest that named it.
#[expect(
    clippy::similar_names,
    reason = "`argv` is the command line and `args` the parsed mode's argument \
              definition; both names are the ones this function is about"
)]
fn parse_cli(argv: &[std::ffi::OsString]) -> Result<Loaded, StartupError> {
    // A command line that names no mode names no key set either, so there is
    // nothing to read a file against: the parser is built on its own defaults
    // and clap answers the missing or misspelt mode itself.
    let layered = match scan_mode(argv) {
        Some(mode) => {
            let (system, run) = config_paths(argv);
            Some(
                Layered::load(mode, system.as_deref(), run.as_deref())
                    .map_err(StartupError::Config)?,
            )
        }
        None => None,
    };
    let cmd = with_env_prefix(with_config_args(Cli::command()));
    let cmd = match &layered {
        Some(layered) => with_file_defaults(cmd, layered),
        None => cmd,
    };
    // The command is cloned before parsing consumes it, so the report can list
    // the mode's arguments rather than guess at them from the matches.
    let definition = cmd.clone();
    let matches = cmd.try_get_matches_from(argv).map_err(StartupError::Cli)?;
    #[expect(
        clippy::expect_used,
        reason = "`Cli` requires the subcommand and takes no argument of its own, \
                  so a parse that succeeded named a mode as the word `scan_mode` \
                  read — which means the files were read for it"
    )]
    let layered = layered.expect("a parsed Cli named the mode the scan found");
    #[expect(
        clippy::expect_used,
        reason = "`Cli` has `subcommand_required`, so a successful parse named a \
                  mode, and that mode is by construction one the definition carries"
    )]
    let (mode, sub) = matches
        .subcommand()
        .expect("a parsed Cli names one of its modes");
    #[expect(
        clippy::expect_used,
        reason = "as above — `mode` came out of the same command definition"
    )]
    let args = definition
        .find_subcommand(mode)
        .expect("the parsed mode is one of the command's subcommands");
    let report = effective_config(mode, args, sub, &layered);
    let print_config = sub.get_flag(PRINT_CONFIG_ARG);
    let cli = Cli::from_arg_matches(&matches).map_err(StartupError::Cli)?;

    // The one thing clap's derive cannot state: a flag whose requirement or
    // conflict depends on another flag's *value*
    // ([`ClipConfig::trigger_argument_fault`]). Checked as part of parsing, so
    // both ways of naming a trigger wrongly end the process with the flag at
    // fault named and nothing written — no output directory, no clip.
    if let Mode::Clip(cfg) = &cli.mode
        && let Some(fault) = cfg.trigger_argument_fault()
    {
        // Raised against `clipper clip` so the usage line clap prints under
        // the message is the mode's own flag list, not the mode listing.
        #[expect(
            clippy::expect_used,
            reason = "reached only from the `Mode::Clip` arm above, so the parse \
                      already resolved `CLIP_MODE` against this same definition"
        )]
        let mut clip = definition
            .find_subcommand(CLIP_MODE)
            .expect("clip is a mode of clipper")
            .clone()
            .bin_name(format!("{PROGRAM} {CLIP_MODE}"));
        return Err(StartupError::Cli(clip.error(fault.kind(), fault.message())));
    }

    Ok(Loaded {
        cli,
        selection: layered.into_selection(),
        report,
        print_config,
    })
}

/// Parse the process's command line, or end the process saying why.
///
/// Diverges the way [`clap::Error::exit`] does — printing the message and ending
/// the process — for `--help`, `--version` and any parse error, so this returns
/// only a fully-populated mode. A failure that left the mode unnamed carries
/// [`mode_hint`] after clap's own text. A configuration file that cannot be used
/// ends the run the same way, at clap's usage exit code: it is the same class of
/// mistake, made in a file instead of on the command line.
fn load_cli() -> Loaded {
    match parse_cli(&std::env::args_os().collect::<Vec<_>>()) {
        Ok(loaded) => loaded,
        Err(StartupError::Cli(err)) => {
            let hint = mode_hint(err.kind());
            let code = err.exit_code();
            let _ = err.print();
            if let Some(hint) = hint {
                eprintln!("\n{hint}");
            }
            std::process::exit(code);
        }
        Err(StartupError::Config(err)) => {
            eprintln!("clipper: {err:#}");
            std::process::exit(CONFIG_EXIT_CODE);
        }
    }
}

/// The status a configuration file this run cannot use exits with — clap's own
/// usage code, since a file that names an unknown key is the same mistake as a
/// command line that does.
const CONFIG_EXIT_CODE: i32 = 2;

/// Deliver SIGINT/SIGTERM as a message on the returned channel: a dedicated
/// thread blocks on signal-hook's iterator and forwards the first shutdown
/// signal for [`supervise`] to select on. Both signals mean the same
/// requested, orderly stop (the process exits zero) — SIGTERM is what process
/// supervisors send first.
fn signal_channel() -> anyhow::Result<Receiver<i32>> {
    let (tx, rx) = bounded(1);
    let mut signals = signal_hook::iterator::Signals::new([SIGINT, SIGTERM])?;
    thread::Builder::new()
        .name("signals".to_string())
        .spawn(move || {
            if let Some(sig) = signals.forever().next() {
                let _ = tx.send(sig);
            }
        })?;
    Ok(rx)
}

fn signal_name(sig: i32) -> &'static str {
    match sig {
        SIGINT => "SIGINT",
        SIGTERM => "SIGTERM",
        _ => "shutdown signal",
    }
}

/// Bounded admission for trigger handlers: [`MAX_ACTIVE_TRIGGERS`] permits
/// bound how many may be active (admitted, waiting, or extracting) at once.
/// The consumer takes a permit without waiting before spawning a handler
/// thread; the permit rides in the thread and returns when the handler
/// finishes — panic included, since it returns on drop, which unwinding
/// covers — so the bound never ratchets down.
struct Admission {
    active: AtomicUsize,
    limit: usize,
}

impl Admission {
    fn new(limit: usize) -> Arc<Self> {
        Arc::new(Admission {
            active: AtomicUsize::new(0),
            limit,
        })
    }

    /// Take a permit if one is free, without waiting. `None` means every
    /// permit is held by an active handler — the caller rejects the trigger.
    fn try_acquire(self: Arc<Self>) -> Option<AdmissionPermit> {
        self.active
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| {
                (n < self.limit).then_some(n + 1)
            })
            .ok()
            .map(|_| AdmissionPermit(self))
    }
}

/// An admitted handler's slot; returns to the [`Admission`] count on drop.
struct AdmissionPermit(Arc<Admission>);

impl Drop for AdmissionPermit {
    fn drop(&mut self) {
        self.0.active.fetch_sub(1, Ordering::AcqRel);
    }
}

/// Entry point: install logging, then run the mode the command line named.
///
/// The `match` is the dispatch table over [`Mode`], so a mode added to the enum
/// is a compile error here until it has a body to run.
fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Logs go to stdout: there is no machine-readable contract on that stream
    // and there will not be one — a run's result is the contents of `out_dir`
    // when the process exits, each clip carrying its own metadata record — so
    // the stream an operator reads first is free for human-readable output.
    // `Target` comes through the `env_logger` that `pretty_env_logger`
    // re-exports; no direct dependency on it.
    pretty_env_logger::formatted_builder()
        .target(pretty_env_logger::env_logger::fmt::Target::Stdout)
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let loaded = load_cli();
    // The effective configuration goes out on demand and at startup alike, from
    // the one report, so a run's log states the configuration it ran with in the
    // words `--print-config` would have used.
    if loaded.print_config {
        println!("{}", loaded.report);
        return Ok(());
    }
    info!("{}", loaded.report);

    // The producer is read off the mode before it is destructured, so every clip
    // the run writes is stamped with the subcommand that produced it.
    let mode = loaded.cli.mode;
    let producer = mode.producer();
    let selection = loaded.selection;
    match mode {
        Mode::Tail(cfg) => tail_mode(cfg, producer, selection),
        Mode::Clip(cfg) => clip_mode(cfg, producer, selection).map_err(Into::into),
    }
}

/// `clipper tail`: supervisor for the device recorder. Spawns the long-lived
/// threads — the tail, the interface (which owns its own trigger source, and for
/// ROS its node spin), the staging worker pool, and the signal forwarder — then
/// blocks in [`supervise`] until a shutdown signal (exit 0) or the first dead
/// critical thread (exit non-zero, for a supervisor to restart the process).
///
/// Returning ends the process, which kills the remaining threads: the
/// immortal tail and interface loops, parked handlers, and any in-flight
/// extraction.
/// That is safe for clips by construction — the capturing-dir reset at
/// startup reclaims any stranded staged file, and `out_dir` only ever holds
/// complete clips.
fn tail_mode(
    cfg: Config,
    producer: Producer,
    selection: clip::ChannelSelection,
) -> Result<(), Box<dyn std::error::Error>> {
    let cfg = Arc::new(cfg);

    // Start each run with a clean capturing dir: a crash mid-publish can strand
    // a stale staged file there, and clearing it at startup bounds that clutter
    // to a single run. This also creates out_dir, so the first clip can be
    // published without further setup. Fatal if it fails — a recorder that
    // cannot prepare its output directory must not start.
    clip::cut::reset_capturing_dir(&cfg.out_dir)?;

    // One staging worker per allowed concurrent clip copy; see
    // Config::extract_parallelism. The clip compression codec and the topic
    // selection are process-global, captured in the workers; each window's clock
    // domain travels with its job.
    let extract_tx = segment::spawn_stage_workers(
        cfg.extract_parallelism,
        cfg.clip_compression.to_mcap(),
        selection,
    );

    // Admission gate for trigger handlers; see [`Admission`].
    let admission = Admission::new(MAX_ACTIVE_TRIGGERS);

    if !cfg.record_dir.is_dir() {
        warn!(
            "record dir {} does not exist; the tail idles until the continuous \
             recording (scripts/record.sh) creates it",
            cfg.record_dir.display()
        );
    }

    // Build the tailer and the interface the trigger source selects together,
    // then drive the recorder with them. This match is the pairing: the source
    // names both halves of the seam, so `ros` takes the live subscription and
    // the `Recorded` publish that answers it, and `mcap` takes the in-recording
    // triggers and the clip's move into `out_dir` as its only completion signal
    // (`Interface::SOURCE` on each is the same fact, stated on the type).
    // Exactly one is active; `drive` is generic over it (static dispatch, no
    // `Box<dyn>`).
    //
    // The MCAP interface drives off a decode-free trigger tap — the tail lifts
    // trigger-topic messages out of the recording — so its arm wires the tap
    // channel and hands the receiver to the interface; the ROS interface reads
    // triggers from a live subscription and needs no tap, so its tailer is built
    // without one. The ROS arm exists only where the `ros` feature compiled that
    // interface in — without it the variant does not exist and this match has
    // two arms.
    let result = match cfg.trigger_source {
        #[cfg(feature = "ros")]
        TriggerSource::Ros => {
            let (tailer, coverage) = Tailer::new();
            let iface = RosInterface::new(TRIGGER_TOPIC, ANNOUNCE_TOPIC, cfg.time_source)?;
            drive(
                iface, cfg, tailer, coverage, extract_tx, admission, producer,
            )
        }
        TriggerSource::Mcap => {
            let (tx, rx) = unbounded();
            let (tailer, coverage) = Tailer::with_trigger_tap(TRIGGER_TOPIC, tx);
            let iface = McapInterface::new(TRIGGER_TOPIC, rx, cfg.time_source);
            drive(
                iface, cfg, tailer, coverage, extract_tx, admission, producer,
            )
        }
        TriggerSource::Param => Err(unaccepted_source(TAIL_MODE, TriggerSource::Param)),
    };
    result.map_err(Into::into)
}

/// The clock domain `clipper clip` cuts on.
///
/// Not a flag: a finished recording's summary states its message times on
/// `log_time` alone, so that is the clock a window over one can be planned and a
/// completeness claim made on. `--time-source` is the tail's, where the choice
/// changes what a handler waits for.
const CLIP_TIME_SOURCE: TimeSource = TimeSource::Log;

/// The codec `clipper clip` writes its clip with — the recorder's default,
/// spelled once here because this mode takes no compression flag.
const CLIP_MODE_COMPRESSION: ClipCompression = ClipCompression::Zstd;

/// The triggers this run cuts, in the order their clips are written: the one the
/// `--trigger-*` flags name, or every one the recording carries.
///
/// The `param` arm cannot fail on a command line [`parse_cli`] accepted — the
/// same [`ClipConfig::param_trigger`] decided both — so a fault here is that
/// gate's bug and is reported as one.
fn clip_triggers(cfg: &ClipConfig) -> anyhow::Result<Vec<AnchoredTrigger>> {
    match cfg.trigger_source {
        TriggerSource::Param => cfg
            .param_trigger()
            .map(|trigger| vec![trigger])
            .map_err(|fault| anyhow::anyhow!("{}", fault.message())),
        TriggerSource::Mcap => embedded_triggers(&cfg.recording),
        #[cfg(feature = "ros")]
        TriggerSource::Ros => Err(unaccepted_source(CLIP_MODE, TriggerSource::Ros)),
    }
}

/// The triggers the recording itself carries on [`TRIGGER_TOPIC`], each anchored
/// on the `log_time` the recording stamped its trigger message with.
///
/// The list exists before the first cut, because the input has an end: reading
/// it is a summary read plus the chunks that summary names as holding the
/// trigger channel ([`clip::embedded::read_triggers`]), and no other chunk is
/// decompressed to find it. A bag directory is read split by split, in the same
/// order its windows are planned in ([`clip::bag::splits`]), so the run's clips
/// come out in the order the triggers were recorded.
///
/// An undecodable trigger is logged and skipped rather than fatal, exactly as
/// the recorder's `mcap` source treats one: a single trigger nobody can read
/// must not cost the caller every other clip in the recording. A `cdr` payload
/// is undecodable in a build without the `ros` feature; `json` decodes in every
/// build.
fn embedded_triggers(recording: &std::path::Path) -> anyhow::Result<Vec<AnchoredTrigger>> {
    let mut triggers = Vec::new();
    for split in clip::bag::splits(recording)? {
        for record in clip::embedded::read_triggers(&split, TRIGGER_TOPIC)? {
            match clip::decode::decode_trigger(&record.message_encoding, &record.body) {
                Ok(trigger) => triggers.push(AnchoredTrigger {
                    trigger,
                    anchor_ns: record.log_time,
                }),
                Err(e) => warn!(
                    "skipping the trigger at log_time {} in {} (encoding={}): {e:#}",
                    record.log_time,
                    split.display(),
                    record.message_encoding,
                ),
            }
        }
    }
    Ok(triggers)
}

/// The triggers of `triggers` whose names may reach the filesystem, or the fault
/// that ends the run.
///
/// A trigger name is embedded in the clip's pathname, so it passes the gate
/// every trigger passes, whichever source it arrived from ([`validate_name`]).
/// What an unsafe one costs differs with who wrote it: a name the operator typed
/// is a command line to fix and ends the run, while one the recording carried
/// costs that trigger its clip and no more — the same isolation an undecodable
/// trigger gets.
fn with_usable_names(
    cfg: &ClipConfig,
    triggers: Vec<AnchoredTrigger>,
) -> anyhow::Result<Vec<AnchoredTrigger>> {
    let mut cuts = Vec::with_capacity(triggers.len());
    for anchored in triggers {
        let Err(why) = validate_name(&anchored.trigger.name) else {
            cuts.push(anchored);
            continue;
        };
        match cfg.trigger_source {
            TriggerSource::Param => {
                anyhow::bail!("--trigger-name {:?} {why}", anchored.trigger.name)
            }
            TriggerSource::Mcap => warn!(
                "skipping the trigger at log_time {} in {}: its name {:?} {why}",
                anchored.anchor_ns,
                cfg.recording.display(),
                anchored.trigger.name,
            ),
            #[cfg(feature = "ros")]
            TriggerSource::Ros => return Err(unaccepted_source(CLIP_MODE, TriggerSource::Ros)),
        }
    }
    Ok(cuts)
}

/// `clipper clip`: cut one clip per trigger out of one finished recording and
/// exit.
///
/// The recording is indexed from its own summary ([`clip::whole::WholeFileIndex`])
/// — a footer seek and one read, no chunk decompressed — and handed to the same
/// [`clip::segment::cut_window`] the recorder drives, so each clip is
/// byte-for-byte what the device would have cut from the same recording and
/// window.
///
/// **The input is one recording or a bag directory of them.** A directory is
/// read as one time-ordered collection ([`clip::bag`]), so a window straddling
/// a split is cut into one segment per contributing recording — the same
/// `<anchor>_<name>_NN.mcap` set the device writes when a window straddles a
/// rollover — and a segment number is the position after the recordings that
/// contributed nothing are dropped, not the split's place in the directory.
///
/// **How many clips a run writes is the trigger source's answer**
/// ([`clip_triggers`]). `--trigger-source param` names one trigger and writes
/// one clip. `--trigger-source mcap` writes one per trigger the recording
/// carries — none at all for a recording that carries none, which is a normal,
/// zero-status run that says so and leaves the output directory untouched.
///
/// **The waits are what is absent.** `tail::handler` sleeps until the wall clock
/// passes the window end, then blocks until the tail's coverage reaches it,
/// because a window may reach past the last byte on disk. This input has an end:
/// there is nothing to wait for, so a window reaching past it is simply short,
/// and the manifest says so ([`clip::manifest::WindowCoverage`]).
///
/// **A recording it cannot index is refused by name**, from that same footer
/// and summary, before anything is created: [`clip::whole::IndexRefusal`] is
/// the taxonomy, the message names the fault and the `mcap` command that
/// repairs it, and the run exits non-zero having written nothing — not the
/// output directory, not the staging directory inside it. Every split of a bag
/// directory faces that contract on its own, so a directory is refused naming
/// the one recording in it that failed. clipper runs no repair itself;
/// recovering and re-indexing a recording are the operator's.
///
/// **A clip that is already there is refused too**
/// ([`clip::segment::Publication::Refuse`]). A finished recording and a trigger
/// describe one window and one copy of its bytes, so a re-run over both writes
/// the clip that is already in the output directory: the run names it, exits
/// non-zero, and stages nothing, rather than publishing a second copy beside it
/// the way the recorder does for a second live trigger. There is no flag to
/// override it — an operator who wants the clip again removes it or names
/// another `--out-dir`.
///
/// Nothing is printed for a caller to parse. The result is the output
/// directory's contents when the process exits, each clip carrying its own
/// manifest; the exit status is the verdict.
#[expect(
    clippy::needless_pass_by_value,
    reason = "the mode owns the config clap parsed for it; `main` hands it over and \
              keeps nothing"
)]
fn clip_mode(
    cfg: ClipConfig,
    producer: Producer,
    selection: clip::ChannelSelection,
) -> anyhow::Result<()> {
    // Index the recording — every split of it, for a bag directory — before
    // anything else touches it, including before its own triggers are read: a
    // recording clipper cannot index is refused by name
    // (`clip::whole::IndexRefusal`) from the footer and summary alone, so the
    // refusal is the same whichever source the run's triggers come from and no
    // chunk is decompressed to reach it. Reading triggers first would let an
    // `mcap` run walk an input the contract refuses.
    let index = clip::whole::WholeFileIndex::open(&cfg.recording)?;

    let triggers = clip_triggers(&cfg)?;

    let cuts = with_usable_names(&cfg, triggers)?;

    // A run with nothing to cut is a normal run: it writes no clip, creates no
    // output directory, and says why. Only `mcap` reaches this — `param` either
    // yields its one trigger or has already failed.
    if cuts.is_empty() {
        info!(
            "{} carries no trigger on {TRIGGER_TOPIC}; nothing to cut",
            cfg.recording.display(),
        );
        return Ok(());
    }

    // Start from a clean capturing dir, which also creates out_dir: a clip is
    // assembled there and hard-linked into place, so the output directory only
    // ever holds complete clips.
    clip::cut::reset_capturing_dir(&cfg.out_dir)?;

    // One copy at a time: the windows are cut in trigger order, and the pool is
    // sized to the work in front of it. It exists at all because staging is the
    // pool's job either way.
    let stage_tx = segment::spawn_stage_workers(1, CLIP_MODE_COMPRESSION.to_mcap(), selection);

    info!(
        "cutting {} clip(s) from {} recording(s) under {} into {} (source={CLIP_TIME_SOURCE})",
        cuts.len(),
        index.splits().len(),
        cfg.recording.display(),
        cfg.out_dir.display(),
    );

    for AnchoredTrigger { trigger, anchor_ns } in cuts {
        let request = Arc::new(clip::CutRequest::new(
            producer,
            trigger.clone(),
            anchor_ns,
            CLIP_TIME_SOURCE,
        ));

        // The one thing a finished clip cannot show from its own contents:
        // whether the recording ever reached the window end, or simply stops
        // inside it.
        let coverage = if index
            .log_end_ns()
            .is_some_and(|end_ns| end_ns >= request.end_ns())
        {
            clip::WindowCoverage::Covered
        } else {
            warn!(
                "{} ends before the window end {}; the clip stops where the \
                 recording does",
                cfg.recording.display(),
                request.end_ns(),
            );
            clip::WindowCoverage::Short
        };

        info!(
            "cutting {} window=[{}, {}] anchor={anchor_ns}",
            trigger.name,
            request.start_ns(),
            request.end_ns(),
        );

        let base_out_path = cfg.out_dir.join(format!(
            "{anchor_ns}_{}.mcap",
            segment::sanitize(&trigger.name)
        ));
        let segments = segment::cut_window(
            &index,
            &request,
            coverage,
            &base_out_path,
            segment::Publication::Refuse,
            &stage_tx,
        )?;

        #[expect(
            clippy::cast_precision_loss,
            reason = "a log line's MiB figure; the loss starts past 8 PiB in one clip"
        )]
        for stats in &segments {
            info!(
                "clip {} written: {} msgs from {} extents, {:.1} MiB",
                stats.out_path.display(),
                stats.messages_copied,
                stats.extents_read,
                stats.bytes_copied as f64 / 1_048_576.0,
            );
            if stats.records_skipped > 0 || stats.chunks_dropped > 0 {
                warn!(
                    "clip {} is missing data over damage in the recording: \
                     {} records skipped, {} chunks dropped",
                    stats.out_path.display(),
                    stats.records_skipped,
                    stats.chunks_dropped,
                );
            }
        }
    }
    Ok(())
}

/// The largest `preroll` or `postroll` a trigger may request, in nanoseconds
/// (30 minutes). A window wider than this is a malformed or runaway request, not
/// a real clip; the cap bounds how far a cut reaches back into the retained
/// recordings and how long a handler parks. The exact value is accepted.
const MAX_ROLL_NS: u64 = 1_800_000_000_000; // 30 * 60 * 1e9

/// The largest amount a resolved anchor may sit in the future of `now`, in
/// nanoseconds (30 minutes — the same horizon as [`MAX_ROLL_NS`], since both
/// bound how long one trigger can wedge a handler). The anchor drives the
/// postroll wall-floor sleep (`anchor + postroll`), so an anchor far in the
/// future parks a handler for that long; a wildly future anchor is a producer
/// clock fault or a hostile record stamp, never a real request. The guard is on
/// the *resolved* anchor, whatever cell produced it: `--trigger-source ros
/// --time-source log` resolves it to `now` and always passes, while a
/// `ros`+`publish` `trigger_time` or a tail record's own stamp is exactly what it
/// bites on — and a tail record's stamp is the only one a build without the
/// `ros` feature can present.
const MAX_ANCHOR_FUTURE_SKEW_NS: u64 = 1_800_000_000_000; // 30 * 60 * 1e9

/// The largest trigger `name`, in bytes. The name is embedded in the clip
/// pathname `<anchor_ns>_<name>.mcap`, so it is bounded and kept filename-safe
/// (see [`validate_name`]).
const MAX_TRIGGER_NAME_LEN: usize = 128;

/// The trigger-admission gate: whether a resolved trigger is cut into a clip, or
/// the reason it is rejected. Every incoming trigger passes through here (in the
/// interface-fired callback) before any handler work; a rejection logs at
/// `error!` and produces no clip and no `Recorded`. `now_ns` is the current
/// system clock, passed in so the gate stays a pure function (the future-skew
/// guard is the only clock-relative check). The checks, any one of which
/// rejects:
///
/// - **`trigger_time` in a cell that ignores it.** At most one cell of the
///   trigger-source × `--time-source` matrix reads `trigger_time` —
///   `--trigger-source ros --time-source publish`, where it *is* the anchor
///   ([`Anchor::from_trigger_time`]);
///   a build without the `ros` feature has no such cell, and every other cell
///   anchors on a transport stamp. Sending `trigger_time` where
///   it is ignored would silently anchor the window on the trigger's arrival
///   rather than the requested instant, so it is refused loudly. `trigger_time == 0`
///   is always accepted.
/// - **`preroll`/`postroll` past [`MAX_ROLL_NS`].**
/// - **A resolved anchor more than [`MAX_ANCHOR_FUTURE_SKEW_NS`] past `now`.**
///   The anchor — not `trigger_time` specifically — is the guarded value, since
///   it is what parks a handler through its postroll sleep whatever cell resolved
///   it.
/// - **A `name` that is empty, past [`MAX_TRIGGER_NAME_LEN`], or unsafe to embed
///   in the clip pathname** (see [`validate_name`]).
fn validate_trigger(trig: &Trigger, anchor: Anchor, now_ns: u64) -> Result<(), String> {
    if !anchor.from_trigger_time && trig.trigger_time.ns() != 0 {
        return Err(format!(
            "name={:?} sets trigger_time={} but the active --trigger-source and \
             --time-source anchor on a transport stamp and ignore it; send \
             trigger_time=0",
            trig.name,
            trig.trigger_time.ns(),
        ));
    }
    if trig.preroll > MAX_ROLL_NS {
        return Err(format!(
            "name={:?} preroll={} ns exceeds the {MAX_ROLL_NS} ns maximum",
            trig.name, trig.preroll,
        ));
    }
    if trig.postroll > MAX_ROLL_NS {
        return Err(format!(
            "name={:?} postroll={} ns exceeds the {MAX_ROLL_NS} ns maximum",
            trig.name, trig.postroll,
        ));
    }
    if anchor.ns > now_ns.saturating_add(MAX_ANCHOR_FUTURE_SKEW_NS) {
        return Err(format!(
            "name={:?} anchor {} ns is more than {MAX_ANCHOR_FUTURE_SKEW_NS} ns \
             past now ({now_ns} ns)",
            trig.name, anchor.ns,
        ));
    }
    if let Err(why) = validate_name(&trig.name) {
        return Err(format!("name={:?} {why}", trig.name));
    }
    Ok(())
}

/// Reject a trigger `name` that cannot be safely embedded in the clip pathname
/// `<anchor_ns>_<name>.mcap`. [`clip::segment::sanitize`] maps stray characters to `_`
/// at clip creation, but structural hazards — an empty name, a path separator or
/// NUL, a leading dot (a hidden file), or an embedded `..` (a parent-directory
/// escape) — are refused whole here rather than silently rewritten, so a
/// malformed request never reaches the filesystem in a surprising shape.
fn validate_name(name: &str) -> Result<(), &'static str> {
    if name.is_empty() {
        return Err("is empty");
    }
    if name.len() > MAX_TRIGGER_NAME_LEN {
        return Err("exceeds the name length limit");
    }
    if name.contains('\0') {
        return Err("contains a NUL byte");
    }
    if name.contains('/') || name.contains('\\') {
        return Err("contains a path separator");
    }
    if name.starts_with('.') {
        return Err("starts with a dot");
    }
    if name.contains("..") {
        return Err("contains '..'");
    }
    Ok(())
}

/// Wire one interface to the tail and run the recorder for the process's
/// lifetime, then supervise. Generic over the active [`Interface`] — static
/// dispatch, no `Box<dyn>`.
///
/// Spawns two long-lived companions over the supervision channels: the **tail**
/// thread (file scan feeding coverage and the extent index) and the
/// **interface** thread (`iface.run`, which drains its trigger source — a ROS
/// subscription or the MCAP tap — and fires the per-trigger callback). The
/// callback admits the trigger (a flood bound), then spawns one handler thread
/// that cuts the clip and announces through the interface's announcer; per-trigger
/// errors are isolated (logged, the permit returned on drop). The ROS interface
/// owns its own node spin internally, so supervision is uniform in either mode.
#[expect(
    clippy::needless_pass_by_value,
    reason = "`drive` owns the recorder's shared handles for the process's lifetime \
              and moves its own clones of them into the `'static` interface \
              callback; borrowing would push that lifetime back onto `main`"
)]
fn drive<I: Interface>(
    iface: I,
    cfg: Arc<Config>,
    tailer: Arc<Tailer>,
    coverage: Arc<Watch<Coverage>>,
    extract_tx: Sender<segment::StageJob>,
    admission: Arc<Admission>,
    producer: Producer,
) -> anyhow::Result<()> {
    let announcer = iface.announcer();

    // The callback the interface fires per decoded Trigger. `Fn` + `Send`: it is
    // moved into the single interface thread and called from there, never shared.
    let fire = {
        // The seam: the handler half reads only these settings, so unpack them
        // here rather than handing the CLI parser's `Config` down. The interface
        // resolves each trigger's `anchor_ns` (the window centre) — the ROS
        // interface from `trigger_time`, the MCAP interface from the trigger
        // record's own stamp on the active `--time-source` — and the handler
        // takes that resolved anchor rather than re-deriving one.
        let out_dir = cfg.out_dir.clone();
        let grace = cfg.grace();
        let time_source = cfg.time_source;
        let tailer = tailer.clone();
        let coverage = coverage.clone();
        let extract_tx = extract_tx.clone();
        let admission = admission.clone();
        move |trig: Trigger, anchor: Anchor| {
            // The single validation gate every resolved trigger passes before a
            // handler is spawned. A rejected trigger cuts no clip and announces
            // nothing — the `error!` log is its only trace.
            if let Err(reason) = validate_trigger(&trig, anchor, now_ns()) {
                error!("trigger rejected: {reason}");
                return;
            }
            let anchor_ns = anchor.ns;
            let Some(permit) = admission.clone().try_acquire() else {
                error!(
                    "trigger rejected: all {MAX_ACTIVE_TRIGGERS} trigger handlers are busy; \
                     ignoring name={:?} anchor={anchor_ns}",
                    trig.name,
                );
                return;
            };
            let out_dir = out_dir.clone();
            let tailer = tailer.clone();
            let coverage = coverage.clone();
            let extract_tx = extract_tx.clone();
            let announcer = announcer.clone();
            // Per-trigger error isolation: a failed cut is logged and counted but
            // does not tear down the interface loop, and a panic dies with the
            // handler's own thread (its permit returns on drop either way).
            let spawned = thread::Builder::new()
                .name(format!("trigger-{anchor_ns}"))
                .spawn(move || {
                    let _permit = permit;
                    if let Err(e) = handler::handle_trigger(
                        trig,
                        anchor_ns,
                        &out_dir,
                        grace,
                        tailer,
                        coverage,
                        extract_tx,
                        announcer,
                        time_source,
                        producer,
                    ) {
                        error!("trigger handling failed: {e:#}");
                    }
                });
            if let Err(e) = spawned {
                error!("spawning a trigger handler failed: {e}");
            }
        }
    };

    // The tail thread: discovers and scans the recording for the process's
    // lifetime (blocking IO on its own thread). Supervised: with a dead tailer
    // every clip degrades to a grace-timeout cut, so the process exits rather
    // than limping on silently.
    let tail = {
        let tailer = tailer.clone();
        let record_dir = cfg.record_dir.clone();
        let watch = cfg.watch_old_files();
        let delete_old_files = cfg.delete_old_files;
        spawn_supervised("tail", move || {
            tailer.run(&record_dir, watch, delete_old_files)
        })
    };

    // The interface thread: drains its trigger source and fires the callback for
    // the process's lifetime. Supervised: a dead interface silently stops acting
    // on triggers, so the process exits rather than going quiet.
    let interface = spawn_supervised("interface", move || iface.run(fire));

    let signal_rx = signal_channel().context("signal handler failed to install")?;

    info!(
        "clipper tail up: {} interface, triggers on {TRIGGER_TOPIC}, \
         tailing {}, writing clips to {}",
        I::SOURCE,
        cfg.record_dir.display(),
        cfg.out_dir.display(),
    );

    supervise(tail, interface, signal_rx)
}

/// Watch the two critical long-lived threads and the shutdown signal; return
/// when any of them resolves.
///
/// Each supervised thread reports on its channel (see [`spawn_supervised`]): a
/// received value is its verdict, a disconnect without a value is a panic,
/// harvested through the join handle so the payload lands in the error chain.
///
/// Returns `Ok(())` when the signal channel delivers SIGINT or SIGTERM — the
/// requested, orderly stop path; the signal is logged here and the caller exits
/// zero. Every other arm returns `Err`: a thread exiting (clean or panic) is a
/// fault that a supervisor must respond to by restarting the process, and the
/// signal channel disconnecting (the forwarder thread died) must not be silent
/// either, since it means SIGINT could never trigger a clean shutdown.
///
/// Both threads carry a typed `anyhow::Result<()>` and loop for the process's
/// lifetime, so a clean `Ok(())` return is as unexpected as a fault; a fault
/// surfaces as the inner `Err`, wrapped so the operator sees the root cause. The
/// **tail** thread feeds coverage and the extent index (a dead tailer silently
/// degrades every clip to a grace-timeout cut); the **interface** thread drains
/// the trigger source and, for the ROS interface, owns the node spin (a dead
/// interface silently stops acting on triggers).
#[expect(
    clippy::needless_pass_by_value,
    reason = "supervision owns the signal receiver: dropping it here is what \
              releases the handler's channel when the process winds down"
)]
fn supervise(
    tail: Supervised<anyhow::Result<()>>,
    interface: Supervised<anyhow::Result<()>>,
    signal: Receiver<i32>,
) -> anyhow::Result<()> {
    let (tail_rx, tail_handle) = tail;
    let (interface_rx, interface_handle) = interface;
    select! {
        recv(tail_rx) -> res => match res {
            // run() loops for the process's lifetime, so a clean return is
            // as unexpected as a fault. A scan fault it could not retry past
            // comes back as the inner Err, wrapped so the operator sees the
            // root cause; a panic is the disconnect.
            Ok(Ok(())) => anyhow::bail!("tail thread exited unexpectedly"),
            Ok(Err(e)) => Err(e.context("tail thread failed")),
            Err(_) => Err(harvest_panic(tail_handle).context("tail thread exited unexpectedly")),
        },
        recv(interface_rx) -> res => match res {
            // The interface drains its trigger source for the process's
            // lifetime; a clean return or a fault both mean it stopped.
            Ok(Ok(())) => anyhow::bail!("interface thread exited unexpectedly"),
            Ok(Err(e)) => Err(e.context("interface thread failed")),
            Err(_) => Err(harvest_panic(interface_handle)
                .context("interface thread exited unexpectedly")),
        },
        recv(signal) -> res => match res {
            // Requested shutdown — not a fault; the caller exits zero.
            Ok(sig) => {
                info!("{} received; shutting down", signal_name(sig));
                Ok(())
            }
            Err(_) => anyhow::bail!("signal handler thread exited unexpectedly"),
        },
    }
}

#[cfg(test)]
mod tests {
    #![allow(
        clippy::unwrap_used,
        clippy::expect_used,
        clippy::indexing_slicing,
        clippy::cast_possible_truncation,
        clippy::format_push_string,
        clippy::case_sensitive_file_extension_comparisons,
        reason = "a failed unwrap, a panicking index or a truncated stamp is a \
                  failing test, and a fixture that writes `.mcap` in one case reads \
                  it back in the same one"
    )]

    use std::path::Path;

    use super::*;

    /// Parse a `Cli` from an explicit argv through [`parse_cli`] — the same
    /// env-prefixed command and the same cross-field checks `load_cli` runs, so
    /// the tests exercise the real wiring rather than half of it.
    fn cli_from<I, T>(argv: I) -> Result<Cli, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        let argv = without_a_system_file(argv.into_iter().map(Into::into).collect());
        let _env = env_lock();
        match parse_cli(&argv) {
            Ok(loaded) => Ok(loaded.cli),
            Err(StartupError::Cli(err)) => Err(err),
            // These argvs name no configuration file, so the layers are the
            // built-in defaults and this arm is unreachable in practice.
            Err(StartupError::Config(err)) => panic!("no configuration file is named: {err}"),
        }
    }

    /// A path no system configuration file is at, named so a parse under test
    /// resolves against the built-in defaults.
    ///
    /// [`clip::config::SYSTEM_CONFIG_PATH`] is a real path on a machine with
    /// the package installed — a deployment target, and any CI runner that
    /// installs the deb — and a file there would decide the defaults every one
    /// of these tests reads. A missing system file is legal, so naming one that
    /// cannot exist is the same run with none, deterministically.
    const NO_SYSTEM_FILE: &str = "/nonexistent/momentedge/clipper.toml";

    /// `argv` with [`NO_SYSTEM_FILE`] appended, unless it names its own system
    /// file — the tests that are *about* the system layer pass their own.
    fn without_a_system_file(mut argv: Vec<std::ffi::OsString>) -> Vec<std::ffi::OsString> {
        let names_one = argv.iter().any(|arg| {
            arg == "--system-config" || arg.to_string_lossy().starts_with("--system-config=")
        });
        if !names_one {
            argv.push("--system-config".into());
            argv.push(NO_SYSTEM_FILE.into());
        }
        argv
    }

    /// A `clipper clip` argv over `rec.mcap` into `/data/clips`, with `extra`
    /// appended — the trigger flags each test names for itself.
    fn clip_argv(extra: &[&str]) -> Vec<String> {
        ["clipper", "clip", "rec.mcap", "--out-dir", "/data/clips"]
            .into_iter()
            .chain(extra.iter().copied())
            .map(str::to_string)
            .collect()
    }

    /// A `--trigger-source param` config over `recording` writing into
    /// `out_dir`, with a trigger a test overrides field by field through struct
    /// update syntax.
    fn param_clip_cfg(recording: &Path, out_dir: &Path) -> ClipConfig {
        ClipConfig {
            recording: recording.to_path_buf(),
            out_dir: out_dir.to_path_buf(),
            trigger_source: TriggerSource::Param,
            trigger_time: Some(0),
            preroll: Some(0),
            postroll: Some(0),
            trigger_name: None,
            trigger_description: None,
        }
    }

    /// The producer `clip_mode` is handed when a test builds its config
    /// directly rather than through [`Mode::producer`].
    const CLIP_PRODUCER: Producer = Producer {
        program: PROGRAM,
        mode: CLIP_MODE,
    };

    /// The recorder's `Config` out of an argv naming the `tail` mode.
    fn parse_from<I, T>(argv: I) -> Result<Config, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        cli_from(argv).map(|cli| match cli.mode {
            Mode::Tail(cfg) => cfg,
            Mode::Clip(_) => panic!("this argv names the tail mode"),
        })
    }

    /// The cutter's `ClipConfig` out of an argv naming the `clip` mode.
    fn clip_from<I, T>(argv: I) -> Result<ClipConfig, clap::Error>
    where
        I: IntoIterator<Item = T>,
        T: Into<std::ffi::OsString> + Clone,
    {
        cli_from(argv).map(|cli| match cli.mode {
            Mode::Clip(cfg) => cfg,
            Mode::Tail(_) => panic!("this argv names the clip mode"),
        })
    }

    #[test]
    fn config_defaults_when_no_flags_given() {
        let cfg = parse_from(["clipper", "tail"]).unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("./record"));
        assert_eq!(cfg.out_dir, PathBuf::from("./clipped"));
        assert_eq!(cfg.grace(), Duration::from_secs(30));
        assert_eq!(cfg.extract_parallelism, 1);
        assert_eq!(cfg.clip_compression, ClipCompression::Zstd);
        assert_eq!(cfg.time_source, TimeSource::Log);
    }

    #[test]
    fn config_cli_flags_populate_every_field() {
        let cfg = parse_from([
            "clipper",
            "tail",
            "--record-dir",
            "/data/record",
            "--out-dir",
            "/data/clips",
            "--grace-secs",
            "7",
            "--extract-parallelism",
            "3",
            "--clip-compression",
            "lz4",
            "--trigger-source",
            "mcap",
            "--time-source",
            "publish",
            "--watch-old-files-duration",
            "90",
            "--delete-old-files",
        ])
        .unwrap();
        assert_eq!(cfg.record_dir, PathBuf::from("/data/record"));
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.grace(), Duration::from_secs(7));
        assert_eq!(cfg.extract_parallelism, 3);
        assert_eq!(cfg.clip_compression, ClipCompression::Lz4);
        assert_eq!(cfg.trigger_source, TriggerSource::Mcap);
        assert_eq!(cfg.time_source, TimeSource::Publish);
        assert_eq!(cfg.watch_old_files(), Duration::from_secs(90));
        assert!(cfg.delete_old_files);
    }

    /// `with_env_prefix` binds every argument generically, so assert the
    /// invariant generically: each one carries `MOMENTEDGE_<FIELD>`, and the
    /// auto-generated `--help`/`--version` carry none. Written over
    /// `get_arguments()` rather than a hand-listed set so a field added to
    /// a mode's config is covered the moment it exists — a per-field list would
    /// silently leave the newest field, the one most likely to be mis-wired,
    /// untested. Every mode is walked, since the binding is applied per
    /// subcommand.
    #[test]
    fn env_prefix_binds_a_momentedge_name_to_every_field() {
        let cli = with_env_prefix(Cli::command());
        let bound = |mode: &str| -> usize {
            let cmd = cli
                .find_subcommand(mode)
                .unwrap_or_else(|| panic!("{mode} is a subcommand of clipper"));
            let mut bound = 0;
            for arg in cmd.get_arguments() {
                let id = arg.get_id().as_str();
                let env = arg.get_env().map(|e| e.to_string_lossy().into_owned());
                if matches!(id, "help" | "version") {
                    assert_eq!(env, None, "{id} must keep clap's own handling");
                    continue;
                }
                assert_eq!(
                    env.as_deref(),
                    Some(format!("MOMENTEDGE_{}", id.to_uppercase()).as_str()),
                    "{mode}: {id} must fall back to its MOMENTEDGE_* env var",
                );
                bound += 1;
            }
            bound
        };
        assert_eq!(
            bound("tail"),
            9,
            "every Config field is bound (update on a new field)"
        );
        assert_eq!(
            bound("clip"),
            8,
            "every ClipConfig field is bound (update on a new field)"
        );
    }

    /// `--trigger-source mcap` selects the in-recording triggers in every
    /// build, and an unknown value is rejected. (The
    /// `MOMENTEDGE_TRIGGER_SOURCE` env fallback is covered by
    /// `env_prefix_binds_a_momentedge_name_to_every_field`.)
    #[test]
    fn tail_trigger_source_parses_mcap_and_rejects_an_unknown_value() {
        assert_eq!(
            parse_from(["clipper", "tail", "--trigger-source", "mcap"])
                .unwrap()
                .trigger_source,
            TriggerSource::Mcap
        );
        assert!(parse_from(["clipper", "tail", "--trigger-source", "bogus"]).is_err());
    }

    /// The recorder does not take `param`, in any build: it runs until a
    /// shutdown signal, and a source naming one window would leave its loop with
    /// nothing to do after the single cut. The refusal is the parser's, so it
    /// names the value and the sources that *are* accepted while the command
    /// line is being read.
    #[test]
    fn tail_refuses_the_param_trigger_source() {
        let err = cli_from(["clipper", "tail", "--trigger-source", "param"])
            .expect_err("`clipper tail` takes no param trigger source");
        assert_eq!(err.kind(), clap::error::ErrorKind::InvalidValue);
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        let message = err.to_string();
        assert!(
            message.contains("invalid value 'param'"),
            "the refusal names the value: {message}"
        );
        assert!(
            message.contains("mcap"),
            "the refusal names what the recorder does accept: {message}"
        );
    }

    /// The device build: the `ros` feature offers the live subscription, and
    /// clipper takes it when `--trigger-source` is absent — the deployed
    /// behaviour.
    #[cfg(feature = "ros")]
    #[test]
    fn tail_trigger_source_defaults_to_ros_under_the_ros_feature() {
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().trigger_source,
            TriggerSource::Ros
        );
        assert_eq!(
            parse_from(["clipper", "tail", "--trigger-source", "ros"])
                .unwrap()
                .trigger_source,
            TriggerSource::Ros
        );
    }

    /// The ROS-free build has no live subscription to select: `--trigger-source
    /// ros` is refused like any other unknown value, and `mcap` is what an
    /// absent flag means.
    #[cfg(not(feature = "ros"))]
    #[test]
    fn tail_trigger_source_is_mcap_only_without_the_ros_feature() {
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().trigger_source,
            TriggerSource::Mcap
        );
        assert!(
            parse_from(["clipper", "tail", "--trigger-source", "ros"]).is_err(),
            "a build that links no ROS must not accept --trigger-source ros"
        );
    }

    #[test]
    fn config_time_source_defaults_to_log_and_parses_publish() {
        // Default is the log domain; --time-source selects publish; an unknown
        // value is rejected. (Its MOMENTEDGE_TIME_SOURCE env fallback is covered
        // by env_prefix_binds_a_momentedge_name_to_every_field.)
        assert_eq!(
            parse_from(["clipper", "tail"]).unwrap().time_source,
            TimeSource::Log
        );
        assert_eq!(
            parse_from(["clipper", "tail", "--time-source", "publish"])
                .unwrap()
                .time_source,
            TimeSource::Publish
        );
        assert!(parse_from(["clipper", "tail", "--time-source", "bogus"]).is_err());
    }

    #[test]
    fn config_rejects_a_non_numeric_grace() {
        assert!(parse_from(["clipper", "tail", "--grace-secs", "soon"]).is_err());
    }

    // ── the command shape: one binary, the mode is a subcommand ────────────

    /// A recorder flag offered to `clipper` itself belongs to `clipper tail`,
    /// and the message says so. The process exits non-zero on it: `load_cli`
    /// exits with `Error::exit_code`, which is 2 for a parse failure.
    #[test]
    fn a_bare_recorder_flag_is_rejected_and_points_at_the_tail_mode() {
        let err = cli_from(["clipper", "--record-dir", "/data/record"])
            .expect_err("a recorder flag on the bare command is not a valid command line");
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        let hint = mode_hint(err.kind()).expect("the failure left the mode unnamed");
        assert!(
            hint.contains("clipper tail"),
            "the hint names the subcommand that owns the flag: {hint}"
        );
    }

    /// Naming no mode at all is the same rejection: `clipper` alone runs
    /// nothing.
    #[test]
    fn naming_no_mode_is_rejected_and_points_at_the_tail_mode() {
        let err = cli_from(["clipper"]).expect_err("clipper with no mode runs nothing");
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        assert!(mode_hint(err.kind()).is_some_and(|h| h.contains("clipper tail")));
    }

    /// `clipper --help` lists the modes. It is not a failure — exit code 0 and
    /// no hint, since nothing went wrong.
    #[test]
    fn top_level_help_lists_the_modes() {
        let err = cli_from(["clipper", "--help"]).expect_err("--help short-circuits the parse");
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp);
        assert_eq!(err.exit_code(), 0);
        assert_eq!(mode_hint(err.kind()), None);
        let help = err.to_string();
        assert!(help.contains("Commands:"), "{help}");
        for mode in ["tail", "clip"] {
            assert!(help.contains(mode), "the mode listing names {mode}: {help}");
        }
    }

    /// `clipper tail --help` is the recorder's own surface: its flags, and no
    /// list of modes to descend into.
    #[test]
    fn tail_help_lists_the_recorder_flags_and_no_modes() {
        let err =
            cli_from(["clipper", "tail", "--help"]).expect_err("--help short-circuits the parse");
        assert_eq!(err.kind(), clap::error::ErrorKind::DisplayHelp);
        let help = err.to_string();
        for flag in [
            "--record-dir",
            "--out-dir",
            "--grace-secs",
            "--extract-parallelism",
            "--clip-compression",
            "--trigger-source",
            "--time-source",
            "--watch-old-files-duration",
            "--delete-old-files",
        ] {
            assert!(
                help.contains(flag),
                "clipper tail --help lists {flag}: {help}"
            );
        }
        assert!(
            !help.contains("Commands:"),
            "the recorder mode has no submodes: {help}"
        );
    }

    /// `clipper tail --help` names the trigger sources *this* build of *this*
    /// subcommand can select — the one place the `ros` cargo feature is visible
    /// on the command line. The feature build offers `ros` and `mcap` and
    /// defaults to `ros`; the ROS-free build offers `mcap` alone and says
    /// outright that the ROS source was not built in, so an operator reading
    /// `--help` on a host with no ROS is told why rather than left guessing.
    /// `param` is the cutter's and appears in neither.
    #[test]
    fn tail_help_names_the_trigger_sources_this_build_offers() {
        let err =
            cli_from(["clipper", "tail", "--help"]).expect_err("--help short-circuits the parse");
        let help = err.to_string();
        // `--help` renders the accepted values as a `Possible values:` list, one
        // `- <name>:` line each.
        assert!(help.contains("- mcap:"), "every build offers mcap: {help}");
        assert!(
            !help.contains("- param:"),
            "`param` belongs to `clipper clip`: {help}"
        );
        #[cfg(feature = "ros")]
        {
            assert!(help.contains("- ros:"), "{help}");
            assert!(help.contains("[default: ros]"), "{help}");
        }
        #[cfg(not(feature = "ros"))]
        {
            assert!(
                !help.contains("- ros:"),
                "a build that links no ROS must not offer --trigger-source ros: {help}"
            );
            assert!(help.contains("[default: mcap]"), "{help}");
            assert!(
                help.contains("this build was made without it"),
                "the help says the ros source is absent from this build: {help}"
            );
        }
    }

    /// A failure *inside* a named mode is the mode's own problem, so it carries
    /// no subcommand hint — the caller already said `tail`.
    #[test]
    fn a_bad_flag_value_inside_the_mode_carries_no_hint() {
        let err = cli_from(["clipper", "tail", "--time-source", "bogus"])
            .expect_err("an unknown --time-source value is rejected");
        assert_eq!(mode_hint(err.kind()), None);
    }

    // ── `clipper clip`: one clip out of one finished recording ─────────────

    /// A full `clipper clip` command line populates every field of the cutter's
    /// config.
    #[test]
    fn clip_cli_flags_populate_every_field() {
        let cfg = clip_from([
            "clipper",
            "clip",
            "/data/record/rosbag2_0.mcap",
            "--out-dir",
            "/data/clips",
            "--trigger-time",
            "1738000000000000000",
            "--preroll",
            "5000000000",
            "--postroll",
            "2000000000",
            "--trigger-name",
            "brake-event",
            "--trigger-description",
            "hard brake over 0.8 g",
        ])
        .unwrap();
        assert_eq!(
            cfg.recording,
            PathBuf::from("/data/record/rosbag2_0.mcap"),
            "the recording is the positional argument"
        );
        assert_eq!(cfg.out_dir, PathBuf::from("/data/clips"));
        assert_eq!(cfg.trigger_time, Some(1_738_000_000_000_000_000));
        assert_eq!(cfg.preroll, Some(5_000_000_000));
        assert_eq!(cfg.postroll, Some(2_000_000_000));
        assert_eq!(cfg.trigger_name.as_deref(), Some("brake-event"));
        assert_eq!(
            cfg.trigger_description.as_deref(),
            Some("hard brake over 0.8 g")
        );
        assert_eq!(
            cfg.trigger_source,
            TriggerSource::Param,
            "a command line that names a trigger takes it from the command line"
        );
    }

    /// The window is what only the caller knows, so leaving out the instant it
    /// centres on is refused by name rather than defaulted to something.
    ///
    /// The refusal is clap's, so it happens before `clip_mode` runs: no output
    /// directory is created and no clip is written.
    #[test]
    fn clip_without_a_trigger_time_is_refused_by_name() {
        let err = cli_from([
            "clipper",
            "clip",
            "rec.mcap",
            "--out-dir",
            "/data/clips",
            "--preroll",
            "1000",
            "--postroll",
            "1000",
        ])
        .expect_err("a clip with no trigger time names no window");
        assert_eq!(err.kind(), clap::error::ErrorKind::MissingRequiredArgument);
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        assert!(
            err.to_string().contains("--trigger-time"),
            "the refusal names the missing flag: {err}"
        );
    }

    /// The cutter has no clock-domain flag: a window over a finished recording
    /// lives on `log_time`, the clock its summary states message times on.
    /// Passing `--time-source` is refused rather than quietly ignored.
    #[test]
    fn clip_offers_no_clock_domain_flag() {
        assert!(
            clip_from([
                "clipper",
                "clip",
                "rec.mcap",
                "--out-dir",
                "/data/clips",
                "--trigger-time",
                "1000",
                "--preroll",
                "0",
                "--postroll",
                "0",
                "--time-source",
                "log",
            ])
            .is_err(),
            "the cutter takes no --time-source"
        );

        let err =
            cli_from(["clipper", "clip", "--help"]).expect_err("--help short-circuits the parse");
        let help = err.to_string();
        assert!(
            !help.contains("--time-source"),
            "clipper clip --help offers no clock domain: {help}"
        );
        for flag in [
            "--out-dir",
            "--trigger-time",
            "--preroll",
            "--postroll",
            "--trigger-name",
            "--trigger-description",
        ] {
            assert!(
                help.contains(flag),
                "clipper clip --help lists {flag}: {help}"
            );
        }
    }

    /// The end-to-end cut: a real chunked recording in, one clip out, stamped
    /// with the mode that produced it.
    ///
    /// `producer.mode` is what tells a clip cut here from one the recorder cut,
    /// and it is read off the parsed mode rather than spelled at the cut, so it
    /// is taken through `Mode::producer` exactly as `main` takes it.
    #[test]
    fn clip_mode_cuts_the_window_and_stamps_the_clip_as_its_own() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-mode")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(
            &rec,
            true,
            &[("/t", 1_000), ("/t", 2_000), ("/t", 3_000), ("/t", 4_000)],
        )?;
        let out_dir = root.join("clipped");

        let mode = Mode::Clip(ClipConfig {
            trigger_time: Some(3_000),
            preroll: Some(1_500),
            postroll: Some(500),
            trigger_name: Some("brake".to_string()),
            trigger_description: Some("hard brake".to_string()),
            ..param_clip_cfg(&rec, &out_dir)
        });
        let producer = mode.producer();
        let Mode::Clip(cfg) = mode else {
            unreachable!("the mode was just built as Clip")
        };
        clip_mode(cfg, producer, clip::ChannelSelection::default())?;

        let clip_path = out_dir.join("3000_brake.mcap");
        assert_eq!(
            clip::testing::read_clip(&clip_path)?,
            vec![("/t".to_string(), 2_000), ("/t".to_string(), 3_000)],
            "the window [1500, 3500] holds exactly these two messages"
        );

        let m = clip::manifest::read_manifest(&clip_path)?.expect("every clip carries a manifest");
        assert_eq!(m["producer.name"], "clipper");
        assert_eq!(
            m["producer.mode"], "clip",
            "a clip cut here is told from a recorder's without opening the recording"
        );
        assert_eq!(m["trigger.name"], "brake");
        assert_eq!(m["trigger.description"], "hard brake");
        assert_eq!(m["trigger.anchor_ns"], "3000");
        assert_eq!(m["trigger.preroll_ns"], "1500");
        assert_eq!(m["trigger.postroll_ns"], "500");
        assert_eq!(m["window.time_source"], "log");
        assert_eq!(m["window.start_ns"], "1500");
        assert_eq!(m["window.end_ns"], "3500");
        assert_eq!(m["source.path"], rec.display().to_string());
        assert_eq!(m["source.files_planned"], "1");
        assert_eq!(m["clip.messages"], "2");
        assert_eq!(
            m["clip.short"], "false",
            "the recording runs past the window end"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The cut runs with both of the recorder's waits absent.
    ///
    /// The window ends a minute in the wall-clock future and a minute past the
    /// last recorded message — the two things `tail::handler` sleeps for. The
    /// recording is finished, so there is nothing to wait for: the clip is
    /// written at once and says it stops short of what was asked for.
    #[test]
    fn clip_mode_over_a_finished_recording_does_not_wait() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-nowait")?;
        let rec = root.join("rec.mcap");
        let base = clip::trigger::now_ns();
        clip::testing::write_recording(&rec, true, &[("/t", base), ("/t", base + 1_000)])?;
        let out_dir = root.join("clipped");

        let postroll = 60_000_000_000;
        let began = std::time::Instant::now();
        clip_mode(
            ClipConfig {
                trigger_time: Some(base),
                postroll: Some(postroll),
                trigger_name: Some("late".to_string()),
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;
        let elapsed = began.elapsed();

        let clip_path = out_dir.join(format!("{base}_late.mcap"));
        let m = clip::manifest::read_manifest(&clip_path)?.expect("every clip carries a manifest");
        assert_eq!(m["clip.messages"], "2");
        assert_eq!(
            m["clip.short"], "true",
            "the recording stops well inside the window, and the clip says so"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "the cut must not sleep out the window's {postroll} ns of postroll: took {elapsed:?}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A trigger name that cannot be safely embedded in the clip pathname is
    /// refused here exactly as it is when it arrives on a topic, and nothing is
    /// written.
    #[test]
    fn clip_mode_refuses_an_unsafe_trigger_name() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-badname")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, true, &[("/t", 1_000)])?;
        let out_dir = root.join("clipped");

        let err = clip_mode(
            ClipConfig {
                trigger_time: Some(1_000),
                trigger_name: Some("../escape".to_string()),
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("--trigger-name"),
            "the refusal names the flag: {err:#}"
        );
        assert!(
            !out_dir.exists(),
            "a refused command line writes nothing at all"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording clipper cannot index reaches the operator as the refusal it
    /// is: the fault named, the repair named, a non-zero exit, and nothing
    /// written anywhere.
    ///
    /// The fixture is an unchunked recording — a perfectly valid MCAP that
    /// carries no chunk index to plan a window from — so this exercises the
    /// refusal rather than a corrupt file. `clip::whole` owns the taxonomy and
    /// tests every variant of it; what is tested here is that a refusal
    /// survives the trip out of `clip_mode` with its message intact and takes
    /// the output directory with it.
    #[test]
    fn clip_mode_refuses_a_recording_it_cannot_index() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-unindexable")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, false, &[("/t", 1_000), ("/t", 2_000)])?;
        let out_dir = root.join("clipped");

        let err = clip_mode(
            ClipConfig {
                recording: rec.clone(),
                out_dir: out_dir.clone(),
                trigger_source: TriggerSource::Param,
                trigger_time: Some(1_500),
                preroll: Some(500),
                postroll: Some(500),
                trigger_name: Some("brake".to_string()),
                trigger_description: None,
            },
            Producer {
                program: PROGRAM,
                mode: "clip",
            },
            clip::ChannelSelection::default(),
        )
        .unwrap_err();

        let text = format!("{err:#}");
        assert!(
            text.contains(&rec.display().to_string()),
            "the refusal names the recording: {text}"
        );
        assert!(
            text.contains("indexes no chunk"),
            "the refusal names the fault: {text}"
        );
        assert!(
            text.contains("mcap recover"),
            "the refusal names the repair: {text}"
        );
        assert!(
            err.downcast_ref::<clip::OpenError>().is_some(),
            "the refusal keeps its type all the way out: {text}"
        );
        assert!(
            !out_dir.exists(),
            "a refusal writes nothing, not even the staging directory"
        );

        // What the operator actually sees: `main` boxes the error and returns
        // it, and the runtime renders that box's `Debug` before exiting
        // non-zero. A refusal whose text is lost on the way out is a refusal
        // nobody can act on.
        let boxed: Box<dyn std::error::Error> = err.into();
        let printed = format!("{boxed:?}");
        assert!(
            printed.contains("indexes no chunk") && printed.contains("mcap recover"),
            "the refusal survives the boxing `main` does: {printed}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// The input contract guards `mcap` runs too: a recording clipper cannot
    /// index is refused by name before its own triggers are read.
    ///
    /// Reading the triggers first would walk an input the contract refuses —
    /// and, because a recording with no readable trigger cuts nothing and exits
    /// zero, would answer an unindexable recording with silence instead of the
    /// fault and its repair. The unchunked fixture is the sharp case: it is a
    /// perfectly valid MCAP carrying no chunk index, so a trigger read over it
    /// succeeds and finds nothing, while the contract refuses it outright.
    #[test]
    fn clip_mode_refuses_an_unindexable_recording_under_the_mcap_source() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-unindexable-mcap")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, false, &[("/t", 1_000), ("/t", 2_000)])?;
        let out_dir = root.join("clipped");

        let err = clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )
        .unwrap_err();

        let text = format!("{err:#}");
        assert!(
            text.contains("indexes no chunk") && text.contains("mcap recover"),
            "an `mcap` run gets the same refusal a `param` run gets: {text}"
        );
        assert!(
            err.downcast_ref::<clip::OpenError>().is_some(),
            "the refusal keeps its type all the way out: {text}"
        );
        assert!(
            !out_dir.exists(),
            "a refusal writes nothing, not even the staging directory"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A bag directory is cut as one collection: a window straddling a split
    /// becomes one segment per contributing recording, and each segment states
    /// which recording it came from.
    ///
    /// This is the whole of what the directory buys — the window is the same
    /// window either way, and a clip that stopped at the split it started in
    /// would be short two seconds of data nobody asked to lose.
    #[test]
    fn clip_mode_cuts_a_bag_directory_as_one_collection() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-bag")?;
        let bag = root.join("record");
        std::fs::create_dir_all(&bag)?;
        let first = bag.join("bag_0.mcap");
        let second = bag.join("bag_1.mcap");
        clip::testing::write_recording(&first, true, &[("/t", 1_000), ("/t", 2_000)])?;
        clip::testing::write_recording(&second, true, &[("/t", 3_000), ("/t", 4_000)])?;
        clip::testing::write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/t", 4)])?;
        let out_dir = root.join("clipped");

        clip_mode(
            ClipConfig {
                trigger_time: Some(3_000),
                preroll: Some(1_500),
                postroll: Some(500),
                trigger_name: Some("brake".to_string()),
                ..param_clip_cfg(&bag, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        let mut written: Vec<String> = std::fs::read_dir(&out_dir)?
            .map(|entry| Ok(entry?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<Vec<_>>>()?
            .into_iter()
            .filter(|name| name.ends_with(".mcap"))
            .collect();
        written.sort();
        assert_eq!(
            written,
            vec!["3000_brake_00.mcap", "3000_brake_01.mcap"],
            "one segment per contributing recording, numbered in collection order"
        );

        // The window [1500, 3500] straddles the split; the segments together
        // are the two messages inside it, in recording order.
        assert_eq!(clip_data(&out_dir.join("3000_brake_00.mcap"))?, vec![2_000]);
        assert_eq!(clip_data(&out_dir.join("3000_brake_01.mcap"))?, vec![3_000]);

        let manifest = |name: &str| -> anyhow::Result<_> {
            clip::manifest::read_manifest(&out_dir.join(name))?
                .context("every clip carries a manifest")
        };
        assert_eq!(
            manifest("3000_brake_00.mcap")?["source.path"],
            first.display().to_string()
        );
        assert_eq!(
            manifest("3000_brake_01.mcap")?["source.path"],
            second.display().to_string()
        );
        assert_eq!(
            manifest("3000_brake_00.mcap")?["source.files_planned"],
            "2",
            "the clip states how many recordings of the collection the window crossed"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A bag directory holding one recording that fails the index contract is
    /// refused naming *that recording* — the file an operator repairs — and
    /// writes nothing.
    ///
    /// The fixture is the case this exists for: a directory copied off a device
    /// mid-recording, whose last split never got its footer. Its good splits
    /// buy it nothing; a collection is only as plannable as the recordings in
    /// it.
    #[test]
    fn clip_mode_refuses_a_collection_naming_the_recording_that_fails() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-bag-refused")?;
        let bag = root.join("record");
        std::fs::create_dir_all(&bag)?;
        clip::testing::write_recording(&bag.join("bag_0.mcap"), true, &[("/t", 1_000)])?;
        let truncated = bag.join("bag_1.mcap");
        clip::testing::write_unfinished_recording(&truncated, "/t", &[2_000, 3_000])?;
        let out_dir = root.join("clipped");

        let err = clip_mode(
            ClipConfig {
                trigger_time: Some(2_000),
                preroll: Some(1_000),
                postroll: Some(1_000),
                ..param_clip_cfg(&bag, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )
        .unwrap_err();

        let text = format!("{err:#}");
        assert!(
            text.contains(&truncated.display().to_string()),
            "the refusal names the recording that failed, not the directory: {text}"
        );
        assert!(
            !text.contains("bag_0.mcap"),
            "and says nothing about the recordings that passed: {text}"
        );
        assert!(
            text.contains("does not end with the MCAP magic"),
            "the refusal names the fault: {text}"
        );
        assert!(
            !out_dir.exists(),
            "a refusal writes nothing, not even the staging directory"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Cutting the same window out of the same recording twice writes the clip
    /// once: the second run names the clip that is already there, exits
    /// non-zero, and leaves the output directory exactly as the first left it.
    ///
    /// This is what separates a cut from a finished recording from the
    /// recorder's cut on a vehicle. There a colliding name is a *second*
    /// trigger, whose clip is data no re-run can produce again, so it is
    /// published beside the first. Here both runs describe one window over one
    /// finished file and copy the same bytes, so a second file would be a
    /// duplicate — and there is no flag that turns the refusal off.
    #[test]
    fn clip_mode_refuses_a_re_run_rather_than_duplicating() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-rerun")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, true, &[("/t", 1_000), ("/t", 2_000)])?;
        let out_dir = root.join("clipped");
        let cut_it = || {
            clip_mode(
                ClipConfig {
                    trigger_time: Some(1_500),
                    preroll: Some(1_000),
                    postroll: Some(1_000),
                    trigger_name: Some("brake".to_string()),
                    ..param_clip_cfg(&rec, &out_dir)
                },
                CLIP_PRODUCER,
                clip::ChannelSelection::default(),
            )
        };

        cut_it()?;
        let clip_path = out_dir.join("1500_brake.mcap");
        assert_eq!(
            clip::testing::read_clip(&clip_path)?,
            vec![("/t".to_string(), 1_000), ("/t".to_string(), 2_000)],
            "the first run writes the clip"
        );

        let err = cut_it().unwrap_err();
        let text = format!("{err:#}");
        assert!(
            text.contains(&clip_path.display().to_string()),
            "the refusal names the clip that is already there: {text}"
        );
        assert!(
            err.downcast_ref::<clip::segment::ClipExists>().is_some(),
            "the refusal keeps its type all the way out: {text}"
        );

        // Nothing else reached the output directory: no second copy under a
        // suffixed name, and nothing stranded in the staging area.
        let mut published: Vec<String> = std::fs::read_dir(&out_dir)?
            .map(|e| Ok::<_, anyhow::Error>(e?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<Vec<_>>>()?;
        published.sort();
        assert_eq!(
            published,
            vec![".capturing".to_string(), "1500_brake.mcap".to_string()],
            "the refused run publishes nothing, least of all a suffixed sibling"
        );
        assert_eq!(
            std::fs::read_dir(out_dir.join(".capturing"))?.count(),
            0,
            "the refusal happens before staging, so the staging area stays empty"
        );

        // What the operator actually sees: `main` boxes the error and returns
        // it, and the runtime renders that box's `Debug` before exiting
        // non-zero. A refusal that does not name the clip on the way out is one
        // nobody can act on.
        let boxed: Box<dyn std::error::Error> = err.into();
        let printed = format!("{boxed:?}");
        assert!(
            printed.contains(&clip_path.display().to_string()),
            "the refusal survives the boxing `main` does: {printed}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── `clipper clip --trigger-source`: where the run's triggers come from ─

    /// The topic a fixture recording carries its triggers on: the recorder's
    /// own, since that is the one `clipper clip --trigger-source mcap` reads.
    const FIXTURE_TRIGGER_TOPIC: &str = TRIGGER_TOPIC;

    /// A trigger a fixture recording carries, stamped `log_time`.
    fn embedded(
        log_time: u64,
        name: &str,
        preroll: u64,
        postroll: u64,
    ) -> clip::testing::FixtureMsg<'static> {
        clip::testing::FixtureMsg::Trigger {
            log_time,
            trigger: Trigger {
                name: name.to_string(),
                description: format!("{name} happened"),
                // A recorded trigger states its instant on the wire too, but the
                // window anchors on the record's own log time; a distinct value
                // here is what proves which of the two was read.
                trigger_time: clip::Stamp { sec: 0, nanosec: 0 },
                preroll,
                postroll,
            },
        }
    }

    /// A data message on `/t` at `log_time` — the messages a clip copies.
    fn recorded(log_time: u64) -> clip::testing::FixtureMsg<'static> {
        clip::testing::FixtureMsg::Data {
            topic: "/t",
            log_time,
        }
    }

    /// The `/t` messages a clip holds. A window that contains a trigger message
    /// copies that message too, like any other; the data topic is what the
    /// window assertions are about.
    fn clip_data(path: &Path) -> anyhow::Result<Vec<u64>> {
        Ok(clip::testing::read_clip(path)?
            .into_iter()
            .filter(|(topic, _)| topic == "/t")
            .map(|(_, log_time)| log_time)
            .collect())
    }

    /// [`MODES`] spells the names clap gave the two variants — `scan_mode` finds
    /// the mode in argv by them and `parse_cli` raises a trigger fault against
    /// `clip` by name, so a rename that left a constant behind would read no
    /// configuration file and panic instead of reporting the fault.
    #[test]
    fn the_mode_names_are_the_ones_clap_built() {
        let cmd = Cli::command();
        for (name, _) in MODES {
            assert!(
                cmd.find_subcommand(name).is_some(),
                "{name} must name a subcommand clipper actually has"
            );
        }
        assert_eq!(
            cmd.get_subcommands().count(),
            MODES.len(),
            "every subcommand clipper has must be in MODES"
        );
        // A mode reachable by a second spelling is a mode `scan_mode` would miss
        // while clap accepted it, which is the one way `parse_cli` can hold a
        // parsed command line whose files were never read — and there it panics
        // rather than reporting anything. An alias or `infer_subcommands` is
        // fine to add; the scan has to learn the same spellings in the same
        // change, and this is what says so.
        for sub in cmd.get_subcommands() {
            assert!(
                sub.get_all_aliases().next().is_none(),
                "{} carries an alias, which `scan_mode` does not know",
                sub.get_name()
            );
        }
    }

    /// The flag defaults to the command line, parses `mcap`, and refuses
    /// anything else. (Its `MOMENTEDGE_TRIGGER_SOURCE` env fallback is covered
    /// by `env_prefix_binds_a_momentedge_name_to_every_field`.)
    #[test]
    fn clip_trigger_source_defaults_to_param_and_parses_mcap() {
        let source = |extra: &[&str]| clip_from(clip_argv(extra)).map(|cfg| cfg.trigger_source);
        assert_eq!(
            source(&[
                "--trigger-time",
                "1000",
                "--preroll",
                "0",
                "--postroll",
                "0"
            ])
            .unwrap(),
            TriggerSource::Param,
            "an absent --trigger-source takes the trigger from the command line"
        );
        assert_eq!(
            source(&["--trigger-source", "mcap"]).unwrap(),
            TriggerSource::Mcap
        );
        assert_eq!(
            source(&[
                "--trigger-source",
                "param",
                "--trigger-time",
                "1000",
                "--preroll",
                "0",
                "--postroll",
                "0"
            ])
            .unwrap(),
            TriggerSource::Param,
            "naming the default source explicitly is not a conflict with the flags it reads"
        );
        assert!(source(&["--trigger-source", "bogus"]).is_err());
    }

    /// `clipper clip --help` names the sources it accepts and the default it
    /// takes, rendered from the `ValueEnum` itself, and says where a recorded
    /// trigger is read from. `ros` is a live subscription and appears in no
    /// build of this subcommand.
    #[test]
    fn clip_help_names_the_trigger_sources() {
        let err =
            cli_from(["clipper", "clip", "--help"]).expect_err("--help short-circuits the parse");
        let help = err.to_string();
        assert!(help.contains("--trigger-source"), "{help}");
        // `--help` renders the accepted values as a `Possible values:` list, one
        // `- <name>:` line each.
        for value in ["- param:", "- mcap:"] {
            assert!(help.contains(value), "the help lists {value}: {help}");
        }
        assert!(
            !help.contains("- ros:"),
            "a finished recording has no live topic: {help}"
        );
        assert!(
            help.contains("[default: param]"),
            "the default is rendered from the enum's own value name: {help}"
        );
        assert!(
            help.contains(TRIGGER_TOPIC),
            "the long help names the topic a recorded trigger is read from: {help}"
        );
    }

    /// The cutter does not take `ros`, in any build: a finished recording has no
    /// live topic to subscribe to. Under the `ros` feature the variant exists
    /// and the cutter's own subset is what refuses it; without the feature there
    /// is no such value at all. Either way the refusal is clap's, names the
    /// value, and happens before anything is written.
    #[test]
    fn clip_refuses_the_ros_trigger_source() {
        let err = cli_from(clip_argv(&["--trigger-source", "ros"]))
            .expect_err("`clipper clip` takes no ros trigger source");
        assert_eq!(err.kind(), clap::error::ErrorKind::InvalidValue);
        assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
        let message = err.to_string();
        assert!(
            message.contains("invalid value 'ros'"),
            "the refusal names the value: {message}"
        );
        for accepted in ["mcap", "param"] {
            assert!(
                message.contains(accepted),
                "the refusal names what the cutter does accept ({accepted}): {message}"
            );
        }
    }

    /// The subsets are the type's, and every subcommand's `--trigger-source`
    /// surface is derived from them: each subcommand accepts exactly the sources
    /// [`TriggerSource::modes`] lists for it, no source belongs to nothing, and
    /// the default a subcommand takes with the flag absent is one it accepts.
    ///
    /// Stated over the parser clap actually built, so the table and the two
    /// arguments cannot drift: a variant added without a `modes` entry does not
    /// compile, and one whose entry disagrees with the surface fails here.
    #[test]
    fn each_subcommand_accepts_exactly_its_own_trigger_sources() {
        let cmd = Cli::command();
        for (name, mode) in MODES {
            let expected: Vec<String> = TriggerSource::accepted_by(mode)
                .map(|source| source.to_string())
                .collect();
            assert!(
                !expected.is_empty(),
                "{name} must accept at least one trigger source"
            );
            let arg = cmd
                .find_subcommand(name)
                .unwrap_or_else(|| panic!("{name} is a subcommand of clipper"))
                .get_arguments()
                .find(|arg| arg.get_id().as_str() == "trigger_source")
                .unwrap_or_else(|| panic!("{name} carries --trigger-source"));
            let offered: Vec<String> = arg
                .get_value_parser()
                .possible_values()
                .expect("--trigger-source is parsed against a fixed value set")
                .map(|value| value.get_name().to_string())
                .collect();
            assert_eq!(offered, expected, "{name}: --trigger-source value set");
            let default: Vec<String> = arg
                .get_default_values()
                .iter()
                .map(|value| value.to_string_lossy().into_owned())
                .collect();
            assert_eq!(default.len(), 1, "{name}: one default");
            assert!(
                expected.contains(&default[0]),
                "{name}: the default {} is not a source it accepts ({expected:?})",
                default[0],
            );
        }

        for source in TriggerSource::value_variants() {
            assert!(
                !source.modes().is_empty(),
                "{source} belongs to no subcommand"
            );
        }
    }

    /// The completion half follows from the trigger source: each interface the
    /// recorder can drive names the source that selects it, and carries the
    /// announcer that pairing implies. `ros` answers each clip with a `Recorded`
    /// publish; `mcap` has the clip's move into `out_dir` as its only signal, so
    /// its announcer is the no-op one. There is no separate announcer setting —
    /// these two cells are the whole matrix.
    #[test]
    fn each_tail_trigger_source_carries_the_announcer_its_interface_implies() {
        assert_eq!(McapInterface::SOURCE, TriggerSource::Mcap);
        // A compile-time assertion on the associated type: this coerces only if
        // the MCAP interface's announcer *is* the no-op one.
        let _: fn(interface::NullAnnouncer) -> <McapInterface as Interface>::Announcer =
            std::convert::identity;
        #[cfg(feature = "ros")]
        {
            assert_eq!(RosInterface::SOURCE, TriggerSource::Ros);
            let _: fn(interface::ros::RosAnnouncer) -> <RosInterface as Interface>::Announcer =
                std::convert::identity;
        }
        // And the two sides are the same set: every source the recorder accepts
        // names an interface this build compiled in, and every such interface is
        // selectable.
        #[cfg(feature = "ros")]
        let interfaces = [McapInterface::SOURCE, RosInterface::SOURCE];
        #[cfg(not(feature = "ros"))]
        let interfaces = [McapInterface::SOURCE];
        let mut served: Vec<String> = interfaces.iter().map(ToString::to_string).collect();
        served.sort_unstable();
        let mut accepted: Vec<String> = TriggerSource::accepted_by(config::Mode::Tail)
            .map(|source| source.to_string())
            .collect();
        accepted.sort_unstable();
        assert_eq!(
            served, accepted,
            "every source `clipper tail` accepts names an interface, and back"
        );
    }

    /// The `Display` a `--help` default is rendered through and the values clap
    /// accepts are two spellings of one thing, and must not drift.
    #[test]
    fn trigger_source_display_matches_the_clap_value_names() {
        for source in TriggerSource::value_variants() {
            assert_eq!(
                source.to_string(),
                source
                    .to_possible_value()
                    .expect("no TriggerSource variant is skipped")
                    .get_name(),
            );
        }
    }

    /// `param` cuts the trigger the flags name, so each flag it cannot do
    /// without is refused by name — while the parse is running, so nothing is
    /// written.
    #[test]
    fn param_without_a_trigger_flag_is_refused_by_name() {
        let full = [
            ("--trigger-time", "1000"),
            ("--preroll", "10"),
            ("--postroll", "20"),
        ];
        for missing in 0..full.len() {
            let extra: Vec<&str> = full
                .iter()
                .enumerate()
                .filter(|(i, _)| *i != missing)
                .flat_map(|(_, (flag, value))| [*flag, *value])
                .collect();
            let err = cli_from(clip_argv(&extra))
                .expect_err("a param run missing a trigger flag names no window");
            assert_eq!(err.kind(), clap::error::ErrorKind::MissingRequiredArgument);
            assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
            assert!(
                err.to_string().contains(full[missing].0),
                "the refusal names the missing flag {}: {err}",
                full[missing].0
            );
        }
    }

    /// `mcap` takes every trigger from the recording, so any flag that states
    /// part of a trigger is a conflict — named, refused, and refused while the
    /// parse is running.
    #[test]
    fn mcap_with_any_trigger_parameter_names_the_conflict() {
        for (flag, value) in [
            ("--trigger-time", "1000"),
            ("--preroll", "10"),
            ("--postroll", "20"),
            ("--trigger-name", "brake"),
            ("--trigger-description", "hard brake"),
        ] {
            let err = cli_from(clip_argv(&["--trigger-source", "mcap", flag, value]))
                .expect_err("a recorded trigger leaves nothing for the flags to say");
            assert_eq!(err.kind(), clap::error::ErrorKind::ArgumentConflict);
            assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
            let msg = err.to_string();
            assert!(msg.contains(flag), "the refusal names {flag}: {msg}");
            assert!(
                msg.contains("--trigger-source mcap"),
                "the refusal names the source it conflicts with: {msg}"
            );
        }
        assert!(
            cli_from(clip_argv(&["--trigger-source", "mcap"])).is_ok(),
            "mcap on its own is a complete command line"
        );
    }

    /// A refused trigger command line writes nothing at all: both faults are
    /// raised while the arguments are being read, so `clip_mode` never runs and
    /// the output directory it named is never created.
    #[test]
    fn a_refused_trigger_command_line_writes_nothing() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-refused")?;
        let out_dir = root.join("clipped");
        let out = out_dir.to_string_lossy().into_owned();
        for extra in [
            // `param` (by default) without the instant it needs.
            vec!["--preroll", "10", "--postroll", "20"],
            // `mcap` alongside a flag it has no use for.
            vec!["--trigger-source", "mcap", "--trigger-time", "1000"],
        ] {
            let argv: Vec<String> = ["clipper", "clip", "rec.mcap", "--out-dir", out.as_str()]
                .into_iter()
                .chain(extra)
                .map(str::to_string)
                .collect();
            let err = cli_from(argv).expect_err("these trigger arguments are wrong");
            assert_ne!(err.exit_code(), 0, "a rejected command line exits non-zero");
            assert!(
                !out_dir.exists(),
                "a refused command line writes nothing, not even an output directory"
            );
        }

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording carrying N triggers produces N clips, each anchored on its
    /// own trigger and holding the window that trigger asked for.
    #[test]
    fn mcap_cuts_one_clip_per_embedded_trigger() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-embedded")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording_with_triggers(
            &rec,
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(1_000), recorded(2_000)],
                &[embedded(2_500, "first", 1_000, 500)],
                &[recorded(3_000), recorded(4_000)],
                &[embedded(4_200, "second", 300, 1_000)],
                &[recorded(5_000), recorded(6_000)],
            ],
        )?;
        let out_dir = root.join("clipped");

        clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        // One clip per trigger, each named by its own trigger's log time.
        let mut written: Vec<String> = std::fs::read_dir(&out_dir)?
            .map(|entry| Ok(entry?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<Vec<_>>>()?
            .into_iter()
            .filter(|name| name.ends_with(".mcap"))
            .collect();
        written.sort();
        assert_eq!(written, vec!["2500_first.mcap", "4200_second.mcap"]);

        // Each window is its own trigger's, both bounds inclusive:
        // [2500-1000, 2500+500] and [4200-300, 4200+1000].
        assert_eq!(
            clip_data(&out_dir.join("2500_first.mcap"))?,
            vec![2_000, 3_000]
        );
        assert_eq!(
            clip_data(&out_dir.join("4200_second.mcap"))?,
            vec![4_000, 5_000]
        );

        let first = clip::manifest::read_manifest(&out_dir.join("2500_first.mcap"))?
            .expect("every clip carries a manifest");
        assert_eq!(first["trigger.name"], "first");
        assert_eq!(first["trigger.description"], "first happened");
        assert_eq!(
            first["trigger.anchor_ns"], "2500",
            "the window anchors on the recorded trigger message's own log time"
        );
        assert_eq!(first["trigger.preroll_ns"], "1000");
        assert_eq!(first["trigger.postroll_ns"], "500");
        assert_eq!(first["window.start_ns"], "1500");
        assert_eq!(first["window.end_ns"], "3000");
        assert_eq!(first["producer.mode"], "clip");

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// Over a bag directory, every split is read for triggers, in the order the
    /// collection is planned in.
    ///
    /// A trigger sits in the recording that was being written when it fired, so
    /// a reader that stopped at the first split would cut the clips of the
    /// first few minutes and silently drop the rest of the run's.
    #[test]
    fn mcap_reads_every_recording_of_a_bag_directory_for_triggers() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-embedded-bag")?;
        let bag = root.join("record");
        std::fs::create_dir_all(&bag)?;
        clip::testing::write_recording_with_triggers(
            &bag.join("bag_0.mcap"),
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(1_000), recorded(2_000), recorded(3_000)],
                &[embedded(2_000, "first", 500, 500)],
            ],
        )?;
        clip::testing::write_recording_with_triggers(
            &bag.join("bag_1.mcap"),
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(5_000), recorded(6_000), recorded(7_000)],
                &[embedded(6_000, "second", 500, 500)],
            ],
        )?;
        clip::testing::write_bag_metadata(&bag, &["bag_0.mcap", "bag_1.mcap"], &[("/t", 6)])?;
        let out_dir = root.join("clipped");

        clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&bag, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        let mut written: Vec<String> = std::fs::read_dir(&out_dir)?
            .map(|entry| Ok(entry?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<Vec<_>>>()?
            .into_iter()
            .filter(|name| name.ends_with(".mcap"))
            .collect();
        written.sort();
        assert_eq!(
            written,
            vec!["2000_first.mcap", "6000_second.mcap"],
            "one clip per trigger, whichever recording of the collection carried it"
        );
        // Each window sits inside the recording its trigger was written to, so
        // each clip is one segment and holds that recording's messages.
        assert_eq!(clip_data(&out_dir.join("2000_first.mcap"))?, vec![2_000]);
        assert_eq!(clip_data(&out_dir.join("6000_second.mcap"))?, vec![6_000]);

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A clip cut from a command-line trigger and one cut from the equivalent
    /// trigger inside the recording carry the same trigger record — the whole
    /// manifest, key for key, since the two runs differ in nothing else.
    #[test]
    fn a_param_clip_and_an_equivalent_embedded_one_agree() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-agree")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording_with_triggers(
            &rec,
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(1_000), recorded(2_000)],
                &[embedded(3_000, "brake", 1_500, 500)],
                &[recorded(3_500), recorded(4_000)],
            ],
        )?;

        let from_recording = root.join("from-recording");
        clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&rec, &from_recording)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        let from_flags = root.join("from-flags");
        clip_mode(
            ClipConfig {
                trigger_time: Some(3_000),
                preroll: Some(1_500),
                postroll: Some(500),
                trigger_name: Some("brake".to_string()),
                trigger_description: Some("brake happened".to_string()),
                ..param_clip_cfg(&rec, &from_flags)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        let clip_name = "3000_brake.mcap";
        let recorded_manifest = clip::manifest::read_manifest(&from_recording.join(clip_name))?
            .expect("every clip carries a manifest");
        let flagged_manifest = clip::manifest::read_manifest(&from_flags.join(clip_name))?
            .expect("every clip carries a manifest");
        assert_eq!(
            recorded_manifest, flagged_manifest,
            "the same trigger states the same clip, whichever source stated it"
        );
        assert_eq!(
            clip_data(&from_recording.join(clip_name))?,
            clip_data(&from_flags.join(clip_name))?,
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recording holding no trigger, read under `mcap`, is a normal run: it
    /// exits zero, writes nothing at all, and says so.
    #[test]
    fn a_recording_with_no_triggers_cuts_nothing() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-notriggers")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(&rec, true, &[("/t", 1_000), ("/t", 2_000)])?;
        let out_dir = root.join("clipped");

        clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        assert!(
            !out_dir.exists(),
            "a run with nothing to cut writes nothing, not even an output directory"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A `param` run decompresses no chunk to find its trigger, and an `mcap`
    /// run over the same recording proves it: the chunk holding the trigger
    /// channel is destroyed, so the run that reads it fails and the run that
    /// does not cuts its clip.
    #[test]
    fn a_param_run_decompresses_no_chunk_before_the_cut() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-param-nochunk")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording_with_triggers(
            &rec,
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(1_000), recorded(2_000)],
                &[embedded(9_000, "late", 100, 100)],
            ],
        )?;
        // Chunk 1 holds the trigger channel and nothing else. Destroyed, it is
        // unreadable to anyone who opens it.
        let gutted = clip::testing::clobber_chunks(&rec, &root.join("gutted.mcap"), &[1])?;

        // The window covers the surviving chunk only, so the cut itself never
        // asks for the destroyed one.
        let out_dir = root.join("clipped");
        clip_mode(
            ClipConfig {
                trigger_time: Some(1_500),
                preroll: Some(1_000),
                postroll: Some(1_000),
                trigger_name: Some("window".to_string()),
                ..param_clip_cfg(&gutted, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;
        assert_eq!(
            clip_data(&out_dir.join("1500_window.mcap"))?,
            vec![1_000, 2_000],
            "a param run reads the chunks its window needs and no others"
        );

        // The same recording under `mcap` must touch that chunk, and cannot.
        let err = clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&gutted, &root.join("clipped-mcap"))
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )
        .unwrap_err();
        assert!(
            format!("{err:#}").contains("decompressing the chunk"),
            "the trigger chunk really is destroyed: {err:#}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    /// A recorded trigger whose name cannot be embedded in a clip pathname
    /// costs that trigger its clip and no more — the same isolation an
    /// undecodable trigger gets, and the opposite of what an operator's own
    /// `--trigger-name` gets.
    #[test]
    fn an_unsafe_embedded_trigger_name_skips_only_that_trigger() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-badembedded")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording_with_triggers(
            &rec,
            FIXTURE_TRIGGER_TOPIC,
            &[
                &[recorded(1_000), recorded(2_000)],
                &[
                    embedded(2_500, "../escape", 1_000, 500),
                    embedded(2_600, "good", 1_000, 500),
                ],
            ],
        )?;
        let out_dir = root.join("clipped");

        clip_mode(
            ClipConfig {
                trigger_source: TriggerSource::Mcap,
                trigger_time: None,
                preroll: None,
                postroll: None,
                ..param_clip_cfg(&rec, &out_dir)
            },
            CLIP_PRODUCER,
            clip::ChannelSelection::default(),
        )?;

        let written: Vec<String> = std::fs::read_dir(&out_dir)?
            .map(|entry| Ok(entry?.file_name().to_string_lossy().into_owned()))
            .collect::<anyhow::Result<Vec<_>>>()?
            .into_iter()
            .filter(|name| name.ends_with(".mcap"))
            .collect();
        assert_eq!(
            written,
            vec!["2600_good.mcap"],
            "the safe trigger still gets its clip, and the unsafe one none"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }

    // ── supervise() tests ──────────────────────────────────────────────────

    /// A supervised arm that never resolves: the sender is leaked so the
    /// channel never disconnects, and the handle is a finished no-op thread
    /// ([`supervise`] joins a handle only after a disconnect).
    fn pending<T: Send + 'static>() -> Supervised<T> {
        let (tx, rx) = bounded::<T>(1);
        std::mem::forget(tx);
        (rx, thread::spawn(|| {}))
    }

    /// A signal channel that never fires (and never disconnects).
    fn no_signal() -> Receiver<i32> {
        let (tx, rx) = bounded::<i32>(1);
        std::mem::forget(tx);
        rx
    }

    #[test]
    fn supervise_carries_tail_failure_cause() {
        // The tail thread resolves a typed anyhow::Result. A scan fault it
        // could not retry past comes back as a received Err; supervise must
        // wrap it so the formatted chain names the tail thread AND carries
        // the scan-fault root cause for the operator.
        let tail = spawn_supervised("tail", || -> anyhow::Result<()> {
            Err(anyhow::anyhow!("scan of X faulted at offset 42"))
        });

        let err = supervise(tail, pending(), no_signal()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("tail thread"),
            "error must name the tail thread, got: {msg}"
        );
        assert!(
            msg.contains("faulted at offset 42"),
            "error must carry the scan-fault root cause, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_interface_end() {
        // An interface thread that exits cleanly (its trigger source ended) is
        // an error: the recorder stops acting on triggers with no noise. The
        // other arms are parked as "pending forever" to isolate it.
        let interface = spawn_supervised("interface", || -> anyhow::Result<()> { Ok(()) });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("interface thread"),
            "error must name the interface thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_carries_interface_failure_cause() {
        // The interface thread resolves a typed anyhow::Result; a fault comes
        // back as a received Err, wrapped so the chain names the interface
        // thread AND carries the root cause for the operator.
        let interface = spawn_supervised("interface", || -> anyhow::Result<()> {
            Err(anyhow::anyhow!("the trigger subscription stream ended"))
        });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        let msg = format!("{err:#}");
        assert!(
            msg.contains("interface thread"),
            "error must name the interface thread, got: {msg}"
        );
        assert!(
            msg.contains("subscription stream ended"),
            "error must carry the interface fault root cause, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_interface_panic() {
        // A panicking interface drops its result sender without a send; the
        // disconnect routes through the join handle so the formatted chain
        // carries the panic payload and the operator knows what went wrong.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));
        let interface: Supervised<anyhow::Result<()>> =
            spawn_supervised("interface", || -> anyhow::Result<()> { panic!("boom") });

        let err = supervise(pending(), interface, no_signal()).unwrap_err();
        std::panic::set_hook(prev_hook);
        let msg = format!("{err:#}");
        assert!(
            msg.contains("interface thread"),
            "error must name the interface thread, got: {msg}"
        );
        assert!(
            msg.contains("boom") || msg.contains("panic"),
            "error must carry the panic context, got: {msg}"
        );
    }

    #[test]
    fn supervise_reports_tail_end() {
        // The tail loop never returns Ok on its own, so a clean Ok(()) return
        // is an unexpected exit: clips would degrade to grace-timeout cuts
        // silently if not caught.
        let tail = spawn_supervised("tail", || -> anyhow::Result<()> { Ok(()) });

        let err = supervise(tail, pending(), no_signal()).unwrap_err();
        assert!(
            format!("{err:#}").contains("tail thread"),
            "error must name the tail thread, got: {err:#}"
        );
    }

    #[test]
    fn supervise_returns_ok_on_shutdown_signal() {
        // A delivered shutdown signal (SIGINT / SIGTERM) is a requested,
        // orderly stop — not a fault. supervise() must return Ok(()) so main
        // can exit zero, distinguishing it from a dead thread.
        let (sig_tx, sig_rx) = bounded(1);
        sig_tx.send(SIGINT).unwrap();

        let result = supervise(pending(), pending(), sig_rx);
        assert!(
            result.is_ok(),
            "a signal must return Ok(()), got: {result:?}"
        );
    }

    #[test]
    fn supervise_reports_signal_handler_failure() {
        // The signal forwarder dying (its channel disconnecting) must surface
        // as an error naming the signal handler — losing it silently would
        // mean SIGINT/SIGTERM could never trigger a clean shutdown. (An
        // installation failure carries the same attribution, raised in main()
        // before supervision starts.)
        let (sig_tx, sig_rx) = bounded::<i32>(1);
        drop(sig_tx);

        let err = supervise(pending(), pending(), sig_rx).unwrap_err();
        assert!(
            format!("{err:#}").contains("signal handler"),
            "error must name the signal handler, got: {err:#}"
        );
    }

    /// Admission at the limit, rejection above it, and slot reuse — the
    /// acceptance test for MAX_ACTIVE_TRIGGERS. This is the exact scenario the
    /// recorder must handle: 16 concurrent trigger handlers admitted, the 17th
    /// rejected (flood-sanity bound), then a completed handler returns its
    /// permit and a later trigger is admitted. Mirrors the consumer loop: a
    /// permit is taken without waiting and rides in the handler thread until
    /// it finishes.
    #[test]
    fn admission_at_the_limit_rejection_above_and_slot_reuse() -> anyhow::Result<()> {
        let active = Admission::new(MAX_ACTIVE_TRIGGERS);
        // Each handler parks on a watch until released, as a real handler
        // does while it waits out its window and extracts.
        let release = Arc::new(Watch::new(false));

        let mut handlers = Vec::new();
        for i in 0..MAX_ACTIVE_TRIGGERS {
            let permit = active
                .clone()
                .try_acquire()
                .ok_or_else(|| anyhow::anyhow!("trigger {i} rejected; expected admission"))?;
            let release = release.clone();
            handlers.push(std::thread::spawn(move || {
                let _permit = permit;
                release.wait_timeout_for(Duration::from_secs(30), |&v| v);
            }));
        }

        // Trigger 17: every permit is held by a parked handler — admission
        // must fail immediately, without waiting.
        assert!(
            active.clone().try_acquire().is_none(),
            "trigger beyond the limit must be rejected"
        );

        // Handlers complete; their permits return, so a later trigger is
        // admitted as soon as the first one comes back.
        release.send_replace(true);
        let deadline = std::time::Instant::now() + Duration::from_secs(5);
        let _readmitted = loop {
            if let Some(permit) = active.clone().try_acquire() {
                break permit;
            }
            if std::time::Instant::now() >= deadline {
                anyhow::bail!("a completed handler must free a slot");
            }
            std::thread::sleep(Duration::from_millis(5));
        };

        for h in handlers {
            h.join().unwrap();
        }
        Ok(())
    }

    /// `ClipCompression::to_mcap` maps each variant to the expected
    /// `mcap::Compression` option: `None` yields uncompressed, `Zstd` and `Lz4`
    /// yield their named codecs.
    #[test]
    fn clip_compression_to_mcap_maps_all_variants() {
        // `mcap::Compression` is not `PartialEq`, so match the option shape.
        assert!(ClipCompression::None.to_mcap().is_none());
        assert!(matches!(
            ClipCompression::Zstd.to_mcap(),
            Some(mcap::Compression::Zstd)
        ));
        assert!(matches!(
            ClipCompression::Lz4.to_mcap(),
            Some(mcap::Compression::Lz4)
        ));
    }

    /// A panicking handler must return its permit: the admission bound would
    /// otherwise ratchet down with every panic until every trigger is
    /// rejected. The permit is held by the handler thread and returns on
    /// drop, which unwinding covers.
    #[test]
    fn panicking_handler_returns_its_permit() -> anyhow::Result<()> {
        // Suppress the panic backtrace noise in test output.
        let prev_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(|_| {}));

        let active = Admission::new(1);
        let permit = active
            .clone()
            .try_acquire()
            .ok_or_else(|| anyhow::anyhow!("a fresh admission gate must admit"))?;
        let handler = std::thread::spawn(move || {
            let _permit = permit;
            panic!("boom");
        });
        let joined = handler.join();
        std::panic::set_hook(prev_hook);
        assert!(joined.is_err(), "the handler must have panicked");

        assert!(
            active.try_acquire().is_some(),
            "the panicked handler's permit must be free again"
        );
        Ok(())
    }

    /// A representative "now" for the pure-function gate tests — far past every
    /// small anchor the trigger_time tests use, so only the future-skew tests
    /// approach the horizon.
    const TEST_NOW: u64 = 1_000_000_000_000_000_000;

    /// An admissible anchor at `TEST_NOW` from a transport stamp (not
    /// `trigger_time`): the shape the field tests vary one axis away from.
    fn valid_anchor() -> Anchor {
        Anchor {
            ns: TEST_NOW,
            from_trigger_time: false,
        }
    }

    /// A fully valid domain `Trigger` — accepted by `validate_trigger` with
    /// `valid_anchor()` at `TEST_NOW`; a test mutates one field to probe a bound.
    fn valid_trigger() -> Trigger {
        Trigger {
            name: "evt".to_string(),
            description: String::new(),
            trigger_time: clip::trigger::Stamp { sec: 0, nanosec: 0 },
            preroll: 0,
            postroll: 0,
        }
    }

    /// [`valid_trigger`] with a chosen `trigger_time`, for the matrix cell tests.
    fn trigger_with_time(trigger_time_ns: u64) -> Trigger {
        Trigger {
            trigger_time: clip::trigger::Stamp {
                sec: (trigger_time_ns / 1_000_000_000) as i32,
                nanosec: (trigger_time_ns % 1_000_000_000) as u32,
            },
            ..valid_trigger()
        }
    }

    /// A cell that ignores `trigger_time` (the anchor is not `from_trigger_time`
    /// — ros+log, mcap+log, mcap+publish) rejects a non-zero `trigger_time` and
    /// accepts zero.
    #[test]
    fn validate_rejects_trigger_time_in_an_ignoring_cell() {
        let ignoring = Anchor {
            ns: 5,
            from_trigger_time: false,
        };
        assert!(
            validate_trigger(&trigger_with_time(0), ignoring, TEST_NOW).is_ok(),
            "trigger_time=0 is accepted where the field is ignored"
        );
        assert!(
            validate_trigger(&trigger_with_time(7_000_000_250), ignoring, TEST_NOW).is_err(),
            "a non-zero trigger_time is rejected where the field is ignored"
        );
    }

    /// The one reading cell (ros+publish, the anchor *is* `from_trigger_time`)
    /// accepts any `trigger_time` — it is the window anchor there.
    #[test]
    fn validate_accepts_trigger_time_in_the_reading_cell() {
        let reading = Anchor {
            ns: 7_000_000_250,
            from_trigger_time: true,
        };
        assert!(validate_trigger(&trigger_with_time(7_000_000_250), reading, TEST_NOW).is_ok());
        assert!(
            validate_trigger(&trigger_with_time(0), reading, TEST_NOW).is_ok(),
            "trigger_time=0 anchors the window at the epoch, but that is the \
             publisher's choice, not a rejected one"
        );
    }

    /// `preroll` and `postroll` are each accepted exactly at [`MAX_ROLL_NS`] and
    /// rejected one nanosecond above it.
    #[test]
    fn validate_bounds_preroll_and_postroll() {
        let ok = |trig: &Trigger| validate_trigger(trig, valid_anchor(), TEST_NOW).is_ok();

        let mut t = valid_trigger();
        t.preroll = MAX_ROLL_NS;
        assert!(ok(&t), "preroll exactly at the maximum is accepted");
        t.preroll = MAX_ROLL_NS + 1;
        assert!(!ok(&t), "preroll one ns over the maximum is rejected");

        let mut t = valid_trigger();
        t.postroll = MAX_ROLL_NS;
        assert!(ok(&t), "postroll exactly at the maximum is accepted");
        t.postroll = MAX_ROLL_NS + 1;
        assert!(!ok(&t), "postroll one ns over the maximum is rejected");
    }

    /// The future-skew guard is on the *resolved anchor*: accepted exactly at the
    /// horizon, rejected beyond it, whatever cell produced the anchor.
    #[test]
    fn validate_guards_the_resolved_anchor_against_future_skew() {
        let check = |anchor: Anchor| validate_trigger(&valid_trigger(), anchor, TEST_NOW);
        let at = |ns: u64, from_trigger_time: bool| Anchor {
            ns,
            from_trigger_time,
        };

        assert!(
            check(at(TEST_NOW + MAX_ANCHOR_FUTURE_SKEW_NS, false)).is_ok(),
            "an anchor exactly at the future horizon is accepted"
        );
        assert!(
            check(at(TEST_NOW + MAX_ANCHOR_FUTURE_SKEW_NS + 1, false)).is_err(),
            "an anchor one ns past the horizon is rejected"
        );
        // The guard bites whatever cell resolved the anchor — a pathological
        // far-future ros+publish `trigger_time` and a hostile mcap record stamp
        // alike.
        let far_future = TEST_NOW + 10 * MAX_ANCHOR_FUTURE_SKEW_NS;
        assert!(
            check(at(far_future, true)).is_err(),
            "a far-future ros+publish trigger_time anchor is rejected"
        );
        assert!(
            check(at(far_future, false)).is_err(),
            "a far-future mcap record-stamp anchor is rejected"
        );
        assert!(
            check(at(TEST_NOW - 1, false)).is_ok(),
            "a past anchor is fine"
        );
    }

    /// A trigger `name` is accepted plain and exactly at [`MAX_TRIGGER_NAME_LEN`],
    /// and rejected when over-length, empty, or carrying a filename hazard (a
    /// path separator, NUL, leading dot, or embedded `..`).
    #[test]
    fn validate_rejects_unsafe_and_oversized_names() {
        let with_name = |name: &str| {
            let mut t = valid_trigger();
            t.name = name.to_string();
            validate_trigger(&t, valid_anchor(), TEST_NOW)
        };

        assert!(with_name("evt-1").is_ok(), "a plain name is accepted");
        assert!(
            with_name(&"a".repeat(MAX_TRIGGER_NAME_LEN)).is_ok(),
            "a name exactly at the length limit is accepted"
        );
        assert!(
            with_name(&"a".repeat(MAX_TRIGGER_NAME_LEN + 1)).is_err(),
            "a name one byte over the limit is rejected"
        );
        assert!(with_name("").is_err(), "an empty name is rejected");
        assert!(with_name("a/b").is_err(), "a path separator is rejected");
        assert!(with_name("a\\b").is_err(), "a backslash is rejected");
        assert!(with_name("a\0b").is_err(), "a NUL byte is rejected");
        assert!(with_name(".hidden").is_err(), "a leading dot is rejected");
        assert!(with_name("..").is_err(), "'..' is rejected");
        assert!(
            with_name("../escape").is_err(),
            "path traversal is rejected"
        );
        assert!(with_name("a..b").is_err(), "an embedded '..' is rejected");
    }

    // ---- the layered configuration file -------------------------------------

    /// Parse an argv through the whole four-layer path `load_cli` drives, minus
    /// the exit-on-failure.
    fn loaded_from(argv: &[&str]) -> Result<Loaded, StartupError> {
        let _env = env_lock();
        parse_cli(&without_a_system_file(
            argv.iter().map(std::ffi::OsString::from).collect(),
        ))
    }

    /// The two configuration files are found by a scan rather than a parse, so
    /// the scan has to accept what clap accepts: either spelling, a path that is
    /// not UTF-8, and nothing past `--`.
    #[test]
    fn the_config_scan_accepts_both_spellings_and_stops_at_the_separator() {
        use std::ffi::OsString;
        use std::os::unix::ffi::OsStringExt;

        let argv = |args: &[&str]| -> Vec<OsString> { args.iter().map(OsString::from).collect() };

        assert_eq!(
            scan_flag(
                &argv(&["clipper", "tail", "--config", "/tmp/a.toml"]),
                "--config"
            ),
            Some(PathBuf::from("/tmp/a.toml"))
        );
        assert_eq!(
            scan_flag(
                &argv(&["clipper", "tail", "--config=/tmp/a.toml"]),
                "--config"
            ),
            Some(PathBuf::from("/tmp/a.toml"))
        );
        assert_eq!(
            scan_flag(&argv(&["clipper", "tail"]), "--config"),
            None,
            "an absent flag names no file"
        );
        assert_eq!(
            scan_flag(
                &argv(&["clipper", "tail", "--", "--config", "/tmp/a.toml"]),
                "--config"
            ),
            None,
            "nothing past `--` is a flag"
        );
        assert_eq!(
            scan_flag(
                &argv(&["clipper", "tail", "--system-config", "/tmp/s.toml"]),
                "--config"
            ),
            None,
            "a longer flag that starts the same way is a different flag"
        );

        // A path that is not UTF-8 survives both spellings.
        let raw = OsString::from_vec(b"/tmp/\xff.toml".to_vec());
        let mut eq = OsString::from("--config=");
        eq.push(&raw);
        assert_eq!(
            scan_flag(&[OsString::from("--config"), raw.clone()], "--config"),
            Some(PathBuf::from(&raw))
        );
        assert_eq!(scan_flag(&[eq], "--config"), Some(PathBuf::from(&raw)));
    }

    /// The mode is found by a scan of one word, and this is the pair of claims
    /// that makes reading `argv[1]` sound: every mode is named there, and a
    /// command line that names no mode there names none at all.
    #[test]
    fn the_mode_scan_reads_the_word_after_the_program_name() {
        let argv = |args: &[&str]| -> Vec<std::ffi::OsString> {
            args.iter().map(std::ffi::OsString::from).collect()
        };

        assert_eq!(
            scan_mode(&argv(&["clipper", "tail", "--grace-secs", "5"])),
            Some(config::Mode::Tail)
        );
        assert_eq!(
            scan_mode(&argv(&["clipper", "clip", "/tmp/rec.mcap"])),
            Some(config::Mode::Clip)
        );
        for line in [
            vec!["clipper"],
            vec!["clipper", "--help"],
            vec!["clipper", "--version"],
            vec!["clipper", "bogus"],
            vec!["clipper", "--config", "/tmp/a.toml", "tail"],
        ] {
            assert_eq!(
                scan_mode(&argv(&line)),
                None,
                "{line:?} names no mode at argv[1]"
            );
        }
    }

    /// What makes the scan's one word enough: a mode can only ever be the word
    /// after the program name, because `clipper` itself takes no argument that
    /// could precede it. Every command line the scan reads as mode-less is one
    /// clap refuses, so a parse that succeeds is a parse whose files were read.
    #[test]
    fn nothing_may_stand_between_clipper_and_its_mode() {
        let mut root = with_config_args(Cli::command());
        root.build();
        for arg in root.get_arguments() {
            assert!(
                !arg.get_action().takes_values(),
                "clipper's own `{}` takes a value, so it could stand before the mode",
                arg.get_id()
            );
        }

        for line in [
            vec!["clipper"],
            vec!["clipper", "--help"],
            vec!["clipper", "bogus"],
            vec!["clipper", "--", "tail"],
            vec!["clipper", "--config", "/tmp/a.toml", "tail"],
        ] {
            let argv: Vec<std::ffi::OsString> = line.iter().map(std::ffi::OsString::from).collect();
            assert!(
                scan_mode(&argv).is_none(),
                "{line:?} must be mode-less to the scan"
            );
            assert!(
                matches!(loaded_from(&line), Err(StartupError::Cli(_))),
                "{line:?} must not parse"
            );
        }
    }

    /// The `(value, origin)` a report line carries for `key`.
    fn reported(report: &str, key: &str) -> (String, String) {
        let line = report
            .lines()
            .find(|line| line.trim_start().starts_with(&format!("{key} ")))
            .unwrap_or_else(|| panic!("the report has a line for {key}:\n{report}"));
        let (name, rest) = line.split_once(" = ").expect("a report line has a value");
        assert_eq!(name.trim(), key);
        let (value, origin) = rest
            .split_once(" <- ")
            .expect("a report line has an origin");
        (value.trim().to_string(), origin.trim().to_string())
    }

    /// Serialises a test that sets a `MOMENTEDGE_*` variable against every test
    /// that parses a command line.
    ///
    /// The environment is process-global and [`with_env_prefix`] makes every
    /// variable the fallback for its flag, so a variable one test sets is read
    /// by any parse running beside it — and a value that arrives from the
    /// environment can trip a conflict the argv never asked for. The lock is
    /// held for the guard's whole lifetime, so the variable is set and removed
    /// with no parse in between.
    static ENV_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    thread_local! {
        /// How many [`EnvGuard`]s this thread holds. A test that sets a
        /// variable and then parses an argv takes the lock twice on one thread,
        /// which a plain `Mutex` deadlocks on, so only the outermost guard
        /// holds it.
        static ENV_DEPTH: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    }

    /// A reentrant hold on [`ENV_LOCK`].
    struct EnvGuard {
        /// Held only to be dropped: releasing it releases [`ENV_LOCK`]. The
        /// outermost guard on a thread holds `Some`, every nested one `None`.
        _held: Option<std::sync::MutexGuard<'static, ()>>,
    }

    impl Drop for EnvGuard {
        fn drop(&mut self) {
            ENV_DEPTH.with(|depth| depth.set(depth.get() - 1));
        }
    }

    /// Take [`ENV_LOCK`] unless this thread already holds it, ignoring
    /// poisoning: a test that panicked while holding it has already failed, and
    /// cascading that into every later parse reports one fault as dozens.
    fn env_lock() -> EnvGuard {
        let depth = ENV_DEPTH.with(std::cell::Cell::get);
        let held = (depth == 0).then(|| {
            ENV_LOCK
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner)
        });
        ENV_DEPTH.with(|d| d.set(depth + 1));
        EnvGuard { _held: held }
    }

    /// A `MOMENTEDGE_*` variable set for one test and removed again.
    ///
    /// The guard holds [`ENV_LOCK`] for as long as the variable is set, so no
    /// parse runs beside it and reads a value its own argv never named. The
    /// variable is removed before the lock is released: `EnvVar`'s own `Drop`
    /// runs before its fields do, and `_guard` releases the lock only then.
    struct EnvVar {
        name: &'static str,
        _guard: EnvGuard,
    }

    impl EnvVar {
        fn set(name: &'static str, value: &str) -> Self {
            let guard = env_lock();
            #[expect(
                unsafe_code,
                reason = "`set_var` is unsafe since edition 2024; the type's own \
                          note above states the lock discipline that makes it sound"
            )]
            // SAFETY: see the type's note — the lock keeps every other reader
            // out for as long as this is set, and it is removed on drop.
            unsafe {
                std::env::set_var(name, value);
            };
            EnvVar {
                name,
                _guard: guard,
            }
        }
    }

    impl Drop for EnvVar {
        fn drop(&mut self) {
            #[expect(
                unsafe_code,
                reason = "as above — the guard still holds the env lock here"
            )]
            // SAFETY: as above.
            unsafe {
                std::env::remove_var(self.name);
            };
        }
    }

    /// A `clipper clip` system file answering every required argument, so a test
    /// argv can be about the one key it is testing.
    fn write_clip_system_file(
        dir: &std::path::Path,
        name: &str,
        description: Option<&str>,
    ) -> anyhow::Result<PathBuf> {
        let path = dir.join(name);
        let mut text = String::from(
            "[settings]\n\
             recording = \"/tmp/rec.mcap\"\n\
             out_dir = \"/tmp/out\"\n\
             trigger_time = 1\n\
             preroll = 2\n\
             postroll = 3\n",
        );
        if let Some(description) = description {
            text.push_str(&format!("trigger_description = {description:?}\n"));
        }
        std::fs::write(&path, text)?;
        Ok(path)
    }

    /// A setting present in all four layers resolves to the flag, and removing
    /// layers walks it back in the documented order — environment, per-run file,
    /// system file, built-in default.
    #[test]
    fn a_setting_in_all_four_layers_resolves_to_the_flag_and_walks_back() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-four-layers")?;
        let system = write_clip_system_file(&dir, "system.toml", Some("from the system file"))?;
        let run = dir.join("run.toml");
        std::fs::write(
            &run,
            "[settings]\ntrigger_description = \"from the per-run file\"\n",
        )?;
        // The same required arguments, with nothing to say about the key under
        // test: dropping the description's system layer must not drop the
        // arguments that make the command line parse at all.
        let bare = write_clip_system_file(&dir, "bare.toml", None)?;
        let (system, run, bare) = (
            system.display().to_string(),
            run.display().to_string(),
            bare.display().to_string(),
        );

        let described = |argv: &[&str]| -> anyhow::Result<(String, String, String)> {
            let loaded = loaded_from(argv).map_err(|e| match e {
                StartupError::Cli(e) => anyhow::anyhow!("{e}"),
                StartupError::Config(e) => e,
            })?;
            let Mode::Clip(cfg) = loaded.cli.mode else {
                unreachable!("this argv names the clip mode")
            };
            let (value, origin) = reported(&loaded.report, "trigger_description");
            Ok((cfg.trigger_description.unwrap_or_default(), value, origin))
        };

        let all_four = [
            "clipper",
            "clip",
            "--system-config",
            &system,
            "--config",
            &run,
            "--trigger-description",
            "from the flag",
        ];
        let without_flag = &all_four[..6];
        let without_run_file = ["clipper", "clip", "--system-config", &system];
        let without_files = ["clipper", "clip", "--system-config", &bare];

        {
            let _env = EnvVar::set("MOMENTEDGE_TRIGGER_DESCRIPTION", "from the environment");
            let (used, reported, origin) = described(&all_four)?;
            assert_eq!(used, "from the flag");
            assert_eq!(
                (reported.as_str(), origin.as_str()),
                ("from the flag", "flag")
            );

            let (used, reported, origin) = described(without_flag)?;
            assert_eq!(used, "from the environment");
            assert_eq!(
                (reported.as_str(), origin.as_str()),
                ("from the environment", "environment")
            );
        }

        let (used, reported, origin) = described(without_flag)?;
        assert_eq!(used, "from the per-run file");
        assert_eq!(reported, "from the per-run file");
        assert!(origin.starts_with("per-run file"), "{origin}");

        let (used, reported, origin) = described(&without_run_file)?;
        assert_eq!(used, "from the system file");
        assert_eq!(reported, "from the system file");
        assert!(origin.starts_with("system file"), "{origin}");

        let (used, reported, origin) = described(&without_files)?;
        assert_eq!(used, "", "the built-in default is the empty description");
        assert_eq!(reported, "");
        assert_eq!(origin, "built-in default");

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// A missing system file, a missing per-run file, and both missing are all
    /// legal: the run takes the built-in defaults and starts.
    #[test]
    fn missing_configuration_files_are_legal() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-missing-files")?;
        let absent = dir.join("absent.toml").display().to_string();
        let system = dir.join("system.toml");
        std::fs::write(&system, "[settings]\ngrace_secs = 45\n")?;
        let system = system.display().to_string();

        for argv in [
            vec!["clipper", "tail", "--system-config", &absent],
            vec!["clipper", "tail", "--config", &absent],
            vec![
                "clipper",
                "tail",
                "--system-config",
                &absent,
                "--config",
                &absent,
            ],
        ] {
            let loaded = loaded_from(&argv).unwrap_or_else(|_| panic!("{argv:?} must load"));
            let Mode::Tail(cfg) = loaded.cli.mode else {
                unreachable!("this argv names the tail mode")
            };
            assert_eq!(cfg.grace(), Duration::from_secs(30), "{argv:?}");
        }

        // …and a system file that *is* there, with the per-run one missing, is
        // the layer below an absent one rather than a casualty of it.
        let loaded = loaded_from(&[
            "clipper",
            "tail",
            "--system-config",
            &system,
            "--config",
            &absent,
        ])
        .map_err(|_| anyhow::anyhow!("a present system file and an absent run file must load"))?;
        let Mode::Tail(cfg) = loaded.cli.mode else {
            unreachable!("this argv names the tail mode")
        };
        assert_eq!(cfg.grace(), Duration::from_secs(45));

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// A per-run file setting a key reserved to the system file is reported by
    /// name, and the system value stands.
    #[test]
    fn a_per_run_file_overriding_a_system_key_is_reported_and_ignored() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-refusal")?;
        let system = dir.join("system.toml");
        std::fs::write(
            &system,
            "[settings]\nrecord_dir = \"/data/record\"\ngrace_secs = 45\n",
        )?;
        let run = dir.join("run.toml");
        std::fs::write(
            &run,
            "[settings]\nrecord_dir = \"/tmp/mine\"\ngrace_secs = 20\n",
        )?;
        let (system, run) = (system.display().to_string(), run.display().to_string());

        let loaded = loaded_from(&[
            "clipper",
            "tail",
            "--system-config",
            &system,
            "--config",
            &run,
        ])
        .map_err(|_| anyhow::anyhow!("a refused key must not fail the run"))?;
        let report = loaded.report.clone();
        let Mode::Tail(cfg) = loaded.cli.mode else {
            unreachable!("this argv names the tail mode")
        };

        assert_eq!(
            cfg.record_dir,
            PathBuf::from("/data/record"),
            "the system value stands"
        );
        assert_eq!(
            cfg.grace(),
            Duration::from_secs(20),
            "and the per-run file keeps the keys it may set"
        );
        assert!(
            report.contains("record_dir") && report.contains("run.toml"),
            "the report names the refused key and the file:\n{report}"
        );
        let (_, origin) = reported(&report, "record_dir");
        assert!(origin.starts_with("system file"), "{origin}");

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// The same two files under `clipper clip`, which has no `--record-dir`:
    /// the key is neither refused nor fatal there, so nothing is reported and
    /// the run proceeds on the keys it does have.
    ///
    /// This is what `scan_mode` buys end to end. The scope line is the running
    /// subcommand's, so the same per-run file that `clipper tail` is refused
    /// `record_dir` from costs `clipper clip` nothing — and a device's system
    /// file describing the recorder stays readable by both.
    #[test]
    fn a_key_the_running_mode_does_not_have_is_neither_refused_nor_fatal() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-inert-key")?;
        let system = dir.join("system.toml");
        std::fs::write(
            &system,
            "[settings]\nrecord_dir = \"/data/record\"\nout_dir = \"/data/clipped\"\n",
        )?;
        let run = dir.join("run.toml");
        std::fs::write(&run, "[settings]\nrecord_dir = \"/tmp/mine\"\n")?;
        let (system, run) = (system.display().to_string(), run.display().to_string());

        let loaded = loaded_from(&[
            "clipper",
            "clip",
            "/tmp/rec.mcap",
            "--trigger-time",
            "1",
            "--preroll",
            "2",
            "--postroll",
            "3",
            "--system-config",
            &system,
            "--config",
            &run,
        ])
        .map_err(|_| anyhow::anyhow!("the recorder's keys must not fail a clip run"))?;
        let report = loaded.report.clone();
        let Mode::Clip(cfg) = loaded.cli.mode else {
            unreachable!("this argv names the clip mode")
        };

        assert_eq!(cfg.out_dir, PathBuf::from("/data/clipped"));
        assert!(
            !report.contains("record_dir"),
            "`clipper clip` has no --record-dir, so the key is inert and unreported:\n{report}"
        );

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// The effective configuration a report prints is the configuration the run
    /// uses: every line is checked against the field the mode actually reads.
    #[test]
    fn the_effective_configuration_equals_what_the_run_uses() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-effective")?;
        let system = dir.join("system.toml");
        std::fs::write(
            &system,
            "[settings]\n\
             record_dir = \"/data/record\"\n\
             extract_parallelism = 3\n\
             watch_old_files_duration = 90\n\
             delete_old_files = true\n\
             time_source = \"publish\"\n\
             [topics]\n\
             exclude_regex = \"^/diagnostics\"\n",
        )?;
        let run = dir.join("run.toml");
        std::fs::write(
            &run,
            "[settings]\n\
             out_dir = \"/tmp/clips\"\n\
             clip_compression = \"lz4\"\n\
             [topics]\n\
             include = [\"/imu/data\"]\n",
        )?;
        let (system, run) = (system.display().to_string(), run.display().to_string());

        let loaded = loaded_from(&[
            "clipper",
            "tail",
            "--system-config",
            &system,
            "--config",
            &run,
            "--grace-secs",
            "12",
            // A key the per-run file also sets, so the report is caught out if
            // it re-derives a value from the layers instead of reading the one
            // the parse settled on.
            "--out-dir",
            "/tmp/from-the-flag",
        ])
        .map_err(|_| anyhow::anyhow!("this configuration must load"))?;
        let report = loaded.report.clone();
        let selection = loaded.selection.clone();
        let Mode::Tail(cfg) = loaded.cli.mode else {
            unreachable!("this argv names the tail mode")
        };

        // Every settings line against the field the run reads.
        for (key, used, layer) in [
            ("record_dir", cfg.record_dir.display().to_string(), "system"),
            ("out_dir", cfg.out_dir.display().to_string(), "flag"),
            ("grace_secs", cfg.grace_secs.to_string(), "flag"),
            (
                "extract_parallelism",
                cfg.extract_parallelism.to_string(),
                "system",
            ),
            (
                "clip_compression",
                cfg.clip_compression.to_string(),
                "per-run",
            ),
            ("time_source", cfg.time_source.to_string(), "system"),
            (
                "watch_old_files_duration",
                cfg.watch_old_files_duration.to_string(),
                "system",
            ),
            (
                "delete_old_files",
                cfg.delete_old_files.to_string(),
                "system",
            ),
            ("trigger_source", cfg.trigger_source.to_string(), "built-in"),
        ] {
            let (value, origin) = reported(&report, key);
            assert_eq!(value, used, "{key}: the report is the value in use");
            assert!(origin.starts_with(layer), "{key}: {origin}");
        }

        // …and the topics section against the selection the clips are cut with.
        assert_eq!(reported(&report, "include").0, "[\"/imu/data\"]");
        assert_eq!(reported(&report, "exclude_regex").0, "^/diagnostics");
        assert_eq!(reported(&report, "all").0, "false");
        assert!(selection.selects("/imu/data"));
        assert!(!selection.selects("/camera/image_raw"));
        assert!(!selection.selects("/diagnostics"));

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// A file that exists and cannot be used ends the run rather than being
    /// quietly ignored, and says which file and which key.
    #[test]
    fn an_unusable_configuration_file_fails_the_run() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-bad-file")?;
        let bad = dir.join("bad.toml");
        std::fs::write(&bad, "[settings]\ngrace_secondz = 3\n")?;
        let bad = bad.display().to_string();

        match loaded_from(&["clipper", "tail", "--system-config", &bad]) {
            Err(StartupError::Config(err)) => {
                let rendered = format!("{err:#}");
                assert!(rendered.contains("grace_secondz"), "{rendered}");
                assert!(rendered.contains("bad.toml"), "{rendered}");
            }
            Err(StartupError::Cli(err)) => panic!("a file fault is not a parse fault: {err}"),
            Ok(_) => panic!("an unknown key must not be ignored"),
        }

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// Why an argument of a mode is deliberately not a `[settings]` key of that
    /// mode. The exclusions are not one list with one reason, and stating them
    /// as one would lose the distinction a reader needs.
    #[derive(Clone, Copy, Debug)]
    enum NotASettingsKey {
        /// clap's own `--help` and `--version`. They end the process rather
        /// than configure a run, so there is nothing for a file to say.
        ClapsOwn,
        /// The three configuration flags. They are *about* the configuration
        /// rather than in it, so no file can name another file or ask for the
        /// report of one.
        AboutTheConfiguration,
        /// `--trigger-source`. It says how *this process was launched* rather
        /// than what the machine is configured with — a device sets it once in
        /// the unit file that already carries the rest of the invocation — so
        /// it is the flag and `MOMENTEDGE_TRIGGER_SOURCE` and nothing else.
        HowTheProcessWasLaunched,
    }

    impl NotASettingsKey {
        /// The reason in words, for the assertion that names it. An exhaustive
        /// match, so a reason added here has to be spelled out.
        fn reason(self) -> &'static str {
            match self {
                Self::ClapsOwn => "clap's own; it configures no run",
                Self::AboutTheConfiguration => {
                    "about the configuration rather than in it, so no file names another file"
                }
                Self::HowTheProcessWasLaunched => {
                    "how this process was launched, not what the machine is configured with"
                }
            }
        }
    }

    /// Every argument that is deliberately not a key, and which reason it is.
    ///
    /// An argument named nowhere here and nowhere in the scope table fails
    /// [`every_mode_argument_is_a_settings_key_and_back`], so a flag added
    /// without a key has to say which of these it is rather than being waved
    /// through. This list is the drift test's alone: `effective_config` has its
    /// own and shorter one, because `trigger_source` is a setting the run uses
    /// and belongs in the report even though no file may name it.
    const NOT_SETTINGS_KEYS: &[(&str, NotASettingsKey)] = &[
        ("help", NotASettingsKey::ClapsOwn),
        ("version", NotASettingsKey::ClapsOwn),
        (CONFIG_ARG, NotASettingsKey::AboutTheConfiguration),
        (SYSTEM_CONFIG_ARG, NotASettingsKey::AboutTheConfiguration),
        (PRINT_CONFIG_ARG, NotASettingsKey::AboutTheConfiguration),
        ("trigger_source", NotASettingsKey::HowTheProcessWasLaunched),
    ];

    /// Every argument of a mode is a `[settings]` key **of that mode**, and
    /// every key of a mode is an argument of it.
    ///
    /// The scope table lives in the library and the arguments live here, so
    /// nothing but this test stops the two drifting: a flag added to a mode
    /// would otherwise be unsettable from a file, and a key listed under a mode
    /// that has no such flag would name nothing a run of it can use. Checking
    /// each mode against its own rows rather than against the union is what
    /// makes the second half bite — a `recording` row under `tail` passes a
    /// union check and fails this one. The deliberate exceptions are
    /// [`NOT_SETTINGS_KEYS`], each carrying the reason it is one; an argument
    /// listed there must be a key of no mode, and an argument listed nowhere
    /// must be a key of its own.
    #[test]
    fn every_mode_argument_is_a_settings_key_and_back() {
        let cmd = with_config_args(Cli::command());
        for (name, mode) in MODES {
            let sub = cmd
                .find_subcommand(name)
                .unwrap_or_else(|| panic!("{name} is a subcommand of clipper"));
            let mut named = std::collections::BTreeSet::new();
            for arg in sub.get_arguments() {
                let id = arg.get_id().as_str();
                let excused = NOT_SETTINGS_KEYS
                    .iter()
                    .find_map(|(excluded, why)| (*excluded == id).then_some(*why));
                if let Some(why) = excused {
                    assert_eq!(
                        config::scope_of(mode, id),
                        None,
                        "{name}: {id} must not be a settings key — {}",
                        why.reason()
                    );
                    continue;
                }
                assert!(
                    config::scope_of(mode, id).is_some(),
                    "{name}: {id} is a flag with no `[settings]` key"
                );
                named.insert(id.to_string());
            }
            for key in config::setting_keys(mode) {
                assert!(
                    named.contains(key),
                    "{name}: the `{key}` settings key names no argument of this mode"
                );
            }
        }
    }

    /// An argument both subcommands carry accepts the same values in both,
    /// unless [`VALUE_SETS_MAY_DIVERGE`] excuses it and says why.
    ///
    /// One name shared by two subcommands is one name a *layer* can set for
    /// both. A configuration file is read by whichever subcommand runs, and the
    /// `MOMENTEDGE_*` binding is the same word for both, so a value legal for
    /// one is handed to the other — and a value the other refuses stops it at
    /// parse time, on a machine configured for the first. Sharing a name is safe
    /// only while either subcommand would take what a layer wrote for the other.
    ///
    /// The check is over the arguments, not over the `[settings]` keys, because
    /// a key is only one of the layers that can do this: `trigger_source` is no
    /// file's key at all and still shares `MOMENTEDGE_TRIGGER_SOURCE` across
    /// both subcommands.
    #[test]
    fn an_argument_both_subcommands_share_accepts_the_same_values_in_both() {
        let cmd = with_config_args(Cli::command());
        let subcommand = |mode_name: &str| {
            cmd.find_subcommand(mode_name)
                .unwrap_or_else(|| panic!("{mode_name} is a subcommand of clipper"))
        };
        let values = |mode_name: &str, id: &str| -> Vec<String> {
            subcommand(mode_name)
                .get_arguments()
                .find(|arg| arg.get_id().as_str() == id)
                .unwrap_or_else(|| panic!("{mode_name} carries {id}"))
                .get_possible_values()
                .iter()
                .map(|value| value.get_name().to_string())
                .collect()
        };

        let clip_ids: std::collections::BTreeSet<&str> = subcommand(CLIP_MODE)
            .get_arguments()
            .map(|arg| arg.get_id().as_str())
            .collect();
        let shared: Vec<&str> = subcommand(TAIL_MODE)
            .get_arguments()
            .map(|arg| arg.get_id().as_str())
            .filter(|id| clip_ids.contains(id) && !matches!(*id, "help" | "version"))
            .collect();
        assert!(
            shared.contains(&"out_dir") && shared.contains(&"trigger_source"),
            "the subcommands share at least where clips go and where triggers \
             come from: {shared:?}"
        );

        for id in shared {
            let excused = VALUE_SETS_MAY_DIVERGE
                .iter()
                .find_map(|(excused, why)| (*excused == id).then_some(*why));
            if let Some(why) = excused {
                assert_ne!(
                    values(TAIL_MODE, id),
                    values(CLIP_MODE, id),
                    "`{id}` is excused from sharing a value set — {why} — but the \
                     subcommands agree on it, so the excuse is stale"
                );
                continue;
            }
            assert_eq!(
                values(TAIL_MODE, id),
                values(CLIP_MODE, id),
                "`{id}` is an argument of both subcommands, so a layer setting it \
                 for one must not hand the other a value it refuses"
            );
        }
    }

    /// The arguments both subcommands carry whose value sets deliberately
    /// differ, each with the reason no layer can exploit the difference.
    ///
    /// An entry is a promise that nothing reaching *both* subcommands by default
    /// can set the argument, so only a deliberate act can hand one subcommand
    /// the other's value. It is not a way to wave the check through: an excused
    /// argument whose value sets agree also fails, so an entry that stops being
    /// needed reports itself.
    const VALUE_SETS_MAY_DIVERGE: &[(&str, &str)] = &[(
        "trigger_source",
        "no configuration file may name it, so the only layer reaching both is \
         `MOMENTEDGE_TRIGGER_SOURCE`, which an operator exports deliberately \
         rather than a run picking it up from /etc",
    )];

    /// A configuration file naming `trigger_source` fails the run, whichever
    /// subcommand reads it, and the error names the key.
    ///
    /// It is not a key of either mode, so it is not "some other subcommand's
    /// key" and therefore inert — it is nobody's key, which is the misspelling
    /// case. The trigger source is the flag and `MOMENTEDGE_TRIGGER_SOURCE`,
    /// and a file that tries to decide it is told so at the first start.
    #[test]
    fn a_file_naming_trigger_source_fails_either_subcommand() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-trigger-source-file")?;
        let named = dir.join("named.toml");
        std::fs::write(&named, "[settings]\ntrigger_source = \"ros\"\n")?;
        let named = named.display().to_string();

        for argv in [
            vec!["clipper", "tail", "--system-config", &named],
            vec![
                "clipper",
                "clip",
                "rec.mcap",
                "--out-dir",
                "/data/clips",
                "--trigger-time",
                "1",
                "--preroll",
                "0",
                "--postroll",
                "1",
                "--system-config",
                &named,
            ],
        ] {
            match loaded_from(&argv) {
                Err(StartupError::Config(err)) => {
                    let rendered = format!("{err:#}");
                    assert!(rendered.contains("trigger_source"), "{rendered}");
                    assert!(rendered.contains("named.toml"), "{rendered}");
                }
                Err(StartupError::Cli(err)) => {
                    panic!("{argv:?}: a file fault is not a parse fault: {err}")
                }
                Ok(_) => panic!("{argv:?}: a file may not name `trigger_source`"),
            }
        }

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// A device's system file, describing the recorder that runs on that
    /// machine, leaves `clipper clip` on the same machine able to start.
    ///
    /// What this pins is the hazard of a shared name: a file is read by
    /// whichever subcommand runs, so any key both carry is a value one of them
    /// can hand the other. The trigger source being no file's key is what keeps
    /// the recorder's own settings from reaching the cutter — a system file
    /// describing the machine's recorder has nothing in it that `clipper clip`
    /// must accept, so the cutter starts on a fully configured device.
    #[test]
    fn the_recorders_system_file_leaves_the_cutter_able_to_start() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-device-file-under-clip")?;
        let system = dir.join("clipper.toml");
        std::fs::write(
            &system,
            "[settings]\n\
             record_dir = \"/data/record\"\n\
             out_dir = \"/data/clipped\"\n\
             extract_parallelism = 1\n\
             grace_secs = 45\n",
        )?;
        let system = system.display().to_string();

        let loaded = loaded_from(&[
            "clipper",
            "clip",
            "rec.mcap",
            "--trigger-time",
            "1",
            "--preroll",
            "0",
            "--postroll",
            "1",
            "--system-config",
            &system,
        ])
        .map_err(|e| match e {
            StartupError::Cli(e) => anyhow::anyhow!("{e}"),
            StartupError::Config(e) => e,
        })?;
        let Mode::Clip(cfg) = loaded.cli.mode else {
            unreachable!("this argv names the clip mode")
        };
        assert_eq!(
            cfg.trigger_source,
            TriggerSource::Param,
            "the recorder's file has nothing to say about the cutter's source"
        );
        assert_eq!(
            cfg.out_dir,
            PathBuf::from("/data/clipped"),
            "the keys the two subcommands share still cross"
        );

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// `MOMENTEDGE_TRIGGER_SOURCE` reaches both subcommands' argument, and the
    /// report says the environment decided it.
    ///
    /// The environment layer is the other half of what the trigger source has:
    /// whoever launches the process commands it, from the unit file or from the
    /// command line, and from nowhere below.
    #[test]
    fn the_trigger_source_environment_variable_reaches_both_subcommands() -> anyhow::Result<()> {
        let _env = EnvVar::set("MOMENTEDGE_TRIGGER_SOURCE", "mcap");

        let tail = loaded_from(&["clipper", "tail"])
            .map_err(|_| anyhow::anyhow!("`clipper tail` must parse"))?;
        let Mode::Tail(tail_cfg) = tail.cli.mode else {
            unreachable!("this argv names the tail mode")
        };
        assert_eq!(tail_cfg.trigger_source, TriggerSource::Mcap);
        assert_eq!(
            reported(&tail.report, "trigger_source"),
            ("mcap".to_string(), "environment".to_string())
        );

        let clip = loaded_from(&["clipper", "clip", "rec.mcap", "--out-dir", "/data/clips"])
            .map_err(|_| anyhow::anyhow!("`clipper clip` must parse"))?;
        let Mode::Clip(clip_cfg) = clip.cli.mode else {
            unreachable!("this argv names the clip mode")
        };
        assert_eq!(clip_cfg.trigger_source, TriggerSource::Mcap);
        assert_eq!(
            reported(&clip.report, "trigger_source"),
            ("mcap".to_string(), "environment".to_string())
        );

        Ok(())
    }

    /// `--print-config` reports `trigger_source` under both subcommands, and
    /// never at a file layer.
    ///
    /// It is a setting the run uses, so leaving it out of the report would hide
    /// which source a run took; it simply has no layer below the environment to
    /// resolve to, so the report reads `flag` or `built-in default` and nothing
    /// else. The system file here is the device's, to prove a file present and
    /// full of the recorder's keys still cannot claim the line.
    #[test]
    fn trigger_source_is_reported_by_both_subcommands_at_no_file_layer() -> anyhow::Result<()> {
        let dir = clip::testing::test_dir("cli-trigger-source-report")?;
        let system = dir.join("clipper.toml");
        std::fs::write(
            &system,
            "[settings]\nrecord_dir = \"/data/record\"\nout_dir = \"/data/clipped\"\n",
        )?;
        let system = system.display().to_string();

        let clip_argv = ["clipper", "clip", "rec.mcap"];
        let clip_window = ["--trigger-time", "1", "--preroll", "0", "--postroll", "1"];
        let defaulted: Vec<&str> = ["clipper", "tail", "--system-config", &system]
            .into_iter()
            .collect();
        let clip_defaulted: Vec<&str> = clip_argv
            .into_iter()
            .chain(clip_window)
            .chain(["--system-config", &system])
            .collect();
        let flagged: Vec<&str> = ["clipper", "tail", "--trigger-source", "mcap"]
            .into_iter()
            .chain(["--system-config", &system])
            .collect();
        let clip_flagged: Vec<&str> = clip_argv
            .into_iter()
            .chain(["--trigger-source", "mcap"])
            .chain(["--system-config", &system])
            .collect();

        for (argv, expected) in [
            (defaulted, "built-in default"),
            (clip_defaulted, "built-in default"),
            (flagged, "flag"),
            (clip_flagged, "flag"),
        ] {
            let loaded = loaded_from(&argv).map_err(|_| anyhow::anyhow!("{argv:?} must parse"))?;
            let (_, origin) = reported(&loaded.report, "trigger_source");
            assert_eq!(origin, expected, "{argv:?}");
        }

        std::fs::remove_dir_all(dir)?;
        Ok(())
    }

    /// The three configuration flags reach every mode, and carry the same
    /// `MOMENTEDGE_*` fallback every other argument does.
    #[test]
    fn the_configuration_flags_reach_every_mode_with_their_env_names() {
        let cmd = with_env_prefix(with_config_args(Cli::command()));
        for mode in ["tail", "clip"] {
            let sub = cmd
                .find_subcommand(mode)
                .unwrap_or_else(|| panic!("{mode} is a subcommand of clipper"));
            for (id, env) in [
                (CONFIG_ARG, "MOMENTEDGE_CONFIG"),
                (SYSTEM_CONFIG_ARG, "MOMENTEDGE_SYSTEM_CONFIG"),
                (PRINT_CONFIG_ARG, "MOMENTEDGE_PRINT_CONFIG"),
            ] {
                let arg = sub
                    .get_arguments()
                    .find(|arg| arg.get_id().as_str() == id)
                    .unwrap_or_else(|| panic!("{mode} carries --{id}"));
                assert_eq!(
                    arg.get_env().map(|e| e.to_string_lossy().into_owned()),
                    Some(env.to_string()),
                    "{mode}: {id}"
                );
            }
        }
    }

    /// `--print-config` is what a caller asks the report for; every mode takes
    /// it and nothing else is run.
    ///
    /// Both modes are given the *same* file, one written with `clipper clip`'s
    /// keys: a file is one file, and the keys of a mode that is not running are
    /// inert rather than fatal, so `clipper tail` reads it and simply applies
    /// the `out_dir` they share.
    #[test]
    fn print_config_is_requested_per_mode() -> anyhow::Result<()> {
        for mode in [TAIL_MODE, CLIP_MODE] {
            let dir = clip::testing::test_dir(&format!("cli-print-{mode}"))?;
            let system = write_clip_system_file(&dir, "system.toml", None)?
                .display()
                .to_string();
            let loaded = loaded_from(&[
                "clipper",
                mode,
                "--system-config",
                &system,
                "--print-config",
            ])
            .map_err(|_| anyhow::anyhow!("{mode} --print-config must parse"))?;
            assert!(loaded.print_config, "{mode}");
            assert!(
                loaded.report.starts_with(&format!("clipper {mode} ")),
                "{mode}"
            );

            let quiet = loaded_from(&["clipper", mode, "--system-config", &system])
                .map_err(|_| anyhow::anyhow!("{mode} must parse without --print-config"))?;
            assert!(!quiet.print_config, "{mode}");
            std::fs::remove_dir_all(dir)?;
        }
        Ok(())
    }

    /// The one-shot cutter cuts the topics the configuration selects, so the
    /// mode that runs off a finished recording filters exactly as the recorder
    /// does.
    #[test]
    fn clip_mode_cuts_only_the_selected_topics() -> anyhow::Result<()> {
        let root = clip::testing::test_dir("clip-mode-topics")?;
        let rec = root.join("rec.mcap");
        clip::testing::write_recording(
            &rec,
            true,
            &[("/imu/data", 1_000), ("/camera/image_raw", 1_100)],
        )?;
        let out_dir = root.join("clipped");
        let selection = clip::ChannelSelection::try_from(clip::select::Spec {
            include: vec!["/imu/data".to_string()],
            ..clip::select::Spec::default()
        })?;

        clip_mode(
            ClipConfig {
                trigger_time: Some(1_000),
                preroll: Some(0),
                postroll: Some(1_000),
                trigger_name: Some("sel".to_string()),
                ..param_clip_cfg(&rec, &out_dir)
            },
            Producer {
                program: PROGRAM,
                mode: "clip",
            },
            selection,
        )?;

        let clip_path = out_dir.join("1000_sel.mcap");
        assert_eq!(
            clip::testing::read_clip(&clip_path)?,
            vec![("/imu/data".to_string(), 1_000)],
            "only the selected topic is cut"
        );
        let manifest =
            clip::manifest::read_manifest(&clip_path)?.expect("every clip carries a manifest");
        assert_eq!(
            manifest
                .keys()
                .filter(|k| k.starts_with("channel."))
                .count(),
            3,
            "the excluded topic has no per-channel manifest keys: {manifest:?}"
        );

        std::fs::remove_dir_all(root)?;
        Ok(())
    }
}
