//! Periodic `/events/momentedge/trigger` publisher.
//!
//! Emits a `momentedge_msgs/Trigger` every `--period` seconds (1 s by default)
//! so the triggered recorder (`clipper`) has something to react to during
//! development.
//!
//! `trigger_time` is left zero by default. clipper reads `trigger_time` only
//! under `--interface ros --time-source publish`; every other cell (including the
//! default `--time-source log`) anchors the window on the recorder's own receipt
//! instant and *rejects* a trigger that sets `trigger_time`, so the default dev
//! loop must send zero. Pass `--stamp-trigger-time` to stamp it with the current
//! RosTime at publish — the publish-domain anchor a `--time-source publish` run
//! centres its window on.
//!
//! Flags (all optional):
//!
//! ```text
//! --period <secs>        seconds between triggers          (default 1)
//! --preroll <ns>         nanoseconds kept before the anchor (default: random per trigger)
//! --postroll <ns>        nanoseconds kept after the anchor  (default: random per trigger)
//! --name <prefix>        trigger name prefix; a counter is appended (default "periodic")
//! --description <text>   free-form description carried in the trigger
//! --stamp-trigger-time   stamp trigger_time with RosTime at publish (for
//!                        --time-source publish); off by default (trigger_time=0)
//! ```
//!
//! When `--preroll`/`--postroll` are omitted, each trigger draws a fresh window
//! independently — a random whole number of seconds in `[1, 10]` — so the
//! recorder sees varied clip lengths during development. Pass either flag to pin
//! that side to a fixed nanosecond value.
//!
//! Logging uses the `log` facade with a pretty_env_logger backend; `RUST_LOG`
//! controls verbosity (defaults to `info`).

use std::time::Duration;

use log::info;
use r2r::QosProfile;

/// Inclusive bounds, in seconds, for a randomly drawn pre/postroll window.
const RANDOM_ROLL_SECS: std::ops::RangeInclusive<u64> = 1..=10;

struct Args {
    period: Duration,
    /// `None` means "draw a fresh random window per trigger"; `Some(ns)` pins it.
    preroll: Option<u64>,
    postroll: Option<u64>,
    name: String,
    description: String,
    /// Stamp `trigger_time` with RosTime at publish. Off by default: only
    /// `--interface ros --time-source publish` reads `trigger_time`; the other
    /// cells reject a non-zero one, so the default sends zero.
    stamp_trigger_time: bool,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            period: Duration::from_secs(1),
            preroll: None,
            postroll: None,
            name: "periodic".to_string(),
            description: "periodic test trigger".to_string(),
            stamp_trigger_time: false,
        }
    }
}

fn parse_args() -> Args {
    let mut args = Args::default();
    let mut it = std::env::args().skip(1);
    while let Some(flag) = it.next() {
        let mut value = || it.next().expect("flag needs a value");
        match flag.as_str() {
            "--period" => {
                args.period = Duration::from_secs_f64(value().parse().expect("--period: number"))
            }
            "--preroll" => args.preroll = Some(value().parse().expect("--preroll: u64 ns")),
            "--postroll" => args.postroll = Some(value().parse().expect("--postroll: u64 ns")),
            "--name" => args.name = value(),
            "--description" => args.description = value(),
            "--stamp-trigger-time" => args.stamp_trigger_time = true,
            other => panic!("unknown flag: {other}"),
        }
    }
    args
}

/// Resolve a roll window to nanoseconds: the fixed value if pinned, otherwise a
/// fresh random whole-second draw in [`RANDOM_ROLL_SECS`].
fn resolve_roll(fixed: Option<u64>) -> u64 {
    fixed.unwrap_or_else(|| fastrand::u64(RANDOM_ROLL_SECS) * 1_000_000_000)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let args = parse_args();

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "clipper_trigger_pub", "")?;
    let publisher = node.create_publisher::<r2r::momentedge_msgs::msg::Trigger>(
        "/events/momentedge/trigger",
        QosProfile::default(),
    )?;
    // RosTime clock — sampled into trigger_time only when --stamp-trigger-time
    // is set (the publish-domain anchor).
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;

    let describe_roll = |fixed: Option<u64>| match fixed {
        Some(ns) => format!("{ns} ns"),
        None => format!(
            "random {}-{}s",
            RANDOM_ROLL_SECS.start(),
            RANDOM_ROLL_SECS.end()
        ),
    };
    info!(
        "publishing /events/momentedge/trigger every {:.1}s  (preroll={}, postroll={})",
        args.period.as_secs_f64(),
        describe_roll(args.preroll),
        describe_roll(args.postroll),
    );

    let mut counter: u64 = 0;
    loop {
        // Zero unless --stamp-trigger-time: clipper reads trigger_time only under
        // --interface ros --time-source publish and rejects a non-zero one in
        // every other cell.
        let trigger_time = if args.stamp_trigger_time {
            r2r::Clock::to_builtin_time(&clock.get_now()?)
        } else {
            r2r::builtin_interfaces::msg::Time { sec: 0, nanosec: 0 }
        };
        let name = format!("{}-{counter}", args.name);
        let preroll = resolve_roll(args.preroll);
        let postroll = resolve_roll(args.postroll);
        let msg = r2r::momentedge_msgs::msg::Trigger {
            name: name.clone(),
            description: args.description.clone(),
            trigger_time: trigger_time.clone(),
            preroll,
            postroll,
        };
        publisher.publish(&msg)?;
        info!(
            "trigger #{counter} name={name} stamp={}.{:09} preroll={preroll} ns postroll={postroll} ns",
            trigger_time.sec, trigger_time.nanosec
        );
        counter += 1;

        // No subscriptions to service, but spin briefly to keep the node's
        // graph/publisher machinery healthy, then wait out the period.
        node.spin_once(Duration::from_millis(10));
        std::thread::sleep(args.period);
    }
}
