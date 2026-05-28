//! Periodic `/events/edgestream/trigger` publisher.
//!
//! Emits an `edgestream_msgs/Trigger` every `--period` seconds (5 s by default)
//! so the triggered recorder (`edgestream-rec`) has something to react to during
//! development. Each trigger's `trigger_time` is stamped with the current
//! RosTime at publish — the "original timestamp" the pre/post-roll window is cut
//! around — mirroring how the other recorders in this workspace key off a
//! message's own stamp rather than its arrival time.
//!
//! Flags (all optional):
//!   --period <secs>       seconds between triggers          (default 5)
//!   --preroll <ns>        nanoseconds kept before the stamp (default 3_000_000_000)
//!   --postroll <ns>       nanoseconds kept after the stamp  (default 3_000_000_000)
//!   --name <prefix>       trigger name prefix; a counter is appended (default "periodic")
//!   --description <text>  free-form description carried in the trigger
//!
//! Logging uses the `log` facade with a pretty_env_logger backend; `RUST_LOG`
//! controls verbosity (defaults to `info`).

use std::time::Duration;

use log::info;
use r2r::QosProfile;

struct Args {
    period: Duration,
    preroll: u64,
    postroll: u64,
    name: String,
    description: String,
}

impl Default for Args {
    fn default() -> Self {
        Args {
            period: Duration::from_secs(5),
            preroll: 3_000_000_000,
            postroll: 3_000_000_000,
            name: "periodic".to_string(),
            description: "periodic test trigger".to_string(),
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
            "--preroll" => args.preroll = value().parse().expect("--preroll: u64 ns"),
            "--postroll" => args.postroll = value().parse().expect("--postroll: u64 ns"),
            "--name" => args.name = value(),
            "--description" => args.description = value(),
            other => panic!("unknown flag: {other}"),
        }
    }
    args
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(log::LevelFilter::Info)
        .parse_default_env()
        .init();

    let args = parse_args();

    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "edgestream_trigger_pub", "")?;
    let publisher = node.create_publisher::<r2r::edgestream_msgs::msg::Trigger>(
        "/events/edgestream/trigger",
        QosProfile::default(),
    )?;
    // RosTime clock: the trigger_time stamp the recorder centres its window on.
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;

    info!(
        "publishing /events/edgestream/trigger every {:.1}s  (preroll={} ns, postroll={} ns)",
        args.period.as_secs_f64(),
        args.preroll,
        args.postroll,
    );

    let mut counter: u64 = 0;
    loop {
        let now = clock.get_now()?;
        let trigger_time = r2r::Clock::to_builtin_time(&now);
        let name = format!("{}-{counter}", args.name);
        let msg = r2r::edgestream_msgs::msg::Trigger {
            name: name.clone(),
            description: args.description.clone(),
            trigger_time: trigger_time.clone(),
            preroll: args.preroll,
            postroll: args.postroll,
        };
        publisher.publish(&msg)?;
        info!(
            "trigger #{counter} name={name} stamp={}.{:09}",
            trigger_time.sec, trigger_time.nanosec
        );
        counter += 1;

        // No subscriptions to service, but spin briefly to keep the node's
        // graph/publisher machinery healthy, then wait out the period.
        node.spin_once(Duration::from_millis(10));
        std::thread::sleep(args.period);
    }
}
