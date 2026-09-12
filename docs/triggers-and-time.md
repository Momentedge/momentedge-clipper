# Triggers and time

What a trigger is, how it reaches clipper, and which clock the window it asks
for lives on. For cutting from a recording that is already finished, see
[`clipper clip`](clip-command.md); for the flags named here, see
[Configuration](configuration.md). The words this page leans on — anchor, window,
trigger source, and the several instants a message can be said to have happened
at — are each defined once in the [glossary](../CONTEXT.md).

## The trigger message

A trigger is a `momentedge_msgs/Trigger` message:

| Field | Type | Meaning |
|---|---|---|
| `name` | `string` | trigger identifier; becomes part of the clip filename |
| `description` | `string` | optional free-form context |
| `trigger_time` | `builtin_interfaces/Time` | publish-domain anchor; read only under `--trigger-source ros --time-source publish` (see [the anchor matrix](#the-anchor-which-instant-the-window-centres-on)), must be `0` in every other cell |
| `preroll` | `uint64` | nanoseconds before the anchor to keep |
| `postroll` | `uint64` | nanoseconds after the anchor to keep |

**Validation.** Every field is checked before any work; a trigger failing any
check is logged at `error!` and ignored — no clip, no `Recorded`. The limits
(each value exactly at its bound is accepted):

- `preroll` and `postroll` — at most **30 minutes** (`1_800_000_000_000` ns) each.
- The resolved **anchor** — at most **30 minutes** past the current clock. The
  anchor drives the window's wait, so a far-future one (a producer clock fault or
  a hostile record stamp) is refused rather than parking a handler for that long.
- `name` — non-empty, at most **128 bytes**, and safe to embed in the clip
  pathname: no path separator, NUL, leading `.`, or `..`.
- `trigger_time` — `0` except in the one cell that reads it (`--trigger-source
  ros --time-source publish`); non-zero elsewhere is rejected (see
  [the anchor matrix](#the-anchor-which-instant-the-window-centres-on)).

For each finished clip, clipper publishes a `momentedge_msgs/Recorded` on
`/events/momentedge/recorded`, echoing the trigger's `name`, `description`, and
`trigger_time` and listing every file written in its `string[] filenames`. Every
path it names is already complete and crash-durable on disk.

**Clip naming.** A window that falls inside a single recording produces one
file, `<anchor_ns>_<name>.mcap`, where `<anchor_ns>` is the resolved window
anchor. A window that straddles a rollover (a rosbag2 bag split or a recorder
restart clipper observed while running) produces one segment per source file —
`<anchor_ns>_<name>_00.mcap`, `_01.mcap`, … — tiling the window in time order,
all listed in `filenames`.

## Two ways in: `ros` and `mcap`

Where `clipper tail`'s triggers come from is one choice, set by
`--trigger-source`, and exactly one source is active per launch. The **completion
half follows from it** — there is no second setting for how a finished clip is
announced:

- **`ros`** (the deployed path, and the default where it exists) subscribes to
  `/events/momentedge/trigger` and publishes `momentedge_msgs/Recorded` on
  `/events/momentedge/recorded`.
- **`mcap`** reads triggers straight out of the recording clipper already tails
  (run `ros2 bag record --all` so the trigger topic is captured) and runs
  **ROS-free** — no node, subscription, or publisher. It publishes nothing, so
  the clip's atomic move into `--out-dir` is the only completion signal.

Both cut identical clips; only the trigger and completion edges differ.

`ros` is the source the `ros` cargo feature adds, so a
[ROS-free build](../README.md#install) offers `mcap` alone and takes it by default, and
`clipper tail --help` lists the values the binary in front of you accepts. Asking
a build for a source it does not carry is a parse error naming the value and
listing what it does take.

The `mcap` source decodes each trigger by its MCAP `message_encoding`: `json`
decodes in every build, while `cdr` — what `ros2 bag record` writes — needs the
rmw typesupport the same feature links, and a ROS-free build skips such a trigger
with an error naming the feature. So a ROS-free deployment wants a producer that
writes its triggers as `json` (see
[`examples/custom-mcap-writer`](../examples/custom-mcap-writer/README.md)).

**`mcap` means the same thing on both sides of the recording's end.**
`--trigger-source` is one key spanning both subcommands, and its `mcap` value
names the recorded trigger in either: under `clipper tail` the tailed recording
that is still growing, under
[`clipper clip`](clip-command.md#where-a-clip-runs-triggers-come-from-param-and-mcap)
the finished one. Same topic, same decoder, same anchor rule — so a trigger
stream that drives the recorder on the vehicle re-cuts the same clips from the
bag afterwards. The values that do not span both are the ones with nowhere to
land: `ros` is a live subscription, which a finished recording has none of, and
`param` names a single trigger on the command line, which a run with no end has
no use for.

## Time source: `log` or `publish`

`--time-source` picks the clock domain the **whole clip window** lives in — the
anchor it centres on, which messages fall inside it, which bytes are read, and
the coverage a cut waits for. Every MCAP message carries two stamps, and clipper
windows on whichever the flag selects:

- **`log`** (the default) — the message's `log_time`: when the producer received
  it. One writer stamps every recording in receive order, so log times run
  (approximately) non-decreasing on disk. Coverage on `log` is a *completeness*
  proof: once it passes a window end, every in-window message is on disk.
- **`publish`** — the message's `publish_time`: whatever the producer wrote
  there. `ros2 bag record` fills it with the DDS source timestamp; a momentedge
  writer fills it with the capture time (see
  [`examples/custom-mcap-writer`](../examples/custom-mcap-writer/README.md)). clipper
  never interprets it — it windows on the raw value.

## The anchor: which instant the window centres on

The window centres on an **anchor** the active trigger source resolves from what
it has. A live ROS trigger carries no recording stamp, so the `ros` source
anchors on `now` or the publisher's `trigger_time`; an in-recording trigger
carries its own stamps, so the `mcap` source anchors on those. The four
`--trigger-source` × `--time-source` cells resolve it thus:

| | `--time-source log` | `--time-source publish` |
|---|---|---|
| **`--trigger-source ros`** | `now` at the subscription instant | the trigger's `trigger_time` |
| **`--trigger-source mcap`** | the trigger record's `log_time` | the trigger record's `publish_time` |

**`trigger_time` is read in exactly one cell — `ros` + `publish`.** There it is
the anchor: a publisher declaring its own publish-domain instant, standing in for
the `publish_time` it cannot set on the wire, so a request like "clip around ten
minutes ago" lands where it means to. Every other cell anchors on a transport
stamp and **rejects** a trigger that sets a non-zero `trigger_time` — logging it
at `error!` and cutting no clip — rather than silently dropping the field and
mis-anchoring the window. A producer for those cells must send `trigger_time = 0`
(the [`trigger-pub`](../examples/trigger-pub/README.md) example does by default).

Retention is unaffected by the flag — a recording is always aged out on its
`log_time`, so a producer cannot drive file deletion through `publish_time`.

**Publish coverage is a liveness signal, not a completeness proof.** Publish
times carry no ordering guarantee; out-of-order arrival is normal. Under
`--time-source publish` a message can land on disk *after* a cut with a
`publish_time` that falls inside the window, and is then missing from that clip.
`--grace-secs` bounds how long a cut waits, exactly as on `log`.

**On Humble, `--time-source publish` is a no-op.** Humble's
`rosbag2_storage_mcap` writes `publish_time = log_time` verbatim, so the two
domains are identical there. It differs on Jazzy and newer (where `publish_time`
is the DDS source timestamp) and for a momentedge writer (capture time).

