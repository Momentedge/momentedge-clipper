# ros2-pulse-trigger: a clip when a topic rate goes wrong

A small rclpy node that turns [ros2_pulse](https://github.com/TanayK07/ros2_pulse)
diagnostics into clipper triggers. ros2_pulse is an `LD_PRELOAD` probe that
measures topic rates inside a running ROS 2 process and checks them against an
expected-rate spec; its `pulse_bridge` publishes the verdict on `/diagnostics`.
This node watches those statuses and publishes one `momentedge_msgs/Trigger` on
`/events/momentedge/trigger` when one of them leaves `OK`, so the clip holds the
seconds before a topic slowed down or stopped.

It is one Python file with no build step. It needs a sourced ROS 2 environment
with `momentedge_msgs` in it, the same one clipper runs in.

## Behaviour

On each `/diagnostics` message the node looks at the statuses whose name starts
with `prefix` (default `ros2_pulse:`). The bridge names them like this
([full table](https://github.com/TanayK07/ros2_pulse#diagnostic_aggregator-diagnostics)):

| Status | Not `OK` when |
|---|---|
| `ros2_pulse: <topic> (<side>)` | `WARN`: the probe reports the rule's rate or gap bound broken |
| `ros2_pulse: node <name>` | `ERROR`: a node the spec expects is missing |
| `ros2_pulse: log <file>` | `ERROR`: a probed process stopped writing windows |
| `ros2_pulse: <topic>` | `WARN`: a warn no spec rule accounts for |

A status that moves into one of `levels` (default `WARN` and `ERROR`), or is
first seen there, fires one trigger. It does not fire again while it stays
there, `WARN` to `ERROR` included, because the bridge republishes every status
on every poll. It re-arms once it is back at `OK`, and never fires sooner than
`cooldown_s` after its previous trigger, so a rule flapping on its bound does not
cut a clip every window.

The trigger it publishes:

- `name`: the status name, e.g. `ros2_pulse: /chatter (recv)`, cut to clipper's
  128-byte limit.
- `description`: the level, message and values, e.g.
  `WARN rate 0.000 Hz outside [8, inf] (hz=0.000 min_hz=8 max_gap_ms=300)`.
- `trigger_time`: `0`, so clipper anchors the window on the instant it receives
  the trigger.
- `preroll` / `postroll`: `preroll_s` / `postroll_s`, in nanoseconds.

The probe judges a rate once per window (5 s by default), and the bridge polls
once a second, so the trigger reaches clipper up to a window and a poll after
the fault. Size `preroll_s` to cover that delay plus the lead-up you want to see.

Parameters (all optional, set with `--ros-args -p name:=value`):

| Parameter | Default | Meaning |
|---|---|---|
| `preroll_s` | `30.0` | seconds kept before the anchor (at most 1800) |
| `postroll_s` | `10.0` | seconds kept after the anchor (at most 1800) |
| `levels` | `[WARN, ERROR]` | levels that fire (`OK`, `WARN`, `ERROR`, `STALE`) |
| `prefix` | `ros2_pulse:` | only statuses whose name starts with this |
| `cooldown_s` | `60.0` | least seconds between two triggers of one status |

The seconds parameters are doubles, so pass `30.0` rather than `30`.

## Run

Install ros2_pulse (0.7.0 or newer, which added `/diagnostics`) on Humble,
Jazzy or Kilted:

```bash
sudo apt install ros-$ROS_DISTRO-ros2-pulse
```

If apt still has an older version, build it
[from source](https://github.com/TanayK07/ros2_pulse#from-source).

Write a spec, `expected.yaml`: at least 8 Hz arriving on `/chatter`, with no gap
over 300 ms. The format is in
[Expected-rate alerting](https://github.com/TanayK07/ros2_pulse#expected-rate-alerting).

```yaml
topics:
  /chatter: {min_hz: 8, max_gap_ms: 300}
```

Then, each in its own shell sharing one ROS 2 environment, from the repo root:

```bash
# 1. Continuous recording -> ./record
ros2 bag record --all --storage mcap --output ./record

# 2. clipper, tailing ./record, writing clips to ./clipped
clipper tail --record-dir ./record --out-dir ./clipped --clip-compression zstd

# 3. A 10 Hz publisher
ros2 topic pub -r 10 /chatter std_msgs/msg/String "{data: hello}"

# 4. The app, with the probe preloaded and checking the spec
ROS_TOPIC_STATS_FORMAT=jsonl ROS_TOPIC_STATS_EXPECTED=$PWD/expected.yaml \
LD_PRELOAD=$(ros2 pkg prefix ros2_pulse)/lib/libros2_pulse.so \
  ros2 run demo_nodes_cpp listener

# 5. The bridge, publishing the probe's verdict on /diagnostics
ros2 run ros2_pulse pulse_bridge --ros-args \
  -p diagnostics:=true -p expected_file:=$PWD/expected.yaml

# 6. This node
python3 examples/ros2-pulse-trigger/pulse_trigger.py --ros-args \
  -p preroll_s:=30.0 -p postroll_s:=10.0
```

To see it fire, stop the publisher in shell 3 with Ctrl-C. Within a window the
listener's `/chatter` rate drops to 0, the bridge turns
`ros2_pulse: /chatter (recv)` to `WARN`, this node logs the trigger, and once the
postroll has passed clipper writes the clip under `./clipped`. Start the
publisher again and the status returns to `OK`; stopping it once more, after the
cooldown, cuts a second clip.

Stopping the probed listener itself fires nothing: a process that exits cleanly
takes its statuses with it. A process that crashes or hangs does, because its
log goes stale and `ros2_pulse: log <file>` turns `ERROR`.

The tests cover the edge detection and need no ROS 2:

```bash
python3 -m unittest examples/ros2-pulse-trigger/test_pulse_trigger.py
```

The field-by-field contract this node writes to is
[The trigger message](../../docs/triggers-and-time.md#the-trigger-message).
