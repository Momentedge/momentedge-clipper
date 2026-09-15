# Cut a clip on your laptop

Seeing clipper work without a robot, a ROS 2 installation or a bag from the
field. The route is [`clipper clip`](clip-command.md), which cuts windows out of
a *finished* recording: no live topic, no ROS, no waiting — so a sample
recording written by one of the repo's own examples is enough to produce a real
clip you can open in [Foxglove](https://foxglove.dev/).

**What it costs:** a Rust toolchain (1.85 or newer, for edition 2024) and
nothing else — no ROS, no `colcon`, no device. The first `cargo build` dominates;
everything after it is about two minutes, twenty-five seconds of which is the
recording writing itself.

For what clipper is and how it runs against a live recorder, start at the
[README](../README.md).

## 1. Build the two binaries

```bash
git clone https://github.com/Momentedge/momentedge-clipper
cd momentedge-clipper
cargo build -p clipper -p chunked-mcap-writer
```

`clipper` is the recorder and the clip tool in one binary. The default build
links no ROS at all, which is why this works on a plain laptop;
[`chunked-mcap-writer`](../examples/chunked-mcap-writer/README.md) is the
example producer that stands in for `ros2 bag record`.

## 2. Write a sample recording

The writer emits a 50 Hz `/pose` channel and one synthetic
`momentedge_msgs/Trigger` every ten seconds, each asking for three seconds
either side of itself. Let it run for about twenty-five seconds — two triggers —
then stop it with Ctrl+C, which finalises the file:

```console
$ mkdir -p record
$ ./target/debug/chunked-mcap-writer --out record/demo.mcap
recording /pose to record/demo.mcap every 20 ms (zstd chunks close every 4096 uncompressed bytes), one trigger every 10s — Ctrl+C to stop
trigger emitted on /events/momentedge/trigger: anchor=1789454041904972135ns preroll=3000000000ns postroll=3000000000ns
trigger emitted on /events/momentedge/trigger: anchor=1789454051897716282ns preroll=3000000000ns postroll=3000000000ns
^C
wrote 1243 /pose messages to record/demo.mcap
```

That is an ordinary MCAP file of about 70 kB, and every `anchor=` it printed is
a nanosecond instant you can cut around in step 5. Both `record/` and `clipped/`
are in the repo's `.gitignore`, so the clone stays clean whatever you do here.

## 3. Cut a clip per recorded trigger

The recording carries its own triggers, so `--trigger-source mcap` needs nothing
else: one clip per trigger, each anchored on the instant the recording stamped
that trigger message with, each taking the preroll and postroll the trigger
itself asked for.

```console
$ ./target/debug/clipper clip ./record --out-dir ./clipped --trigger-source mcap
INFO  clipper > clipper clip effective configuration
  ...
INFO  clipper   > cutting 2 clip(s) from 1 recording(s) under ./record into ./clipped (source=log)
INFO  clipper   > cutting chunked-mcap-writer-example window=[1789454038904972135, 1789454044904972135] anchor=1789454041904972135
INFO  clip::cut > clip ./clipped/1789454041904972135_7984-4afc-5e09-946e/1789454041904972135_7984-4afc-5e09-946e_0.mcap written: 299 msgs from 8 extents, 0.0 MiB
INFO  clipper   > cutting chunked-mcap-writer-example window=[1789454048897716282, 1789454054897716282] anchor=1789454051897716282
INFO  clip::cut > clip ./clipped/1789454051897716282_708d-c929-ffe0-4bfd/1789454051897716282_708d-c929-ffe0-4bfd_0.mcap written: 300 msgs from 9 extents, 0.0 MiB
INFO  clipper   > 2 clip(s) cut, 0 skipped as already there in ./clipped
```

`./record` is passed as a directory, the same way a bag directory of splits is:
a window straddling two splits comes out whole. Every run prints its effective
configuration first, which the `...` above stands in for.

## 4. Look at what you got

```console
$ ls ./clipped/
1789454041904972135_7984-4afc-5e09-946e
1789454051897716282_708d-c929-ffe0-4bfd

$ ls ./clipped/1789454041904972135_7984-4afc-5e09-946e/
1789454041904972135_7984-4afc-5e09-946e_0.mcap  clip_metadata.yaml
```

A clip is a directory named by its **clip id** — the window's anchor plus a
digest of the trigger — holding one standalone MCAP per contributing recording
and the document that says what it is. `clip_metadata.yaml` is written last, and
its presence is what "complete" means:

```console
$ cat ./clipped/1789454041904972135_7984-4afc-5e09-946e/clip_metadata.yaml
version: '1'
clip:
  id: 1789454041904972135_7984-4afc-5e09-946e
  messages: 299
  short: false
producer:
  name: clipper
  mode: clip
  version: 0.1.3
  url: https://github.com/Momentedge/momentedge-clipper
trigger:
  name: chunked-mcap-writer-example
  description: synthetic trigger emitted by chunked-mcap-writer
  anchor_ns: 1789454041904972135
  preroll_ns: 3000000000
  postroll_ns: 3000000000
window:
  time_source: log
  start_ns: 1789454038904972135
  end_ns: 1789454044904972135
  files_planned: 1
sources:
- file: 1789454041904972135_7984-4afc-5e09-946e_0.mcap
  path: ./record/demo.mcap
  extents_read: 8
  bytes_read: 13258
  messages: 299
  channels:
    1:
      messages: 298
      first_ns: 1789454038924522455
      last_ns: 1789454044901346198
    2:
      messages: 1
      first_ns: 1789454041904972135
      last_ns: 1789454041904972135
```

Every field is explained in [What a clip carries](clip-manifest.md). The `.mcap`
beside it is a complete, standard recording — drag it into Foxglove, or read it
with the [`mcap` CLI](https://mcap.dev/guides/cli):

```console
$ mcap info ./clipped/1789454041904972135_7984-4afc-5e09-946e/1789454041904972135_7984-4afc-5e09-946e_0.mcap
library:     mcap-rust/0.25.0
messages:    299
duration:    5.976823743s
start:       2026-09-15T06:33:58.924522455Z (1789454038.924522455)
end:         2026-09-15T06:34:04.901346198Z (1789454044.901346198)
...
channels:
	(1) /pose                     	298 msgs (49.7..49.9Hz)	 : Pose [jsonschema]
	(2) /events/momentedge/trigger	  1 msgs               	 : <no schema>
```

Six seconds and 299 messages out of a twenty-five-second, 1245-message
recording — three seconds of them from *before* the trigger fired, which is the
whole point of clipping from a recording that was already running.

## 5. Cut a window you choose

Where the recording carries no trigger, or you want a different window than the
one it asked for, name the window on the command line instead. Any instant
inside the recording works: the numbers printed in step 2 are nanoseconds since
the Unix epoch, so pick one and adjust it.

```console
$ ./target/debug/clipper clip ./record --out-dir ./clipped \
    --trigger-time 1789454046000000000 \
    --preroll 1500000000 --postroll 1500000000 \
    --trigger-name my-first-clip
INFO  clipper > cutting 1 clip(s) from 1 recording(s) under ./record into ./clipped (source=log)
INFO  clipper > cutting my-first-clip window=[1789454044500000000, 1789454047500000000] anchor=1789454046000000000
INFO  clipper > 1 clip(s) cut, 0 skipped as already there in ./clipped
```

The name lands in the clip's document under `trigger.name` and is one of the six
fields the clip id hashes, so no part of it reaches a path. Run the same command
twice and the second run skips the window it has already cut and exits 0 —
that is what makes a re-run a resume.

## If your own recording is refused

`clipper clip` plans a window from the recording's own summary rather than by
walking it, so the recording has to carry one. An unchunked recording carries
nothing to plan from and is refused by name before anything is written:

```
Error: plain.mcap's summary indexes no chunk: the writer used an unchunked
profile, and an unchunked recording carries nothing to plan a window from. ...
```

This is the one trap between you and a clip, and it catches real recordings, not
just examples. `ros2 bag record --storage mcap` writes chunked output on its
default profile, as does `chunked-mcap-writer` above. Unchunked is a deliberate
choice both the `fastwrite` storage profile — which
[`examples/continuous/`](../examples/continuous/README.md) recommends for the
lowest live-tail latency — and
[`custom-mcap-writer`](../examples/custom-mcap-writer/README.md) make, because a
top-level record is visible to a tailer the instant it lands. It costs nothing
for `clipper tail`, which reads the growing file itself; it is what `clipper
clip` refuses, because there is no summary to plan from.

`mcap recover in.mcap -o out.mcap` rewrites such a file into the chunked,
summarised, message-indexed shape this reads. clipper never rewrites, recovers
or re-indexes a recording itself. Every other refusal and its cause is in
[`clipper clip`](clip-command.md#when-a-recording-is-refused).

## Where to go next

| Page | What it answers |
|---|---|
| [`clipper clip`](clip-command.md) | bag directories, refusals, re-runs, exit statuses |
| [What a clip carries](clip-manifest.md) | every field of `clip_metadata.yaml`, and how a clip id is derived |
| [Triggers and time](triggers-and-time.md) | the `Trigger` message, and which clock a window lives on |
| [Configuration](configuration.md) | every flag, environment variable and TOML key |
| [Installing and building](install.md) | the `.deb`s, the two cargo builds, the nix package |

To run the live recorder instead — clipper tailing a recording as it is written,
cutting on triggers as they arrive — you need a ROS 2 environment and
`--features ros`; the [README's quickstart](../README.md#quickstart) is that
path, and [`examples/`](../examples/README.md) has the setup guides.
