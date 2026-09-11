# What a clip carries

Every clip is a standalone MCAP that says what it is. This page is the
`momentedge.clip` metadata record: its keys, what each says, and how to read a
clip that came out empty.

Every clip is a standalone MCAP holding the copied messages **and one metadata
record naming what it is**, written under `momentedge.clip`. It is indexed in
the clip's summary and counted in its statistics, so any MCAP tool finds it
without scanning the file:

```console
$ mcap get metadata --name momentedge.clip ./clipped/1738000000000000000_brake-event.mcap
{
  "manifest.version": "1",
  "producer.name": "clipper",
  "producer.mode": "tail",
  "producer.version": "0.1.3",
  "producer.url": "https://github.com/Momentedge/momentedge-clipper",
  "trigger.name": "brake-event",
  "trigger.description": "hard brake over 0.8 g",
  "trigger.anchor_ns": "1738000000000000000",
  "trigger.preroll_ns": "5000000000",
  "trigger.postroll_ns": "5000000000",
  "window.time_source": "log",
  "window.start_ns": "1737999995000000000",
  "window.end_ns": "1738000005000000000",
  "source.path": "/data/record/rosbag2_0.mcap",
  "source.files_planned": "1",
  "source.extents_read": "3",
  "source.bytes_read": "12058624",
  "clip.messages": "4211",
  "clip.short": "false",
  "channel.1.messages": "251",
  "channel.1.first_ns": "1737999995012000000",
  "channel.1.last_ns": "1738000004988000000"
}
```

The keys are flat and dotted, the values all strings:

| Group | What it says |
|---|---|
| `manifest.version` | the record's own schema version; a reader checks it before trusting the rest |
| `producer.*` | the binary, the subcommand that cut the clip (`tail` or `clip`), the cut path's crate version, the project URL |
| `trigger.*` | the trigger that asked: name, description, the anchor it resolved to, preroll and postroll |
| `window.*` | the clock the window lives on (`log`/`publish`) and its inclusive bounds |
| `source.*` | the recording the bytes came from, how many recordings the window was planned over, and how much was read |
| `clip.*` | how many messages were copied, and whether the cut ended short of the window |
| `channel.<id>.*` | per channel of *this clip* — message count and the earliest and latest stamp; a channel the clip holds nothing of has no keys |

Which channels the clip holds at all is the
[`[topics]` selection](configuration.md#which-topics-a-clip-contains): an excluded topic is
absent from the clip's channels, its schemas, and these keys alike.

A window straddling a bag split publishes one segment per source recording, and
each segment carries its own record naming its own `source.path`.

**An empty clip still explains itself.** Every trigger produces a clip even when
there was nothing to copy, and three keys say which kind of empty it is:

| `source.files_planned` | `clip.short` | What happened |
|---|---|---|
| `0` | `true` | nothing covered the window — the recording never reached it (`--grace-secs` ran out) |
| `0` | `false` | the window fell in a gap between splits — the recording ran past it but held no bytes inside it |
| `>= 1` | `false` | a recording was read, and no message fell inside the window |

`clip.short` is the one fact a clip cannot show from its own contents: a clip
whose last message sits well before the window end looks the same whether the
recorded topics went quiet or the recorder never got there.

The record survives the MCAP CLI's rewrite commands (`compress`, `decompress`,
`sort`, `filter`, `recover`). `mcap merge` refuses two clips by default, since
both carry a record under the same name — pass `--allow-duplicate-metadata`.

