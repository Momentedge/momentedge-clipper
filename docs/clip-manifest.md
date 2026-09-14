# What a clip carries

Every clip is a standalone MCAP that says what it is. This page is the
`momentedge.clip` metadata record: its keys, what each says, how a clip's id is
derived, and how to read a clip that came out empty.

Every clip is a standalone MCAP holding the copied messages **and one metadata
record naming what it is**, written under `momentedge.clip`. It is indexed in
the clip's summary and counted in its statistics, so any MCAP tool finds it
without scanning the file:

```console
$ mcap get metadata --name momentedge.clip \
    ./clipped/1726300000000000000_fc43-6475-ade8-4730.mcap
{
  "manifest.version": "1",
  "producer.name": "clipper",
  "producer.mode": "tail",
  "producer.version": "0.1.3",
  "producer.url": "https://github.com/Momentedge/momentedge-clipper",
  "trigger.name": "brake-event",
  "trigger.description": "hard brake over 0.8 g",
  "trigger.anchor_ns": "1726300000000000000",
  "trigger.preroll_ns": "5000000000",
  "trigger.postroll_ns": "5000000000",
  "window.time_source": "log",
  "window.start_ns": "1726299995000000000",
  "window.end_ns": "1726300005000000000",
  "source.path": "/data/record/rosbag2_0.mcap",
  "source.files_planned": "1",
  "source.extents_read": "3",
  "source.bytes_read": "12058624",
  "clip.id": "1726300000000000000_fc43-6475-ade8-4730",
  "clip.messages": "4211",
  "clip.short": "false",
  "channel.1.messages": "251",
  "channel.1.first_ns": "1726299995012000000",
  "channel.1.last_ns": "1726300004988000000"
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
| `clip.*` | the clip's id, how many messages were copied, and whether the cut ended short of the window |
| `channel.<id>.*` | per channel of *this clip* — message count and the earliest and latest stamp; a channel the clip holds nothing of has no keys |

Which channels the clip holds at all is the
[`[topics]` selection](configuration.md#which-topics-a-clip-contains): an excluded topic is
absent from the clip's channels, its schemas, and these keys alike.

A window straddling a bag split publishes one segment per source recording, and
each segment carries its own record naming its own `source.path`. Every segment
of one window states the same `clip.id`, so files separated from the directory
they were written into can still be grouped.

## The clip id

A clip's **id** is what it is named by — `clip.id` in the record above is the
file's own stem — and it is a function of the request that asked for the clip and
of nothing else:

```
<anchor_ns>_<hash>          1726300000000000000_fc43-6475-ade8-4730
```

- **`anchor_ns`** is the resolved anchor in nanoseconds on the run's
  [time source](triggers-and-time.md#the-anchor-which-instant-the-window-centres-on).
  It leads so that ids sort in time order.
- **`hash`** is the first 16 lower-case hex characters of SHA-256 over the
  canonical encoding below, written as four groups of four separated by `-`.

Two properties follow, and both are worth relying on:

- **The same request yields the same id**, on every machine and every clipper
  version, so an id quoted in a report stays valid.
- **A change to any of the six fields yields a different id** — the description
  included, which changes no byte of the clip's data. Two detectors firing on one
  instant with different names or windows therefore get their own clips rather
  than colliding.

No trigger text appears in the id, so a `name` holding `/`, `..`, unicode or
nothing at all shapes no path.

### The canonical encoding

The encoding is a **published contract** and does not change: predict a clip's
name before it exists, or check one you were given. It is nine lines, each
terminated by `\n`:

```
momentedge.clip.id/1
<anchor_ns>
<byte length of name>
<name>
<byte length of description>
<description>
<preroll_ns>
<postroll_ns>
log | publish
```

- The first line is the encoding's own name and version, and is inside the hash:
  a future encoding is a new tag and a new id space, never the same bytes quietly
  meaning something else.
- The integers are decimal, without separators, padding or a sign.
- The time source is the word its command line takes, `log` or `publish`.
- The **byte length** — not the character count — precedes each of the two
  free-text fields, and is what delimits them. The newline after each is part of
  the encoding but carries no meaning, so a name that itself holds newlines, or
  ends in one, still encodes to exactly one byte string.

### A worked vector

A trigger named `brake-event`, described `hard brake over 0.8 g`, with five
seconds of roll each way, anchored at `1726300000000000000` on `log`:

```console
$ printf 'momentedge.clip.id/1\n1726300000000000000\n11\nbrake-event\n21\nhard brake over 0.8 g\n5000000000\n5000000000\nlog\n' | sha256sum
fc436475ade84730780c870fb500412fe9d0ee985bbebdbff8ce54ce6f2222dd  -
```

The leading 16 characters, in groups of four, are the hash — so the clip is

```
1726300000000000000_fc43-6475-ade8-4730
```

That vector is pinned by a test, so the encoding cannot drift without a
deliberate edit.

## An empty clip still explains itself

Every trigger produces a clip even when there was nothing to copy, and three keys
say which kind of empty it is:

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
