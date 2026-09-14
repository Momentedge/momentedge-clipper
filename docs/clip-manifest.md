# What a clip carries

A clip is a **directory** holding one or more MCAP files and the
`clip_metadata.yaml` that says what they are. This page is that record: the
layout it describes, every field of the document, how a clip's id is derived, and
how to read a clip that came out empty.

```
clipped/
  1726300000000000000_fc43-6475-ade8-4730/
    1726300000000000000_fc43-6475-ade8-4730_0.mcap
    1726300000000000000_fc43-6475-ade8-4730_1.mcap   # only when the window straddled a split
    clip_metadata.yaml                                # written last; present = complete
```

- **The directory is the clip**, and its name is the clip's
  [id](#the-clip-id). `Recorded` names it, an upload pipeline syncs it, and
  `--out-dir` holds nothing but directories like it.
- **Each file is `<id>_N.mcap`**, `N` counting from `0` over the source
  recordings that contributed — always numbered, so a clip that fits in one file
  is `<id>_0.mcap` and a consumer reads one rule rather than two. One file per
  source recording; they are never merged. The names match what
  `ros2 bag record -o <id>` would have written, so `ros2 bag reindex <id> mcap`
  produces a rosbag2 `metadata.yaml` for the directory on demand.
- **`clip_metadata.yaml` is written last**, after every MCAP file in the
  directory is durable, and it arrives at that name by a rename, so it is there
  whole or not at all. **Its presence is what "complete" means:** a directory
  without it is a clip still being written or the residue of one that died, and
  is never a clip to upload.

Inside each MCAP file, one `momentedge.clip` metadata record carries the clip's
id and nothing else — so a file separated from its directory can still be grouped
— while everything a clip has to say about itself is stated once, in the document
beside the files.

```console
$ mcap get metadata --name momentedge.clip \
    ./clipped/1726300000000000000_fc43-6475-ade8-4730/1726300000000000000_fc43-6475-ade8-4730_0.mcap
{
  "clip.id": "1726300000000000000_fc43-6475-ade8-4730"
}
```

## `clip_metadata.yaml`

A window over a recording that rolled over mid-window, cut by the device
recorder:

```yaml
version: '1'
clip:
  id: 1726300000000000000_fc43-6475-ade8-4730
  messages: 4211
  short: false
producer:
  name: clipper
  mode: tail
  version: 0.1.3
  url: https://github.com/Momentedge/momentedge-clipper
trigger:
  name: brake-event
  description: hard brake over 0.8 g
  anchor_ns: 1726300000000000000
  preroll_ns: 5000000000
  postroll_ns: 5000000000
window:
  time_source: log
  start_ns: 1726299995000000000
  end_ns: 1726300005000000000
  files_planned: 2
sources:
- file: 1726300000000000000_fc43-6475-ade8-4730_0.mcap
  path: /data/record/rosbag2_0.mcap
  extents_read: 3
  bytes_read: 12058624
  messages: 2118
  channels:
    1:
      messages: 126
      first_ns: 1726299995012000000
      last_ns: 1726299999988000000
    2:
      messages: 1992
      first_ns: 1726299995004000000
      last_ns: 1726299999999000000
- file: 1726300000000000000_fc43-6475-ade8-4730_1.mcap
  path: /data/record/rosbag2_1.mcap
  extents_read: 2
  bytes_read: 9437184
  messages: 2093
  channels:
    1:
      messages: 125
      first_ns: 1726300000012000000
      last_ns: 1726300004988000000
    2:
      messages: 1968
      first_ns: 1726300000001000000
      last_ns: 1726300004999000000
```

A clip that fits in one file has the same shape with a single `sources` entry.

### `version`

The document's own schema version, and its first field. Fields may be added
within a version; a field that changes meaning takes a new version, so a reader
checks this before trusting the rest.

### `clip`

What the clip is, independent of how many files it took to hold it.

| Field | What it says |
|---|---|
| `id` | the clip's [id](#the-clip-id) — also the directory's name and the stem every file in it opens with |
| `messages` | how many messages the whole clip holds, across every file |
| `short` | `true` when the cut ran without the recording ever reaching the window end, so the clip may stop short of what was asked for |

### `producer`

Which build of which program cut the clip.

| Field | What it says |
|---|---|
| `name` | the binary, as an operator invokes it (`clipper`) |
| `mode` | the subcommand that cut it: `tail` for the device recorder, `clip` for a cut over a finished recording |
| `version` | the cut path's own crate version — the version to quote when a clip looks wrong |
| `url` | the project the cut path came from |

### `trigger`

The trigger that asked for the clip, echoed verbatim: `name`, `description`,
`anchor_ns` (the instant the window centres on, as the trigger source resolved
it), `preroll_ns` and `postroll_ns`. The [`Trigger`
message](triggers-and-time.md) is where those come from.

### `window`

The window the clip was cut for.

| Field | What it says |
|---|---|
| `time_source` | the clock domain the whole window lives on, `log` or `publish` |
| `start_ns`, `end_ns` | the window's inclusive bounds, `[anchor − preroll, anchor + postroll]` |
| `files_planned` | how many source recordings the window was planned over — which is **not** `len(sources)`: a recording that was read and contributed nothing is planned and does not appear below |

### `sources`

One entry per MCAP file in the directory, in `_0`, `_1`, … order.

| Field | What it says |
|---|---|
| `file` | the clip's own file this entry describes, a name inside the clip directory |
| `path` | the recording it was copied from. **Absent** for the one file a window no recording covered still produces — there was no source to name |
| `extents_read` | how many planned byte ranges of that recording the copy read |
| `bytes_read` | how many bytes those ranges spanned |
| `messages` | how many messages were copied into this file |
| `channels` | per channel id **of this file**, what that channel contributed: `messages`, and the earliest and latest stamp among them on the window's clock. A channel the file holds nothing of has no entry |

Channel ids are per file — each MCAP numbers its own channels from scratch — so
join a `channels` key to the `Channel` records of the file its entry names, never
to another file's.

Which channels a clip holds at all is the
[`[topics]` selection](configuration.md#which-topics-a-clip-contains): an excluded
topic is absent from the clip's channels, its schemas, and these entries alike.

## The clip id

A clip's **id** is what its directory is named by — `clip.id` in the document
above — and it is a function of the request that asked for the clip and of
nothing else:

```
<anchor_ns>_<hash>          1726300000000000000_fc43-6475-ade8-4730
```

- **`anchor_ns`** is the resolved anchor in nanoseconds on the run's
  [time source](triggers-and-time.md#the-anchor-which-instant-the-window-centres-on).
  It leads so that ids sort in time order.
- **`hash`** is the first 16 lower-case hex characters of SHA-256 over the
  canonical encoding below, written as four groups of four separated by `-`.

Three properties follow, and all three are worth relying on:

- **The same request yields the same id**, on every machine and every clipper
  version, so an id quoted in a report stays valid.
- **A change to any of the six fields yields a different id** — the description
  included, which changes no byte of the clip's data. Two detectors firing on one
  instant with different names or windows therefore get their own clips rather
  than colliding.
- **An id already on disk is never written twice.** A window whose directory
  exists is skipped with a warning, whatever that directory holds, so a repeated
  trigger, a restarted recorder and a re-run of `clipper clip` all leave the clip
  that is there exactly as it is.

No trigger text appears in the id, so a `name` holding `/`, `..`, unicode or
nothing at all shapes no path.

### The canonical encoding

The encoding is a **published contract** and does not change: predict a clip's
directory before it exists, or check one you were given. It is nine lines, each
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

A trigger named `brake-event`, described `hard brake over 0.8 g`, anchored at
`1726300000000000000` on `log`, with five seconds of preroll and three of
postroll:

```console
$ printf 'momentedge.clip.id/1\n1726300000000000000\n11\nbrake-event\n21\nhard brake over 0.8 g\n5000000000\n3000000000\nlog\n' | sha256sum
896d87139fb1f609aaeea772f8c860d7c274672ed821d1e204aefc3ae2a0fa2c  -
```

The leading 16 characters, in groups of four, are the hash — so this trigger's
clip directory is

```
1726300000000000000_896d-8713-9fb1-f609
```

It shares its anchor with the clip at the top of this page and not its id: that
one's postroll is five seconds, and a change to any of the six fields is a
different clip.

Every number in this vector differs from every other, which is deliberate. Two
lines carrying one value — a preroll and a postroll that agree — encode the same
bytes when they are exchanged, so a vector built from them would go on matching
an encoding that had swapped the two while every real id moved. The vector is
pinned by a test, so the encoding cannot drift without a deliberate edit.

## An empty clip still explains itself

Every trigger produces a clip even when there was nothing to copy — one
`<id>_0.mcap` holding an empty MCAP — and two fields say which kind of empty it
is:

| `window.files_planned` | `clip.short` | What happened |
|---|---|---|
| `0` | `true` | nothing covered the window — the recording never reached it (`--grace-secs` ran out) |
| `0` | `false` | the window fell in a gap between splits — the recording ran past it but held no bytes inside it |
| `>= 1` | `false` | a recording was read, and no message fell inside the window |

`clip.short` is the one fact a clip cannot show from its own contents: a clip
whose last message sits well before the window end looks the same whether the
recorded topics went quiet or the recorder never got there.

## Reading a clip's files

Each file is a complete, standalone MCAP: a summary, a footer and closing magic,
so every MCAP tool reads one directly. The `momentedge.clip` record inside it
survives the MCAP CLI's rewrite commands (`compress`, `decompress`, `sort`,
`filter`, `recover`). `mcap merge` refuses two files of one clip by default, since
both carry a record under the same name — pass `--allow-duplicate-metadata`.
