# clip

What every consumer of a recording shares: the MCAP format layer and its
recording index, the copy that cuts a window out of one, the neutral trigger and
completion contract, the segment assembly that turns one window into published
clips, the manifest each of those clips carries, the channel selection that says
which topics it holds, and the layered configuration file behind both.

**No ROS anywhere by default**, and no async runtime. Cutting a window out of a
recording nobody is writing links this crate alone — no successor to find, no
lifecycle to run, nothing to wait for. What following a recording *still being
written* adds is [`tail`](../tail/CLAUDE.md); what only a ROS node needs is
[`clipper`](../clipper/CLAUDE.md). The seam between the three, the feature
matrix, and the clock domain every window lives in are one level up in
[`crates/CLAUDE.md`](../CLAUDE.md).

## The format layer: why a live MCAP can be read (`clip::index`)

The **format** reasoning — why bytes already on disk can be read while the
writer is still appending, what a pass over them yields, and how much damage a
pass survives — is this crate's. The **tailing** reasoning that sits on top of
it — discovery, the recording collection and its lifecycle, coverage, and what
to do when a pass faults — is [`tail`](../tail/CLAUDE.md), and the two only make
full sense read together.

Two properties of the format carry the whole design (`clip::index`):

1. **The MCAP writer is append-only while recording.** Bytes below the current
   end of file never change; the summary/footer is appended only at close. So
   everything behind the last complete record is immutable.
2. **Every record is length-prefixed** (1-byte opcode + u64le length). A record
   whose declared extent runs past the current file length is still being
   appended — the scan stops there and resumes on a later pass. An in-progress
   file is indistinguishable from a crash-truncated one, which MCAP readers are
   designed to tolerate.

Both properties are a producer requirement in disguise — append complete
records, never seek back to rewrite one (the producer-facing statement lives in
[ARCHITECTURE.md](../../ARCHITECTURE.md#tailing-a-live-mcap)). The Rust `mcap`
crate's chunked writer (`use_chunks(true)`, its default) is exactly the
violation: it leaves a `Chunk` header's length as the placeholder `u64::MAX`
until the chunk closes and the true length is back-patched in, so a `len >
clip::index::MAX_RECORD_LEN` framing fault (below) is guaranteed the moment the
scan meets one mid-write. Because `u64::MAX` is a value only a seek-back
writer's placeholder could ever produce — no valid record reaches it under
`MAX_RECORD_LEN` — the scan special-cases that one length to name the cause instead
of reading as bare corruption: "record at offset {offset} declares u64::MAX
bytes — an unpatched length from a seek-back (chunked) writer? such a recording
cannot be tailed until it is finalised". (The compliant chunked configuration —
buffered chunks via `disable_seeking(true)`, which appends each chunk as one
complete record — is demonstrated by
[`examples/chunked-mcap-writer`](../../examples/chunked-mcap-writer/README.md).)
This is also why the scan never waits
out an over-`MAX_RECORD_LEN` length instead of faulting on it: `u64::MAX` (or
anything else past the ceiling) has no path to becoming a valid record, so
retrying it as though it were a transient stall would only delay the identical
fault. Treating it as fatal once the caller's retry budget is exhausted is the
deliberate choice: the alternative — waiting it out — would silently degrade
every clip to a grace-timeout cut instead of failing fast.

A pass is `clip::index::scan_available(file, offset, file_len, seed)` — a free
function over bytes, holding nothing. The caller supplies a `ScanSeed` (the
still-open extent, the trigger tap, the trigger channels already known) and folds
the returned `ScanDelta` into that recording's `RecordingIndex` (`apply_delta`),
recording the new offset. Holding no state is what lets a caller scan with no
lock held; the lock choreography that buys from it is
[`tail`'s](../tail/CLAUDE.md#the-scan-pass).

Two artefacts of the delta serve the per-trigger handlers. A third, collection-wide
one — the [coverage watch](../tail/CLAUDE.md#the-coverage-watch) — is `tail`'s,
because only something holding every recording can say how far they provably
reach:

- **Extent index** — contiguous byte ranges (closed at 4 MiB) carrying the
  min/max `log_time` and `publish_time` of the messages they hold. Extraction
  reads only the extents whose span on the active [time source](../CLAUDE.md#time-source)
  overlaps its window; the overlap test is exact on both spans (real min/max,
  not a heuristic), so no in-window message can be missed on either clock
  domain. Retention ages on the `log` span alone.
- **Schema/channel registry** — owned copies of every `Schema`/`Channel`
  record, keyed by channel ID (unique within one continuous file). The MCAP
  spec puts a Schema before any Channel referencing it, so resolution always
  succeeds on conformant files; an inverted (invalid) file degrades the
  channel to schemaless rather than erroring. Chunked recordings carry these
  records *inside* chunks, so chunks are decompressed during the tail
  (zstd, lz4 and uncompressed chunks all work — mcap's default features);
  the default fastwrite profile is unchunked and skips that cost entirely.

Per top-level `Message` record only the 22-byte fixed header is read, in one
read (channel id, sequence, `log_time`, `publish_time`); bodies are first
touched at extraction. Both stamps are indexed so a window can live on either
[time source](../CLAUDE.md#time-source); the gap between the two — recorder queue backlog
plus producer clock skew — is also observable (logged per scan pass at debug). A
record too short
to hold the full 22-byte header is skipped like other localized damage: warned
and consumed via its intact framing, contributing to neither bounds nor
coverage. The same "decode only the timestamps" discipline as the rest of the
workspace, applied to file tailing.

The one exception is an **opt-in trigger tap**: the tailer takes it at
construction (`Tailer::with_trigger_tap`, wired only by the `mcap` interface)
and passes it into each pass through `ScanSeed`. With the tap on, the scan
additionally reads the *full body* of every message on the configured trigger
topic, lifting it as a raw `(message_encoding, body, log_time, publish_time)`
quadruple the MCAP interface decodes by `message_encoding` — the two record
stamps are what the mcap interface resolves its anchor from; the tap learns the
trigger topic's channel IDs from the same registry pass, so a body is read only
for a message it has already matched to that topic. A trigger is sent the moment
it is durable: a top-level record as soon as its framing is read, a
chunk-interior one only once the chunk's CRC has verified. With the tap disabled
— the `ros` interface, the default — the scan is byte-for-byte the
timestamp-only walk above: no message body is ever read.

**Damage in the recording is survivable up to the point of framing desync.**
The scan tolerates localized damage the same way extraction does (see "The copy
is direct (`clip::cut`)"): a chunk that fails to decompress, fails its CRC, or
carries an unsupported compression algorithm contributes nothing — its interior
is absorbed into a throwaway sub-delta merged into the live state only once the
chunk iterates cleanly, so a chunk whose CRC fails mid-iteration leaves no
registry entry and no time folded into coverage or extent bounds, and coverage
never claims data the cut would silently drop. An unparseable top-level
`Schema`/`Channel` record (spec-legal bytes the parser rejects, e.g. an
invalid-UTF-8 name) is warned and skipped. Both keep the framing intact — the
length prefix is self-consistent — so the record is consumed and the scan keeps
indexing the records behind it.

A **framing** fault has no resync point and so cannot be skipped: a record whose
declared length exceeds `MAX_RECORD_LEN`, or an IO error reading a record's
header or body. The scan stops at the faulted record, returns the delta it
accumulated before it, and reports the fault with that record's offset; the tail
applies that partial delta like any other pass's, so everything before the fault
stays plannable — clips cut from the pre-fault index still extract and announce.
**The resume invariant** binds the two halves: the tail must retry from exactly
the returned offset, never earlier, and never re-attaches or rescans from
scratch. The index is attached once per recording, so a retry that resumed
earlier would make the scan measure `record_end - open.offset` across bytes the
open extent already spans, and underflow. How many faults in a row a caller
tolerates before giving up is the caller's policy and not the scan's; the tail's
is its [scan-fault budget](../tail/CLAUDE.md#the-scan-fault-budget).

## The copy is direct (`clip::cut`)

Extraction reads each planned extent with `read_at` (no seek state shared with
the tail) and walks its records with **its own opcode + length framing** — the
same walk the tail performed to build the extent, so the boundaries are known
to tile. Owning the framing makes extraction damage-tolerant the way the MCAP
format is designed to be (length prefixes delimit every record; chunk CRCs
exist to detect and discard a damaged chunk — the format the official
`mcap recover` tool salvages by): a record whose body fails to parse, or a
message on a channel the recording never declared, is skipped with an error
log; a chunk that fails decompression, CRC, or interior parsing is dropped
whole — its messages are buffered and written only once the chunk iterates
cleanly (`mcap::read::ChunkReader` verifies the CRC at the end of iteration),
since a bad CRC cannot say which of the chunk's bytes are lying. The mcap
library readers are unsuitable for this walk: they halt at the first error,
and the `LinearReader::sans_magic` constructor additionally caps every
record — including chunk-interior records after decompression — at the slice
length, failing any conformant chunk whose contents out-compress it. A message
is in the window when its stamp on the window's [time source](../CLAUDE.md#time-source) —
its `log_time` or its `publish_time` — falls in the inclusive bounds; those that
are get written through with their **raw serialized bytes**
(`write_to_known_channel`); CDR bodies are never decoded.

The clip writer is built from `mcap::WriteOptions` carrying exactly one
deliberate setting. `.compression(..)` is it: the codec — a deliberate choice
(`--clip-compression`, default zstd; see the
[Configuration](../../docs/configuration.md)) — travelling as an
`Option<mcap::Compression>` (`None` = uncompressed) from `Config` into the
staging worker pool, which captures it for its lifetime (it is a property of the
output, not of any one window, and no window may disagree with it) and through
`stage_clip` into the copy.

Everything else is the mcap crate's. Chunking is on and the size a chunk is cut
at is `WriteOptions`' own default, which is what sets a clip's seek granularity
and the memory a reader spends decompressing one chunk. This crate holds no
opinion about that layout: a bump of the `mcap` dependency may move it, and no
test here objects (beads clipper-bf3). This paragraph is where that fact lives —
the other documents link here rather than restate it.

Output channels are registered from the registry per source channel ID and
cached; `mcap::Writer` deduplicates schemas/channels by content. The clip ends
with `Writer::finish()`, which writes the summary section, footer and closing
magic — every clip is a complete, standalone MCAP file
(`mcap::MessageStream` over a clip is the validity check the
unit tests use).

**Which topics a clip is cut from** is the other condition beside the window, and
`ClipWriter::route` is where it is asked — once per recording channel ID, in the
same step that registers the channel. That placement is the whole design: a topic
`clip::select::ChannelSelection` refuses is never registered, so the clip carries
no `Channel` record for it, no `Schema` record that only it referenced, no
message (a copy needs a registration), and no `channel.<id>.*` manifest keys (the
tally is filled by the step that writes a message through). `Route` keeps the two
ways out apart: `Excluded` is this clip's scope and costs nothing, `Unregistered`
is a recording that declares no channel for an ID and is counted with the rest of
the damage. The selection travels in the staging worker pool beside the
compression codec — both are properties of the output rather than of any one
window — which is why one configuration cuts the same channel set whether the
window was found by a scan (`clipper tail`) or by a finished recording's summary
(`clipper clip`); `whole.rs`'s
`one_selection_cuts_one_channel_set_from_either_index` pins that. Two rules are
not the configuration's to make: the announcement topic
(`clip::trigger::ANNOUNCE_TOPIC`) is refused unconditionally, and the trigger
topic is kept unless `exclude_trigger_topic` drops it. The keys and their
`ros2 bag record` counterparts are in the
[Configuration](../../docs/configuration.md#which-topics-a-clip-contains).

**Two-staged atomic publication.** The cut is two separate calls so the output
directory only ever holds finished clips, and so a window that straddled a
rollover can settle its segments' names once their count is known. `stage_clip`
assembles the clip in a `.capturing` subdirectory of `out_dir`,
`Writer::finish()`es it, and `sync_all`s the file; `publish_clip` then moves it
into `out_dir` under the desired name. A staging worker runs only the first
call, `cut_window` the second. (`extract_clip` composes both in one, for the
clip-assembly tests.) The capturing area is a *subdirectory* of the output directory
rather than a sibling so the two always share a filesystem — the move is a true
atomic link, never a cross-device copy. The move is `hard_link` + unlink of the
staged path, not `rename`: a duplicate trigger (same stamp and name) must not
clobber the earlier clip, and `rename` replaces an existing destination
silently, whereas `hard_link` is equally atomic but fails with `AlreadyExists`,
which the `_<n>`-suffix retry (`with_suffix_retry`, cap 1000) resolves against
the *desired* final name. The link is the commit point: once it succeeds the
output directory holds a complete clip (the staged file was already fsynced), so
the staged name is unlinked and `out_dir` itself is fsynced to make the new
directory entry crash-durable. A `StagedClip` is `#[must_use]` and its `Drop`
unlinks an unpublished staged file, so an early return or panic between the
stages — or a failed publish — strands nothing in `.capturing` and never
reaches `out_dir`. The capturing-dir name may carry its own `_<n>` suffix to
avoid colliding with a concurrent stage, independent of the final name a
duplicate trigger resolves to at publish — and whether the final name is allowed
to take a suffix at all is the cut's `clip::segment::Publication`, the recorder's
`Suffix` against `clipper clip`'s `Refuse`. The one leftover `Drop` cannot
reclaim is a crash *between* the publish link and the staged-file unlink, which
strands a stale link in `.capturing` (harmless — only `out_dir` is observed);
`clip::cut::reset_capturing_dir`, called once from `main` at startup, deletes
and recreates `.capturing` (and ensures `out_dir` exists) so that clutter never
outlives a single run. Failing that reset is fatal: a recorder that cannot prepare its
output directory must not start.

Extraction degrades over localized damage and aborts on anything else.
Skipped records and dropped chunks are counted in `ClipStats`
(`records_skipped` / `chunks_dropped`) and surfaced as a warning by the
trigger handler, so a degraded clip is announced but never silent. What stays
fatal — the recording truncated under the plan, extent framing that no longer
matches the tail's scan (the bytes changed since the scan, so there is no
boundary to resync at), and output IO errors — confines its cleanup to the
capturing directory, so the output directory never holds a footer-less file
that could be mistaken for a clip. A *deleted* recording is not an error — the
plan's `Arc<File>` keeps the inode readable, so extractions in flight across a
recorder restart still complete.

**Detection limit:** the leniency applies to damage loud enough to break
parsing or a CRC. The default fastwrite profile is unchunked and carries no
CRCs, so corruption inside a message *body* that leaves the framing and the
22-byte message header intact is invisible to every MCAP reader and is copied
into clips as-is — only a CDR decode downstream would notice.

## Every clip carries its manifest (`clip::manifest`)

A clip leaves the output directory and is read somewhere with neither the
recorder's logs nor the recording beside it, so it states what it is: one
`mcap::records::Metadata` record under the vendor-namespaced name
`momentedge.clip`, flat dotted keys and string values. The key groups and what a
consumer does with them are in the
[What a clip carries](../../docs/clip-manifest.md); this section is where they come
from. The name is namespaced because a recording `ros2 bag record` wrote carries
its *own* metadata record under the bare name `rosbag2` — a manifest under that
name would be found by whichever record a tool read first.

**Written between the last message and `finish`.** `copy_window` walks the
extents, then calls `ClipWriter::write_manifest`, then `Writer::finish()`. That
position is what earns the two properties a reader depends on: the mcap writer
appends a `MetadataIndex` to the summary and increments the statistics'
`metadata_count`, both of which are written by `finish`, so a reader finds the
record by name through the index rather than by walking the file (`mcap get
metadata --name momentedge.clip`, and `mcap info` reports `metadata: 1`).
Writing it earlier would mean guessing counters the copy has not finished
producing.

**Two halves meet at the writer.** The copy knows what it read and wrote; it
does not know who asked or what the planner offered. So:

- `clip::manifest::CutRequest` is the caller's half — the `Producer` (the binary
  and the subcommand: `clipper` / `tail`, from `Mode::producer()` in `main.rs`),
  the neutral `Trigger`, the resolved anchor, and the time source. It **derives**
  `start_ns`/`end_ns` from the anchor and the trigger's rolls and exposes them
  read-only, so the window the manifest states, the window the planner selects
  extents for, and the window each message's membership is tested against are
  one value that cannot drift. `tail::handler::handle_trigger` builds one per
  trigger, in an `Arc` shared by that window's segments.
- `clip::manifest::Planned` is the window-level pair every segment repeats:
  `files`, the number of source recordings `cut_window` was given plans for, and
  `coverage`, a `WindowCoverage` the *caller* supplies — `Short` when the
  coverage wait timed out. Both ride in the `StageJob` alongside the request.
- The copy's own half is per segment: `PlanSource::path` (which split this
  segment came from), the extents and bytes read, the messages copied, and the
  per-channel tallies.

**Per-channel accounting sits on the write.** `ClipWriter::count` is called
immediately after `write_to_known_channel` and does both the whole-clip counters
and the `BTreeMap<u16, ChannelTally>` the manifest's `channel.<id>.*` keys come
from, keyed by the **output** channel id so a reader can join the keys to the
clip's own `Channel` records. One call site is the point: everything that
decides which messages are copied — the window test and the channel selection
beside it — sits in front of that one call, so nothing can leave the accounting
behind, and a channel nothing was copied from simply never gets an entry, so it
has no keys rather than a row of zeroes.

**An empty clip says which kind of empty it is.** `source.files_planned`,
`clip.messages` and `clip.short` are what separate "nothing covered the window"
from "the window fell in a gap between splits" from "no message matched" —
three outcomes whose clips are otherwise byte-identical, and the reason
`WindowCoverage` is plumbed from the wait rather than logged and dropped. The
table is in the README; `segment::tests::an_empty_clip_says_which_kind_of_empty_it_is`
builds all three for real and asserts they differ.

`clip::manifest::read_manifest` is the other half of the contract: it reads the
record back through the summary's metadata index — a bounded seek and one
record, never a walk — and is what the tests assert through.

## Segment assembly and publication (`clip::segment`)

`cut_window` is the whole of what turns one window into published clips, and it
is the same code whichever index found the window: it takes a `&dyn
WindowPlanner`, asks it for the window's plans, stages one clip per plan through
the worker pool, drops the empty ones, names what is left, and publishes each
atomically. Its caller is either a [trigger
handler](../tail/CLAUDE.md#per-trigger-flow) over a growing recording or
`clipper clip` over a finished one; `cut_window` cannot tell them apart.

**Staging worker pool.** `clip::segment::spawn_stage_workers` starts
`extract_parallelism` threads (at least one) sharing one unbounded FIFO channel,
started once in `main` and handed to every handler. `cut_window` enqueues one
`StageJob` per `WindowPlan` — the plan snapshot, the `CutRequest` the window was
cut from, the `Planned` facts of that window, the base output path, and a
bounded(1) reply channel — and blocks
on each reply. The worker dequeues FIFO, runs `clip::cut::stage_clip` into
`.capturing/`, and replies a `StagedClip`. `cut_window` — not the worker —
publishes the staged segments once the window's segment count is known, so
naming (`_00`/`_01`) and atomic publication happen together.
`std::panic::catch_unwind` isolates a panicking stage per job; the
pool thread survives and continues processing. With the default
`extract_parallelism = 1` bulk copies serialize in submission order; postroll
and coverage waiting are always concurrent.

**What the pool captures and what the job carries** is the one distinction worth
holding onto. The compression codec and the `ChannelSelection` are captured for
the pool's lifetime: both are properties of the output, process-global, and no
window may disagree with either. The
`time_source` rides in each job, because it is a property of the *window* — the
same value has to choose the extents the planner returns *and* the stamp each
message's membership is tested on. A pool-level `time_source` would let those two
be answered from different clocks, and the result is not an error but a clip
silently holding the wrong messages;
`cut_window_selects_extents_and_messages_on_the_time_source` drives one
recording through both domains and pins them together.

**A clip already in `out_dir` is the other refusal** (`clip::segment::ClipExists`).
`clipper clip` cuts under `Publication::Refuse`, the recorder under
`Publication::Suffix`, and the two halves of that policy are the same collision
answered for different inputs: a live trigger colliding with an earlier clip's
name is a *second* trigger whose data no re-run can produce again, while a cut
from a finished recording is replayable, so the same name means the same bytes
and a second file is a duplicate. `cut_window` applies the policy as its first
step — before `plan_window`, before a `StageJob` is queued — so a refused window
publishes nothing, stages nothing, and is refused whole even where only one of
its segments' names is taken. What it checks is the base name **or any
`<stem>_<digits><ext>` beside it** (`existing_clip`), the one shape both a
segment name and a suffix-retry sibling take, because a window's segment count is
settled only once staging has run; the resulting over-refusal — a stray
`<stem>_00.mcap` blocks a single-segment window — is the cheap direction of that
trade. The lowest-sorting collision is the one named, so the message is the same
on every run. There is no override flag: an operator removes the clip or names
another `--out-dir`.

## Cutting from a finished recording (`clip::whole`, `clip::bag`, `clip::embedded`)

A recording with an end needs no tail: its own summary already holds what an
incremental scan would rebuild. `clip::whole::WholeFileIndex` serves that summary
through the same `WindowPlanner` the tailer implements, so the cut path above
runs unchanged. This is the half `clipper clip` drives — its command line, its
trigger sources and what it refuses on the way in are
[`clipper`'s](../clipper/CLAUDE.md#clipper-clip-one-window-one-finished-recording).

**A bag directory is one time-ordered collection** (`clip::bag`). What an
operator names — `clipper clip`'s `<recording>` positional — is one `.mcap` or
the directory a recording run left behind, and `bag::open`
answers which recordings that is and in what order — the one decision, made in
one place, that `whole.rs` and `embedded_triggers` both read. The recorder's
`metadata.yaml` states it where the recorder wrote one: its ordered
`relative_file_paths` is authoritative, since the order is a clip's *segment*
order and getting it wrong reorders a clip rather than merely renaming it.
Without that file the order is modification time, oldest first — and its absence
is itself the signal, the file being written at shutdown: a directory without it
was copied off a device while the recording was still growing. What the file
states is checked rather than trusted: a recording it names that is missing and
one present it never named are both logged, neither is fatal, and the
collection-wide `topics_with_message_count` is cross-checked against what the
splits' summaries add up to (`whole::CountDisagreement`, one per topic the two
differ on, reported and not refused).

Two things follow at the cut. **Each split satisfies the index contract on its
own**, so a directory holding one that does not is refused naming *that
recording* — the file an operator repairs or removes — and the last split of a
directory copied mid-recording is the usual offender. And the collection is one
`WindowPlanner`: `plan_window` returns one plan per contributing split, in
recording order, so `cut_window` publishes one `_NN` segment each. The number is
the position **after** the segments that copied nothing are dropped, not the
split's place in the directory — a window over three splits whose middle
recording contributes nothing yields `_00` and `_01`, where `_01` holds the third
recording's data.

**The index is the summary** (`clip::whole::WholeFileIndex`), per recording. A finalised MCAP
already carries a chunk index per chunk (byte range plus the `log_time` span
inside it), the resolved schema/channel registry, and the file's statistics —
everything the incremental scan rebuilds by walking the data section. `open`
drives the mcap crate's sans-io `SummaryReader`, which seeks to the footer and
reads the summary back, and nothing else: accepting a recording costs a seek and
one read whatever the file's size, and no chunk is decompressed until the copy
asks for one. Each chunk index becomes one `Extent` — `chunk_start_offset` is the
opcode and `chunk_length` counts the 9-byte record header in, which is exactly
the framed `Chunk` record `clip::cut` walks — and the filled `RecordingIndex` is
served through the same `WindowPlanner` the tailer implements, so the cut path
cannot tell the two apart.

Two consequences worth knowing:

- **Only chunk-indexed bytes are planned.** A message a chunked recording wrote
  outside a chunk is in no extent, which is why a summary that indexes no chunk
  is refused rather than cut into a silently empty clip.
- **The summary bounds no publish time.** A chunk index's span and the
  statistics' bounds are both `log_time`, so an extent built here carries the
  unbounded publish span: a `publish` window selects every chunk rather than
  dropping one the summary cannot vouch for, and the copy's per-message test
  decides membership. `clipper clip` itself never asks — it has no clock-domain
  flag and cuts on `log` (`CLIP_TIME_SOURCE`), because that is the clock a
  summary states and the only one a completeness claim over a finished recording
  can be made on. Passing `--time-source` is a parse error.

**Every input it cannot index is refused by name** (`clip::whole::OpenError`,
whose `Refused` arm carries the `IndexRefusal` taxonomy). One variant per fault
an operator repairs differently:

| Variant | The recording | Decided by |
|---|---|---|
| `NotMcap` | shorter than the 45-byte magic-footer-magic frame | the file length |
| `Unfinalised` | no closing magic — truncated or copied mid-write | the last eight bytes |
| `NoSummary` | a footer whose `summary_start` is zero | `SummaryReader::finish() == None` |
| `Empty` | holds no message | `stats.message_count == 0` |
| `Unchunked` | an unchunked writer profile | no `ChunkIndex` in the summary |
| `Unindexed` | message indexing disabled | every chunk index's `message_index_length == 0` |

The order is load-bearing at one place: the statistics are read before the chunk
indexes, because a chunked recording holding no message emits no chunk either, so
testing the chunk indexes first would call every empty recording unchunked. Each
variant's `Display` names the recording, the fault and the same three commands —
`mcap recover`, `mcap compress`, `mcap list chunks` (whose `message index length`
column is what `Unindexed` reads). `clipper clip` opens the index *before*
`reset_capturing_dir`, so a refusal creates neither `out_dir` nor the staging
directory inside it, and the input is opened read-only and left byte for byte and
mtime for mtime as it was found. clipper runs no repair; `mcap` does.

`OpenError`'s other three arms are not refusals: `Unreadable` (the file could not
be opened, stat'd, seeked or read), `Unparsable` (a footer that is not a footer
record, or a summary section that does not parse), and `Bag` (a `bag::BagError`:
a directory that cannot be listed, one holding no `*.mcap` at all — usually the
directory *above* the splits — or a `metadata.yaml` that is there and does not
parse). They are separate because they are separate things to do — point at the
directory the splits are in, fix the path, treat the file as corrupt, or run the
repair the refusal names.

**A finished recording can also state its own trigger list**
(`clip::embedded::read_triggers`), which is what
`clipper clip --trigger-source mcap` cuts from: the same undecoded
`TriggerRecord`s the tail's trigger tap emits, answered all at once instead of a
record at a time, because a recording with an end makes that possible.

**Reading that list costs the summary and the chunks it names.** A
finalised MCAP's chunk index carries, per chunk, the offset of a message index
for every channel with a message in it, so naming the trigger channel names its
chunks: `read_triggers` drives the mcap crate's sans-io `IndexedReader` with that
one topic and seeks to those chunks alone. A recording that never carried the
topic is answered from the summary with no chunk read at all — and that early
return is load-bearing, because an empty channel filter is no filter to the
indexed reader, which would then stream the whole file. Where the caller states
the trigger itself instead, no trigger is read and no chunk is decompressed
before the cut asks for one.

## The configuration file (`clip::config`)

`clip::config` owns the file half of a setting's resolution: reading the system
and per-run TOML files, and `Layered`, which resolves a key across them and
records what it refused. The `SETTINGS` key table carries the one thing no
argument definition can — whether a **per-run** file may set a key at all, since
a key naming the machine or the resources it may spend there is the system
file's alone, and a per-run file setting one is reported by name with the system
value left standing (`Layered::refusals`).

The crate stops there on purpose. How those two layers join the environment
variable and the flag above them is `clap`'s side of the seam and lives with the
binary that owns the parser — see
[the four configuration layers](../clipper/CLAUDE.md#the-four-configuration-layers).
The `[topics]` half of the same files becomes the `clip::select::ChannelSelection`
described under [the copy](#the-copy-is-direct-clipcut). The schema, the layering
rule and the per-key scope are documented in the
[Configuration](../../docs/configuration.md#the-configuration-file).
