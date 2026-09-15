# clip

What every consumer of a recording shares: the MCAP format layer and its
recording index, the copy that cuts a window out of one, the neutral trigger and
completion contract, the segment assembly that turns one window into a durable
clip, the id each clip is named by, the on-disk layout a clip is, the document
each clip carries, the channel selection that says which topics it holds, and the
layered configuration file behind both.

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
  (zstd, lz4 and uncompressed chunks all work — mcap's default features); an
  unchunked recording skips that cost entirely. `fastwrite` is the one
  `ros2 bag record` preset that turns chunking off — rosbag2's own default,
  `none`, is chunked, as are both `zstd_*` presets.

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
— the `ros` interface, which the device build takes by default — the scan is
byte-for-byte the timestamp-only walk above: no message body is ever read.

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
test here objects (beads clipper-bf3). This paragraph is where that fact lives
for the crates; another `CLAUDE.md` links here rather than restate it. A
human-facing page must not — what a consumer of a clip needs to know about its
chunk layout belongs in [What a clip carries](../../docs/clip-manifest.md), and a
page that could only be completed by linking here is filed wrong.

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
message (a copy needs a registration), and no per-channel tally (the
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

**The copy writes one file and names nothing.** `stage_clip` creates a file with
`create_new` inside the clip directory its caller has already claimed,
`copy_window`s into it, `Writer::finish()`es it and `sync_all`s it, and hands
back a `StagedClip` — a complete MCAP under a `.part` staging name. It cannot
name the file it wrote, because a file's number is its position among the
recordings that *contributed*, and the empty ones are dropped only once every
copy has run. `cut_window` renames each survivor into place afterwards
([segment assembly](#segment-assembly-and-the-clip-directory-clipsegment)).
`create_new` rather than a collision retry is the point: the directory was
claimed by one atomic `mkdir` and nothing else may write into it, so a taken name
is a bug rather than a race, and failing loudly beats two writers interleaving
bytes into one file.

**A `StagedClip` is either placed or discarded, never merely forgotten.**
`place` renames a file that contributed to its `<id>_N.mcap`; `discard` removes
one that copied nothing in a window that produced data elsewhere. Both consume
the value, so the pair is exhaustive — and a dropped file has to be *removed*
rather than dropped from a `Vec`, because it is already on disk under its
staging name and the directory this cut is about to complete must hold the files
its document names, its document, and nothing else. A staging name left beside
them would be noise in the one directory an operator points a sync tool at, and
an entry no consumer has a rule for.

**A clip directory that is never completed needs no cleanup at all**, which is
why `discard` is best effort and returns no `Result`. [A cut that fails removes
the whole directory](#what-a-clip-is-on-disk-cliplayout) — one rule covering a
partially written clip, a panicking copy and a crash — so a staging file
outlives its copy only inside a directory that *does* get completed, and there
the worst a failed removal costs is a warning and one stray name beside a
correct clip. Failing the cut over it would trade a good clip for a tidy
directory. The only local tidying the copy itself does is removing its own
half-written file when `copy_window` fails, so that a `StagedClip` that exists
always names a complete MCAP and `is_empty()` can be read off it.

Extraction degrades over localized damage and aborts on anything else.
Skipped records and dropped chunks are counted in `ClipStats`
(`records_skipped` / `chunks_dropped`) and surfaced as a warning by the
trigger handler, so a degraded clip is announced but never silent. What stays
fatal — the recording truncated under the plan, extent framing that no longer
matches the tail's scan (the bytes changed since the scan, so there is no
boundary to resync at), and output IO errors — takes the whole clip directory
with it, so the output directory never holds a footer-less file that could be
mistaken for a clip. A *deleted* recording is not an error — the
plan's `Arc<File>` keeps the inode readable, so extractions in flight across a
recorder restart still complete.

**A framing abort costs the extent, not the byte.** The walk enters at the
extent's own offset and has no resync point, so one broken record header takes
with it every record behind it in that extent — and an extent closes at
`EXTENT_CAP_BYTES` (4 MiB), which at a modest data rate is minutes of
recording. Every window whose plan includes that extent is refused, windows over
data written long after the damage included, for as long as the recording is
tailed. It is the tail, not the cut, that fails fast, so the recorder stays up:
the symptom is clips that stop arriving, not a process that stops.
`corrupt_tail_framing_damage_live` holds that shape down.

That refusal is a **type**, `clip::cut::FramingDesync`, and not a message,
because the repetition has to be answered by whoever is cutting repeatedly: it
carries the `recording()` whose bytes changed and the `extent_offset()` the walk
entered at — the fault's identity and its blast radius. What a caller does with
that is the caller's, and the two callers differ. `clipper clip` cuts one window
once and only prints it. The recorder meets the same bytes on every trigger, so
it keeps a per-recording tally and announces the first refusal in full: the
[cut-fault tally](../tail/CLAUDE.md#the-cut-fault-tally), which also says why
that is counted rather than made fatal. Typing it is what keeps file damage
apart from a full disk or an output failure — different faults, different
remedies, and only this one permanent for the recording it names.

**Detection limit:** the leniency applies to damage loud enough to break
parsing or a CRC. `fastwrite` disables both chunking and CRCs, so a recording
written under it — the preset
[`examples/continuous`](../../examples/continuous/README.md) selects for minimal
tail latency, and the one the live e2e suite records its tail scenarios with —
offers a reader nothing to check a message body against: corruption inside a
*body* that leaves the framing and the 22-byte message header intact is
invisible to every MCAP reader and is copied into clips as-is — only a CDR
decode downstream would notice.

## A clip is named by its id (`clip::id`)

`ClipId::of(&CutRequest)` is what a clip is called: `<anchor_ns>_<hash>`, the
resolved anchor followed by the first 8 bytes of SHA-256 over a canonical
encoding of the six fields a request *is* — the anchor, `name`, `description`,
`preroll`, `postroll`, and the time source — rendered as four lower-case hex
groups of four. `layout::ClipDir` names the clip's directory and every file in
it from that id, and `manifest`'s `clip.id` states it, all from that one
function, so the paths, the document and the record cannot disagree.

**The encoding is a published contract**, stated with a worked vector in
[What a clip carries](../../docs/clip-manifest.md#the-clip-id) and pinned by
`the_published_vector_encodes_and_hashes_to_its_documented_id`. An id quoted in
an incident report has to stay valid across versions, so that test failing is
the point: changing the field order, a separator, the tag or how a value is
rendered is a new id space and must be a new `ENCODING_TAG`, never the same bytes
meaning something else. The two free-text fields are **byte-length-prefixed**
rather than delimited — the newline after each is decoration — so no pair of
distinct triggers can encode alike however their text is split.

**Every line of the published vector carries a different value**, and that is a
property of the vector rather than a coincidence of the example. Two lines
holding one number — a preroll and a postroll of five seconds each — encode the
same bytes when they are exchanged, so a vector built from them would go on
matching an encoding whose fields had been reordered while every id a real
trigger produces moved. `every_field_of_the_request_moves_the_id` does not
cover the gap, since a swap moves the id too. The vector test asserts the
distinctness itself, so an edit that reintroduces the blind spot fails there.

**What the id buys is that no trigger text reaches a path.** A `name` holding
`/`, `..`, a leading dot, unicode or nothing at all is hashed like any other, so
there is no sanitizer and nothing for the recorder's admission gate to refuse on
those grounds; what is left of `validate_name` is the recorder's own length bound
on a free-text message field. The cost is that a clip's name says nothing about
what it is about, which is what `clip.id` and `trigger.name` in the document are
for — and why the e2e suite finds a clip by reading documents rather than by
matching a path.

## What a clip is on disk (`clip::layout`)

A clip is a **directory** named by its [id](#a-clip-is-named-by-its-id-clipid),
holding one `<id>_N.mcap` per contributing source recording and the
`clip_metadata.yaml` that says it is complete. The shape a consumer sees is in
[What a clip carries](../../docs/clip-manifest.md); this section is the three
filesystem operations it is built from, and why each is one operation rather than
a protocol.

**The directory is the claim.** `ClipDir::claim` is one `mkdir` with no parents,
which the kernel makes atomic: exactly one caller creates a given path and every
other sees `AlreadyExists`. That single fact answers a repeated trigger, a
recorder restart meeting its own earlier clips, two concurrent windows that
resolve to one id, and two processes writing into one output directory — with no
lock file, no staging area, no startup wipe and no same-device check. It returns
a `Claim`, two variants rather than an `Option`, because a caller has to say
something about each and a taken id must be impossible to mistake for a free one.
No parents on purpose, and the reason is the exclusion rather than the tree:
`create_dir_all` answers `Ok` for a directory that is already there, so it would
report a taken id as a won race and the claim would exclude nothing.
`create_dir` is the one call whose *failure* is the answer. The tree above it is
`prepare_out_dir`'s, which `cut_window` runs before every claim, so the parent is
always there by the time the claim runs.

**The document is the completion signal**, and two things make that true:
how the name appears, and the order of the fsyncs around it.

The document is written and fsynced under a `.part` staging name and then
*renamed* onto `clip_metadata.yaml`, so a rename is the only operation that ever
touches that name. A `File::create` on the final name would publish it holding
zero bytes and fill it afterwards — and on ext4 with delayed allocation, a
create-then-write with no rename is the textbook zero-length-file-after-crash
case, which under a presence rule is a clip that reads as complete and is empty.
The rename closes that window: the name is absent or it holds the whole
document.

Every MCAP file was fsynced by its own copy, so `ClipDir::complete` fsyncs the
staged document, renames it, then fsyncs the clip directory (making every name in
it durable), then fsyncs the output directory (making the clip directory's own
entry durable). A crash can therefore lose a clip but can never leave one that
carries the document and is missing a file the document names. The last fsync is
what lets the recorder announce a clip as on disk: `Recorded` goes out after
`cut_window` returns, and by then the clip survives power loss.
`layout::tests::the_completion_signal_is_never_seen_half_written` pins the
mechanism — it obstructs the staging name and shows the final name untouched —
and says in its own doc what that does and does not prove.

**A failed cut takes its directory with it.** `ClipDir::discard` consumes the
claim, removes the tree and hands back the error that got there — and a removal
that itself fails is folded in as context naming the directory, because that
directory is the one thing an operator has to act on (every later window with
that id is skipped until it is gone). Being *consumed* is what makes this
unforgettable rather than a rule: the claim is either `complete`d or `discard`ed,
and `cut_window`'s match over the two is where both arms are visible at once.

**What a crash leaves is deliberate.** A directory with no `clip_metadata.yaml`
is crash residue by construction, since every ordinary failure removes its own.
Consumers treat it as incomplete, and a later trigger with that id *skips* it
rather than repairing or overwriting it — the residue is the evidence that
something died, and clipper does not destroy evidence to tidy up.

**`.part` is the one name a consumer never sees**, and two things are staged
under it for the same reason: neither may be seen under its final name before it
is whole. An MCAP file's number is its position after the empty files are
dropped, which is known only once every copy has run, so each copy writes under a
name derived from its plan's position (`ClipDir::staging`) and is renamed into
place by `ClipDir::place`. The document goes through `document_staging` because
its name *is* the completion signal. The two staging spaces cannot collide: an
MCAP file's is `<id>_<n>` and an id is digits, hex and `-`, so it carries no `.`
before the extension, while the document's is `clip_metadata.yaml` and does —
and every one of these names is spelled in this module, beside the name it
becomes. The window in which a `.part` exists is a window in which the directory
has no document and is therefore incomplete anyway.

## Every clip carries its document (`clip::manifest`)

A clip leaves the output directory and is read somewhere with neither the
recorder's logs nor the recording beside it, so it states what it is. Two
artefacts do that, and the split between them is the design:

- **One `ClipMetadata` per clip**, serialized to `clip_metadata.yaml` by
  `serde_norway` — the workspace's existing YAML crate, whose only other use is
  *reading* rosbag2's own `metadata.yaml` in `clip::bag`. It is a typed struct
  with nested groups (`clip`, `producer`, `trigger`, `window`, `sources`) rather
  than a hand-built map, so the document and the type cannot drift, and it derives
  `Deserialize` as well so `layout::read_document` reads one back for a consumer
  and for every test that asserts on a clip.
- **One key inside every MCAP file** (`manifest::id_record`): an
  `mcap::records::Metadata` record under the vendor-namespaced name
  `momentedge.clip` carrying `clip.id` and nothing else. The name is namespaced
  because a recording `ros2 bag record` wrote carries its *own* metadata record
  under the bare name `rosbag2`, and a record under that name would be found by
  whichever a tool read first.

**Why the file carries one key and not the account.** A clip's facts are stated
once, where one copy cannot disagree with another after a partial rewrite; what a
*file* has to answer on its own is only "which clip is this?", so that one carried
away from its directory can still be grouped. Everything else — who cut it, what
asked for the window, which recordings it came from — is in the document beside
it.

**`clip.id` is derived, never carried:** it is
[`ClipId::of`](#a-clip-is-named-by-its-id-clipid) over the same `CutRequest` the
`trigger` and `window` groups are written from, and over the same request
`layout` names the directory and its files from. One computation over one value.

**Written between the last message and `finish`.** `copy_window` walks the
extents, then calls `ClipWriter::write_id_record`, then `Writer::finish()`. That
position is what earns the two properties a reader depends on: the mcap writer
appends a `MetadataIndex` to the summary and increments the statistics'
`metadata_count`, both of which are written by `finish`, so a reader finds the
record by name through the index rather than by walking the file (`mcap get
metadata --name momentedge.clip`, and `mcap info` reports `metadata: 1`).

**Two halves meet at `ClipMetadata::of`**, which is pure. The copy knows what it
read and wrote; it does not know who asked or what the planner offered. So:

- `clip::manifest::CutRequest` is the caller's half — the `Producer` (the binary
  and the subcommand: `clipper` / `tail`, from `Mode::producer()` in `main.rs`),
  the neutral `Trigger`, the resolved anchor, and the time source. It **derives**
  `start_ns`/`end_ns` from the anchor and the trigger's rolls and exposes them
  read-only, so the window the document states, the window the planner selects
  extents for, and the window each message's membership is tested against are
  one value that cannot drift. `tail::handler::handle_trigger` builds one per
  trigger, in an `Arc` shared by that window's files.
- `clip::manifest::Planned` is the window-level pair: `files`, the number of
  source recordings `cut_window` was given plans for, and `coverage`, a
  `WindowCoverage` the *caller* supplies — `Short` when the coverage wait timed
  out. `cut_window` holds it, and it rides in no `StageJob`, because the copy
  has nothing to write it into.
- The copy's own half is one `cut::ClipStats` per file: `PlanSource::path`
  (which recording this file came from), the extents and bytes read, the messages
  copied, and the per-channel tallies. Those become one `sources` entry each.

**Per-channel accounting sits on the write.** `ClipWriter::count` is called
immediately after `write_to_known_channel` and does both the whole-file counters
and the `BTreeMap<u16, ChannelTally>` that becomes that file's `channels` map,
keyed by the **output** channel id so a reader can join it to the clip's own
`Channel` records — *of that file*, since each MCAP numbers its channels from
scratch, which is why the tallies are per `sources` entry and not clip-wide. One
call site is the point: everything that decides which messages are copied — the
window test and the channel selection beside it — sits in front of that one call,
so nothing can leave the accounting behind, and a channel nothing was copied from
simply never gets an entry, so it has no keys rather than a row of zeroes.

**An empty clip says which kind of empty it is.** `window.files_planned`,
`clip.messages` and `clip.short` are what separate "nothing covered the window"
from "the window fell in a gap between splits" from "no message matched" —
three outcomes whose clips are otherwise byte-identical, and the reason
`WindowCoverage` is plumbed from the wait rather than logged and dropped. The
table is in [What a clip carries](../../docs/clip-manifest.md#an-empty-clip-still-explains-itself);
`segment::tests::an_empty_clip_says_which_kind_of_empty_it_is` builds all three
for real and asserts they differ.

Two readers close the contract. `layout::read_document` parses a clip
directory's document and is what most tests assert through — and what a consumer
filters on, since an error from it *is* "not a complete clip".
`manifest::read_manifest` reads a file's own record back through the summary's
metadata index — a bounded seek and one record, never a walk — and returns the
whole map rather than just the id, so a reader can tell "clipper wrote this and
said nothing else" from a record under the same name written by something else.

**Three literal names are the contract, and each is pinned by a literal.**
`clip_metadata.yaml` (`DOCUMENT_FILE`), `momentedge.clip` (`MANIFEST_NAME`) and
`clip.id` (`CLIP_ID_KEY`) are what an upload pipeline filters on and what
`mcap get metadata --name` is invoked with. Every other assertion in the
workspace reaches them through the constants, so renaming a constant's *value*
would leave the whole suite green while breaking every consumer — which is why
`the_completion_signal_is_the_name_the_contract_publishes` and
`a_files_record_is_named_and_keyed_as_the_contract_publishes` spell the strings
out. Same reasoning as `cut`'s `ROSBAG2_METADATA_NAME`: a test that compares a
constant to itself asserts nothing.

**The clip's file is `DOCUMENT_FILE`; the producer's is `bag::METADATA_FILE`.**
Two different files written by two different writers, on either side of the seam
this crate is organised around, so they carry different words and the crate's
readers do too — `layout::document_path`, `layout::document_staging` and
`layout::read_document` against `bag::read_metadata`. The glossary
([CONTEXT.md](../../CONTEXT.md)) settles which word names which file: **document**
is the clip's `clip_metadata.yaml`, and **metadata file** already names the
`metadata.yaml` a producer writes beside its splits. Naming both `METADATA_FILE`
is legal — module scoping keeps it compiling — and is exactly the trap: a bare
`METADATA_FILE` in either module reads right and can be wrong, and the two cannot
be imported into one file at all.

## Segment assembly and the clip directory (`clip::segment`)

`cut_window` is the whole of what turns one window into a durable clip, and it is
the same code whichever index found the window: it takes a `&dyn WindowPlanner`,
claims the clip's directory under the output directory it was given, asks the
planner for the window's plans, copies one file per plan through the worker pool,
drops the empty ones, numbers what is left, and writes the document last. Its
caller is either a [trigger handler](../tail/CLAUDE.md#per-trigger-flow) over a
growing recording or `clipper clip` over a finished one; `cut_window` cannot tell
them apart and has no reason to.

**It is the one entry point that decides where a clip goes**, which is why a
caller hands it an `out_dir` rather than a path. Everything under that directory
— the name, the file numbering, the document — is
[`clip::layout`](#what-a-clip-is-on-disk-cliplayout)'s, so neither binary formats
a clip path and the layout moves in one edit.

**Three outcomes, and the type says which.** `cut_window` returns
`anyhow::Result<CutOutcome>`, and `CutOutcome` is `Cut(Clip)` or
`Skipped(PathBuf)`: a clip was written, the id was already taken, or the cut
failed. A skip is neither of the other two — nothing was written and nothing went
wrong — and making it a variant rather than an empty `Clip` is what stops a
caller announcing a clip it did not cut. `Clip` carries the `dir` (what a
completion names and a consumer syncs) and one `cut::ClipStats` per file (what a
caller logs).

**The skip's warning is the cut's, not the caller's**, so both subcommands say
the same thing about a taken id and neither can forget to. The claim runs before
`plan_window`, so a skipped window plans nothing, copies nothing and costs a busy
device nothing; `segment::tests` hands the second cut an `Unplannable` planner
that panics if it is asked, which is how that ordering is pinned.

**A cut that fails after the claim removes its directory**, and the two ways out
of a claim — `complete` or `discard` — are the two arms of one match in
`cut_window`. `fill` is the separate function everything between them lives in,
so it may `?` anywhere without leaving a rule to remember at each site.

**Staging worker pool.** `clip::segment::spawn_stage_workers` starts
`extract_parallelism` threads (at least one) sharing one unbounded FIFO channel,
started once in `main` and handed to every handler. `cut_window` enqueues one
`StageJob` per `WindowPlan` — the plan snapshot, the `CutRequest` the window was
cut from, the path inside the claimed directory to write at, and a bounded(1)
reply channel — and blocks on each reply. The worker dequeues FIFO, runs
`clip::cut::stage_clip`, and replies a `StagedClip`. `cut_window` — not the
worker — names the finished files once the clip's file count is known.
`std::panic::catch_unwind` isolates a panicking copy per job; the pool thread
survives and continues processing. With the default `extract_parallelism = 1`
bulk copies serialize in submission order; postroll and coverage waiting are
always concurrent.

**What the pool captures and what the job carries** is the one distinction worth
holding onto. The compression codec and the `ChannelSelection` are captured for
the pool's lifetime: both are properties of the output, process-global, and no
window may disagree with either. The `time_source` rides in each job, because it
is a property of the *window* — the same value has to choose the extents the
planner returns *and* the stamp each message's membership is tested on. A
pool-level `time_source` would let those two be answered from different clocks,
and the result is not an error but a clip silently holding the wrong messages;
`cut_window_selects_extents_and_messages_on_the_time_source` drives one recording
through both domains and pins them together.

**Numbering is settled at close and is internal to the write.** Empty files are
dropped when the window produced real data elsewhere, and one is kept when they
are all empty, so `<id>_N.mcap` counts the files that *contributed*, not the
source recordings that were planned. `window.files_planned` in the document is
what states the latter, which is how a consumer tells "three splits, one of them
silent" from "two splits".

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
`relative_file_paths` is authoritative, since the order is the order a clip's
files are numbered in, and getting it wrong reorders a clip rather than merely
renaming it.
Without that file the order is modification time, oldest first — and its absence
is itself the signal, the file being written at shutdown: a directory without it
was copied off a device while the recording was still growing.

That fallback is why `bag::METADATA_FILE` is pinned by a literal
(`the_sidecar_carries_the_name_rosbag2_writes`) even though the name is
rosbag2's rather than clipper's — a different argument from the three names
[the clip contract publishes](#every-clip-carries-its-document-clipmanifest).
`open` and the fixture writers both reach the name through the constant, so a
changed value refuses nothing: `bag::read_metadata` answers `None` for a directory
with no sidecar, the order silently becomes modification time, and a straddling
clip's `_0` and `_1` swap. The whole suite stays green for a wrong clip. Since
the name is not ours to choose, a change to it is either a rosbag2 change being
tracked on purpose or a typo, and the literal is what tells the two apart.
`MCAP_EXT` needs no such test — `bag`'s fixtures write literal `bag_0.mcap`
files and assert they are listed. What the file
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
recording order, so the clip holds one `<id>_N.mcap` each. The number is the
position **after** the files that copied nothing are dropped, not the split's
place in the directory — a window over three splits whose middle recording
contributes nothing yields `_0` and `_1`, where `_1` holds the third recording's
data, and the document's `window.files_planned` is `3`.

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
| `ChunkOutOfRange` | a summary that does not describe its own bytes | a chunk index's `offset + len` past the file length |

The order is load-bearing at one place: the statistics are read before the chunk
indexes, because a chunked recording holding no message emits no chunk either, so
testing the chunk indexes first would call every empty recording unchunked. Each
variant's `Display` names the recording, the fault and the same three commands —
`mcap recover`, `mcap compress`, `mcap list chunks` (whose `message index length`
column is what `Unindexed` reads). `clipper clip` opens the index *before* it
prepares the output directory, so a refusal does not even create `out_dir`, and
the input is opened read-only and left byte for byte and mtime for mtime as it
was found. clipper runs no repair; `mcap` does.

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

**The table is keyed by subcommand.** Its rows are `(Mode, key, Scope)`, and
`scope_of`, `setting_keys` and `Layered::load` take the `Mode` — the subcommand
about to run — because the scope of a key is a fact about the subcommand that
reads it, not about the process: `record_dir` names the machine `clipper tail`
records on, and a subcommand without that flag has no opinion about it. A key
both subcommands carry (`out_dir`) is written out once per mode, and that
duplication is the point: two rows are what let one name answer differently
depending on which subcommand is asking. No shipped key answers differently
today — `out_dir` is `Scope::Any` under each — so the capability is stated by
`one_name_can_carry_a_different_scope_under_each_mode` over a table written for
it (`scope_in` takes the table as an argument for exactly that), not over
`SETTINGS`. The shape stays because the scope of a key *is* a fact about the
subcommand reading it, whether or not two rows currently disagree.

**Some flags are deliberately not keys, for two distinct reasons.** `--config`,
`--system-config` and `--print-config` are *about* the configuration rather than
in it, so no file can name another file. `--trigger-source` is absent because it
says how *this process was launched* rather than what the machine is configured
with: a device sets it once in the unit file that already carries the rest of
the invocation, so it lives in the flag and in `MOMENTEDGE_TRIGGER_SOURCE` and
nowhere else, and a file naming `trigger_source` fails the run the way any
unknown key does. The recorder's `every_mode_argument_is_a_settings_key_and_back`
carries the same list with the same two reasons spelled out, so an argument
added without a key has to say which it is.

**The mode governs the scope and nothing else.** Whether a name is a key at all
is `is_setting_key`, read across the modes, so one file serves every subcommand:
a device's system file describing the recorder is read unchanged by a
`clipper clip` run on the same machine. A key the running subcommand does not
have is *inert* — `scope_of` answers `None`, nothing refuses it, and it matches
no argument — while a key **no** subcommand has is a misspelling and fails the
run, naming the key and where each key belongs. Keeping those two questions
apart is what lets the scope line move per subcommand without the key set moving
with it.

The crate stops there on purpose. How those two layers join the environment
variable and the flag above them is `clap`'s side of the seam and lives with the
binary that owns the parser — see
[the four configuration layers](../clipper/CLAUDE.md#the-four-configuration-layers).
The `[topics]` half of the same files becomes the `clip::select::ChannelSelection`
described under [the copy](#the-copy-is-direct-clipcut). The schema, the layering
rule and the per-key scope are documented in the
[Configuration](../../docs/configuration.md#the-configuration-file).
