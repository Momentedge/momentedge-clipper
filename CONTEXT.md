# Clipper

Clipper cuts clips out of a continuous recording it does not own, on trigger
events. This glossary fixes the words for the things it reasons about — above
all the several distinct instants a message can be said to have happened at,
which are easy to conflate and expensive to conflate.

## Language

### Recordings

**Recording**:
One continuous MCAP file, written by a producer and tailed by clipper. Clipper
opens and reads recordings; it never writes one.
_Avoid_: bag, bagfile, split

**Producer**:
Whatever wrote a recording. `ros2 bag record` is one; a momentedge MCAP writer
is another. The producer — not clipper — decides what the timestamps in its
records mean.
_Avoid_: recorder, writer, source

**Extent**:
A contiguous byte range of a recording, with the smallest and largest
timestamps of the messages inside it. A window is served by reading only the
extents that overlap it.

**Coverage**:
How far a recording provably reaches on the active time source — the highest
timestamp the tail has seen. A trigger's clip is cut once coverage passes the
end of its window.
_Avoid_: watermark, progress, high-water

### The clip window

**Trigger**:
A request to cut one clip: a name, a description, a preroll, a postroll, and —
where the transport cannot supply one — a `trigger_time`.

**Window**:
The inclusive interval `[anchor − preroll, anchor + postroll]` a clip is cut
from. Expressed entirely in one time source.

**Anchor**:
The single instant a window is centred on. It is *resolved*, not carried: which
timestamp becomes the anchor depends on the interface and the time source.
_Avoid_: reference time, trigger time, centre, `trigger_time` (that is one
possible source of an anchor, not a synonym for it)

**Preroll** / **Postroll**:
How far before / after the anchor the window reaches.

**Clip**:
A standalone, complete MCAP file holding every message whose timestamp on the
active time source falls inside one trigger's window. Split across several
segments when the window spans a rollover.

**Recorded**:
The completion announcement for a finished clip: its name, its segment
filenames, and the window it was cut with.
_Avoid_: completion event, done message

### Time

**Time source**:
The clock domain a window lives in — `log` or `publish`. It governs the anchor,
which messages fall inside the window, which extents are read, and the coverage
a handler waits for. It governs nothing else.
_Avoid_: clock, time base, time domain, trigger time

**Log time**:
The MCAP record's `log_time`: when the producer received the message. One writer
stamps every recording, so log times are approximately non-decreasing in file
order — which is what makes coverage on this source a completeness proof.
_Avoid_: receive time, write time, record time, disk time

**Publish time**:
The MCAP record's `publish_time`: per the MCAP spec, "the time at which the
message was published". **Its meaning belongs to the producer.** `ros2 bag
record` fills it with the DDS source timestamp; a momentedge writer fills it
with the capture time. Clipper never interprets it and never guesses which it
is — it windows on whatever is there. Publish times may arrive out of order, so
coverage on this source is a liveness signal, not a completeness proof.
_Avoid_: send time, source timestamp, DDS timestamp

**Capture time**:
The instant the data in a message came into existence at its sensor. Clipper
cannot read it: it is not a field, and clipper never opens a message payload. It
reaches a recording only when a producer writes it into `publish_time`.
_Avoid_: sensor time, PTS, header stamp, event time

**`trigger_time`**:
The `Trigger` field a publisher fills with its own publish-domain timestamp,
standing in for the `publish_time` it cannot set on the wire. Read only where no
transport timestamp exists to resolve an anchor from.
_Avoid_: capture time, timestamp, stamp

### Interfaces

**Interface**:
The paired trigger input and completion output, chosen as one unit. The **ros**
interface subscribes for triggers and publishes `Recorded`. The **mcap**
interface lifts triggers out of the recording it already tails, and the clip's
appearance in the output directory is the only completion signal.
_Avoid_: mode, transport, backend
