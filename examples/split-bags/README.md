# Split-bag recording

Splitting caps each bag file at a size or a duration; rosbag2 rolls over to a
fresh `<bag>_<n>.mcap` when a cap is hit, so old files can be pruned to bound
disk use. clipper follows the rollovers (see below), so this pairs with live
clipping as well as plain capture-and-retain — within the split-boundary
trade-offs the two caveats below describe.

```bash
ros2 bag record --all \
  --storage mcap \
  --storage-preset-profile zstd_fast \
  --max-bag-size 0 \
  --max-bag-duration 300 \
  --output ./record
```

## Split controls

| flag                  | unit    | `0` (default) means |
|-----------------------|---------|---------------------|
| `--max-bag-size` / `-b` | bytes   | no size split       |
| `--max-bag-duration` / `-d` | seconds | no duration split   |

With both set, the bag splits at whichever cap is reached first. rosbag2
publishes a `rosbag2_interfaces/WriteSplitEvent` on `/events/write_split` at
each boundary.

## Caveat: messages are dropped during a split

While rosbag2 closes the current file and opens the next, the writer is
blocked; messages arriving in that window can be lost, the more so at high data
rates with small subscription queues. The recording therefore has a small gap
at every split boundary. This is a known rosbag2 limitation, not a setting to
tune away:

- [ros2/rosbag2#640 — Bag splitting blocks the writer which can cause message loss](https://github.com/ros2/rosbag2/issues/640)
- [ros2/rosbag2#2108 — Bag recorder sometimes loses/skips data on bag split](https://github.com/ros2/rosbag2/issues/2108)

A larger `--max-cache-size` (and a non-blocking storage profile) reduces but
does not eliminate the loss.

## clipper follows the newest split

clipper tails the newest `*.mcap` under its record directory, chosen by ctime,
and keeps re-checking as it tails. When rosbag2 rolls over to `<bag>_<n+1>.mcap`,
clipper finishes the file it is on, then advances to the new split and tails it
from the start. No `WriteSplitEvent` subscription is involved — discovery is by
ctime, the same mechanism that follows a recorder restart, so a split is just
another way the newest file changes.

**A clip is cut only from the split clipper is currently on.** Advancing to a
new split resets clipper's in-memory index, so data in a split clipper has
already moved past is not pulled into a later clip — the same cross-file
non-recovery rule it applies across a recorder restart. A trigger whose pre-roll
reaches back across a split boundary therefore captures only the portion in the
current split. Where a window must never straddle a boundary, set the split
duration/size well above the pre + post-roll, or use
[`../continuous`](../continuous/README.md) (one growing file), which has no
boundaries at all.

## Retention: prune old splits

Splitting only helps if old files are removed. A periodic delete by age keeps
disk bounded:

```bash
# delete split files older than 24h (1440 min); run on a timer
find ./record -mindepth 1 -type f -mmin +1440 -delete
```

Run it from cron (or any scheduler) beside the recorder and clipper, or as a
standalone loop with no scheduler:

```bash
# same sweep every 30s, printing each file it removes
while true; do find ./record -mindepth 1 -type f -mmin +1440 -print -delete; sleep 30; done
```
