# Split-bag recording

Splitting caps each bag file at a size or a duration; rosbag2 rolls over to a
fresh `<bag>_<n>.mcap` when a cap is hit, so old files can be pruned to bound
disk use. This is the recording shape for plain capture with retention — **not**
for clipper (see the limitation below).

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

## Caveat: clipper does not follow splits

clipper tails **one** MCAP file. It discovers the newest `*.mcap` once and
follows that file until its path is deleted or recreated (a recorder restart
wiping the directory). A split keeps every file in place and adds a new
`_<n+1>.mcap` alongside, so clipper stays pinned to the file it opened and never
advances to later splits. Point clipper at a splitting recorder and it cuts
clips only from the split it started on.

So split recording is for capture-and-retain workflows where clips are not cut
live. For clipper, use [`../continuous`](../continuous/README.md) (one growing
file) instead. Teaching the tail to follow splits is tracked in beads
(`clipper-wjm`).

## Retention: prune old splits

Splitting only helps if old files are removed. A periodic delete by age keeps
disk bounded:

```bash
# delete split files older than 24h (1440 min); run on a timer
find ./record -mindepth 1 -type f -mmin +1440 -delete
```

Run it from cron (or any scheduler) beside the recorder and clipper.
