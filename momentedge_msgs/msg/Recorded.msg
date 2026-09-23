# Emitted on /events/momentedge/recorded once a triggered clip is complete on
# disk, echoing the originating trigger's identity.

string name                            # trigger identifier this clip was recorded for
string[] filenames                     # exactly one entry: the clip's directory, named by the clip id, holding one <id>_N.mcap per source recording the window spanned plus clip_metadata.yaml
string description                     # description carried over from the trigger
builtin_interfaces/Time trigger_time   # echo of the trigger's trigger_time (0 unless set under ros+publish); the clip's anchor, resolved, leads the directory name
uint64 preroll                         # preroll window the clip was cut with, nanoseconds
