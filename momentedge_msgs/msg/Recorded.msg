# Emitted on /events/momentedge/recorded once a triggered clip has been written
# to disk, echoing the originating trigger's identity.

string name                            # trigger identifier this clip was recorded for
string[] filenames                     # paths of the written clip mcap segments (one per source file the window spanned; usually one)
string description                     # description carried over from the trigger
builtin_interfaces/Time trigger_time   # echo of the trigger's trigger_time (0 unless set under ros+publish); the clip's anchor prefixes each filename
uint64 preroll                         # preroll window the clip was cut with, nanoseconds
