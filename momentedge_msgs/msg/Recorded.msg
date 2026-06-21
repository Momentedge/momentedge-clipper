# Emitted on /events/momentedge/recorded once a triggered clip has been written
# to disk, echoing the originating trigger's identity.

string name                            # trigger identifier this clip was recorded for
string[] filenames                     # paths of the written clip mcap segments (one per source file the window spanned; usually one)
string description                     # description carried over from the trigger
builtin_interfaces/Time trigger_time   # reference stamp the clip was centred on
uint64 preroll                         # preroll window the clip was cut with, nanoseconds
