# Emitted on /events/edgestream/recorded once a triggered clip has been written
# to disk, echoing the originating trigger's identity.

string name                            # trigger identifier this clip was recorded for
string filename                        # path of the written clip mcap
string description                     # description carried over from the trigger
builtin_interfaces/Time trigger_time   # reference stamp the clip was centred on
uint64 preroll                         # preroll window the clip was cut with, nanoseconds
