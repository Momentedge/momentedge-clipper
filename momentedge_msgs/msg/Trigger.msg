# Request to extract a clip from the continuously recording rosbag2 around a
# reference timestamp. Published on /events/momentedge/trigger.

string name                            # trigger identifier; part of the output filename
string description                     # optional free-form context, may be empty
builtin_interfaces/Time trigger_time   # reference stamp the roll window is centred on
uint64 preroll                         # nanoseconds before trigger_time to keep
uint64 postroll                        # nanoseconds after trigger_time to keep
