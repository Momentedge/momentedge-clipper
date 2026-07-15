# Request to extract a clip from the continuously recording rosbag2 around a
# reference timestamp. Published on /events/momentedge/trigger.

string name                            # trigger identifier; part of the output filename
string description                     # optional free-form context, may be empty
builtin_interfaces/Time trigger_time   # publish-domain anchor; read ONLY under --interface ros --time-source publish, must be 0 in every other cell (which anchor on a transport stamp and reject a non-zero value)
uint64 preroll                         # nanoseconds before the anchor to keep
uint64 postroll                        # nanoseconds after the anchor to keep
