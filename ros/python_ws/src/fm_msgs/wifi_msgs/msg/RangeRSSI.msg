time stamp
string ap_mac # A unique number (1,2,3...) per anchor
bool valid_ap_position # flag that determines if the ap position is known
geometry_msgs/Point ap_position # The (x,y,z) position (in meter) of the tag in the reference frame
bool valid_range # Flag that determines if the measured range is valid
float32 distance # Distance (in meter) between the tag and the anchor
DiagnosticsRSSI diagnostics # Signal specific data
