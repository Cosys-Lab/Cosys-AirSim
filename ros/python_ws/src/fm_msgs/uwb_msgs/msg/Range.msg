time stamp
string anchorid # A unique number (1,2,3...) per anchor
geometry_msgs/Point anchor_position # The (x,y,z) position (in meter) of the tag in the reference frame
bool valid_range # Flag that determines if the measured range is valid
float32 distance # Distance (in meter) between the tag and the anchor
Diagnostics diagnostics # Signal specific data
