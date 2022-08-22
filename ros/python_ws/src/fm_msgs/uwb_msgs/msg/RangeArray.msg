Header header
string tagid # A unique number (1,2,3...) per tag
geometry_msgs/Point tag_position # The (x,y,z) position (in meter) of the tag in the body frame
Range[] ranges # A list of ranges, one for each anchor a range is measured to
