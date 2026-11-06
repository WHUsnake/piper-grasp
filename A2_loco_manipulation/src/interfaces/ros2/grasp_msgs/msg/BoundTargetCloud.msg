# Source header is the original RGB capture, never inference completion time.
std_msgs/Header header
string exporter_session
string tracker_session
int64 track_id
int64 class_id
uint64 sequence
sensor_msgs/PointCloud2 cloud
sensor_msgs/Image mask
bool valid
string reason
