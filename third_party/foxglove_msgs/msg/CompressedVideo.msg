# A single frame of a compressed video bitstream.
# Compatible with foxglove_msgs/msg/CompressedVideo 2.3.x.

builtin_interfaces/Time timestamp
string frame_id
uint8[] data
string format
