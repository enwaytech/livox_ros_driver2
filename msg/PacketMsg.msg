# Livox publish raw packet msg format.
#std_msgs/Header header    # ROS standard message header
uint8 lidar_type
uint32 handle
bool extrinsic_enable
uint32 point_num
uint8 data_type
uint8 line_num
uint64 time_stamp           # The time of first point
uint64 point_interval
uint8[] raw_data
