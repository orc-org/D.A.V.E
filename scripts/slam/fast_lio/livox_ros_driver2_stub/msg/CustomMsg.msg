# Livox point cloud (field layout of livox_ros_driver2/msg/CustomMsg)
std_msgs/Header header  # ROS standard message header
uint64 timebase         # the time of first point
uint32 point_num        # total number of points
uint8 lidar_id          # lidar device id number
uint8[3] rsvd           # reserved use
CustomPoint[] points    # pointcloud data
