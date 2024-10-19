# OdometryArray.msg
# 这是一个自定义的ROS消息，包含了多个nav_msgs/Odometry消息

# 引入标准Header
std_msgs/Header header

# 飞机数量
std_msgs/Int32 swarm_size

# 包含多个Odometry消息的数组
nav_msgs/Odometry[] odometry_array