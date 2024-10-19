# GridMap.msg
Header header
# 原点，使用三个浮点数表示x, y, z坐标
float64 origin_x
float64 origin_y
float64 origin_z

# 地图的边界坐标
float64 x_min
float64 x_max
float64 y_min
float64 y_max
float64 z_min
float64 z_max

# 分辨率 (m/格)
float64 resolution

# 最大的索引，使用三个整数表示
int32 max_index_x
int32 max_index_y
int32 max_index_z

# 这里我们不能直接表示三维向量的网格，因为ROS消息不支持这样的复杂类型
# 你需要将网格数据转换成一种ROS可以处理的格式，例如，通过线性化为一维数组
# 或者考虑使用其他方法在ROS节点之间传输复杂的网格数据

# 可选：如果GridNode是一个已经定义的消息类型，则可以包含一个GridNode数组
GridNodeMsg[] grid_nodes
