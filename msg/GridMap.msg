# Define the GridMap message
# This is just an example, you might need to adjust it based on your actual data structure
Header header
float64 origin_x
float64 origin_y
float64 origin_z
float64 x_width
float64 y_width
float64 z_width
float64 resolution
int32 max_index_x
int32 max_index_y
int32 max_index_z
GridNode[] grid_data
