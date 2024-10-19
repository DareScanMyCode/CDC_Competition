#include <Eigen/Dense>
#include <vector>
#include <iostream>

// 定义 grid_node 类，包括位置、索引、是否被占据、是否被检测过
class GridNode {
public:
    // 构造函数
    GridNode(double x, double y, double z, int idx) : pos(x, y, z){}

    Eigen::Vector3d pos; // 中心点的位置         // 根据地图原点和改点的位置确定的索引
    bool is_occupied{false};   // 是否被占据
    int is_inspected{0};  // 是否被检测过
    bool need_to_inspect{false}; // 是否需要被检测
    bool is_in_bbox{false};
    bool is_octo_inspected{false}; //whether is updated by octomap
    bool in_flat{false};
    bool is_face{false};
};

// 定义 grid_map 类，包括地图原点、分辨率、最大索引、栅格点信息等
class GridMap {
public:
    // 构造函数
    GridMap() {}
    GridMap(double origin_x, double origin_y, double origin_z, double x_width, double y_width, double z_width, double res) 
        : map_origin(origin_x, origin_y, origin_z), resolution(res) {
            x_min = origin_x - x_width / 2;
            x_max = origin_x + x_width / 2;
            y_min = origin_y - y_width / 2;
            y_max = origin_y + y_width / 2;
            z_min = origin_z - z_width / 2;
            z_max = origin_z + z_width / 2;
            max_index <<(int)(x_max-x_min)/res,(int)(y_max-y_min)/res,(int)(z_max-z_min)/res;//这种取整是四舍五入
            //先x，后y， 再z
            for (int i = 0; i < max_index[0]; i++) {
                std::vector<std::vector<GridNode>> grid_map_i;
                for (int j = 0; j < max_index[1]; j++) {
                    std::vector<GridNode> grid_map_ij;
                    for (int k = 0; k < max_index[2]; k++) {
                        grid_map_ij.push_back(GridNode(x_min + i * res, y_min + j * res, z_min + k * res, i * max_index[1] * max_index[2] + j * max_index[2] + k));
                    }
                    grid_map_i.push_back(grid_map_ij);
                }
                grid_map.push_back(grid_map_i);
            }
        }

    Eigen::Vector3d map_origin; // 原点
    double x_min, x_max, y_min, y_max, z_min, z_max; 
    double resolution;          // 分辨率 (m/格)
    Eigen::Vector3i max_index;   // 最大的索引
    //grid是一个三维的vector，每个元素是一个grid_node
    std::vector<std::vector<std::vector<GridNode>>> grid_map;

    // 将坐标转换为索引
    bool pos2index(const Eigen::Vector3d& position,Eigen::Vector3i& idx) {
        bool flag=true;
        double x,y,z;
        x=position[0];
        y=position[1];
        z=position[2];
        int id_x=(int)(x-x_min)/resolution;
        int id_y=(int)(y-y_min)/resolution;
        int id_z=(int)(z-z_min)/resolution;

        idx<<id_x,id_y,id_z;
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        if(idx[0]<0||idx[1]<0||idx[2]<0)
        {
            // std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        return flag;
    }

    // 将索引转换为坐标
    Eigen::Vector3d index2pos(const Eigen::Vector3i& idx) {
        double x,y,z;
        x=x_min+idx[0]*resolution;
        y=y_min+idx[1]*resolution;
        z=z_min+idx[2]*resolution;
        Eigen::Vector3d pos(x,y,z);
        return pos;
    }

    // 判断栅格是否被占据
    bool get_occupancy_pos(const Eigen::Vector3d& position) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(idx[0]<=-1||idx[1]<=-1||idx[2]<=-1)//如果索引不合法
        {
            // std::cout<<"error:index smaller than 0"<<std::endl;
            return false;
        }    
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            //std::cout<<"error:the position is out of the map, code: 10"<<std::endl;
            return false;
        }
        if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_flat_pos(const Eigen::Vector3d& position) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(idx[0]<=-1||idx[1]<=-1||idx[2]<=-1)//如果索引不合法
        {
            // std::cout<<"error:index smaller than 0"<<std::endl;
            return false;
        }    
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            //std::cout<<"error:the position is out of the map, code: 10"<<std::endl;
            return false;
        }
        if(grid_map[idx[0]][idx[1]][idx[2]].in_flat==true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_occupancy_idx(const Eigen::Vector3i idx) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]==-1||idx[1]==-1||idx[2]==-1)//如果索引不合法,false(默认空)
        {
            // std::cout<<"error:the position is out of the map, code: 11"<<std::endl;
            return false;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map, code: 12"<<std::endl;
            return false;
        }
        if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void set_face_pos(const Eigen::Vector3d& position, bool is_face_) {
        // 根据位置找到相应的栅格并设置占据情况
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_face=is_face_;
    }

    bool get_face_pos(const Eigen::Vector3d& position) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(idx[0]<=-1||idx[1]<=-1||idx[2]<=-1)//如果索引不合法
        {
            // std::cout<<"error:index smaller than 0"<<std::endl;
            return false;
        }    
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            //std::cout<<"error:the position is out of the map, code: 10"<<std::endl;
            return false;
        }
        return grid_map[idx[0]][idx[1]][idx[2]].is_face==true;

    }

    // 设置栅格占据情况
    void set_occupancy_pos(const Eigen::Vector3d& position, bool occupied) {
        // 根据位置找到相应的栅格并设置占据情况
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_occupied=occupied;
    }

    // 设置栅格占据情况
    void set_flat_pos(const Eigen::Vector3d& position, bool flat) {
        // 根据位置找到相应的栅格并设置占据情况
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].in_flat=flat;
    }

    // 设置栅格占据情况
    void set_octo_inspected(const Eigen::Vector3d& position, bool inspected) {
        // 根据位置找到相应的栅格并设置占据情况
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_octo_inspected=inspected;
    }

    void set_occupancy_idx(const Eigen::Vector3d idx, bool occupied) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            // std::cout<<"error:the position is out of the map, code: 13"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map, code: 14"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_occupied=occupied;
    }

    void set_in_bbox_pos(const Eigen::Vector3d& position, bool in_bbox) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox=in_bbox;
    }

    // 更新检测情况
    void update_state_pos(const Eigen::Vector3d& position, bool inspected) {
        // 根据位置找到相应的栅格并更新检测状态
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_inspected=inspected;
    }

    void update_state_idx(const Eigen::Vector3d idx, bool inspected) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            // std::cout<<"error:the position is out of the map, code: 15"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map, code: 16"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_inspected=inspected;
    }

    void set_need_to_inspect_pos(const Eigen::Vector3d& position, bool need_to_inspect) {
        // 根据位置找到相应的栅格并设置是否需要被检测
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=need_to_inspect;
    }

    void set_need_to_inspect_idx(const Eigen::Vector3d idx, bool need_to_inspect) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            // std::cout<<"error:the position is out of the map, code: 17"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map, code: 18"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=need_to_inspect;
    }

    bool get_surrounding_pos(const Eigen::Vector3d& position,int range){
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        for(int i=std::max(idx[0]-range,0);i<std::min(max_index[0],idx[0]+range);i++){
            for(int j=std::max(idx[1]-range,0);j<std::min(max_index[1],idx[1]+range);j++){
                for(int k=std::max(idx[2]-range,0);k<std::min(max_index[2],idx[2]+range);k++){
                    if(grid_map[i][j][k].is_occupied==true)
                    {
                        return false;
                    }
                }
            }
        }
        return true; 
    }

    bool get_surrounding_idx(const Eigen::Vector3i idx,int range){
        for(int i=std::max(idx[0]-range,0);i<std::min(max_index[0],idx[0]+range);i++){
            for(int j=std::max(idx[1]-range,0);j<std::min(max_index[1],idx[1]+range);j++){
                for(int k=std::max(idx[2]-range,0);k<std::min(max_index[2],idx[2]+range);k++){
                    if(grid_map[i][j][k].is_occupied==true)
                    {
                        return false;
                    }
                }
            }
        }
    }
};

