#include <vector>
#include <queue> 
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "inspector/trajectory_generator.h"

#ifndef GRID_MAP_MINI_H
#define GRID_MAP_MINI_H
//这个我不用
// 定义 search_point 类，包括位置、是否被visit、是否为frontier ponit
// class SearchPoint {
// public:
//     // 构造函数
//     SearchPoint(){};
//     SearchPoint(Eigen::Vector3d pos, int x, int y, int z, bool is_obstacle):pos(pos),x(x),y(y),z(z),is_obstacle(is_obstacle){}

//     Eigen::Vector3d pos; 
//     int x,y,z;
//     bool is_obstacle;   
//     bool is_visited{false}; 
//     bool is_in_frontier{false}; 
// };

struct waypoint{
    Eigen::Vector3i idx;  // 格点坐标
    double cost;      // 从起点到该格点的总代价
    double heuristic; // 启发式函数的值
    waypoint* parent;  // 父节点
    double yaw = 0;
};

// 定义 grid_node 类，包括位置、索引、是否被占据、是否被检测过
class GridNode_mini {
public:
    // 构造函数
    GridNode_mini(double x, double y, double z, int idx) : pos(x, y, z){}

    Eigen::Vector3d pos; // 中心点的位置         // 根据地图原点和改点的位置确定的索引
    bool is_occupied{false};   // 是否被占据
    int is_inspected{0};  // 是否被检测过
    bool need_to_inspect{false}; // 是否需要被检测
    bool is_inflated{false};
    bool is_octo_inspected{false};
};

// 定义 grid_map 类，包括地图原点、分辨率、最大索引、栅格点信息等
class GridMap_mini {
public:
    // 构造函数 啥都不干的来一个
    GridMap_mini() {}
    GridMap_mini(double origin_x, double origin_y, double origin_z, double x_width, double y_width, double z_width, double res) 
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
                std::vector<std::vector<GridNode_mini>> grid_map_i;
                for (int j = 0; j < max_index[1]; j++) {
                    std::vector<GridNode_mini> grid_map_ij;
                    for (int k = 0; k < max_index[2]; k++) {
                        grid_map_ij.push_back(GridNode_mini(x_min + i * res, y_min + j * res, z_min + k * res, i * max_index[1] * max_index[2] + j * max_index[2] + k));
                    }
                    grid_map_i.push_back(grid_map_ij);
                }
                grid_map.push_back(grid_map_i);
            }
        }
    // set_param(double origin_x, double origin_y, double origin_z, double x_width, double y_width, double z_width, double res){
    //     map_origin<<origin_x,origin_y,origin_z;
    //     resolution=res;
    //     x_min = origin_x - x_width / 2;
    //     x_max = origin_x + x_width / 2;
    //     y_min = origin_y - y_width / 2;
    //     y_max = origin_y + y_width / 2;
    //     z_min = origin_z - z_width / 2;
    //     z_max = origin_z + z_width / 2;
    //     max_index <<(int)(x_max-x_min)/res,(int)(y_max-y_min)/res,(int)(z_max-z_min)/res;//这种取整是四舍五入
    //     //先x，后y， 再z
    //     for (int i = 0; i < max_index[0]; i++) {
    //         std::vector<std::vector<GridNode>> grid_map_i;
    //         for (int j = 0; j < max_index[1]; j++) {
    //             std::vector<GridNode> grid_map_ij;
    //             for (int k = 0; k < max_index[2]; k++) {
    //                 grid_map_ij.push_back(GridNode(x_min + i * res, y_min + j * res, z_min + k * res, i * max_index[1] * max_index[2] + j * max_index[2] + k));
    //             }
    //             grid_map_i.push_back(grid_map_ij);
    //         }
    //         grid_map.push_back(grid_map_i);
    //     }
    // }


    Eigen::Vector3d map_origin; // 原点
    double x_min, x_max, y_min, y_max, z_min, z_max; 
    double resolution;          // 分辨率 (m/格)
    Eigen::Vector3i max_index;   // 最大的索引
    //grid是一个三维的vector，每个元素是一个grid_node
    std::vector<std::vector<std::vector<GridNode_mini>>> grid_map;

    bool check_inx_legal(const Eigen::Vector3i& idx, bool print=false){
        bool flag=true;
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            if(print)
                std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        if((idx[0]<0||idx[1]<0||idx[2]<0))
        {   
            if(print)
                std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        if(!flag && print)
            std::cout<<"the idx is "<<idx.transpose()<<std::endl;
        return flag;
    }

    bool inmap(const Eigen::Vector3d& position,bool print=false){
        Eigen::Vector3d idx;
        bool flag=true;
        double x,y,z;
        x=position[0];
        y=position[1];
        z=position[2];
        int id_x=(int)(x-x_min)/resolution;
        int id_y=(int)(y-y_min)/resolution;
        int id_z=(int)(z-z_min)/resolution;
        flag=check_inx_legal(Eigen::Vector3i(id_x,id_y,id_z));
        if(!flag && print)
            std::cout<<"the position "<<x<<" "<<y<<" "<<z<<std::endl;
        return flag;
    }
    bool inmap(const Eigen::Vector3i& idx,bool print=false){
        bool flag=true;
        flag=check_inx_legal(idx);
        if(!flag && print)
            std::cout<<"the index"<<idx.transpose()<<std::endl;
        return flag;
    }
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
        flag=check_inx_legal(Eigen::Vector3i(id_x,id_y,id_z));
        return flag;
    }

    // 将索引转换为坐标
    Eigen::Vector3d index2pos(const Eigen::Vector3i& idx) {
        double x,y,z;
        // std::cout<<"index2pos";
        // std::cout<<" "<<x_min<<" "<<y_min<<" "<<z_min;
        // std::cout<<" "<<idx[0]<<" "<<idx[1]<<" "<<idx[2];
        x=x_min+idx[0]*resolution;
        y=y_min+idx[1]*resolution;
        z=z_min+idx[2]*resolution;
        Eigen::Vector3d pos(x,y,z);
        // std::cout<<" "<<x<<" "<<y<<" "<<z;
        return pos;
    }

    // 判断栅格是否被占据
    bool get_occupancy_pos(const Eigen::Vector3d& position,bool no_inflate=true) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(re){//如果索引合法
            if(no_inflate){
                if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
                {
                    return true;
                }
                return false;
            }
            else{
                if(grid_map[idx[0]][idx[1]][idx[2]].is_inflated==true||grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
                {
                    return true;
                }
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    bool get_occupancy_idx(const Eigen::Vector3i idx,bool no_inflate=true) {
        // 根据位置找到相应的栅格并判断是否被占据
        bool flag=true;
        flag=check_inx_legal(idx);
        if(!flag)
            return flag;
        if(no_inflate){
            if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else{
            if(grid_map[idx[0]][idx[1]][idx[2]].is_inflated==true||grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
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

    void set_occupancy_idx(const Eigen::Vector3i idx, bool occupied) {
        // 根据位置找到相应的栅格并判断是否被占据
        bool flag=true;
        flag=check_inx_legal(idx);
        if(!flag)
            return;
        grid_map[idx[0]][idx[1]][idx[2]].is_occupied=occupied;
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

    void update_state_idx(const Eigen::Vector3i idx, bool inspected) {
        // 根据位置找到相应的栅格并判断是否被占据
        bool flag=check_inx_legal(idx);
        if(!flag)
            return;
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

    void set_need_to_inspect_idx(const Eigen::Vector3i idx, bool need_to_inspect) {
        // 根据位置找到相应的栅格并判断是否被占据
        bool flag=check_inx_legal(idx);
        if(!flag)
          return;
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

    bool is_free_around(Eigen::Vector3i idx,bool no_flate=true,int radius=2){
        for(int i=std::max(idx[0]-radius,0);i<std::min(max_index[0],idx[0]+radius);i++){
            for(int j=std::max(idx[1]-radius,0);j<std::min(max_index[1],idx[1]+radius);j++){
                for(int k=std::max(idx[2]-radius,0);k<std::min(max_index[2],idx[2]+radius);k++){
                    if(no_flate){
                        if(grid_map[i][j][k].is_occupied==true)
                        {
                            return false;
                        }
                    }
                    else{
                        if(grid_map[i][j][k].is_occupied==true||grid_map[i][j][k].is_inflated==true)
                        {
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    void search_near_free_point(Eigen::Vector3d& pos,bool no_inflate=true){

        //在这个点附近搜索一个没有occupied的点
        Eigen::Vector3i idx;
        auto re=pos2index(pos,idx);//在photo_taker应用中，应该是在地图内的。而且应该能找到吧。。。
        //搜索是以idx为中心的，从中心开始
        int range=0;
        while(range<10){
            for(int i=std::max(idx[0]-range,0);i<=std::min(max_index[0],idx[0]+range);i+=1){
                for(int j=std::max(idx[1]-range,0);j<=std::min(max_index[1],idx[1]+range);j+=1){
                    //z从大到小
                    for(int k=std::min(max_index[2],idx[2]+range);k>=std::max(idx[2]-range,0);k-=1){
                        // std::cout<<"ENTERING FREE PASS";
                        if(index2pos(Eigen::Vector3i(i,j,k)).z()<0.5){
                            Eigen::Vector3d tmp_pos=index2pos(Eigen::Vector3i(i,j,k));
                            // std::cout<<"zzzz"<<tmp_pos.z();
                            // std::cout<<"height not enough";
                            continue;
                        }
                        if(no_inflate){
                            if(is_free_around(Eigen::Vector3i(i,j,k)))
                            {
                                // std::cout<<"NEAR FREE PASS no flate"<<" "<<range;
                                pos=index2pos(Eigen::Vector3i(i,j,k));
                                // pos.x()=pos.x()+resolution*(i-idx[0]);
                                // pos.y()=pos.y()+resolution*(j-idx[1]);
                                // pos.z()=pos.z()+resolution*(k-idx[2]);
                                // if(pos.z()>0.5)
                                return;
                            }
                        }
                        else{
                            // if(grid_map[i][j][k].is_inflated==false && grid_map[idx[0]][idx[1]][idx[2]].is_occupied==false)
                            if(is_free_around(Eigen::Vector3i(i,j,k),false))
                            {   
                                // std::cout<<"NEAR FREE PASS"<<" "<<range;
                                pos=index2pos(Eigen::Vector3i(i,j,k));
                                // pos.x()=pos.x()+resolution*(i-idx[0]);
                                // pos.y()=pos.y()+resolution*(j-idx[1]);
                                // pos.z()=pos.z()+resolution*(k-idx[2]);
                                // if(pos.z()>0.5)
                                return;
                            }
                        }
                    }
                }
            }
            range++;
        }
    }

    // 执行地图膨胀操作
    void inflate_map() {
        double inflation_radius=2;//1118变大
        //全部不膨胀
        for (int i = 0; i < max_index[0]; i++) {
            for (int j = 0; j < max_index[1]; j++) {
                for (int k = 0; k < max_index[2]; k++) {
                    grid_map[i][j][k].is_inflated=false;
                }
            }
        }
        for (int i = 0; i < max_index[0]; i++) {
            for (int j = 0; j < max_index[1]; j++) {
                for (int k = 0; k < max_index[2]; k++) {
                    if(!grid_map[i][j][k].is_occupied){
                        bool occupancy_surroud=false;
                        for (int m = std::max(int(i - inflation_radius / resolution), 0);
                            m < std::min(int(i + inflation_radius / resolution), max_index[0]); m++) {
                            for (int n = std::max(int(j - inflation_radius / resolution), 0);
                                n < std::min(int(j + inflation_radius / resolution), max_index[1]); n++) {
                                for (int p = std::max(int(k - inflation_radius / resolution), 0);
                                    p < std::min(int(k + inflation_radius / resolution), max_index[2]); p++) {
                                        if(grid_map[m][n][p].is_occupied==true){
                                            occupancy_surroud=true;
                                            break;
                                        }
                                    }
                                }
                            }
                            if(occupancy_surroud){
                                grid_map[i][j][k].is_inflated=true;
                            }
                        }
                        
                    }
                }
            }
        }
        // for (int i = 0; i < max_index[0]; i++) {
        //     for (int j = 0; j < max_index[1]; j++) {
        //         for (int k = 0; k < max_index[2]; k++) {
        //             if (grid_map[i][j][k].is_occupied) {
        //                 // 根据膨胀半径来标记周围的格子
        //                 for (int m = std::max(int(i - inflation_radius / resolution), 0);
        //                      m < std::min(int(i + inflation_radius / resolution), max_index[0]); m++) {
        //                     for (int n = std::max(int(j - inflation_radius / resolution), 0);
        //                          n < std::min(int(j + inflation_radius / resolution), max_index[1]); n++) {
        //                         for (int p = std::max(int(k - inflation_radius / resolution), 0);
        //                              p < std::min(int(k + inflation_radius / resolution), max_index[2]); p++) {
        //                             // 计算格子与膨胀源的距离
        //                             Eigen::Vector3i source_idx(i, j, k);
        //                             Eigen::Vector3i current_idx(m, n, p);
        //                             double distance = (index2pos(current_idx) - index2pos(source_idx)).norm();
        //                             if (distance <= inflation_radius) {
        //                                 grid_map[m][n][p].is_inflated = true;
        //                             }
        //                         }
        //                     }
        //                 }
        //             }
        //         }
        //     }
        // }
    // }
};



// 定义 inspector 类，包括 grid_map、位置、姿态信息等
class Inspector_mini {
public:
    // 构造函数
    Inspector_mini() {}

    GridMap_mini grid_map;       // 栅格地图
    Eigen::Vector3d pos;    // 当前位置
    Eigen::Vector3d ori;             // 姿态


    std::vector<waypoint*> waypoint_path;
    std::vector<waypoint*> simplified_waypoint_path;
    std::vector<bool> sign_allow_simplify;
    Eigen::MatrixXd _polyCoeff;     // 位置多项式
    Eigen::MatrixXd _polyCoeff_vel; // 速度多项式
    Eigen::MatrixXd _polyCoeff_acc; // 加速度多项式
    Eigen::VectorXd _polyTime;      // 时间分配（每段）
    
    int curr_seg = 0;
    ros::Time start_t;
    ros::Time generate_t;
    bool is_new_tarjectory_generate=false;
    bool is_last_trajectory_finish=false;


    // std::vector<Eigen::Vector3d> search_points;  // 初始搜索点集 vector
    // int search_points_x_length = 12; 
    // int search_points_y_length = 7;
    // int search_points_z_length = 6;
    // std::vector<std::vector<std::vector<SearchPoint>>> search_points_array; // 初始搜索点集 array
    // std::vector<SearchPoint> frontier_points;  // 边界点

    void init(double x, double y, double z, double roll, double pitch, double yaw,
                double x_min,double x_max,double y_min,double y_max,double z_min,double z_max,
                double resolution,std::vector<Eigen::Vector3d> point_set)
                // double origin_x,double origin_y,double origin_z,)
    {
        grid_map=GridMap_mini((x_max-x_min)/2+x_min, (y_max-y_min)/2+y_min, (z_max-z_min)/2+z_min, (x_max-x_min)+10, (y_max-y_min)+10, (z_max-z_min)+10, resolution);
        pos<<x,y,z;
        ori<<roll,pitch,yaw;
        // search_points=point_set;

        // search_points_array.resize(search_points_x_length, std::vector<std::vector<SearchPoint>>(search_points_y_length,std::vector<SearchPoint>(search_points_z_length)));
        // search vector -> search array
        // int count=0;
        // for(int i=0;i<search_points_x_length;i++){
        //     for(int j=0;j<search_points_y_length;j++){
        //         for(int k=0;k<search_points_z_length;k++){
        //             // 考虑是否为障碍
        //             if(!grid_map.get_occupancy_pos(search_points[count]))
        //                 search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,false);
        //             else search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,true);
        //             count++;
        //         }
        //     }
        // }
        // 假定从 0 0 0 开始搜索
        // update_frontier_points(0,0,0);
    }

    void update_state(nav_msgs::Odometry odom){
        // 更新位置和姿态信息
        pos[0]=odom.pose.pose.position.x;
        pos[1]=odom.pose.pose.position.y;
        pos[2]=odom.pose.pose.position.z;
        ori[0]=odom.pose.pose.orientation.x;
        ori[1]=odom.pose.pose.orientation.y;
        ori[2]=odom.pose.pose.orientation.z;
    }

    // 初始化地图
    void init_map(std::vector<Eigen::Vector3d>& point_set) {
        // 根据 bounding box 的范围建立地图，设置 box 表面的点为占据状态
        // x y z 的范围都经过了适当的放缩
        // int multi_occupied=0;
        for(auto point:point_set){
            grid_map.set_occupancy_pos(point,true);//只设置是否被占据，inspected统一为false
        }
        ROS_INFO("SET OCCUPIED");
        //所有高度小于-1的点都被占据
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.index2pos(Eigen::Vector3i(i,j,k)).z()<=1)
                        grid_map.set_occupancy_idx(Eigen::Vector3i(i,j,k),true);
                }
            }
        }
        ROS_INFO("SET OCCUPIED -1");
        grid_map.inflate_map();
        ROS_INFO("INFLATE MAP");
    } 

    // void discrete_map(){
    //     double discrete_resolution=10*grid_map.resolution;
    //     for(double start_x=grid_map.x_min;start_x<=grid_map.x_max;start_x+=discrete_resolution){
    //         for(double start_y=grid_map.y_min;start_y<=grid_map.y_max;start_y+=discrete_resolution){
    //             for(double start_z=grid_map.z_min;start_z<=grid_map.z_max;start_z+=discrete_resolution){
    //                 Eigen::Vector3d now_point{start_x,start_y,start_z};
    //                 if(!grid_map.get_surrounding_pos(now_point,3))//如果周围没有障碍物，初筛
    //                     search_points.push_back(now_point);//离散地图空间，获取我要去的点
    //             }
    //         }
    //     }
    // }

    void set_inpect_grid(std::vector<Eigen::Vector3d>& point_set){
        for(auto point:point_set){
            grid_map.set_need_to_inspect_pos(point,true);
        }
    }

    std::vector<Eigen::Vector3i> ray_casting(Eigen::Vector3d& position,Eigen::Vector3d& target_position){
        // 判断两点之间是否存在遮挡
        // 先来简单版本，不考虑姿态
        std::vector<Eigen::Vector3i> idxs;
        int num_slides=(target_position-position).norm()/grid_map.resolution;
        Eigen::Vector3d middle_position;
        for(double a=1.0/num_slides;a<1.0-1.0/num_slides;a+=1.0/num_slides){
            middle_position=position+a*(target_position-position);
            Eigen::Vector3i idx;
            auto re=grid_map.pos2index(middle_position,idx);
            if(!re)//如果索引不合法,false(默认空)
            {
                continue;
            }
            if(grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
            {
                idxs.push_back(idx);
            }
        }
        return idxs;
    }

    bool have_collision(Eigen::Vector3d& position,Eigen::Vector3d& target_position){
        auto re=ray_casting(position,target_position);
        if(re.size()>0)
            return true;
        else
            return false;
    }

    // 获取能被检测到的未被检测过的点的索引
    std::vector<Eigen::Vector3i> get_covered_points(Eigen::Vector3d& position, Eigen::Vector3d& orientation) {
        // 根据当前的位置和姿态获取能被检测到的未被检测过的点
        //先来简单版本，不考虑姿态
        Eigen::Vector3i idx;
        // auto re=grid_map.pos2index(position,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-10;//这里5对应分辨率乘5
        int x_max=idx[0]+10;
        int y_min=idx[1]-10;
        int y_max=idx[1]+10;
        int z_min=idx[2]-10;
        int z_max=idx[2]+10;
        std::vector<Eigen::Vector3i> idxs;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i++){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j++){
                for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k++){
                    auto target_position=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                    if(!have_collision(position,target_position))
                    {
                        if(grid_map.grid_map[i][j][k].is_inspected==false)
                        {
                            Eigen::Vector3i idx(i,j,k);
                            idxs.push_back(idx);
                        }
                    }
                    // else
                    // {

                    // }    // std::cout<<"have collision"<<std::endl;
                }
            }
        }
        
        return idxs;
    }
    
    // 获取启发式值
    double get_heuristic(Eigen::Vector3d& position, Eigen::Vector3d& orientation, Eigen::Vector3d new_point) {
        // 遍历关键点，综合考虑距离和新观测量，获取前往的目标点
        std::vector<Eigen::Vector3i> idxs = get_covered_points(new_point,orientation);
        int num_covered=idxs.size();
        double distance=0;
        distance = (new_point-position).norm();
        double heuristic=num_covered/distance;
        return heuristic;
    }

    //获取新的目标点
    // SearchPoint get_target_point(){
    //     SearchPoint target_point;
    //     int target_point_idx;
    //     double max_heuristic=-1;
    //     for(int i=0;i<frontier_points.size();i++){
    //         auto tmp_cost=get_heuristic(pos,ori,frontier_points[i].pos);
    //         if(tmp_cost>max_heuristic){
    //             max_heuristic=tmp_cost;
    //             target_point=frontier_points[i];
    //             target_point_idx = i;
    //         }
    //         // if(max_heuristic>10)
    //         //     break;//为了避免搜搜时间过长
    //     }
    //     frontier_points.erase(frontier_points.begin()+target_point_idx);
    //     // test
    //     update_frontier_points(target_point.x,target_point.y,target_point.z);

    //     return target_point;
    // }

    // 更新边界点集合
    // void update_frontier_points(int x,int y,int z){
    //     search_points_array[x][y][z].is_visited = true;
    //     search_points_array[x][y][z].is_in_frontier = false;

    //     // 遍历可能的相邻位置
    //     for (int dx = -1; dx <= 1; dx++) {
    //         for (int dy = -1; dy <= 1; dy++) {
    //             for (int dz = -1; dz <= 1; dz++) {
    //                 int newX = x + dx;
    //                 int newY = y + dy;
    //                 int newZ = z + dz;

    //                 // 检查是否越界
    //                 if (newX >= 0 && newX < search_points_x_length && newY >= 0 && newY < search_points_y_length && newZ >= 0 && newZ < search_points_z_length) {
    //                     // 如果相邻点不是障碍、边界点以及没有被访问，将其加入边界队列
    //                     if (!search_points_array[newX][newY][newZ].is_in_frontier && !search_points_array[newX][newY][newZ].is_obstacle && !search_points_array[newX][newY][newZ].is_visited) {
    //                         search_points_array[newX][newY][newZ].is_in_frontier=true;
    //                         frontier_points.push_back(search_points_array[newX][newY][newZ]);        
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }


    // //获取新的目标点
    // Eigen::Vector3d get_target_point(){
    //     Eigen::Vector3d target_point;
    //     double max_heuristic=-1;
    //     for(auto point:search_points){
    //         auto tmp_cost=get_heuristic(pos,ori,point);
    //         if(tmp_cost>max_heuristic){
    //             max_heuristic=tmp_cost;
    //             target_point=point;
    //         }
    //         if(max_heuristic>10)
    //             break;//为了避免搜搜时间过长
    //     }
    //     return target_point;
    // }

    // 更新地图状态
    void update_map() {
        // 根据当前的位置和姿态，更新地图，设置点状态为已检测
        int num_updated=0;
        Eigen::Vector3i idx;
        auto re=grid_map.pos2index(pos,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-5;//这里5对应分辨率乘5
        int x_max=idx[0]+5;
        int y_min=idx[1]-5;
        int y_max=idx[1]+5;
        int z_min=idx[2]-5;
        int z_max=idx[2]+5;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i++){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j++){
                for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k++){
                    if(grid_map.grid_map[i][j][k].is_inspected==false && grid_map.grid_map[i][j][k].is_occupied==true)
                    {   
                        auto target_pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        if(!have_collision(pos,target_pos))
                        {
                            grid_map.grid_map[i][j][k].is_inspected=true;
                            num_updated++;
                        }
                        grid_map.grid_map[i][j][k].is_inspected=true;
                        num_updated++;
                    }
                }
            }
        }
        std::cout<<"num_updated:"<<num_updated<<std::endl;
    }

    void get_occupied_points(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_occupied==true && grid_map.grid_map[i][j][k].need_to_inspect==false)
                    {
                        Eigen::Vector3d pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        geometry_msgs::Point p;
                        p.x=pos[0];
                        p.y=pos[1];
                        p.z=pos[2];
                        marker.points.push_back(p);
                    }
                }
            }
        }
    }

    void get_inflated_points(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_inflated==false && grid_map.grid_map[i][j][k].is_occupied==false)
                    {
                        Eigen::Vector3d pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        geometry_msgs::Point p;
                        p.x=pos[0];
                        p.y=pos[1];
                        p.z=pos[2];
                        marker.points.push_back(p);
                    }
                }
            }
        }
    }

    void get_uninspected_points(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_inspected== false && grid_map.grid_map[i][j][k].is_occupied==true
                     && grid_map.grid_map[i][j][k].need_to_inspect==true)
                    {
                        Eigen::Vector3d pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        geometry_msgs::Point p;
                        p.x=pos[0];
                        p.y=pos[1];
                        p.z=pos[2];
                        marker.points.push_back(p);
                    }
                }
            }
        }
    }
    void get_inspected_points(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_inspected== true && grid_map.grid_map[i][j][k].is_occupied==true
                    && grid_map.grid_map[i][j][k].need_to_inspect==true)
                    {
                        Eigen::Vector3d pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        geometry_msgs::Point p;
                        p.x=pos[0];
                        p.y=pos[1];
                        p.z=pos[2];
                        marker.points.push_back(p);
                    }
                }
            }
        }
    }
    // void get_search_points(visualization_msgs::Marker& marker){
    //     for(auto point:search_points){
    //         geometry_msgs::Point p;
    //         p.x=point[0];
    //         p.y=point[1];
    //         p.z=point[2];
    //         marker.points.push_back(p);
    //     }
    // }

    double epsilon = 2;  // A star 简化阈值
    // 比较两个节点的总代价
    struct CompareWaypoint {
        bool operator()(const waypoint* a, const waypoint* b) const {
            return (a->cost + a->heuristic) > (b->cost + b->heuristic);
        }
    };

    std::vector<waypoint*> AStar(waypoint& start, waypoint& goal, GridMap_mini map) {
        std::vector<waypoint*> path;

        // 定义移动方向
        const int dx[] = {1, 1, 1, 0, 0, 0, -1, -1, -1, 1, 1, 1, 0,  0, -1, -1, -1, 1, 1, 1, 0, 0, 0, -1, -1, -1};
        const int dy[] = {1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1,  -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1};
        const int dz[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,  0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1};

        const int x_length = map.max_index[0];
        const int y_length = map.max_index[1];
        const int z_length = map.max_index[2];

        // 创建数组来记录节点是否已经被访问
        std::vector<std::vector<std::vector<bool>>> visited(x_length, std::vector<std::vector<bool>>(y_length, std::vector<bool>(z_length, false)));

        // 创建数组来记录节点是否已经被add
        std::vector<std::vector<std::vector<bool>>> added(x_length, std::vector<std::vector<bool>>(y_length, std::vector<bool>(z_length, false)));

        // 创建优先队列用于节点扩展
        std::priority_queue<waypoint*, std::vector<waypoint*>, CompareWaypoint> openSet;

        // 初始化起始节点
        waypoint* startNode = new  waypoint{start.idx, 0, 0, nullptr};
        openSet.push(startNode);
        added[start.idx[0]][start.idx[1]][start.idx[2]] = true;

        if(start.idx==goal.idx){
            path.push_back(startNode);
            return path;
        }

        while (!openSet.empty()) {
            // 从优先队列中取出代价最小的节点
            waypoint* current = openSet.top();
            openSet.pop();

            // 如果当前节点是目标节点，构建并返回路径
            if (current->idx[0] == goal.idx[0] && current->idx[1] == goal.idx[1] && current->idx[2] == goal.idx[2]) {
                
                while (current != nullptr) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(),path.end());
                return path;
            }            
            visited[current->idx[0]][current->idx[1]][current->idx[2]] = true;

            // 扩展当前节点
            for (int i = 0; i < 26; i++) {
                int nextX = current->idx[0] + dx[i];
                int nextY = current->idx[1] + dy[i];
                int nextZ = current->idx[2] + dz[i];
                Eigen::Vector3i nextWaypoint(nextX,nextY,nextZ);

                // 检查是否越界或者是障碍物
                if (nextX < 0 || nextX >= x_length || nextY < 0 || nextY >= y_length || nextZ < 0 || nextZ >= z_length || map.grid_map[nextX][nextY][nextZ].is_occupied || map.grid_map[nextX][nextY][nextZ].is_inflated)
                    continue;

                // 如果节点已经访问过 or added，跳过
                if (visited[nextX][nextY][nextZ] || added[nextX][nextY][nextZ])
                    continue;

                // 计算下一个节点的代价和启发式函数值
                double nextCost = current->cost + std::pow(std::pow(nextX - current->idx[0],2) + std::pow(nextY - current->idx[1],2) + std::pow(nextZ - current->idx[2],2),0.5); 
                double heuristic = std::pow(std::pow(nextX - goal.idx[0],2) + std::pow(nextY - goal.idx[1],2) + std::pow(nextZ - goal.idx[2],2),0.5); 
                // 创建下一个节点
                waypoint* nextNode = new waypoint{nextWaypoint, nextCost, heuristic, current};

                openSet.push(nextNode);
                added[nextX][nextY][nextZ] = true;
            }
        }

        // // 如果无法找到路径，返回一个空路径
        // ROS_INFO("[%s]A* can not find valid waypoints to reach target!", name.c_str());
        return std::vector<waypoint*>();
    }

    // 计算点到直线的距离
    double pointToLineDistance(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d P) {
        // 计算向量 AB 和 AP
        double ABx = B[0] - A[0];
        double ABy = B[1] - A[1];
        double ABz = B[2] - A[2];

        double APx = P[0] - A[0];
        double APy = P[1] - A[1];
        double APz = P[2] - A[2];

        // 计算法向量 N
        double Nx = ABy * APz - ABz * APy;
        double Ny = ABz * APx - ABx * APz;
        double Nz = ABx * APy - ABy * APx;

        // 计算法向量 N 的模
        double N_length = std::sqrt(Nx * Nx + Ny * Ny + Nz * Nz);

        // 计算点到直线的距离
        double AB_length = std::sqrt(ABx * ABx + ABy * ABy + ABz * ABz);

        // 避免除以零的情况
        if (AB_length == 0.0) {
            throw std::invalid_argument("The length of line segment AB is zero.");
        }

        double distance = N_length / AB_length;

        return distance;
    }


    // 执行 Douglas-Peucker 算法 精简A* 不然中间点过多一卡一卡
    std::vector<waypoint *> douglasPeucker(std::vector<waypoint *> points, std::vector<bool>& sign_, double epsilon) {
        std::vector<bool> sign_simplify;
        sign_simplify.insert(sign_simplify.end(),sign_.begin(),sign_.end());
        sign_.clear();
        std::vector<waypoint *> simplified;

        if (points.size() < 3) {
            sign_=sign_simplify;
            return points;
        }

        // 寻找最远点
        double maxDistance = 0.0;
        size_t maxIndex = 0;
        size_t middleIndex = 0;
        waypoint * start = points.front();
        waypoint * end = points.back();


        for (size_t i = 1; i < points.size() - 1; ++i) {
            double d = pointToLineDistance(grid_map.index2pos(start->idx), grid_map.index2pos(end->idx), grid_map.index2pos(points[i]->idx));
            if (d > maxDistance) {
                maxDistance = d;
                maxIndex = i;
            }
            if(sign_simplify[i]==true){//不能越过视点
                middleIndex = i;//middleIndex一定大于等于maxIndex
                break;
            }
        }

        // 如果最大距离大于阈值，则保留最远点，否则继续递归简化
        if (maxDistance > epsilon) {
            std::vector<waypoint *> firstPart(points.begin(), points.begin() + maxIndex + 1);//+1的原因是为了保证最远点也被保留
            std::vector<waypoint *> secondPart(points.begin() + maxIndex, points.end());
            std::vector<bool> firstPart_sign(sign_simplify.begin(), sign_simplify.begin() + maxIndex + 1);
            std::vector<bool> secondPart_sign(sign_simplify.begin() + maxIndex, sign_simplify.end());
            std::vector<waypoint *> simplifiedFirst, simplifiedSecond;

            simplifiedFirst = douglasPeucker(firstPart,firstPart_sign, epsilon);
            simplifiedSecond = douglasPeucker(secondPart,secondPart_sign, epsilon);
            if(simplifiedFirst.empty()||simplifiedSecond.empty()){
                ROS_ERROR("simplifiedFirst or simplifiedSecond is empty");
            }
            if(simplifiedFirst.empty()&&simplifiedSecond.empty()){
                ROS_ERROR("simplifiedFirst and simplifiedSecond are empty");
            }
            sign_.insert(sign_.end(), firstPart_sign.begin(), firstPart_sign.end() - 1);
            sign_.insert(sign_.end(), secondPart_sign.begin(), secondPart_sign.end());
            simplified.insert(simplified.end(), simplifiedFirst.begin(), simplifiedFirst.end() - 1);
            simplified.insert(simplified.end(), simplifiedSecond.begin(), simplifiedSecond.end());
        } 
        else {//这种还有可能是，我遇到了第一个对应视点的点，且前面没有超过阈值的点，所以之前的直接保留，探索后面的能否简化
            if(middleIndex==0){//说明就起点终点必须经过，终点是下一个视点
                simplified.clear();
                sign_.clear();
                simplified.push_back(start);
                simplified.push_back(end);
                // sign_.push_back(true);
                // sign_.push_back(true);
                sign_.push_back(sign_simplify[0]);
                sign_.push_back(sign_simplify[points.size()-1]);
            }
            else{
                simplified.clear();
                sign_.clear();
                // simplified.insert(simplified.end(),points.begin(),points.begin()+middleIndex);//不包含middleIndex
                // sign_.insert(sign_.end(),sign_simplify.begin(),sign_simplify.begin()+middleIndex);
                simplified.push_back(start);
                sign_.push_back(sign_simplify[0]);

                std::vector<waypoint*> left_part=std::vector<waypoint*>(points.begin()+middleIndex,points.end());
                std::vector<bool> left_part_sign=std::vector<bool>(sign_simplify.begin()+middleIndex,sign_simplify.end());//应该不会出问题
                std::vector<waypoint*> simplified_left=douglasPeucker(left_part,left_part_sign,epsilon);
                simplified.insert(simplified.end(),simplified_left.begin(),simplified_left.end());
                sign_.insert(sign_.end(),left_part_sign.begin(),left_part_sign.end());
            }
        }
        return simplified;
    }


    // 找到trajectory不安全的首个点距离精简前的waypoints中最近的那一个 然后加进去
    std::vector<waypoint *> correct_trajectory(){
        Eigen::Vector3d unsafe_point;
        waypoint * add_point;
        std::vector<waypoint *> added_path;
        bool already_in = false;
        for(int i=0;i<_polyTime.size();i++){
            for(double t=0;t<=_polyTime[i];t+=0.1){
                unsafe_point = getPosPoly(_polyCoeff,i,t);
                if(grid_map.get_occupancy_pos(unsafe_point) || grid_map.get_occupancy_pos(unsafe_point,true)){
                    // std::cout<<"trajectory unsafe" << std::endl;
                    if(simplified_waypoint_path.size()==waypoint_path.size()){
                        // std::cout<<"simplified_waypoint_path have alread become waypoint_path" << std::endl;
                    }
                    double min_dis = DBL_MAX;
                    for(auto waypoint:waypoint_path){
                        double tmp = (grid_map.index2pos(waypoint->idx)-unsafe_point).norm();
                        if(tmp<min_dis){
                            for(auto s_waypoint:simplified_waypoint_path){
                                if(waypoint->idx==s_waypoint->idx){
                                    already_in = true;
                                    break;
                                }
                            }
                            if(already_in){
                                already_in = false;
                                continue;
                            }
                            min_dis = tmp;
                            add_point = waypoint;
                        }
                    }
                    // 保证顺序
                    for(auto waypoint:waypoint_path){
                        for(auto s_waypoint:simplified_waypoint_path){
                            if(waypoint->idx==s_waypoint->idx){
                                added_path.push_back(waypoint);
                                break;
                            }
                        }
                        if(waypoint->idx == add_point->idx) 
                            added_path.push_back(waypoint);
                            
                    }
                    return added_path;
                }
            }
        }
        return added_path;
    }

    bool reassign_sign(std::vector<waypoint*> pre_path,std::vector<waypoint*> now_path,std::vector<bool>& sign_simplify){
        std::vector<bool> sign_;
        sign_.insert(sign_.end(),sign_simplify.begin(),sign_simplify.end());
        sign_simplify.clear();
        int index=-1;
        for(auto waypoint:now_path){
            bool find = false;
            for(int i=0;i<pre_path.size();i++){
                if(waypoint->idx==pre_path[i]->idx){
                    find = true;
                    index = i;
                    break;
                }
            }
            if(find){
                sign_simplify.push_back(sign_[index]);
            }
            else{
                sign_simplify.push_back(false);
            }
        }
    }

    bool generate_trajetory(std::vector<waypoint *> waypoints, std::vector<bool> sign_simplify,std::vector<Eigen::Vector3d> yaw_point_list){
        if(waypoints.size()==0){
            // ROS_INFO("[%s] error!  A* waypoints is empty",name.c_str());   
            return false;      
        }
        // 当前点即为目标点
        if(waypoints.size()==1){
            waypoints.push_back(waypoints[0]);
        }
        // 简化A*路点
        try{
            simplified_waypoint_path = douglasPeucker(waypoints,sign_simplify,epsilon);
        }catch (const std::invalid_argument& e) {
            // std::cerr << "Error: " << e.what() << std::endl;
        }
        // ROS_INFO("simpled_waypoint_path size: %d",simplified_waypoint_path.size());
        if(simplified_waypoint_path.size()==0){
            // ROS_INFO("[%s] error!  simplified_waypoint_path is empty",name.c_str());   
            return false;      
        }
        // if(simplified_waypoint_path.size()==2){
        //     waypoint* tmp = new waypoint;
        //     tmp->idx = (simplified_waypoint_path[0]->idx + simplified_waypoint_path[1]->idx)/2; 
        //     simplified_waypoint_path.insert(simplified_waypoint_path.begin() + 1, tmp);
        // }
        Eigen::MatrixXd waypoint_path_matrix(simplified_waypoint_path.size(), 3); 
        for(int k = 0; k < simplified_waypoint_path.size(); k++){
            waypoint_path_matrix.row(k) = Eigen::Vector3d(grid_map.index2pos(simplified_waypoint_path[k]->idx)[0],
                                                        grid_map.index2pos(simplified_waypoint_path[k]->idx)[1],
                                                        std::max(0.2,grid_map.index2pos(simplified_waypoint_path[k]->idx)[2]));
        }
        // ROS_INFO("waypoint_path_matrix size: %d",waypoint_path_matrix.rows());
        trajGeneration(waypoint_path_matrix,_polyCoeff,_polyTime);
        for(size_t i=0;i<_polyTime.size();i++){
            if(_polyTime[i]>100){
                return false;
            }
        }
        // ROS_INFO("polyCoeff size: %d",_polyTime.size());
        std::vector<waypoint *> correct_waypoint_path = correct_trajectory();
        // ROS_INFO("correct_waypoint_path size: %d",correct_waypoint_path.size());
        // 如果traj不合法 尝试去对精简后的A*路点集添加原来的路点来使traj更贴近原始A*路点的轨迹
        while(correct_waypoint_path.size()!=0){
            // 即便是原始A*路点也不行 那就放弃这个目标
            if(simplified_waypoint_path.size()==correct_waypoint_path.size()){
                // ROS_INFO("[%s] can not find safe trajectory",name.c_str());  
                return false;
            }
            reassign_sign(simplified_waypoint_path,correct_waypoint_path,sign_simplify);
            simplified_waypoint_path=correct_waypoint_path;
            waypoint_path_matrix = Eigen::MatrixXd(simplified_waypoint_path.size(), 3); 
            for(int k = 0; k < simplified_waypoint_path.size(); k++){
            waypoint_path_matrix.row(k) = Eigen::Vector3d(grid_map.index2pos(simplified_waypoint_path[k]->idx)[0],
                                                        grid_map.index2pos(simplified_waypoint_path[k]->idx)[1],
                                                        std::max(0.2,grid_map.index2pos(simplified_waypoint_path[k]->idx)[2]));
            }
            trajGeneration(waypoint_path_matrix,_polyCoeff,_polyTime);
            std::vector<waypoint *> correct_waypoint_path = correct_trajectory();
        }

        _polyCoeff_vel = calculate_polycoeff_de(_polyCoeff);
        _polyCoeff_acc = calculate_polycoeff_de(_polyCoeff_vel);
        return true;
    }
};


// class Trajectory_{
//     Trajectory_(){}
//     Trajectory_(){

//     }
// }

#endif // GRID_MAP_H