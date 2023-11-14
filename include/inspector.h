#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>


#ifndef GRID_MAP_H
#define GRID_MAP_H
// 定义 search_point 类，包括位置、是否被visit、是否为frontier ponit
class SearchPoint {
public:
    // 构造函数
    SearchPoint(){};
    SearchPoint(Eigen::Vector3d pos, int x, int y, int z, bool is_obstacle):pos(pos),x(x),y(y),z(z),is_obstacle(is_obstacle){}

    Eigen::Vector3d pos; 
    int x,y,z;
    bool is_obstacle;   
    bool is_visited{false}; 
    bool is_in_frontier{false}; 
};

// 定义 grid_node 类，包括位置、索引、是否被占据、是否被检测过
class GridNode {
public:
    // 构造函数
    GridNode(double x, double y, double z, int idx) : pos(x, y, z){}

    Eigen::Vector3d pos; // 中心点的位置         // 根据地图原点和改点的位置确定的索引
    bool is_occupied{false};   // 是否被占据
    bool is_inspected{false};  // 是否被检测过
    bool need_to_inspect{false}; // 是否需要被检测
    bool is_inflated{false};
};

// 定义 grid_map 类，包括地图原点、分辨率、最大索引、栅格点信息等
class GridMap {
public:
    // 构造函数 啥都不干的来一个
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
    std::vector<std::vector<std::vector<GridNode>>> grid_map;

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
        x=x_min+idx[0]*resolution;
        y=y_min+idx[1]*resolution;
        z=z_min+idx[2]*resolution;
        Eigen::Vector3d pos(x,y,z);
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

    bool is_free_around(Eigen::Vector3i idx,bool no_flate=true){
        for(int i=std::max(idx[0]-1,0);i<std::min(max_index[0],idx[0]+1);i++){
            for(int j=std::max(idx[1]-1,0);j<std::min(max_index[1],idx[1]+1);j++){
                for(int k=std::max(idx[2]-1,0);k<std::min(max_index[2],idx[2]+1);k++){
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
            for(int i=std::max(idx[0]-range,0);i<=std::min(max_index[0],idx[0]+range);i++){
                for(int j=std::max(idx[1]-range,0);j<=std::min(max_index[1],idx[1]+range);j++){
                    //z从大到小
                    for(int k=std::min(max_index[2],idx[2]+range);k>=std::max(idx[2]-range,0);k--){
                        if(index2pos(Eigen::Vector3i(i,j,k)).z()<0.5)
                            continue;
                        if(no_inflate){
                            if(is_free_around(Eigen::Vector3i(i,j,k)))
                            {
                                // pos=index2pos(Eigen::Vector3i(i,j,k));
                                pos.x()=pos.x()+resolution*(i-idx[0]);
                                pos.y()=pos.y()+resolution*(j-idx[1]);
                                pos.z()=pos.z()+resolution*(k-idx[2]);
                                // if(pos.z()>0.5)
                                    return;
                            }
                        }
                        else{
                            // if(grid_map[i][j][k].is_inflated==false && grid_map[idx[0]][idx[1]][idx[2]].is_occupied==false)
                            if(is_free_around(Eigen::Vector3i(i,j,k),false))
                            {
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
        double inflation_radius=1;
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
                    if (grid_map[i][j][k].is_occupied) {
                        // 根据膨胀半径来标记周围的格子
                        for (int m = std::max(int(i - inflation_radius / resolution), 0);
                             m < std::min(int(i + inflation_radius / resolution), max_index[0]); m++) {
                            for (int n = std::max(int(j - inflation_radius / resolution), 0);
                                 n < std::min(int(j + inflation_radius / resolution), max_index[1]); n++) {
                                for (int p = std::max(int(k - inflation_radius / resolution), 0);
                                     p < std::min(int(k + inflation_radius / resolution), max_index[2]); p++) {
                                    // 计算格子与膨胀源的距离
                                    Eigen::Vector3i source_idx(i, j, k);
                                    Eigen::Vector3i current_idx(m, n, p);
                                    double distance = (index2pos(current_idx) - index2pos(source_idx)).norm();
                                    if (distance <= inflation_radius) {
                                        grid_map[m][n][p].is_inflated = true;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};



// 定义 inspector 类，包括 grid_map、位置、姿态信息等
class Inspector {
public:
    // 构造函数
    Inspector() {}

    GridMap grid_map;       // 栅格地图
    Eigen::Vector3d pos;    // 当前位置
    Eigen::Vector3d ori;             // 姿态
    std::vector<Eigen::Vector3d> search_points;  // 初始搜索点集 vector
    int search_points_x_length = 12; 
    int search_points_y_length = 7;
    int search_points_z_length = 6;
    std::vector<std::vector<std::vector<SearchPoint>>> search_points_array; // 初始搜索点集 array
    std::vector<SearchPoint> frontier_points;  // 边界点

    void init(double x, double y, double z, double roll, double pitch, double yaw,
                double x_min,double x_max,double y_min,double y_max,double z_min,double z_max,
                double resolution,std::vector<Eigen::Vector3d> point_set)
                // double origin_x,double origin_y,double origin_z,)
    {
        grid_map=GridMap((x_max-x_min)/2+x_min, (y_max-y_min)/2+y_min, (z_max-z_min)/2+z_min, (x_max-x_min)+10, (y_max-y_min)+10, (z_max-z_min)+10, resolution);
        pos<<x,y,z;
        ori<<roll,pitch,yaw;
        search_points=point_set;

        search_points_array.resize(search_points_x_length, std::vector<std::vector<SearchPoint>>(search_points_y_length,std::vector<SearchPoint>(search_points_z_length)));
        // search vector -> search array
        int count=0;
        for(int i=0;i<search_points_x_length;i++){
            for(int j=0;j<search_points_y_length;j++){
                for(int k=0;k<search_points_z_length;k++){
                    // 考虑是否为障碍
                    if(!grid_map.get_occupancy_pos(search_points[count]))
                        search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,false);
                    else search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,true);
                    count++;
                }
            }
        }
        // 假定从 0 0 0 开始搜索
        update_frontier_points(0,0,0);
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
        grid_map.inflate_map();
    } 

    void discrete_map(){
        double discrete_resolution=10*grid_map.resolution;
        for(double start_x=grid_map.x_min;start_x<=grid_map.x_max;start_x+=discrete_resolution){
            for(double start_y=grid_map.y_min;start_y<=grid_map.y_max;start_y+=discrete_resolution){
                for(double start_z=grid_map.z_min;start_z<=grid_map.z_max;start_z+=discrete_resolution){
                    Eigen::Vector3d now_point{start_x,start_y,start_z};
                    if(!grid_map.get_surrounding_pos(now_point,3))//如果周围没有障碍物，初筛
                        search_points.push_back(now_point);//离散地图空间，获取我要去的点
                }
            }
        }
    }

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
    SearchPoint get_target_point(){
        SearchPoint target_point;
        int target_point_idx;
        double max_heuristic=-1;
        for(int i=0;i<frontier_points.size();i++){
            auto tmp_cost=get_heuristic(pos,ori,frontier_points[i].pos);
            if(tmp_cost>max_heuristic){
                max_heuristic=tmp_cost;
                target_point=frontier_points[i];
                target_point_idx = i;
            }
            // if(max_heuristic>10)
            //     break;//为了避免搜搜时间过长
        }
        frontier_points.erase(frontier_points.begin()+target_point_idx);
        // test
        update_frontier_points(target_point.x,target_point.y,target_point.z);

        return target_point;
    }

    // 更新边界点集合
    void update_frontier_points(int x,int y,int z){
        search_points_array[x][y][z].is_visited = true;
        search_points_array[x][y][z].is_in_frontier = false;

        // 遍历可能的相邻位置
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    int newX = x + dx;
                    int newY = y + dy;
                    int newZ = z + dz;

                    // 检查是否越界
                    if (newX >= 0 && newX < search_points_x_length && newY >= 0 && newY < search_points_y_length && newZ >= 0 && newZ < search_points_z_length) {
                        // 如果相邻点不是障碍、边界点以及没有被访问，将其加入边界队列
                        if (!search_points_array[newX][newY][newZ].is_in_frontier && !search_points_array[newX][newY][newZ].is_obstacle && !search_points_array[newX][newY][newZ].is_visited) {
                            search_points_array[newX][newY][newZ].is_in_frontier=true;
                            frontier_points.push_back(search_points_array[newX][newY][newZ]);        
                        }
                    }
                }
            }
        }
    }


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
                    if(grid_map.grid_map[i][j][k].is_inflated==true||grid_map.grid_map[i][j][k].is_occupied==true)
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
    void get_search_points(visualization_msgs::Marker& marker){
        for(auto point:search_points){
            geometry_msgs::Point p;
            p.x=point[0];
            p.y=point[1];
            p.z=point[2];
            marker.points.push_back(p);
        }
    }
};

#endif // GRID_MAP_H