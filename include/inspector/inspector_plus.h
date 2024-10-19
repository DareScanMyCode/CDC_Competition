#include <vector>
#include <iostream>
#include <queue> 
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ctime>
#include <ros/ros.h>
#include <thread>
#include "trajectory_generator.h"
#include "msg_pub.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <geometry_msgs/TwistStamped.h>
#include "inspector/communication.h"
#include <mutex>

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"

// bool is_debuging = true;
bool is_debuging = false;
// 定义 inspector 类，包括 grid_map、位置、姿态信息等
class Inspector {
public:
    int search_points_x_length; 
    int search_points_y_length;
    int search_points_z_length; 
    std::vector<Eigen::Vector3d> search_points;  // 搜索点集 vector
    std::vector<std::vector<std::vector<SearchPoint>>> search_points_array; // 搜索点集 array
    std::vector<std::vector<std::vector<SearchPoint>>> search_points_array_copy; // 搜索点集 array copy
    std::vector<SearchPoint> frontier_points;  // 边界点集
    GridMap grid_map;        // 栅格地图
    Eigen::Vector3d pos;            // 当前位置
    Eigen::Vector3d ori;            // 姿态
    Eigen::Vector3d gimbal_pos;
    Eigen::Vector3d gimbal_ori;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
    SearchPoint target_point;
    Eigen::MatrixXd _polyCoeff;     // 位置多项式
    Eigen::MatrixXd _polyCoeff_vel; // 速度多项式
    Eigen::MatrixXd _polyCoeff_acc; // 加速度多项式
    Eigen::VectorXd _polyTime;      // 时间分配（每段）

    // 定义waypoint
    struct waypoint{
        Eigen::Vector3i idx;  // 格点坐标
        double cost;      // 从起点到该格点的总代价
        double heuristic; // 启发式函数的值
        waypoint* parent;  // 父节点
        double yaw = 0;
    };
    std::vector<waypoint*> waypoint_path;
    std::vector<waypoint*> simplified_waypoint_path;
    double epsilon = 0.95;  // A star 简化阈值
    bool is_takeoff = false;
    bool is_init = false;
    bool is_new_tarjectory_generate = false;
    bool is_last_trajectory_finish = true;
    bool is_start = false;
    bool is_collision_detect = false;
    bool is_finish_mini_uav_task = false;
    double error_threshold = 3;  // 目标点到达阈值
    SearchPoint last_delete_frontier_point;
    bool has_delete_last = false;
    // 离无人机起始点最近的search point
    SearchPoint closest_search_point;
    // 起飞点
    Eigen::Vector3d start_pos;
    std::string name;
    bool is_octomap_occupy_init = false;
    int range; // 地图膨胀范围
    std::mutex mtx;
    Eigen::Vector3d partner_pos;
    int expand_size;
    int uav_id;
    std::vector<nav_msgs::Odometry> swarm_odom;
    std::vector<SearchPoint> frontier_points_copy;
    int dis_threshold = 6; //飞机能照到兴趣点的最远距离
    int step = 3;
    int inspect_threshold = 6; //at least inspect times
    int curr_seg = 0;
    int count_add = 0;
    ros::Time start_t; //每条轨迹段的起始时间
    ros::Time last_t;
    Eigen::Vector3d gimbal_target;
    int count_isolate=0;
    int modify_search_points_count = 0;
    std::vector<Eigen::Vector3d> interest_points;
    int update_interest_point_count = 0; 
    double min_height=1.9;
    Eigen::Vector3d search_point_dis; //searchpoint间隔真实距离

    // 构造函数
    Inspector(std::string name,int x_length, int y_length, int z_length){
        // this->start_pos = start_pos;
        this->name=name;
        if(name=="jurong") uav_id=0;
        else if(name=="raffles") uav_id=1;
        this->search_points_x_length=x_length;
        this->search_points_y_length=y_length;
        this->search_points_z_length=z_length;
        std::cout<<search_points_x_length<<search_points_y_length<<search_points_z_length<<std::endl;
        this->swarm_odom=std::vector<nav_msgs::Odometry>(5);
        for(int i=0;i<5;i++){
            swarm_odom[i].twist.twist.linear.z=-999;
        }
    }

    //静态函数
    void init(double x_min,double x_max,double y_min,double y_max,double z_min,double z_max,
                int resolution,std::vector<Eigen::Vector3d> point_set)
                // double origin_x,double origin_y,double origin_z,)
    {
        grid_map=GridMap((x_max-x_min)/2+x_min, (y_max-y_min)/2+y_min, (z_max-z_min)/2+z_min, (x_max-x_min)+10, (y_max-y_min)+10, (z_max-z_min)+10, resolution);
                search_points=point_set;
        if(resolution==1){
            range=1;
            dis_threshold=8;
            step=2;
            error_threshold=2;
        }
        else if(resolution==2){
            range=1;
            dis_threshold=5; //8
            step=2;           //4
            error_threshold=3;
        }
        else {
            range=1;
            dis_threshold=5;
            step=2;
            error_threshold=4;
        }
        // dis_threshold=10;
        ROS_WARN("resolution:%d, dis_threshold %d",resolution, dis_threshold);
    }

    void init_map(std::vector<Eigen::Vector3d>& point_set) {
        // 根据 bounding box 的范围建立地图，设置 all points in bbox 的点为占据状态
        
        // set points in bbox
        for(auto point:point_set){
            grid_map.set_in_bbox_pos(point,true);
        }
        for(auto point:point_set){
            grid_map.set_occupancy_pos(point,true);
        }

        init_search_array();

        // for bad point init
        for(auto point:point_set){
            grid_map.set_occupancy_pos(point,false);
        }
        update_search_array();
        // now search points are around or in the bbox 

       
    } 

     void init_map_inbox(std::vector<Eigen::Vector3d>& point_set) {
        // 根据 bounding box 的范围建立地图，设置 all points in bbox 的点为占据状态
        
        // for bad point init
        int count=0;
        for(auto point:point_set){
            grid_map.set_occupancy_pos(point,true);
            count++;
        }
        //计算被占据的格点数量
        for(int i=0;i<grid_map.max_index(0);i++){
            for(int j=0;j<grid_map.max_index(1);j++){
                for(int k=0;k<grid_map.max_index(2);k++){
                    if(grid_map.grid_map[i][j][k].is_occupied)count++;
                }
            }
        }
        // std::cout<<"ininbox point num: "<<count<<std::endl;
        // init_search_array();

        // for bad point init
        // for(auto point:point_set){
        //     grid_map.set_occupancy_pos(point,false);
        // }
        // update_search_array();

        // int count=0;
        // for(int i=0;i<search_points_x_length;i++){
        //     for(int j=0;j<search_points_y_length;j++){
        //         for(int k=0;k<search_points_z_length;k++){
        //             if(!search_points_array[i][j][k].is_bad && !search_points_array[i][j][k].is_obstacle)count++;
        //         }
        //     }
        // }
        // std::cout<<count<<std::endl;

        
    } 


    void init_search_array(){
        search_points_array.resize(search_points_x_length, std::vector<std::vector<SearchPoint>>(search_points_y_length,std::vector<SearchPoint>(search_points_z_length)));
        // search vector -> search array
        int count=0, obnum=0;
        for(int i=0;i<search_points_x_length;i++){
            for(int j=0;j<search_points_y_length;j++){
                for(int k=0;k<search_points_z_length;k++){
                    // 考虑是否为障碍
                    if(grid_map.get_occupancy_pos(search_points[count]) || grid_map.get_flat_pos(search_points[count]) || search_points[count][2]<min_height)
                    {
                        search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,true);
                        obnum++;
                    }
                    else search_points_array[i][j][k] = SearchPoint(search_points[count],i,j,k,false);
                    count++;
                }
            }
        }
        search_points_array_copy = search_points_array;
        
        search_point_dis[0] = std::abs(search_points_array[0][0][0].pos[0]-search_points_array[1][0][0].pos[0]);
        search_point_dis[1] = std::abs(search_points_array[0][0][0].pos[1]-search_points_array[0][1][0].pos[1]);
        search_point_dis[2] = std::abs(search_points_array[0][0][0].pos[2]-search_points_array[0][0][1].pos[2]);
        
        modify_search_points();

    }

    // 更新占据信息
    void update_search_array(){
        for(int i=0;i<search_points_x_length;i++){
            for(int j=0;j<search_points_y_length;j++){
                for(int k=0;k<search_points_z_length;k++){
                    // 考虑是否为障碍
                    if(grid_map.get_occupancy_pos(search_points_array[i][j][k].pos) || grid_map.get_flat_pos(search_points_array[i][j][k].pos))
                        search_points_array[i][j][k].is_obstacle = true;
                    else search_points_array[i][j][k].is_obstacle = false;
                }
            }
        }
    }

    void modify_search_points(){
    // 考虑是否离障碍物太远 太远的排除
        // 在 +- search_point_dis/2 范围内利用探测点探测
        int detect_point = 3;
        for(int i=0;i<search_points_x_length;i++){
            for(int j=0;j<search_points_y_length;j++){
                for(int k=0;k<search_points_z_length;k++){
                    if(search_points_array[i][j][k].is_obstacle)continue;
                    bool neighbor = false;
                    int neighbor_num = 0;
                    // 遍历可能的相邻位置 
                    for (double dx = -search_point_dis[0]; dx <= search_point_dis[0] && !neighbor; dx+=2*search_point_dis[0]/detect_point){
                        for (double dy = -search_point_dis[1]; dy <= search_point_dis[1] && !neighbor; dy+=2*search_point_dis[1]/detect_point) {
                            for (double dz = -search_point_dis[2]; dz <= search_point_dis[2] && !neighbor; dz+=2*search_point_dis[2]/detect_point) {
                                double newX = search_points_array[i][j][k].pos.x() + dx;
                                double newY = search_points_array[i][j][k].pos.y() + dy;
                                double newZ = search_points_array[i][j][k].pos.z() + dz;
                                Eigen::Vector3d new_point = Eigen::Vector3d(newX,newY,newZ);
                                Eigen::Vector3i idx;
                                auto re=grid_map.pos2index(new_point,idx);
                                // 检查是否越界 
                                if (newX > grid_map.x_min && newX < grid_map.x_max && newY > grid_map.y_min && newY < grid_map.y_max && newZ > grid_map.z_min && newZ < grid_map.z_max) {
                                    if((grid_map.get_occupancy_pos(new_point) || grid_map.get_flat_pos(new_point)) && newZ > min_height && grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox){
                                        neighbor_num++;
                                        if(neighbor_num>=1)neighbor=true;
                                    }                     
                                }
                            }
                        }
                    }
                    search_points_array[i][j][k].is_bad=!neighbor;
                    if(search_points_array[i][j][k].is_bad){
                        continue;
                    }
                    else{
                        // TODO 有问题 会丢失非常多的点
                        // // 对于留下的距离障碍物近的点 再近一些 
                        if (modify_search_points_count % 5 == 0){
                            // ROS_INFO("(%.2f, %.2f, %.2f)",search_points_array[i][j][k].pos[0],search_points_array[i][j][k].pos[1],search_points_array[i][j][k].pos[2]);
                            search_points_array[i][j][k].pos = move_to_face_center(search_points_array_copy[i][j][k].pos);  
                            // ROS_INFO("-=-=-=-=-=-=-=-=-=-=-(%.2f, %.2f, %.2f)",search_points_array[i][j][k].pos[0],search_points_array[i][j][k].pos[1],search_points_array[i][j][k].pos[2]);
                        }
                    }
                }
            }
        }
        modify_search_points_count++;
           
    }

    void update_uninspected_points(){
     // 根据其他飞机（包含小飞机）检测情况更新
        if(update_interest_point_count++%1==0){
            for(auto point:interest_points){
                Eigen::Vector3i idx;
                auto re=grid_map.pos2index(point,idx);
                // 26
                int x_min=idx[0]-1;
                int x_max=idx[0]+1;
                int y_min=idx[1]-1;
                int y_max=idx[1]+1;
                int z_min=idx[2]-1;
                int z_max=idx[2]+1;

                for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i++){
                    for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j++){
                        for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k++){
                            if(grid_map.grid_map[i][j][k].need_to_inspect)
                            {
                                grid_map.grid_map[i][j][k].is_inspected = inspect_threshold ;
                            }
                        }
                    }
                }
                ROS_INFO("UPDATE!");
            }
                ROS_INFO("UPDATE!");

        }
    }

    void set_inpect_grid(std::vector<Eigen::Vector3d>& point_set){
        for(auto point:point_set){
            grid_map.set_need_to_inspect_pos(point,true);
        }
    }


    void gimbal_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {   
        if(is_debuging) ROS_INFO("gimbal_callback");
        try {
            gimbal_ori[0] = -msg->twist.linear.x;
            gimbal_ori[1] = -msg->twist.linear.y;
            gimbal_ori[2] = -msg->twist.linear.z;
        }
        catch (...) {
            std::cout << "[" + this->name + "]" << "gimbal_cb error !\t" << ros::Time::now() << std::endl;
        }
        if(is_debuging) ROS_INFO("gimbal_callback done =====");

    }

    // Helper function to compute path length
    double compute_length(const std::vector<waypoint*>& path){
        double length = 0;
        for (size_t i = 1; i < path.size(); ++i){
            Eigen::Vector3d pos1 = idx_to_pos(path[i - 1]->idx);
            Eigen::Vector3d pos2 = idx_to_pos(path[i]->idx);
            length += (pos2 - pos1).norm();
        }
        return length;
    }

    // Helper function to map grid indices to positions
    Eigen::Vector3d idx_to_pos(const Eigen::Vector3i& idx){
        double resolution = 1.0; // Define your grid resolution here
        return idx.cast<double>() * resolution;
    }

    // Helper function to compute distance from point to segment
    double point_to_segment_distance(const Eigen::Vector3d& point, const Eigen::Vector3d& seg_start, const Eigen::Vector3d& seg_end, double& t){
        Eigen::Vector3d d = seg_end - seg_start;
        Eigen::Vector3d pa = point - seg_start;
        double len_sq = d.squaredNorm();
        if (len_sq == 0){
            // Segment is a point
            t = 0;
            return pa.norm();
        } else {
            t = pa.dot(d) / len_sq;
            t = std::max(0.0, std::min(1.0, t));
            Eigen::Vector3d projection = seg_start + t * d;
            return (point - projection).norm();
        }
    }
    
    int gen_path_through_to(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, std::vector<Eigen::Vector3d> points_want_to_go,
                        double max_distance, double max_time, std::vector<waypoint*> &total_path){
        // Initialize start time
        ros::Time start_time = ros::Time::now();
        total_path.clear();

        // Generate initial path from start_pos to end_pos
        int re = gen_path_to(start_pos, end_pos, total_path);
        if (!re){
            // Cannot generate initial path
            return 0;
        }

        // Compute initial path length
        double path_length = compute_length(total_path);
        int inserted_points = 0;

        while(true){
            // Check for termination conditions
            if (path_length > max_distance || (ros::Time::now() - start_time).toSec() > max_time || points_want_to_go.empty()){
                break;
            }

            // Find the closest point and corresponding path segment
            double min_distance = std::numeric_limits<double>::max();
            Eigen::Vector3d closest_point;
            int closest_point_idx = -1;
            int closest_wp_idx = -1;

            for (int p_idx = 0; p_idx < points_want_to_go.size(); ++p_idx){
                Eigen::Vector3d point = points_want_to_go[p_idx];

                for (int wp_idx = 1; wp_idx < total_path.size(); ++wp_idx){
                    // Get positions of waypoints
                    Eigen::Vector3d wp_start_pos = idx_to_pos(total_path[wp_idx - 1]->idx);
                    Eigen::Vector3d wp_end_pos = idx_to_pos(total_path[wp_idx]->idx);

                    double t;
                    double distance = point_to_segment_distance(point, wp_start_pos, wp_end_pos, t);

                    if (distance < min_distance){
                        min_distance = distance;
                        closest_point = point;
                        closest_point_idx = p_idx;
                        closest_wp_idx = wp_idx;
                    }
                }
            }

            if (closest_point_idx == -1){
                // No more points to insert
                break;
            }

            // Plan path from start of segment to closest_point
            std::vector<waypoint*> path_to_point;
            Eigen::Vector3d seg_start_pos = idx_to_pos(total_path[closest_wp_idx - 1]->idx);
            re = gen_path_to(seg_start_pos, closest_point, path_to_point);
            if (!re){
                // Cannot plan path to point, remove point from consideration
                points_want_to_go.erase(points_want_to_go.begin() + closest_point_idx);
                continue;
            }

            // Plan path from closest_point to end of segment
            std::vector<waypoint*> path_from_point;
            Eigen::Vector3d seg_end_pos = idx_to_pos(total_path[closest_wp_idx]->idx);
            re = gen_path_to(closest_point, seg_end_pos, path_from_point);
            if (!re){
                // Cannot plan path from point to end, remove point from consideration
                points_want_to_go.erase(points_want_to_go.begin() + closest_point_idx);
                continue;
            }

            // Build new total_path
            std::vector<waypoint*> new_total_path;

            // Copy waypoints before the segment
            new_total_path.insert(new_total_path.end(), total_path.begin(), total_path.begin() + closest_wp_idx);

            // Append path_to_point, avoiding duplicates
            if (!path_to_point.empty()){
                size_t start_idx = 0;
                if (!new_total_path.empty() && new_total_path.back()->idx == path_to_point.front()->idx){
                    start_idx = 1;
                }
                new_total_path.insert(new_total_path.end(), path_to_point.begin() + start_idx, path_to_point.end());
            }

            // Append path_from_point, avoiding duplicates
            if (!path_from_point.empty()){
                size_t start_idx = 0;
                if (!new_total_path.empty() && new_total_path.back()->idx == path_from_point.front()->idx){
                    start_idx = 1;
                }
                new_total_path.insert(new_total_path.end(), path_from_point.begin() + start_idx, path_from_point.end());
            }

            // Append remaining waypoints after the segment
            new_total_path.insert(new_total_path.end(), total_path.begin() + closest_wp_idx + 1, total_path.end());

            // Replace total_path with new_total_path
            total_path = new_total_path;

            // Update path_length
            path_length = compute_length(total_path);

            // Remove the point from points_want_to_go
            points_want_to_go.erase(points_want_to_go.begin() + closest_point_idx);

            // Increment inserted points count
            ++inserted_points;
        }

        // Return the total number of inserted points
        return inserted_points;
    }



    int expend_path(std::vector<waypoint*> &total_path){
        // 从当前点开始，扩展search point
        // 直到超过时间，或者超过距离
        // 扩展原则为：
        // 设p0为开始点，pk为第k个扩展的点，P为所有备选点集合
        // pk* = max_{p\in P} (p0 \dot p) * heu(p)
        // 也就是尽可能沿着原来的方向，同时希望这个点的启发值更大
        // Initialize start time
        ros::Time start_time = ros::Time::now();
        // total_path.clear();
        double path_length = compute_length(total_path);
        int inserted_points = 0;
        double max_distance = 60;
        double max_time = 1.5;
        // 备份原来的path
        std::vector<waypoint*> total_path_copy = total_path;

        std::vector<SearchPoint> tmp_frontier_points = frontier_points;
        while(1){
            // Check for termination conditions
            if (compute_length(total_path) > max_distance || (ros::Time::now() - start_time).toSec() > max_time){
                break;
            }
            double max_J = std::numeric_limits<double>::min();
            Eigen::Vector3d max_point;
            Eigen::Vector3d last2_pos_now = grid_map.index2pos(total_path[total_path.size()-2]->idx);
            Eigen::Vector3d start_pos_now = grid_map.index2pos(total_path[total_path.size()-1]->idx);
            Eigen::Vector3d direction = (start_pos_now - last2_pos_now).normalized();
            int max_index = 0;
            for (int i=0;i<tmp_frontier_points.size();i++){
                
                Eigen::Vector3d point = tmp_frontier_points[i].pos;
                Eigen::Vector3d tmp_ori(0,0,0);
                double J = point.dot(direction) * get_heuristic(start_pos_now, tmp_ori, point);
                if (J > max_J){
                    max_J = J;
                    max_point = point;
                    max_index = i;
                }
            }

            if(max_J == std::numeric_limits<double>::min()){
                // 没有一个可以扩展的点
                break;
            }

            // 扩展path
            struct waypoint start_waypoint_now, end_waypoint_now;
            grid_map.pos2index(start_pos_now,start_waypoint_now.idx);
            grid_map.pos2index(max_point,end_waypoint_now.idx);

            std::vector<waypoint*> candidata_waypoint_path = AStar(end_waypoint_now,end_waypoint_now,grid_map);
            // 如果找不到一条合法的A*路径 换下一个目标
            if (candidata_waypoint_path.size() == 0){
                ROS_INFO("Cannot find a path to the frontier point----------");
                tmp_frontier_points.erase(tmp_frontier_points.begin() + max_index);
                continue;
            }
            // 可以找到，插入
            total_path.insert(total_path.end(), candidata_waypoint_path.begin(), candidata_waypoint_path.end());
            ROS_INFO("Insert a point to the path----------");
            
        }
        ROS_INFO("Expended with length: %f", compute_length(total_path));
        return 0;
        if (generate_trajetory(total_path)){
            return 1;
        }
        else{
            // 恢复path
            total_path = total_path_copy;
            return 0;
        }
    }

    int gen_path_to(const Eigen::Vector3d& start_pos, const Eigen::Vector3d& end_pos, std::vector<waypoint*> &path_gened){
        
    }

    // 读取 odometry 数据 更新 inspector 的位置和姿态信息
    // 并判断是否到达 target point
    void odometry_callback(const nav_msgs::Odometry::ConstPtr& odom){
        ros::Time t = ros::Time::now();
        if(is_debuging) ROS_INFO("odometry_callback");
        try {
            update_state(*odom);
            update_map();
            update_search_array(); //更新search point是否被占用

            
            swarm_odom[uav_id].child_frame_id = odom->child_frame_id;
            swarm_odom[uav_id].header = odom->header;
            swarm_odom[uav_id].pose = odom->pose;
            swarm_odom[uav_id].twist = odom->twist;
            caric_competition_xmu::swarm_odom_pub.publish(caric_competition_xmu::swarm_odom2msg(swarm_odom));
            // ROS_WARN("[%d] %f %f %f",uav_id,swarm_odom[uav_id].pose.pose.position.x,swarm_odom[uav_id].pose.pose.position.y,swarm_odom[uav_id].pose.pose.position.z);

            if(!is_takeoff) return;
            
            if(!is_init){ 
            // 起飞到正上空的一个较高的无碰撞点
                
                // Eigen::Vector3d tmp =Eigen::Vector3d(pos[0],pos[1],grid_map.z_max);
                // std::vector<Eigen::Vector3i>  tmp_points= ray_casting(pos,tmp);
                // if(tmp_points.size()==0) target_point = get_closest_search_point(tmp);
                // else{
                    //     Eigen::Vector3d tmp_target = Eigen::Vector3d(grid_map.index2pos(tmp_points[0])[0],grid_map.index2pos(tmp_points[0])[1],grid_map.index2pos(tmp_points[0])[2] - 5);
                    //     // ROS_WARN("(%f,%f,%f)",tmp_target[0],tmp_target[1],tmp_target[2]);
                    //     target_point = get_closest_search_point(tmp_target);
                // }
                target_point = get_closest_search_point(pos);


                ROS_INFO("[%s] First Target Point:(%f,%f,%f)",name.c_str(),target_point.pos[0],target_point.pos[1],target_point.pos[2]);
                struct waypoint start_waypoint, end_waypoint;
                grid_map.pos2index(pos,start_waypoint.idx);
                grid_map.pos2index(target_point.pos,end_waypoint.idx);
                waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                generate_trajetory(waypoint_path);
                is_init = true; 
                last_t = ros::Time::now();
            }
            else{
                // 避障
                if(is_collision_detect){ 
                    try{
                        if(has_delete_last){
                            frontier_points.push_back(last_delete_frontier_point);  // 因为没有到达目标 把之前删掉的边界点加回去
                            last_delete_frontier_point.is_visited=false;
                            has_delete_last=false;
                        }

                        struct waypoint start_waypoint, end_waypoint;
                        grid_map.pos2index(pos,start_waypoint.idx);
                        // first go to the closest point
                        closest_search_point = get_closest_search_point(pos);
                        grid_map.pos2index(closest_search_point.pos,end_waypoint.idx);
                        waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                        // if no way, back to the last point
                        if(waypoint_path.size()==0){
                            grid_map.pos2index(target_point.pos,end_waypoint.idx);
                            waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                        }
                        else target_point = closest_search_point;
                        // ROS_INFO("[%s] Collision Avoid : Next Target Point:(%f,%f,%f)",name.c_str(),target_point.pos[0],target_point.pos[1],target_point.pos[2]);        
                        generate_trajetory(waypoint_path);
                    }catch(...){}
                    // TODO 这里不建议？
                    is_collision_detect = false;
                }
                else{
                    double error =  std::pow(
                            std::pow(odom->pose.pose.position.x-target_point.pos[0],2)+
                            std::pow(odom->pose.pose.position.y-target_point.pos[1],2)+
                            std::pow(odom->pose.pose.position.z-target_point.pos[2],2)
                        ,0.5);
                    if(error < error_threshold && is_last_trajectory_finish){
                        modify_search_points();
                        // update_uninspected_points();
                        update_frontier_points(target_point.x,target_point.y,target_point.z);
                        struct waypoint start_waypoint, end_waypoint;
                        grid_map.pos2index(target_point.pos,start_waypoint.idx); 
                        std::priority_queue<SearchPoint, std::vector<SearchPoint>, CompareSearchPoint> pq;
                        pq=get_target_point(frontier_points, pos, ori); //优先队列 按优先级依次去找安全的目标
                                            
                        
                        //判断是否是空的
                        if(pq.size()==0){
                            // ROS_WARN("[%s] no target point",name.c_str());
                            return;
                        }
                        target_point = pq.top();
                        pq.pop();
                        grid_map.pos2index(target_point.pos,end_waypoint.idx);
                        waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                        // expend_path(waypoint_path);
                        // 如果找不到一条合法的A*路径 换下一个目标
                        while(waypoint_path.size()==0 && pq.size()>0){
                            target_point = pq.top();
                            pq.pop();
                            grid_map.pos2index(target_point.pos,end_waypoint.idx);
                            waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);    
                        }
                        // 如果所有目标都找不到一条合法的A*路径 换之前删去的启发值很低的边界点
                        if(waypoint_path.size()==0 && pq.size()==0){
                            while(waypoint_path.size()==0 && frontier_points_copy.size()>0){
                                target_point = frontier_points_copy[0];
                                frontier_points_copy.erase(frontier_points_copy.begin()+0);
                                grid_map.pos2index(target_point.pos,end_waypoint.idx);
                                waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);    
                            }
                        }
                        for(auto wp:waypoint_path){
                            const Eigen::Vector3i tmp_idx = wp->idx;
                            grid_map.pos2index(move_to_face_center(grid_map.index2pos(tmp_idx)), wp->idx);
                        }
                        // 极端情况 原始A*合法但是全部A*点作为trajetory仍然不合法 但其实分辨率1时A*已经非常紧密了
                        // 换次优目标点
                        while(!generate_trajetory(waypoint_path) && pq.size()>0){
                            target_point = pq.top();
                            pq.pop();
                            grid_map.pos2index(target_point.pos,end_waypoint.idx);
                            waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                        }
                        // 如果所有目标都找不到一条合法的trajetory 换之前删去的启发值很低的边界点
                        if(!generate_trajetory(waypoint_path) && pq.size()==0){
                            while(!generate_trajetory(waypoint_path) && frontier_points_copy.size()>0){
                                target_point = frontier_points_copy[0];
                                frontier_points_copy.erase(frontier_points_copy.begin()+0);
                                grid_map.pos2index(target_point.pos,end_waypoint.idx);
                                waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);    
                            }
                        }
                        // 删掉边界点
                        for(int i=0;i<frontier_points.size();i++){
                            if(frontier_points[i].pos==target_point.pos){
                                frontier_points.erase(frontier_points.begin() + i);
                                break;
                            }
                        }
                        // 没救了 估计是lidar错误把无人机当前位置当作障碍了
                        if(pq.size()==0  && waypoint_path.size()==0){
                            // if(uav_id==0) curr_mini_id=-1;
                            // ROS_WARN("[%s] error, Lidar Treats the UAVs as error",name.c_str());  // added
                            // target_point = get_closest_search_point(pos);
                            // grid_map.pos2index(target_point.pos,end_waypoint.idx);
                            // waypoint_path = AStar(start_waypoint,end_waypoint,grid_map);
                            Eigen::Vector3d tmp =Eigen::Vector3d(pos[0],pos[1],grid_map.z_max);
                            std::vector<Eigen::Vector3i>  tmp_points= ray_casting(pos,tmp);
                            if(tmp_points.size()!=0){
                                Eigen::Vector3d tmp_target = Eigen::Vector3d(grid_map.index2pos(tmp_points[0])[0],grid_map.index2pos(tmp_points[0])[1],grid_map.index2pos(tmp_points[0])[2] - 5);
                                // ROS_WARN("(%f,%f,%f)",tmp_target[0],tmp_target[1],tmp_target[2]);
                                target_point = get_closest_search_point(tmp_target);
                            }
                            else{
                                target_point = get_closest_search_point(pos);                          
                            }      
                        } 
                            
                        last_delete_frontier_point = target_point;
                        has_delete_last=true;
                        if(target_point.is_visited);
                        else{
                            target_point.is_visited=true;
                            // ROS_INFO("[%s] Next Target Point:(%f,%f,%f)",name.c_str(),target_point.pos[0],target_point.pos[1],target_point.pos[2]); 
                        }
                    }
                }
            }
        
        if(frontier_points.size()<10){
            add_new_frontier_points();
        }
        // ROS_WARN("%f",(ros::Time::now()-t).toSec());
        }
        catch (...) {
            std::cout << "[" + this->name + "]" << "odom_cb error !\t" << ros::Time::now() << std::endl;
        }
        if(is_debuging) ROS_INFO("odometry_callback done=====");
        
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
    std::vector<waypoint *> douglasPeucker(std::vector<waypoint *> points, double epsilon) {
        std::vector<waypoint *> simplified;

        if (points.size() < 3) {
            return points;
        }

        // 寻找最远点
        double maxDistance = 0.0;
        size_t maxIndex = 0;

        waypoint * start = points.front();
        waypoint * end = points.back();

        for (size_t i = 1; i < points.size() - 1; ++i) {
            double d = pointToLineDistance(grid_map.index2pos(start->idx), grid_map.index2pos(end->idx), grid_map.index2pos(points[i]->idx));
            if (d > maxDistance) {
                maxDistance = d;
                maxIndex = i;
            }
        }

        // 如果最大距离大于阈值，则保留最远点，否则继续递归简化
        if (maxDistance > epsilon) {
            std::vector<waypoint *> firstPart(points.begin(), points.begin() + maxIndex + 1);
            std::vector<waypoint *> secondPart(points.begin() + maxIndex, points.end());
            std::vector<waypoint *> simplifiedFirst, simplifiedSecond;

            simplifiedFirst = douglasPeucker(firstPart, epsilon);
            simplifiedSecond = douglasPeucker(secondPart, epsilon);

            simplified.insert(simplified.end(), simplifiedFirst.begin(), simplifiedFirst.end() - 1);
            simplified.insert(simplified.end(), simplifiedSecond.begin(), simplifiedSecond.end());
        } else {
            simplified.clear();
            simplified.push_back(start);
            simplified.push_back(end);
        }
        return simplified;
    }

    bool generate_trajetory(std::vector<waypoint *> waypoints){
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
            simplified_waypoint_path = douglasPeucker(waypoints,epsilon);
        }catch (const std::invalid_argument& e) {
            // std::cerr << "Error: " << e.what() << std::endl;
        }
        // // 两个点的直线snap也能跑歪 
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
        trajGeneration(waypoint_path_matrix,_polyCoeff,_polyTime);
        std::vector<waypoint *> correct_waypoint_path = correct_trajectory();
        // 如果traj不合法 尝试去对精简后的A*路点集添加原来的路点来使traj更贴近原始A*路点的轨迹
        while(correct_waypoint_path.size()!=0){
            // 即便是原始A*路点也不行 那就放弃这个目标
            if(simplified_waypoint_path.size()==correct_waypoint_path.size()){
                // ROS_INFO("[%s] can not find safe trajectory",name.c_str());  
                return false;
            }
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

        // TODO 不一定可以在这里执行
        _polyCoeff_vel = calculate_polycoeff_de(_polyCoeff);
        _polyCoeff_acc = calculate_polycoeff_de(_polyCoeff_vel);
        is_new_tarjectory_generate = true;
        return true;
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
                if(grid_map.get_occupancy_pos(unsafe_point) || grid_map.get_flat_pos(unsafe_point)){
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
                        if(waypoint->idx == add_point->idx) added_path.push_back(waypoint);
                            
                    }
                    return added_path;
                }
            }
        }
        return added_path;
    }

    void update_state(nav_msgs::Odometry odom){
        // 更新位置和姿态信息
        pos[0]=odom.pose.pose.position.x;
        pos[1]=odom.pose.pose.position.y;
        pos[2]=odom.pose.pose.position.z;

        tf::Quaternion quat;
        double roll, pitch, yaw;//定义存储r\p\y的容器
        // tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
        // tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        Eigen::Quaterniond q(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        // q.normalize();
        ori=Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()).toRotationMatrix().eulerAngles(2,1,0);
        if (std::abs(ori[1]) > 1.57 || std::abs(ori[2]) > 1.57){
            // 明显反了
            ori[0] = ori[0] - M_PI;
            // roll 和 pitch我反应不过来，先不管吧！！！
            // TODO 重要！！！
            // ori[1] = -ori[1];

        }
        // ROS_INFO("============[%s] pos:(%f,%f,%f) ori:(%f,%f,%f)",name.c_str(),pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]);
        // ori[0] = -roll;
        // ori[1] = -pitch;
        // ori[2] = -yaw;               // Added TODO 这个注意一下, 记得确认

        linear_vel = Eigen::Vector3d(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
        angular_vel = Eigen::Vector3d(odom.twist.twist.angular.x,odom.twist.twist.angular.y,odom.twist.twist.angular.z);
    }

    // 找到_pos最近的一个search point
    SearchPoint get_closest_search_point(Eigen::Vector3d _pos){
        SearchPoint tmp_point;
        double distance = 100000000;
        for(int i=0;i<search_points_x_length;i++){
            for(int j=0;j<search_points_y_length;j++){
                for(int k=0;k<search_points_z_length;k++){
                    // 考虑是否为障碍
                    if(!grid_map.get_occupancy_pos(search_points_array[i][j][k].pos) && !grid_map.get_flat_pos(search_points_array[i][j][k].pos)){
                        if(search_points_array[i][j][k].is_obstacle || search_points_array[i][j][k].pos[2] < min_height) continue;
                        // 保险起见 找一个直线无碰撞的点
                        std::vector<Eigen::Vector3i>  tmp_points= ray_casting(_pos,search_points_array[i][j][k].pos);
                        if(tmp_points.size()!=0) continue;
                        double tmp = std::pow(std::pow(_pos[0]-search_points_array[i][j][k].pos[0],2)+
                                    std::pow(_pos[1]-search_points_array[i][j][k].pos[1],2)+
                                    std::pow(_pos[2]-search_points_array[i][j][k].pos[2],2),0.5);
                        if(tmp<distance){
                            distance=tmp;
                            tmp_point = search_points_array[i][j][k];
                        }
                    }
                }
            }
        }
        return tmp_point;
    }

        std::vector<Eigen::Vector3i> ray_casting(Eigen::Vector3d& position,Eigen::Vector3d& target_position, bool ignore_flat = false){
        // 判断两点之间是否存在遮挡
        // 先来简单版本，不考虑姿态
        std::vector<Eigen::Vector3i> idxs;
        int num_slides=((target_position-position).norm()/grid_map.resolution+1);
        Eigen::Vector3d middle_position;
        for(double a=1.0/num_slides;a<1.0-1.0/num_slides;a+=1.0/num_slides){
            middle_position=position+a*(target_position-position);
            Eigen::Vector3i idx;
            auto re=grid_map.pos2index(middle_position,idx);
            if(!re)//如果索引不合法,false(默认空)
            {
                continue;
            }
            if(grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_occupied || (grid_map.grid_map[idx[0]][idx[1]][idx[2]].in_flat && !ignore_flat))
            {
                idxs.push_back(idx);
            }
        }
        return idxs;
    }

    bool have_collision(Eigen::Vector3d& position,Eigen::Vector3d& target_position){
        auto re=ray_casting(position,target_position,true);
        if(re.size()>0)
            return true;
        else
            return false;
        // return false; //暂时不考虑遮挡
    }


    // 获取能被检测到的未被检测过的点的索引
    // step 跳点 dis_thres 距离阈值
    std::vector<Eigen::Vector3i> get_covered_points(Eigen::Vector3d& position, Eigen::Vector3d& orientation) {
        // 根据当前的位置和姿态获取能被检测到的未被检测过的点
        //先来简单版本，不考虑姿态
        Eigen::Vector3i idx;
        auto re=grid_map.pos2index(position,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-dis_threshold * 1.5;//这里改成10但是下面step从1改成2 因为其实不需要精确的知道点的数量 只要有一个大概的数量就行
        int x_max=idx[0]+dis_threshold * 1.5;
        int y_min=idx[1]-dis_threshold * 1.5;
        int y_max=idx[1]+dis_threshold * 1.5;
        int z_min=idx[2]-dis_threshold * 1.5;
        int z_max=idx[2]+dis_threshold * 1.5;
        std::vector<Eigen::Vector3i> idxs;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i+=step){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j+=step){
                for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k+=step){
                    auto target_position=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                    if(!have_collision(position,target_position))
                    {
                        if(grid_map.grid_map[i][j][k].need_to_inspect==true&&grid_map.grid_map[i][j][k].is_inspected<inspect_threshold)
                        {
                            Eigen::Vector3i idx(i,j,k);
                            idxs.push_back(idx);
                        }
                    }
                }
            }
        }
        
        return idxs;
    }
    
    inline double my_norm_w_alt(Eigen::Vector3d pos_1, Eigen::Vector3d pos_2) {
        double cost_factor_xy = 1;
        double cost_factor_z  = 0.25;           // 可以调整

        if (pos_1[2] >= pos_2[2]) {
            cost_factor_z = 0.65;               // default: 0.25 可以是负的, 但是设计成负值可能导致一直往高了飞
        } else {
            cost_factor_z = 0.65;               // default: 1.25
        }

        double dx = pos_1[0] - pos_2[0];
        double dy = pos_1[1] - pos_2[1];
        double dz = pos_1[2] - pos_2[2];

        return std::sqrt(cost_factor_xy * (dx * dx + dy * dy) + cost_factor_z * dz * dz);    
    }

    // 获取启发式值
    double get_heuristic(Eigen::Vector3d& position, Eigen::Vector3d& orientation, Eigen::Vector3d new_point) {
        // 遍历关键点，综合考虑距离和新观测量，获取前往的目标点
        std::vector<Eigen::Vector3i> idxs = get_covered_points(new_point,orientation);
        int num_covered = 0;
        if(!idxs.empty()) num_covered=idxs.size();

        double distance;
        //distance = (new_point-position).norm();
        distance = my_norm_w_alt(new_point, position);      // 注意这个顺序不能反

        Eigen::Vector3d parter_realtive_pos_vec = (partner_pos - position);
        Eigen::Vector3d new_point_relative_vec = new_point - position;
        double dotProduct = parter_realtive_pos_vec.dot(new_point_relative_vec);  // 约小越好

        // 有限考虑前往未起飞的飞机的上方
        double heu_goto_untakeoff = 0.0;
        for (int i = 2; i < 5; i++) {
            if (swarm_odom[i].pose.pose.position.z < 1.5 && swarm_odom[i].twist.twist.linear.z > -900) {
                heu_goto_untakeoff += 1.0 / (std::pow(swarm_odom[i].pose.pose.position.x - new_point[0], 2) 
                                        + std::pow(swarm_odom[i].pose.pose.position.y - new_point[1], 2)
                                        + 1 
                                        + std::pow(new_point[2] / 5.0, 2));
            }
        }

        // if(num_covered==0) return - INT16_MAX;
        if(num_covered==0) return 0.;
        else if(partner_pos.norm()==0){
            return (
                (  std::min(num_covered, 100)) 
            + 50 * sqrt(1.0 / (distance + 1))
            + 25 * heu_goto_untakeoff
            + 10  *  new_point[2] / 100.0
            + std::min(std::abs(position[2] - new_point[2]), 5.0)
            + 80
            );
        }
        else{
            return (
                (  std::min(num_covered, 100)) 
                // ( 80 * -1.0 / (num_covered + 1)) 
                + 50 * sqrt(1.0 / (distance + 1))
                + 150 * (-1.0 / (std::sqrt((partner_pos - new_point).norm())/5. + 1))
                + 150 * heu_goto_untakeoff
                + 15  *  new_point[2] / 100.0
                + std::min(std::abs(position[2] - new_point[2]), 5.0)
                + 380 //补偿到非负
                
                );

        }
    }

    struct CompareSearchPoint {
        bool operator()(const SearchPoint& point1, const SearchPoint& point2) {
            return point1.heuristic < point2.heuristic;
        }
    };

    std::vector<visualization_msgs::Marker> text_markers;

    void push_heu_marker(Eigen::Vector3d pos, double heu) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "heu";
        marker.id = text_markers.size();
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos[0];
        marker.pose.position.y = pos[1];
        marker.pose.position.z = pos[2] + 0.65;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 1.8;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << heu;
        marker.text = ss.str();
        text_markers.push_back(marker);
        // ROS_INFO("PUSHED one point")
    }

    std::priority_queue<SearchPoint, std::vector<SearchPoint>, CompareSearchPoint> get_target_point(std::vector<SearchPoint>& frontier_points, Eigen::Vector3d curr_pos, Eigen::Vector3d curr_ori) {
        std::priority_queue<SearchPoint, std::vector<SearchPoint>, CompareSearchPoint> pq;
        // frontier_points_copy.clear();
        time_t t0 = time(nullptr);
        // 将所有前沿点加入优先队列
        text_markers.clear();
        for (int i=0;i<frontier_points.size();i++) {
            if(frontier_points[i].is_obstacle || grid_map.get_occupancy_pos(frontier_points[i].pos) || grid_map.get_flat_pos(frontier_points[i].pos) || frontier_points[i].pos[2]<min_height){
                frontier_points[i].is_obstacle=true;
                frontier_points.erase(frontier_points.begin()+i); 
                continue;
            }
            // another explorer set curr target
            if(frontier_points[i].is_visited){
                continue;
            }
            frontier_points[i].heuristic = get_heuristic(curr_pos, curr_ori, frontier_points[i].pos);
            // 如果边界点周围没有可检测的点，删除它，并将其加入到备选边界点中
            if(frontier_points[i].heuristic == -INT16_MAX && frontier_points.size()>10){ 
                frontier_points_copy.push_back(frontier_points[i]);
                frontier_points[i].is_in_frontier=false;
                frontier_points.erase(frontier_points.begin()+i);         
                continue;
            }
            push_heu_marker(frontier_points[i].pos, frontier_points[i].heuristic);
            pq.push(frontier_points[i]);
            if(time(nullptr)-t0>5) break;
        }
        if(pq.size()==0){
            pq.push(frontier_points[0]);
        }

        // // 获取优先队列中的顶部元素，即具有最大启发式值的点
        // SearchPoint target_point = pq.top();

        // // 如果不是预测，记录上一次删除的点
        // if (!is_predict) {
        //     last_delete_frontier_point = target_point;
        // }

        // // 从前沿点中移除该点
        // frontier_points.erase(std::remove(frontier_points.begin(), frontier_points.end(), target_point), frontier_points.end());

        return pq;
    }

    // 防止frontier_points走完没事干 定期添加合法的search_points到frontier_points 因为会清理周围没有兴趣点的frontier_points 所以不会冗余
    void add_new_frontier_points(){
        int add_num = 0;
        int max_add = 10; //防止花费太多时间
        for(int i=0;i<search_points_x_length && add_num<=max_add;i++){
            for(int j=0;j<search_points_y_length && add_num<=max_add;j++){
                for(int k=0;k<search_points_z_length && add_num<=max_add;k++){
                    // if(!search_points_array[i][j][k].is_obstacle && !search_points_array[i][j][k].is_bad && !search_points_array[i][j][k].is_in_frontier && !search_points_array[i][j][k].is_visited) 
                    if(!search_points_array[i][j][k].is_obstacle && !search_points_array[i][j][k].is_bad && search_points_array[i][j][k].pos[2] > min_height) //
                    {
                        if(grid_map.get_occupancy_pos(search_points_array[i][j][k].pos) || grid_map.get_flat_pos(search_points_array[i][j][k].pos)){
                            search_points_array[i][j][k].is_obstacle=true;
                            continue;
                        }
                        bool is_same = false;
                        for(auto point:frontier_points){
                            if(point.pos == search_points_array[i][j][k].pos){
                                is_same = true;
                                break;
                            }
                        }
                        if(is_same) continue;
                        search_points_array[i][j][k].is_in_frontier=true;
                        // int num_covered = 0;
                        // std::vector<Eigen::Vector3i> idxs = get_covered_points(search_points_array[i][j][k].pos,ori);
                        // if(!idxs.empty()) num_covered=idxs.size();
                        // if (num_covered==0) continue;
                        frontier_points.push_back(search_points_array[i][j][k]);  
                        add_num++;
                    }
                    // ROS_INFO("%d",c++);

                }
            }
        }

    }

    // 计算某个点附近的所有障碍物的重心 只考虑在bbox内 且不是地面
    Eigen::Vector3d get_obstacle_center(Eigen::Vector3d& position) {
        int search_points_x_length = 12;
        int search_points_y_length = 16;
        int search_points_z_length = 8;
        Eigen::Vector3d center(0, 0, 0);
        int num = 0;
        int detect_point = 3;
        // 遍历可能的相邻位置 
        for (double dx = -search_point_dis[0]; dx <= search_point_dis[0] ; dx+=2*search_point_dis[0]/detect_point){
            for (double dy = -search_point_dis[1]; dy <= search_point_dis[1] ; dy+=2*search_point_dis[1]/detect_point) {
                for (double dz = -search_point_dis[2]; dz <= search_point_dis[2] ; dz+=2*search_point_dis[2]/detect_point) {
                    Eigen::Vector3d pos = position + Eigen::Vector3d(dx, dy, dz);
                    Eigen::Vector3i idx;
                    auto re=grid_map.pos2index(pos,idx);
                    // std::cout << grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox << std::endl;
                    // if (grid_map.get_occupancy_pos(pos)) {
                    if ((grid_map.get_flat_pos(pos) || grid_map.get_occupancy_pos(pos)) && pos[2]>min_height && grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox) { //&& grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox
                        
                        center += pos;
                        num++;
                    }
                }
            }
        }
        if (num == 0) {
            return Eigen::Vector3d(INT32_MIN, INT32_MIN, INT32_MIN);
        }
        return center / num;
    }

        // 计算某个点附近的所有障碍物的重心
    Eigen::Vector3d get_face_center(const Eigen::Vector3d& position) {
        int search_points_x_length = 13;
        int search_points_y_length = 13;
        int search_points_z_length = 8;
        Eigen::Vector3d center(0, 0, 0);
        int num = 0;
        for (int i = -search_points_x_length; i < search_points_x_length; i++) {
            for (int j = -search_points_y_length; j < search_points_y_length; j++) {
                for (int k = -search_points_z_length; k < search_points_z_length; k++) {
                    Eigen::Vector3d pos = position + Eigen::Vector3d(i, j, k);
                    if (pos[2] < min_height) continue;
                    // if (grid_map.get_occupancy_pos(pos)) {
                    if (grid_map.get_face_pos(pos)) {
                        
                        center += pos;
                        num++;
                        if (num > 30){
                            // 找到了足够的face点
                            return center / num;
                        }
                    }
                }
            }
        }
        if (num == 0) {
            return Eigen::Vector3d(INT32_MIN, INT32_MIN, INT32_MIN);
        }
        return center / num;
    }

    // 从某个点开始，向障碍物移动，首先计算附近的障碍物的重心，然后向重心移动，并且不能碰到障碍物（保持一定距离）
    Eigen::Vector3d move_to_obstacle_center(Eigen::Vector3d& position) {
        try{
            Eigen::Vector3d center = get_obstacle_center(position);
            if (center[0] == INT32_MIN) {
                return position;
            }
            Eigen::Vector3d direction = center - position;
            float distance = direction.norm();
            direction.normalize();
            float candidate_ratio[] = {
                1, 0.7, 0.5, 0.3, 0.1,  -0.1
            };
            Eigen::Vector3d new_position = position;
            Eigen::Vector3d candidate_position;
            for (int c = 0; c < 6; c++) {
                // 向前移动
                candidate_position = position + direction * candidate_ratio[c] * distance;
                if(candidate_position[2]<min_height) continue;
                bool is_collision = false;
                // 对这个点附近进行检查，检查范围为2*2*2，如果存在障碍物，则尝试下一个点
                for(int i = -1; i <=1 && !is_collision; i++ ){
                    for(int j = -1; j <=1 && !is_collision; j++ ){
                        for(int k = -1; k <=1 && !is_collision; k++ ){
                            Eigen::Vector3d pos = candidate_position + Eigen::Vector3d(i, j, k);
                            if (grid_map.get_occupancy_pos(pos) || grid_map.get_flat_pos(pos)) {
                                is_collision = true;
                            }
                        }
                    }
                }
                if (!is_collision) {
                    new_position = candidate_position;
                    // ROS_INFO("move to obstacle center from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f), center:(%.2f, %.3f, %.3f)",
                    //  position[0], position[1], position[2], 
                    //  new_position[0], new_position[1], new_position[2],
                    //  center[0], center[1], center[2]);
                    break;
                }
            }
            return new_position;
        }
        catch(...){
            std::cout<<"move_to_obstacle_center error"<<std::endl;
        }
    }

    Eigen::Vector3d move_to_face_center(const Eigen::Vector3d& position) {
        Eigen::Vector3d center = get_face_center(position);
        if (center[0] == INT32_MIN) {
            return position;
        }
        Eigen::Vector3d direction = center - position;
        float distance = direction.norm();
        direction.normalize();
        double candidate_ratio[] = {
        //    0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.1, -0.1,  1.4, 1.2, 1.0,
        //    -0.1, 0,0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2
          0., 1., 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
        };
        Eigen::Vector3d new_position = position;
        Eigen::Vector3d candidate_position;
        for (int c = 0; c < 15; c++) {
            // 向前移动
            candidate_position = position + direction * candidate_ratio[c];
            bool is_collision = false;
            // 对这个点附近进行检查，检查范围为2*2*2，如果存在障碍物，则尝试下一个点
            float dect_r = std::max(std::min(candidate_ratio[c], 4.8), 2.5);
            for(float i = -dect_r; i <=dect_r; i+=1.0 ){
                for(float j = -dect_r; j <=dect_r; j+=1.0 ){
                    for(float k = -dect_r; k <=dect_r; k+=1.0 ){
                        Eigen::Vector3d pos = candidate_position + Eigen::Vector3d(i, j, k);
                        if (pos[2] < 1.8) continue;
                        if (grid_map.get_occupancy_pos(pos)) {
                        // if (grid_map.get_face_pos(pos)) {
                            is_collision = true;
                        }
                    }
                }
            }
            if (!is_collision) {
                new_position = candidate_position;
                // ROS_INFO("move to obstacle center from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f), center:(%.2f, %.3f, %.3f)",
                //  position[0], position[1], position[2], 
                //  new_position[0], new_position[1], new_position[2],
                //  center[0], center[1], center[2]);
                // break;
                continue;
            }
            else break;
        }
        return new_position;
    }

    // 更新边界点集合
    void update_frontier_points(int x,int y, int z){
        if (x < 0 || x >= search_points_x_length || y < 0 || y >= search_points_y_length || z < 0 || z >= search_points_z_length) {
            return;
        }
        search_points_array[x][y][z].is_visited = true;
        search_points_array[x][y][z].is_in_frontier = false;

        // 遍历可能的相邻位置 添加当前位置邻域的点
        int range = 3;
        for (int dx = -range; dx <= range; dx++) {
            for (int dy = -range; dy <= range; dy++) {
                for (int dz = -range; dz <= range; dz++) {
                    int newX = x + dx;
                    int newY = y + dy;
                    int newZ = z + dz;

                    // 检查是否越界
                    if (newX >= 0 && newX < search_points_x_length && newY >= 0 && newY < search_points_y_length && newZ >= 0 && newZ < search_points_z_length) {
                        // 如果相邻点不是障碍 or bad point、边界点以及没有被访问，将其加入边界队列
                        // if (!search_points_array[newX][newY][newZ].is_in_frontier && !search_points_array[newX][newY][newZ].is_obstacle && !search_points_array[newX][newY][newZ].is_bad && !search_points_array[newX][newY][newZ].is_visited) {
                        if (!search_points_array[newX][newY][newZ].is_obstacle && !search_points_array[newX][newY][newZ].is_bad && !search_points_array[newX][newY][newZ].is_visited) {       
                            bool is_same = false;
                            for(auto point:frontier_points){
                                if(point.pos == search_points_array[newX][newY][newZ].pos){
                                    is_same = true;
                                    break;
                                }
                            }
                            if(is_same) continue;
                            // search_points_array[newX][newY][newZ].pos = move_to_obstacle_center(search_points_array[newX][newY][newZ].pos);
                            frontier_points.push_back(search_points_array[newX][newY][newZ]); 
                        }
                    }
                }
            }
        }

        // 同步位置 更新为已经靠近的search points
        for(int i=0;i<frontier_points.size();i++){
            frontier_points[i].pos = search_points_array[frontier_points[i].x][frontier_points[i].y][frontier_points[i].z].pos;
        }
    }


    // 比较两个节点的总代价
    struct CompareWaypoint {
        bool operator()(const waypoint* a, const waypoint* b) const {
            return (a->cost + a->heuristic) > (b->cost + b->heuristic);
        }
    };

    std::vector<waypoint*> AStar(waypoint& start, waypoint& goal, GridMap map) {
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
                if (nextX < 0 || nextX >= x_length || nextY < 0 || nextY >= y_length || nextZ < 0 || nextZ >= z_length || map.grid_map[nextX][nextY][nextZ].is_occupied || map.grid_map[nextX][nextY][nextZ].in_flat)
                    continue;

                // 如果节点已经访问过 or added，跳过
                if (visited[nextX][nextY][nextZ] || added[nextX][nextY][nextZ])
                    continue;

                // 计算下一个节点的代价和启发式函数值
                // double nextCost = current->cost + std::pow(std::pow(nextX - current->idx[0],2) + std::pow(nextY - current->idx[1],2) + std::pow(nextZ - current->idx[2],2),0.5); 
                double nextCost = current->cost +std::pow(nextX - current->idx[0],2) + std::pow(nextY - current->idx[1],2) + std::pow(nextZ - current->idx[2],2); 
                // double heuristic = std::pow(std::pow(nextX - goal.idx[0],2) + std::pow(nextY - goal.idx[1],2) + std::pow(nextZ - goal.idx[2],2),0.5); 
                double heuristic = std::pow(nextX - goal.idx[0], 2) + std::pow(nextY - goal.idx[1], 2) + std::pow(nextZ - goal.idx[2], 2);

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
    // zbw
    const double thr60degree = 0.5;
    const double thr45degree = 0.71;
    const double thr37degree = 0.8;
    const double thr30degree = 0.87;
    const double thr25degree = 0.90;
    const double thr20degree = 0.935;
    const double thr15degree = 0.96;
    const double thr10degree = 0.985;
    bool in_sight(Eigen::Vector3d photo_point){
        try{
            // 第一个才是yaw！
            Eigen::Vector2d gimbal_dir_XY(cos(ori[0]),sin(ori[0]));
            Eigen::Vector2d photo_dir_XY(photo_point[0],photo_point[1]);
            // check yaw
            double xy_dot;
            if (photo_dir_XY[0] != 0 && photo_dir_XY[1] != 0) {
                photo_dir_XY = photo_dir_XY.normalized();
                xy_dot = gimbal_dir_XY.dot(photo_dir_XY);
                if (xy_dot < thr37degree) {
                    if (std::abs(gimbal_ori[1]) > 6*M_PI/14){
                        // if(std::abs(xy_dot) > thr10degree){
                        //     // 在看pitch
                        // }else{
                            // 仰角很大，允许dot为负
                            // 此时直接将两个向量dot，小于37度就可以
                            Eigen::Vector3d gimbal_dir_XYZ(cos(ori[0]),sin(ori[0]), sin(gimbal_ori[1]));
                            return (photo_point.dot(gimbal_dir_XYZ) / (photo_point.norm()*gimbal_dir_XYZ.norm())) > thr20degree;
                        // }
                    }
                    else return false;
                } // 继续看pitch
            }
            // check pitch
            double photo_point_pitch;
            if (photo_dir_XY[0] != 0 && photo_dir_XY[1] != 0){
                // XY不为零，计算photo_point的仰角
                photo_dir_XY[0] = photo_point[0];
                photo_dir_XY[1] = photo_point[1];

                photo_point_pitch = atan(photo_point[2]/(photo_dir_XY.norm()));
            }
            else{
                photo_point_pitch = photo_point[2]>0 ? M_PI/2:-M_PI/2;
            }
            return std::abs(photo_point_pitch - gimbal_ori[1]) < (M_PI/8);
        }
        catch(...){
            return false;
        }
        // double angle = acos(gimbal_tagert.dot(photo_point)/ (gimbal_tagert.norm()*photo_point.norm()));
        // return (angle/M_PI*180) < 45  && gimbal_tagert.dot(photo_point) >= 0;
        // tf::Matrix3x3 world2uav, uav2camera;
        // tf::Vector3 target_point(photo_point[0],photo_point[1],photo_point[2]);

        // world2uav.setRPY(ori[0], ori[1], ori[2]);
        // uav2camera.setRPY(gimbal_ori[0],gimbal_ori[1],gimbal_ori[2]);
        // tf::Vector3 tf_target_dir = uav2camera * world2uav * target_point;
        
        // // Eigen::Vector3d target_dir = photo_point-gimbal_pos;
        // Eigen::Vector3d curr_dir(1,0,0); 
        // Eigen::Vector3d target_dir(tf_target_dir[0],tf_target_dir[1],tf_target_dir[2]);
        // double angle = acos((target_dir.dot(curr_dir))/(target_dir.norm()*curr_dir.norm()));
        
        // return (angle/M_PI*180) < 45;
    }

    // 更新占据点是否已被检测
    void update_map() {
        // 根据当前的位置和姿态，更新地图，设置点状态为已检测
        Eigen::Vector3i idx;
        auto re = grid_map.pos2index(pos,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-dis_threshold *1.25;
        int x_max=idx[0]+dis_threshold *1.25;
        int y_min=idx[1]-dis_threshold *1.25;
        int y_max=idx[1]+dis_threshold *1.25;
        int z_min=idx[2]-dis_threshold *1.25;
        int z_max=idx[2]+dis_threshold *1.25;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i++){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j++){
                for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k++){
                    if(grid_map.grid_map[i][j][k].is_inspected < inspect_threshold && grid_map.grid_map[i][j][k].need_to_inspect)
                    {   
                        auto target_pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        if(!have_collision(pos,target_pos) && in_sight(target_pos-pos))
                        {
                            grid_map.grid_map[i][j][k].is_inspected++;
                        }
                    }
                }
            }
        }
    }


    // 找到最近的无遮挡的未被检测的点的质心
    // 要考虑远近 越近权重越高 不然一直往前看而不看近的
    // step 跳点 dis_thres
    Eigen::Vector3d find_inspect_points_center(){
        Eigen::Vector3i idx;
        auto re=grid_map.pos2index(pos,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-(dis_threshold*1.5);
        int x_max=idx[0]+(dis_threshold*1.5);
        int y_min=idx[1]-(dis_threshold*1.5);
        int y_max=idx[1]+(dis_threshold*1.5);
        int z_min=idx[2]-(dis_threshold*1.25);
        int z_max=idx[2]+(dis_threshold*1.25);
        std::vector<Eigen::Vector3d> points;
        std::vector<Eigen::Vector3d> points_obstacle;
        Eigen::Vector3d points_center;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i+=step){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j+=step){
                for(int k=std::max(0,z_min);k<std::min(z_max,grid_map.max_index[2]);k+=step){
                    // if (points.size()>20) break;
                    auto target_pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                    if(grid_map.grid_map[i][j][k].need_to_inspect&& grid_map.grid_map[i][j][k].is_inspected<inspect_threshold)
                    {
                        if(!have_collision(pos,target_pos)){
                            points.push_back(target_pos);
                        }
                        
                    }
                    if(grid_map.grid_map[i][j][k].is_occupied && grid_map.grid_map[i][j][k].is_in_bbox && grid_map.grid_map[i][j][k].pos[2]>min_height) points_obstacle.push_back(target_pos);
                }
            }
        }
        // face points that need to inspect or is_obstacle
        Eigen::Vector3d  sum;
        Eigen::Vector3d  sum_ob;
        for(auto point:points){
            sum+=point;
        }
        if(points.size()>3) points_center=sum/points.size();
        else if(points_obstacle.size()>0){
            for(auto point:points_obstacle){
                sum_ob+=point;     
            }
            points_center=sum_ob/points_obstacle.size();
        }
        else points_center = Eigen::Vector3d(-1,-1,-1);

        gimbal_target = points_center;
        return points_center;

    }

    double saturation(double num,double max,double min){
        if(num>max)
            num=num-max;
        else if(num<min)
            num=num-min;
        return num;
    }

    double cal_desired_yaw(Eigen::Vector3d photo_point){
        photo_point<<photo_point.x(),photo_point.y(),pos[2];//投影到这个平面
        //计算两个向量的yaw
        //1.判断目标相对无人机在哪个象限
        Eigen::Vector3d target_dir = photo_point - pos;
        
        double yaw_dir;
        int phase_index=0;
        if(target_dir.x()>=0 && target_dir.y()>=0){
            yaw_dir=atan2(target_dir.y(),target_dir.x());
            phase_index=1;
            yaw_dir=yaw_dir;
        }
        else if(target_dir.x()<0 && target_dir.y()>0){
            yaw_dir=atan2(target_dir.y(),-target_dir.x());
            phase_index=2;
            yaw_dir=M_PI-yaw_dir;
        }   
        else if(target_dir.x()<=0 && target_dir.y()<=0){
            yaw_dir=atan2(-target_dir.y(),-target_dir.x());
            phase_index=3;
            yaw_dir=-M_PI+yaw_dir;
        }
        else{
            yaw_dir=atan2(-target_dir.y(),target_dir.x());
            phase_index=4;
            yaw_dir=-yaw_dir;
        }

        return yaw_dir;
    }

    double cal_desired_pitch(Eigen::Vector3d photo_point){
        Eigen::Vector3d pos_diff=photo_point-pos;
        double z_diff=pos_diff.z();
        double xy_diff=sqrt(pow(pos_diff.x(),2)+pow(pos_diff.y(),2));
        double desired_pitch;
        if(z_diff<0){//因为逆时针为正
            desired_pitch=atan2(-z_diff,xy_diff);
        }
        else{
            desired_pitch=-atan2(z_diff,xy_diff);
        }
        return desired_pitch;
    }

    void world_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
    {
        if(is_debuging) ROS_INFO("world_octomap");
        try {
            std::thread t2(std::bind(&Inspector::update_occupy,this,msg));
            t2.detach();
        }
        catch (...) {
            std::cout << "[" + this->name + "]" << "world_octomap_cb error !\t" << ros::Time::now() << std::endl;
        }
        if(is_debuging) ROS_INFO("world_octomap done=====");

        // // Added
        // caric_competition_xmu::GridMapMsg grid_map_msg = caric_competition_xmu::map2msg(this->grid_map, this->name);
        // caric_competition_xmu::map_msg_pub.publish(grid_map_msg);
        // std::cout << "[" + this->name + "]" << "published Grid Map !\t" << ros::Time::now() << std::endl;        
    }

    // void update_occupy(const octomap_msgs::Octomap::ConstPtr &msg){
    //     //根据获取的octomap更新grid_map
    //     int count=0;
    //     octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
    //     if (tree) {
    //         octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
    //         if (octree) {
    //             // 防止一开始没有点 全当成孤立点
    //             if(!is_octomap_occupy_init){
    //                 // 防止一开始没有点全当成孤立点
    //                 for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
    //                     // 不管越界
    //                     Eigen::Vector3d tmp(it.getCoordinate().x(),it.getCoordinate().y(),it.getCoordinate().z());
    //                     ROS_INFO("%f %f %f",tmp[0],tmp[1],tmp[2]);
    //                     if(it.getCoordinate().x()<grid_map.x_min 
    //                         || it.getCoordinate().x()>grid_map.x_max
    //                         || it.getCoordinate().y()<grid_map.y_min 
    //                         || it.getCoordinate().y()>grid_map.y_max 
    //                         || it.getCoordinate().z()>grid_map.z_max
    //                         || it.getCoordinate().z()<0) continue;
    //                     grid_map.set_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
    //                     grid_map.set_octo_inspected(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
    //                 }
    //                 for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
    //                     // 不管越界
    //                     if( it.getCoordinate().x()<grid_map.x_min 
    //                         || it.getCoordinate().x()>grid_map.x_max
    //                         || it.getCoordinate().y()<grid_map.y_min 
    //                         || it.getCoordinate().y()>grid_map.y_max 
    //                         || it.getCoordinate().z()>grid_map.z_max
    //                         || it.getCoordinate().z()<0) continue;
    //                     if(octree->isNodeOccupied(*it) && (grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()+1)) ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()-1))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()+1, it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()-1, it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()+1, it.getCoordinate().y(), it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()-1, it.getCoordinate().y(), it.getCoordinate().z())))){
    //                         // 膨胀
    //                         for(int i=-range;i<=range;i++){
    //                             for(int j=-range;j<=range;j++){
    //                                 for(int k=-range;k<=range;k++){
    //                                     Eigen::Vector3d position(it.getCoordinate().x()+i, it.getCoordinate().y()+j, it.getCoordinate().z()+k);
    //                                     grid_map.set_occupancy_pos(position, true);
    //                                 }
    //                             }
    //                         }
    //                     }
    //                     else grid_map.set_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), false);
    //                 }

    //                 is_octomap_occupy_init = true;
    //             }
    //             else{
    //                 for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
    //                     // 不管越界
    //                     if(it.getCoordinate().x()<grid_map.x_min 
    //                         || it.getCoordinate().x()>grid_map.x_max
    //                         || it.getCoordinate().y()<grid_map.y_min 
    //                         || it.getCoordinate().y()>grid_map.y_max 
    //                         || it.getCoordinate().z()>grid_map.z_max) continue;
    //                     grid_map.set_octo_inspected(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
    //                     // 排除孤立点
    //                     if((grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()+1)) ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()-1))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()+1, it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()-1, it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()+1, it.getCoordinate().y(), it.getCoordinate().z()))  ||
    //                     grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()-1, it.getCoordinate().y(), it.getCoordinate().z())))){
    //                     }
    //                     else{
    //                         Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
    //                         grid_map.set_occupancy_pos(position, false);
    //                         continue;
    //                     }
    //                     if (octree->isNodeOccupied(*it)) {
    //                         // 膨胀
    //                         for(int i=-range;i<=range;i++){
    //                             for(int j=-range;j<=range;j++){
    //                                 for(int k=-range;k<=range;k++){
    //                                     Eigen::Vector3d position(it.getCoordinate().x()+i, it.getCoordinate().y()+j, it.getCoordinate().z()+k);
    //                                     // 不管地面
    //                                     if(position[2]<0) continue;
    //                                     grid_map.set_occupancy_pos(position, true);
    //                                 }
    //                             }
    //                         }
    //                         // set inspect points
    //                         Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
    //                         Eigen::Vector3i idx;
    //                         auto re = grid_map.pos2index(position,idx);
    //                         if(re&&position[2]>3&&grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox && !grid_map.grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect)
    //                             grid_map.grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=true;
    //                     }
    //                     else {
    //                         // Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
    //                         // grid_map.set_occupancy_pos(position, false);
    //                         // count++;
    //                     }
    //                 }

    //             }
    //         } 
    //         else {
    //             ROS_ERROR("Failed to cast the abstract tree to OcTree.");
    //         }
    //     } 
    //     else {
    //         ROS_ERROR("Received an unsupported octomap type.");
    //     }
    //     ROS_INFO("[%s]update map %d points",name.c_str(),count);
    // }

 void update_occupy(const octomap_msgs::Octomap::ConstPtr &msg){
        //根据获取的octomap更新grid_map
        int count=0;
        octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
        if (tree) {
            octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
            if (octree) {
                for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
                    // 不管越界
                    if(it.getCoordinate().x()<grid_map.x_min 
                        || it.getCoordinate().x()>grid_map.x_max
                        || it.getCoordinate().y()<grid_map.y_min 
                        || it.getCoordinate().y()>grid_map.y_max 
                        || it.getCoordinate().z()>grid_map.z_max
                        || it.getCoordinate().z()<0) continue;
                    grid_map.set_octo_inspected(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
                    // // 排除孤立点
                    // if((grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()+1)) ||
                    // grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()-1))  ||
                    // grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()+1, it.getCoordinate().z()))  ||
                    // grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y()-1, it.getCoordinate().z()))  ||
                    // grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()+1, it.getCoordinate().y(), it.getCoordinate().z()))  ||
                    // grid_map.get_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x()-1, it.getCoordinate().y(), it.getCoordinate().z())))){
                    // }
                    // else{
                    //     Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
                    //     grid_map.set_occupancy_pos(position, false);
                    //     continue;
                    // }
                    if (octree->isNodeOccupied(*it)) {
                        // occupy
                        grid_map.set_occupancy_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
                        grid_map.set_face_pos(Eigen::Vector3d(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z()), true);
                        // 膨胀
                        for(int i=-range;i<=range;i++){
                            for(int j=-range;j<=range;j++){
                                for(int k=-range;k<=range;k++){
                                    if(i==0&&j==0&&k==0)continue;
                                    Eigen::Vector3d position(it.getCoordinate().x()+i, it.getCoordinate().y()+j, it.getCoordinate().z()+k);
                                    grid_map.set_flat_pos(position, true);
                                }
                            }
                        }
                        // set inspect points
                        Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
                        Eigen::Vector3i idx;
                        auto re = grid_map.pos2index(position,idx);
                        if(re&&position[2]>3&&grid_map.grid_map[idx[0]][idx[1]][idx[2]].is_in_bbox && !grid_map.grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect)
                            grid_map.grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=true;
                    }
                    else {              
                        Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
                        if(!grid_map.get_occupancy_pos(position) || position[2]<3)continue;
                        grid_map.set_occupancy_pos(position, false);
                        grid_map.set_need_to_inspect_pos(position,false);
                        Eigen::Vector3i tmp_idx;
                        auto re = grid_map.pos2index(position,tmp_idx);
                        for (int dx = -range; dx <= range; dx++) {
                            for (int dy = -range; dy <= range; dy++) {
                                for (int dz = -range; dz <= range; dz++) {
                                    int newX = tmp_idx[0] + dx;
                                    int newY = tmp_idx[1] + dy;
                                    int newZ = tmp_idx[2] + dz;

                                    // 检查是否越界
                                    if (newX >= 0 && newX < grid_map.max_index[0] && newY >= 0 && newY < grid_map.max_index[1] && newZ >= 0 && newZ < grid_map.max_index[2]) {
                                        grid_map.grid_map[newX][newY][newZ].in_flat=false;
                                    }
                                }
                            }
                        }
                        
                    }
                    
                }
            } 
            else {
                ROS_ERROR("Failed to cast the abstract tree to OcTree.");
            }
        } 
        else {
            ROS_ERROR("Received an unsupported octomap type.");
        }
        // ROS_INFO("[%s]update map %d points",name.c_str(),count);
        count++;
    }

    // 用于显示waypoints
    void get_waypoints(visualization_msgs::Marker* marker){
        for(auto point:simplified_waypoint_path){
            geometry_msgs::Point p;
            Eigen::Vector3d pos = grid_map.index2pos(point->idx);
            p.x=pos[0];
            p.y=pos[1];
            p.z=pos[2];
            marker->points.push_back(p);
        }
    }

    void get_points_in_sight(visualization_msgs::Marker& marker){
        // 根据当前的位置和姿态，更新地图，设置点状态为已检测
        Eigen::Vector3i idx;
        auto re = grid_map.pos2index(pos,idx);
        //我就在这个idx附近找一圈
        int x_min=idx[0]-dis_threshold * 1.25;
        int x_max=idx[0]+dis_threshold * 1.25;
        int y_min=idx[1]-dis_threshold * 1.25;
        int y_max=idx[1]+dis_threshold * 1.25;
        int z_min=idx[2]-dis_threshold * 1.25;
        int z_max=idx[2]+dis_threshold * 1.25;
        for(int i=std::max(0,x_min);i<std::min(x_max,grid_map.max_index[0]);i++){
            for(int j=std::max(0,y_min);j<std::min(y_max,grid_map.max_index[1]);j++){
                for(int k=std::max(1,z_min);k<std::min(z_max,grid_map.max_index[2]);k++){
 
                    auto target_pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                    if(!have_collision(pos,target_pos) && in_sight(target_pos-pos))
                    {
                        geometry_msgs::Point p;
                        p.x=target_pos[0];
                        p.y=target_pos[1];
                        p.z=target_pos[2];
                        marker.points.push_back(p);
                    }
                }
            }
        }
    }

    // rviz marker显示
    void create_marker(visualization_msgs::Marker* marker,std::string frame_id, std::string ns,int32_t id,int32_t type,int32_t action, 
        double w, double x,double y,double z,double r,double g, double b, double a ){
        marker->header.frame_id = frame_id; // 设置坐标系
        marker->header.stamp = ros::Time::now();
        marker->ns = ns;
        marker->id = id;
        marker->type = type;
        marker->action = action;
        marker->pose.orientation.w = w;


        // 设置点的大小,三维点
        marker->scale.x = x;
        marker->scale.y = y;
        marker->scale.z = z;

        // 设置点的颜色
        marker->color.r = r;
        marker->color.g = g;
        marker->color.b = b;
        marker->color.a = a;
    }

    // 5
    void publish_marker(std::vector<visualization_msgs::Marker*> marker_list, ros::Publisher marker_pub){
        std::lock_guard<std::mutex> lock(mtx); //线程互斥锁
        for(auto marker:marker_list){
            // // 清空 Marker 消息的点
            marker->points.clear();
            if(marker->ns == "occupied"){
                Inspector::get_occupied_points(*marker);
            } 
            else if(marker->ns == "search"){
                get_search_points(*marker);
            }

            else if(marker->ns =="inspected"){
                get_inspected_points(*marker);
            }
            else if(marker->ns =="not_inspected"){
                get_uninspected_points(*marker);
            }
            else if(marker->ns=="bbox"){
                get_bbox(*marker);
            }
            // 起飞前不发布以下marker
            else if (is_takeoff){
                if(marker->ns == "waypoint") {
                    get_waypoints(marker);
                }
                // if(marker->ns == "risk_waypoint") Inspector::get_occupied_points(*marker);
                else if(marker->ns == "trajectory"){
                    get_trajectory(*marker);
                }
                else if(marker->ns=="frontier_points"){
                    get_frontier_points(*marker);
                }
                else if(marker->ns=="gimbal_target"){
                    get_gimbal_target(*marker);
                }
                else if(marker->ns == "target"){
                    geometry_msgs::Point p;
                    p.x=target_point.pos(0);
                    p.y=target_point.pos(1);
                    p.z=target_point.pos(2);
                    marker->points.push_back(p);
                }
                else if(marker->ns == "in_sight"){
                    get_points_in_sight(*marker);
                }
            }

            marker_pub.publish(*marker); 
        }
        if (is_takeoff){
            for(auto& text_marker : this->text_markers){
                marker_pub.publish(text_marker);
            }
        }
        // Inspector::get_occupied_points(marker_occupied);
        // inspector.get_uninspected_points(marker_not_inspected);        
        // Inspector::get_inspected_points(marker_inspected);
            
    }

    void get_bbox(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_in_bbox)
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

    void get_gimbal_target(visualization_msgs::Marker& marker){
        geometry_msgs::Point p;
        p.x=gimbal_target.x();
        p.y=gimbal_target.y();
        p.z=gimbal_target.z();
        marker.points.push_back(p);
    }

    void get_frontier_points(visualization_msgs::Marker& marker){
        for(auto point:this->frontier_points){
            geometry_msgs::Point p;
            p.x=point.pos.x();
            p.y=point.pos.y();
            p.z=point.pos.z();
            marker.points.push_back(p);
        }
    }

    void get_occupied_points(visualization_msgs::Marker& marker){
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_occupied || grid_map.grid_map[i][j][k].in_flat)
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
                    if(grid_map.grid_map[i][j][k].is_inspected < inspect_threshold && grid_map.grid_map[i][j][k].need_to_inspect==true)
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
        // int count=0;
        for(int i=0;i<grid_map.max_index[0];i++){
            for(int j=0;j<grid_map.max_index[1];j++){
                for(int k=0;k<grid_map.max_index[2];k++){
                    if(grid_map.grid_map[i][j][k].is_inspected >=inspect_threshold && grid_map.grid_map[i][j][k].need_to_inspect==true)
                    {
                        Eigen::Vector3d pos=grid_map.index2pos(Eigen::Vector3i(i,j,k));
                        geometry_msgs::Point p;
                        p.x=pos[0];
                        p.y=pos[1];
                        p.z=pos[2];
                        marker.points.push_back(p);
                        // count++;
                    }
                }
            }
        }
        // std::cout<<count<<std::endl;
    }
    void get_search_points(visualization_msgs::Marker& marker){
        for(auto pointss:search_points_array){
            for(auto points:pointss){
                for(auto point:points){
                    // if( !point.is_visited){
                    if(!point.is_bad && !point.is_obstacle && !point.is_visited){
                        geometry_msgs::Point p;
                        p.x=point.pos[0];
                        p.y=point.pos[1];
                        p.z=point.pos[2];
                        marker.points.push_back(p);
                    }
                    
                }
            }
            
        }
    }

    void get_trajectory(visualization_msgs::Marker& marker){
    try{
        for(int i=0;i<_polyTime.size();i++){
            for(double t=0;t<_polyTime[i];t+=0.1){
                
                    Eigen::Vector3d tmp = getPosPoly(_polyCoeff,i,t);
                    geometry_msgs::Point p;
                    p.x=tmp[0];
                    p.y=tmp[1];
                    p.z=tmp[2];
                    marker.points.push_back(p);
                }
            }
        }
    
    catch(...){}
    }

};


