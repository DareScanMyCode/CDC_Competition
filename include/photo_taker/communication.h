#include "ros/ros.h"
#include "caric_competition_xmu/GridMapMsg.h"
#include "caric_competition_xmu/GridNodeMsg.h"
#include "caric_competition_xmu/OdometryArray.h"
#include "caric_competition_xmu/SearchPointArray.h"
#include "caric_competition_xmu/SearchPointMsg.h"
#include "inspector_mini.h"
#include <stdio.h>
#include "caric_mission/CreatePPComTopic.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud.h"
// #include "nav_msgs/Odometry.h"
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include "geometry_msgs/PoseArray.h"
#include <unordered_map>
#include <unordered_set>
#include <set>
// #include "search_point.h"

// 定义 Eigen::Vector3d 的哈希函数
struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& v) const {
        // 使用内置的哈希函数来哈希 Vector3d
        return std::hash<double>()(v.x()) ^ std::hash<double>()(v.y()) ^ std::hash<double>()(v.z());
    }
};

// 定义 Vector3d 的比较函数，确保 unordered_map 可以正确比较两个 Eigen::Vector3d 对象
struct Vector3dEqual {
    bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const {
        return v1.isApprox(v2);  // 使用 Eigen 内置的 isApprox 函数来比较浮点数
    }
};

struct VectorCompare {
    bool operator()(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) const {
        // 使用一个很小的阈值来防止浮点数误差导致的比较问题
        double epsilon = 1e-10;
        if (std::abs(v1.x() - v2.x()) > epsilon) {
            return v1.x() < v2.x();
        } else if (std::abs(v1.y() - v2.y()) > epsilon) {
            return v1.y() < v2.y();
        } else {
            return v1.z() < v2.z();
        }
    }
};

namespace std {
    template <>
    struct hash<Eigen::Matrix<double, 3, 1>> {
        size_t operator()(const Eigen::Matrix<double, 3, 1>& matrix) const {
            // 使用Eigen库中的hash函数来计算哈希值
            return std::hash<double>()(matrix(0)) ^
                   std::hash<double>()(matrix(1)) ^
                   std::hash<double>()(matrix(2));
        }
    };
}

class SearchPoint {
    public:
        // 构造函数
        SearchPoint(){};
        SearchPoint(Eigen::Vector3d pos, int x, int y, int z, bool is_obstacle):pos(pos),x(x),y(y),z(z),is_obstacle(is_obstacle){}

        Eigen::Vector3d pos; 
        // idx in saerch array
        int x,y,z;
        bool is_visited{false}; 
        bool is_in_frontier{false}; 
        bool is_bad; // 每一个searchpoint周围至少有一个点在bbox内 否则为bad
        bool is_obstacle;
        double heuristic;
    };
namespace caric_competition_xmu{
    

    std::vector<std::vector<std::vector<SearchPoint>>> search_points_array;

    std::vector<std::vector<std::vector<SearchPoint>>>  msg2search_point_array3(caric_competition_xmu::SearchPointArray msg)
    {
        std::vector<std::vector<std::vector<SearchPoint>>> search_points_array(msg.max_index_x, std::vector<std::vector<SearchPoint>>(msg.max_index_y, std::vector<SearchPoint>(msg.max_index_z)));

        int index;
        int i=0;
        int j=0;
        int k=0;
        
        for(index=0; index < msg.search_points.size(); index++){
            SearchPoint point;
            caric_competition_xmu::SearchPointMsg search_point = msg.search_points[index];
            
            // 转换 SearchPoint 到 SearchPoint 消息
            point.pos = Eigen::Vector3d(search_point.pos.x, search_point.pos.y, search_point.pos.z);
            point.x = search_point.x;
            point.y = search_point.y;
            point.z = search_point.z;
            point.is_visited = search_point.is_visited;
            point.is_in_frontier = search_point.is_in_frontier;
            point.is_bad = search_point.is_bad;
            point.is_obstacle = search_point.is_obstacle;
            point.heuristic = search_point.heuristic;

            search_points_array[i][j][k] = point;
            k++;
            if(k >= msg.max_index_z){
                j++;
                k=0;
                if(j >= msg.max_index_y){
                    j=0;
                    i++;
                }
            }

        }
        return search_points_array;
    }

    std::vector<SearchPoint> msg2search_point_array(caric_competition_xmu::SearchPointArray msg){
        std::vector<SearchPoint> search_points_arra;
        for(int index=0; index < msg.search_points.size(); index++){
            SearchPoint point;
            caric_competition_xmu::SearchPointMsg search_point = msg.search_points[index];
            // 转换 SearchPoint 到 SearchPoint 消息
            point.pos = Eigen::Vector3d(search_point.pos.x, search_point.pos.y, search_point.pos.z);
            point.x = search_point.x;
            point.y = search_point.y;
            point.z = search_point.z;
            point.is_visited = search_point.is_visited;
            point.is_in_frontier = search_point.is_in_frontier;
            point.is_bad = search_point.is_bad;
            point.is_obstacle = search_point.is_obstacle;
            point.heuristic = search_point.heuristic;
            search_points_arra.push_back(point);
        }
        return search_points_arra;
    }

    caric_competition_xmu::SearchPointArray search_point_array3_2msg(std::vector<std::vector<std::vector<SearchPoint>>> search_points_array){
        caric_competition_xmu::SearchPointArray msg;

        msg.max_index_x = search_points_array.size();
        msg.max_index_y = search_points_array[0].size();
        msg.max_index_z = search_points_array[0][0].size();

        for(int i = 0; i < msg.max_index_x; i++){
            for(int j = 0; j < msg.max_index_y; j++){
                for(int k = 0; k < msg.max_index_z; k++){
                    caric_competition_xmu::SearchPointMsg node;
                    node.x = search_points_array[i][j][k].pos.x();
                    node.y = search_points_array[i][j][k].pos.y();
                    node.z = search_points_array[i][j][k].pos.z();
                    node.is_visited     = search_points_array[i][j][k].is_visited;
                    node.is_in_frontier = search_points_array[i][j][k].is_in_frontier;
                    node.is_bad         = search_points_array[i][j][k].is_bad;
                    node.is_obstacle    = search_points_array[i][j][k].is_obstacle;
                    node.heuristic      = search_points_array[i][j][k].heuristic;
                    msg.search_points.push_back(node);
                }
            }
        }
        return msg;
    }

    caric_competition_xmu::SearchPointArray search_point_array1_2msg(std::vector<SearchPoint> search_points_array){
        caric_competition_xmu::SearchPointArray msg;

        msg.max_index_x = search_points_array.size();


        for(int i = 0; i < msg.max_index_x; i++){
            caric_competition_xmu::SearchPointMsg node;
            node.x = search_points_array[i].pos.x();
            node.y = search_points_array[i].pos.y();
            node.z = search_points_array[i].pos.z();
            node.is_visited         = search_points_array[i].is_visited;
            node.is_in_frontier     = search_points_array[i].is_in_frontier;
            node.is_bad             = search_points_array[i].is_bad;
            node.is_obstacle        = search_points_array[i].is_obstacle;
            node.heuristic          = search_points_array[i].heuristic;
            msg.search_points.push_back(node);
        }
        return msg;
    }

    
    
    /// @brief 根据时间戳合并两个msg
    /// @param old_map_msg old指本身有的
    /// @param new_map_msg new指新收到的。下同
    /// @return 0：合并成功； -1：两条msg参数不一致
    // int merge_grid_map_msg(caric_competition_xmu::GridMapMsg::ConstPtr & old_map_msg, caric_competition_xmu::GridMapMsg::ConstPtr & new_map_msg){
    //     int result = 0;
    //     if(
    //         old_map_msg->max_index_x != new_map_msg->max_index_x ||
    //         old_map_msg->max_index_y != new_map_msg->max_index_y ||
    //         old_map_msg->max_index_z != new_map_msg->max_index_z ||
    //         old_map_msg->origin_x != new_map_msg->origin_x ||
    //         old_map_msg->origin_y != new_map_msg->origin_y ||
    //         old_map_msg->origin_z != new_map_msg->origin_z
    //     ){
    //         ROS_WARN("!!while merging map msgs, origin or max indexes do not euqal!!");
    //         result = -1;
    //     }
        
    //     // 合并两条消息
    //     int max_index = old_map_msg->max_index_x * old_map_msg->max_index_y * old_map_msg->max_index_z;
    //     for(int i = 0; i<max_index; i++){
    //         // if(old_map_msg.grid_nodes[i].header.stamp.toSec() < new_map_msg.grid_nodes[i].header.stamp.toSec()){
    //             // 是否可以直接赋值node？
    //             // old_map_msg.grid_nodes[i].header = new_map_msg.grid_nodes[i].header;
    //             // old_map_msg.grid_nodes[i].x = new_map_msg.grid_nodes[i].x;
    //             // old_map_msg.grid_nodes[i].y = new_map_msg.grid_nodes[i].y;
    //             // old_map_msg.grid_nodes[i].z = new_map_msg.grid_nodes[i].z;
    //             old_map_msg->grid_nodes[i].is_inspected = old_map_msg->grid_nodes[i].is_inspected || new_map_msg->grid_nodes[i].is_inspected;
    //             old_map_msg->grid_nodes[i].is_occupied = old_map_msg->grid_nodes[i].is_occupied || new_map_msg->grid_nodes[i].is_occupied;
    //             // old_map_msg.grid_nodes[i].need_to_inspect = !old_map_msg.grid_nodes[i].is_inspected &&
    //             //                                             (old_map_msg.grid_nodes[i].need_to_inspect ||
    //             //                                             new_map_msg.grid_nodes[i].need_to_inspect);
    //         // }
    //     }
    //     return result;
    // }


    /// @brief 将grid map 转换为 map msg
    /// @param map 
    /// @return 
    caric_competition_xmu::GridMapMsg map2msg(GridMap_mini map){
        caric_competition_xmu::GridMapMsg msg;

        msg.origin_x = map.map_origin.x();
        msg.origin_y = map.map_origin.y();
        msg.origin_z = map.map_origin.z();

        msg.x_min = map.x_min;
        msg.y_min = map.y_min;
        msg.z_min = map.z_min;
        msg.x_max = map.x_max;
        msg.y_max = map.y_max;
        msg.z_max = map.z_max;

        msg.max_index_x = map.max_index.x();
        msg.max_index_y = map.max_index.y();
        msg.max_index_z = map.max_index.z();

        msg.resolution = map.resolution;

        for(int i = 0; i < map.max_index.x(); i++){
            for(int j = 0; j < map.max_index.y(); j++){
                for(int k = 0; k < map.max_index.z(); k++){
                    caric_competition_xmu::GridNodeMsg node;
                    node.x = map.grid_map[i][j][k].pos.x();
                    node.y = map.grid_map[i][j][k].pos.y();
                    node.z = map.grid_map[i][j][k].pos.z();
                    node.is_inspected = map.grid_map[i][j][k].is_inspected;
                    node.is_occupied = map.grid_map[i][j][k].is_occupied;
                    node.need_to_inspect = map.grid_map[i][j][k].need_to_inspect;

                    msg.grid_nodes.push_back(node);
                }
            }
        }
        return msg;
    }

    /// @brief map_msg转map
    /// @param msg 
    /// @return 转换后的map
    GridMap_mini msg2map(caric_competition_xmu::GridMapMsg msg) {
        GridMap_mini map = GridMap_mini(msg.origin_x,
            msg.origin_y, 
            msg.origin_z, 
            msg.x_max - msg.x_min,
            msg.y_max - msg.y_min,
            msg.z_max - msg.z_min,
            msg.resolution
            );

        // Resize and initialize the grid map based on max_index
        // map.grid_map.resize(map.max_index.x());
        // for (int i = 0; i < map.max_index.x(); i++) {
        //     map.grid_map[i].resize(map.max_index.y());
        //     for (int j = 0; j < map.max_index.y(); j++) {
        //         map.grid_map[i][j].resize(map.max_index.z());
        //     }
        // }

        // Iterate over grid_nodes and fill the grid map
        for (const auto& node : msg.grid_nodes) {
            int i = static_cast<int>(round((node.x - map.x_min) / map.resolution));
            int j = static_cast<int>(round((node.y - map.y_min) / map.resolution));
            int k = static_cast<int>(round((node.z - map.z_min) / map.resolution));

            if (i >= 0 && i < map.max_index.x() &&
                j >= 0 && j < map.max_index.y() &&
                k >= 0 && k < map.max_index.z()) {
                map.grid_map[i][j][k].pos = Eigen::Vector3d(node.x, node.y, node.z);
                map.grid_map[i][j][k].is_inspected = node.is_inspected;
                map.grid_map[i][j][k].is_occupied = node.is_occupied;
                map.grid_map[i][j][k].need_to_inspect = node.need_to_inspect;
                map.grid_map[i][j][k].is_octo_inspected=node.is_octo_inspected;
            }
        }

        return map;
    }

    GridMap_mini small_msg2map(caric_competition_xmu::GridMapMsg msg) {
        GridMap_mini map = GridMap_mini(msg.origin_x,
            msg.origin_y, 
            msg.origin_z, 
            msg.x_max - msg.x_min,
            msg.y_max - msg.y_min,
            msg.z_max - msg.z_min,
            msg.resolution
            );

        // Resize and initialize the grid map based on max_index
        // map.grid_map.resize(map.max_index.x());
        // for (int i = 0; i < map.max_index.x(); i++) {
        //     map.grid_map[i].resize(map.max_index.y());
        //     for (int j = 0; j < map.max_index.y(); j++) {
        //         map.grid_map[i][j].resize(map.max_index.z());
        //     }
        // }

        // Iterate over grid_nodes and fill the grid map
        for (const auto& node : msg.grid_nodes) {
            int i = static_cast<int>(round((node.x - map.x_min) / map.resolution));
            int j = static_cast<int>(round((node.y - map.y_min) / map.resolution));
            int k = static_cast<int>(round((node.z - map.z_min) / map.resolution));

            if (i >= 0 && i < map.max_index.x() &&
                j >= 0 && j < map.max_index.y() &&
                k >= 0 && k < map.max_index.z()) {
                map.grid_map[i][j][k].pos = Eigen::Vector3d(node.x, node.y, node.z);
                map.grid_map[i][j][k].is_inspected = node.is_inspected;
                map.grid_map[i][j][k].need_to_inspect = node.need_to_inspect;
                if(!node.is_octo_inspected||node.is_occupied||node.in_flat){
                    map.grid_map[i][j][k].is_occupied=true;
                }
                else{
                    map.grid_map[i][j][k].is_occupied=false;
                }
            }
        }

        return map;
    }

    // void copy_odom_msg(caric_competition_xmu::OdometryArray to_msg, caric_competition_xmu::OdometryArray from_msg){
    //     to_msg.header = from_msg.header; // TODO header的传递
    //     if(to_msg.swarm_size != from_msg.swarm_size) ROS_WARN("两个OdometryArray数组大小不同");
    //     for(int i = 0; i < to_msg.swarm_size.data; i++){
    //         to_msg.odometry_array[i] = from_msg.odometry_array[i];
    //     }
    // }

//do not use
    // int merge_swarm_odom(nav_msgs::Odometry[] swarm_odom, caric_competition_xmu::OdometryArray new_odom_msg){
    //     int result = 0;
    //     if(swarm_odom. != new_odom_msg.swarm_size) {result = -1; ROS_WARN("两个OdometryArray数组大小不同");}
    //     for(int i = 0; i < new_odom_msg.swarm_size.data; i++){
    //         if(new_odom_msg.odometry_array[i].header.stamp.nsec > old_odom_msg.odometry_array[i].header.stamp.nsec){
    //             old_odom_msg.odometry_array[i] = new_odom_msg.odometry_array[i];
    //         }
    //     }
    //     // if(old_odom_msg)
    //     return result;
    // }

    caric_competition_xmu::OdometryArray swarm_odom2msg(const std::vector<nav_msgs::Odometry>& swarm_odom) {
        caric_competition_xmu::OdometryArray odometryArrayMsg;

        // 填充 Header 和 swarm_size
        odometryArrayMsg.header.stamp = ros::Time::now();
        odometryArrayMsg.header.frame_id = "world";  // 根据需要设置合适的 frame_id
        odometryArrayMsg.swarm_size.data = swarm_odom.size();

        // 填充 Odometry 数组
        for (const auto& odom : swarm_odom) {
            odometryArrayMsg.odometry_array.push_back(odom);
        }

        return odometryArrayMsg;
    }

    // 定义变量
    ros::Publisher msg_pub;
    ros::Publisher map_msg_pub;
    ros::Publisher interest_point_pub;
    ros::Publisher detected_interest_point_pub;
    ros::Publisher checked_interest_point_pub;
    ros::Publisher search_points_array_pub;
    ros::Publisher frontier_points_pub;
    ros::Publisher odometry_pub;
    ros::Publisher swarm_odom_pub;

    ros::Subscriber map_msg_sub;
    ros::Subscriber interest_point_sub;
    ros::Subscriber detected_interest_point_sub;
    ros::Subscriber checked_interest_point_sub;
    ros::Subscriber swarm_odom_sub;
    ros::Publisher gcs_score_pub;

    GridMap_mini newest_grid_map;

    sensor_msgs::PointCloud newest_interested_msg;  // msg 和 cloud

    sensor_msgs::PointCloud newest_detected_points_msg;
    sensor_msgs::PointCloud newest_checked_points_msg;


    caric_competition_xmu::OdometryArray newest_swarm_odom_msg;
    caric_competition_xmu::GridMapMsg newest_grid_map_msg;

    bool is_debug;
    int uav_id;

    // 回调函数
    // void map_msg_cb(caric_competition_xmu::GridMapMsg msg){
    //     if(is_debug) ROS_INFO("get a new map msg");
    //     caric_competition_xmu::merge_grid_map_msg(newest_grid_map_msg, msg);
    //     newest_grid_map = msg2map(newest_grid_map_msg);
    // }

    // void interest_point_msg_cb(sensor_msgs::PointCloud msg){
    //     if(is_debug) ROS_INFO("get a nbr cloud msg");
    //     merge_interest_point_msg(newest_interested_msg, msg);
    // }

    // void swarm_odom_msg_cb(caric_competition_xmu::OdometryArray msg){
    //     if(is_debug) ROS_INFO("get a swarm odom msg");
    //     merge_swarm_odom(newest_swarm_odom_msg, msg);
    // }

    // void detected_interest_point_msg_cb(sensor_msgs::PointCloud msg){
    //     merge_interest_point_msg(newest_detected_points_msg, msg);
    // }

    // void checked_interest_point_msg_cb(sensor_msgs::PointCloud msg){
    //     merge_interest_point_msg(newest_checked_points_msg, msg);
    // }

    // get和set方法

    // void set_map(GridMap_mini map){
    //     newest_grid_map = map;
    // }

    // GridMap_mini get_map(){
    //     return newest_grid_map;
    // }


    // void set_interest_pointcloud_msg(sensor_msgs::PointCloud cloud_msg){
    //     newest_interested_msg = cloud_msg;
    // }

    /// @brief 兴趣点向量转换为msg
    /// @param interest_point 
    /// @return 
    geometry_msgs::PoseArray interest_point2msg(std::vector<Eigen::Vector3d> interest_points) {
        geometry_msgs::PoseArray pose_array;
        for (const auto& point : interest_points) {
            geometry_msgs::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            pose.position.z = point.z();

            // 设置方向为无旋转（默认四元数）
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;

            pose_array.poses.push_back(pose);
        }
        return pose_array;
    }

    geometry_msgs::PoseArray checked_point2msg(std::unordered_set<Eigen::Vector3d> checked_points) {
        geometry_msgs::PoseArray pose_array;
        for (const auto& point : checked_points) {
            geometry_msgs::Pose pose;
            pose.position.x = point.x();
            pose.position.y = point.y();
            pose.position.z = point.z();
            // 设置方向为无旋转（默认四元数）
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;

            pose_array.poses.push_back(pose);
        }
        return pose_array;
    }

    /// @brief msg转换为兴趣点向量
    /// @param  
    /// @return 
    std::vector<Eigen::Vector3d> msg2interest_point(geometry_msgs::PoseArray msg){
        std::vector<Eigen::Vector3d> interest_points;
        for (const auto& pose : msg.poses) {
            Eigen::Vector3d point(pose.position.x, pose.position.y, pose.position.z);
            interest_points.push_back(point);
        }
        return interest_points;
    }
    
    // 检测到的
    // void set_detected_interest_pointcloud_msg(geometry_msgs::PoseArray cloud_msg){
    //     newest_detected_points_msg = cloud_msg;
    // }

    // geometry_msgs::PoseArray get_detected_point_msg(){
    //     return newest_detected_points_msg;
    // }

    // std::vector<Eigen::Vector3d> get_detected_point(){
    //     return msg2interest_point(newest_detected_points_msg);
    // }


    // // 检查过的
    // void set_cheked_interest_pointcloud_msg(geometry_msgs::PoseArray cloud_msg){
    //     newest_checked_points_msg = cloud_msg;
    // }

    // geometry_msgs::PoseArray get_checked_point_msg(){
    //     return newest_checked_points_msg;
    // }

    // std::vector<Eigen::Vector3d> get_checke_point(){
    //     return msg2interest_point(newest_checked_points_msg);
    // }

    // 编队几何位置
    // void set_swarm_odom_msg(caric_competition_xmu::OdometryArray odom_msg){
    //     newest_swarm_odom_msg = odom_msg;
    // }

    // caric_competition_xmu::OdometryArray get_swarm_msg(){
    //     return newest_swarm_odom_msg;
    // }

    // 更新msg或者其他变量
    void update_swarm_odom(nav_msgs::Odometry my_odom, int self_id_from_0){
        newest_swarm_odom_msg.odometry_array[self_id_from_0] = my_odom;
    }

    /// @brief 初始化
    /// @param nh 
    /// @param source 
    /// @param is_debug_ 
    /// @return 
    int init_Communicator(ros::NodeHandle nh, std::string source, bool is_debug_){
        is_debug = is_debug_;
        caric_mission::CreatePPComTopic srv;
        // ROS_INFO("Wait for service to appear");
        ros::service::waitForService("/create_ppcom_topic");  // 可以在之前就wait
        ROS_INFO("Got Service");
        ros::ServiceClient create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");

        // 地图通讯
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/grid_map";
        srv.request.package_name = "caric_competition_xmu";
        srv.request.message_type = "GridMapMsg";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : map communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        map_msg_pub = nh.advertise<caric_competition_xmu::GridMapMsg>(srv.request.topic_name,1);
        
        // 位置通讯
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/swarm_odometry";
        srv.request.package_name = "caric_competition_xmu";
        srv.request.message_type = "OdometryArray";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : swarm_odometry communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        swarm_odom_pub = nh.advertise<caric_competition_xmu::OdometryArray>(srv.request.topic_name,1);


        // 兴趣点通讯
        // 已探索到的
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/detected_interested_points";
        srv.request.package_name = "sensor_msgs";
        srv.request.message_type = "PointCloud";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : detected_interested_points communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        detected_interest_point_pub = nh.advertise<sensor_msgs::PointCloud>(srv.request.topic_name,1);

        // 已经check的
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/checked_interested_points";
        srv.request.package_name = "geometry_msgs";
        srv.request.message_type = "PoseArray";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : checked_interested_points communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        checked_interest_point_pub = nh.advertise<geometry_msgs::PoseArray>(srv.request.topic_name,1);

        // search_points_array
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/search_points_array";
        srv.request.package_name = "caric_competition_xmu";
        srv.request.message_type = "SearchPointArray";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : search_points_array communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        search_points_array_pub = nh.advertise<caric_competition_xmu::SearchPointArray>(srv.request.topic_name,1);

        // frontier_points, array设置成n*1*1
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/frontier_points";
        srv.request.package_name = "caric_competition_xmu";
        srv.request.message_type = "SearchPointArray";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : frontier_points communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        frontier_points_pub = nh.advertise<caric_competition_xmu::SearchPointArray>(srv.request.topic_name,1);

        // frontier_points, array设置成n*1*1
        srv.request.source = source;
        srv.request.targets = {"all"};
        srv.request.topic_name = "/gcs_score";
        srv.request.package_name = "sensor_msgs";
        srv.request.message_type = "PointCloud";
        create_ppcom_topic.call(srv);
        // ROS_INFO("%s : gcs_score communicator gened, result: %s", source.c_str(), srv.response.result.c_str());
        gcs_score_pub = nh.advertise<sensor_msgs::PointCloud>(srv.request.topic_name,1);

        
        // 设置订阅和回调函数
        // std::string map_msg_sub_topic_name("/grid_map/");
        // map_msg_sub_topic_name += source;
        // map_msg_sub = nh.subscribe(map_msg_sub_topic_name.c_str(), 1, map_msg_cb);

        // std::string detected_interest_point_sub_topic_name("/detected_interested_points/");
        // detected_interest_point_sub_topic_name += source;
        // detected_interest_point_sub = nh.subscribe(detected_interest_point_sub_topic_name.c_str(), 1, detected_interest_point_msg_cb);

        // std::string checked_interest_point_sub_topic_name("/checked_interested_points/");
        // checked_interest_point_sub_topic_name += source;
        // checked_interest_point_sub = nh.subscribe(checked_interest_point_sub_topic_name.c_str(), 1, checked_interest_point_msg_cb);

        // std::string swarm_odometry_sub_topic_name("/swarm_odometry/");
        // swarm_odometry_sub_topic_name += source;
        // swarm_odom_sub = nh.subscribe(swarm_odometry_sub_topic_name.c_str(), 1, swarm_odom_msg_cb);
    }

    // void pub_t_fun(){
    //     // 或者睡眠2000毫秒
    //     while(ros::ok){
    //         std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //         detected_interest_point_pub.publish(newest_detected_points_msg);
    //         checked_interest_point_pub.publish(newest_checked_points_msg);
    //         map_msg_pub.publish(newest_grid_map_msg);
    //         swarm_odom_pub.publish(newest_swarm_odom_msg);
    //         ROS_INFO("pubed ");
    //     }
    // }

    // /// @brief 开始
    // void begin_comminicate(){
    //     std::thread pub_t(pub_t_fun);
    //     pub_t.join();
    // }
}