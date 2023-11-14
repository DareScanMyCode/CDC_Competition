#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "../include/inspector.h"
#include "../include/photo_taker.h"
#include "GridMap.h"
#include "GridNode.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <ctime>

Inspector inspector;
Eigen::Vector3d target_point;
void odometry_callback(nav_msgs::Odometry odom){
    // 读取 uav0 的 odometry 数据
    // 更新 inspector 的位置和姿态信息
    inspector.update_state(odom);
    inspector.update_map();
    time_t t0 = time(nullptr);
    target_point=inspector.get_target_point().pos;
    
    std::cout<<"target_point"<<target_point<<std::endl;
    ROS_INFO("odometry_callback");
    std::cout << "Update Target Time Cost:" << (time(nullptr)-t0) << std::endl;
}

int main(int argc, char** argv) {
    // 定义两个 double 变量和一个存储 Eigen::Vector3i 的 vector
    double x_min,x_max,y_min,y_max,z_min,z_max,resolution;
    resolution=2.0;
    std::vector<Eigen::Vector3d> vector3d_face_data;
    std::vector<Eigen::Vector3d> vector3d_all_data;
    std::vector<Eigen::Vector3d> vector3d_search_data;


    // 打开文本文件
    std::ifstream input_file_face("/home/gjr/WorkSpace/ws_caric/src/2023caric_competition_xmu/files/face_points.txt");
    std::ifstream input_file_all("/home/gjr/WorkSpace/ws_caric/src/2023caric_competition_xmu/files/all_points.txt");
    std::ifstream input_file_search("/home/gjr/WorkSpace/ws_caric/src/2023caric_competition_xmu/files/search_points.txt");

    // 检查文件是否成功打开
    if (!input_file_face.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    input_file_face >> x_min>>x_max>>y_min>>y_max>>z_min>>z_max;
    x_min=x_min-5;
    x_max=x_max+5;
    y_min=y_min-5;
    y_max=y_max+5;
    z_min=z_min-5;
    z_max=z_max+5;
    if (!input_file_all.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    if (!input_file_search.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    //有换行符
    input_file_face.ignore(1);
    double inx, iny, inz;
    while (input_file_face >> inx >> iny >> inz) {
        vector3d_face_data.push_back(Eigen::Vector3d(inx, iny, inz));
        input_file_face.ignore(1);
    }
    input_file_face.close();

    double inx1, iny1, inz1;
    while (input_file_all >> inx1 >> iny1 >> inz1) {
        vector3d_all_data.push_back(Eigen::Vector3d(inx1, iny1, inz1));
        input_file_all.ignore(1);
    }
    input_file_all.close();

    double inx2, iny2, inz2;
    while (input_file_search >> inx2 >> iny2 >> inz2) {
        vector3d_search_data.push_back(Eigen::Vector3d(inx2, iny2, inz2));
        input_file_search.ignore(1);
    }
    input_file_search.close();

    // std::cout << "Eigen::Vector3d data:" << std::endl;
    // for (const auto& v : vector3d_face_data) {
    //     std::cout << v.transpose() << std::endl;
    // }
    std::cout<<"num of face points"<<vector3d_face_data.size()<<std::endl;
    std::cout<<"num of all points"<<vector3d_all_data.size()<<std::endl;
    
    double x,y,z,roll,pitch,yaw;
    x=0;y=0;z=0;roll=0;pitch=0;yaw=0;
    inspector.init(x,y,z,roll,pitch,yaw,x_min,x_max,y_min,y_max,z_min,z_max,resolution,vector3d_search_data);
    inspector.init_map(vector3d_all_data);
    inspector.set_inpect_grid(vector3d_face_data);
    // inspector.discrete_map();

    //ros
    ros::init(argc, argv, "search_map_publisher");
    ros::NodeHandle nh;

    //订阅odometry
    ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("/jurong/ground_truth/odometry", 10, odometry_callback);
    // 创建一个发布者来发布 Marker 消息
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("grid_map_marker", 10);
    //测试发送地图
    ros::Publisher map_pub = nh.advertise<GridMap>("my_gridmap", 1, true);


    // 创建一个 std::vector<Eigen::Vector3d> 来模拟网格地图数据
    std::vector<Eigen::Vector3d> grid_map_data;
    // 填充 grid_map_data

    // 创建一个 Marker 消息
    visualization_msgs::Marker marker_not_inspected;
    marker_not_inspected.header.frame_id = "world"; // 设置坐标系
    marker_not_inspected.header.stamp = ros::Time::now();
    marker_not_inspected.ns = "search_map_3";
    marker_not_inspected.id = 0;
    marker_not_inspected.type = visualization_msgs::Marker::POINTS;
    marker_not_inspected.action = visualization_msgs::Marker::ADD;
    marker_not_inspected.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker_not_inspected.scale.x = 1;
    marker_not_inspected.scale.y = 1;
    marker_not_inspected.scale.z = 1;

    // 设置点的颜色
    marker_not_inspected.color.r = 0.0;
    marker_not_inspected.color.g = 0.0;
    marker_not_inspected.color.b = 1.0;
    marker_not_inspected.color.a = 1.0;

    //另外一个marker
    visualization_msgs::Marker marker_inspected;
    marker_inspected.header.frame_id = "world"; // 设置坐标系
    marker_inspected.header.stamp = ros::Time::now();
    marker_inspected.ns = "search_map_2";
    marker_inspected.id = 1;
    marker_inspected.type = visualization_msgs::Marker::POINTS;
    marker_inspected.action = visualization_msgs::Marker::ADD;
    marker_inspected.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker_inspected.scale.x = 2;
    marker_inspected.scale.y = 2;
    marker_inspected.scale.z = 2;

    marker_inspected.color.r = 1.0;
    marker_inspected.color.g = 0.0;
    marker_inspected.color.b = 0.0;
    marker_inspected.color.a = 1.0;

    //
    visualization_msgs::Marker marker_occupied;
    marker_occupied.header.frame_id = "world"; // 设置坐标系
    marker_occupied.header.stamp = ros::Time::now();
    marker_occupied.ns = "search_map_1";
    marker_occupied.id = 2;
    marker_occupied.type = visualization_msgs::Marker::POINTS;
    marker_occupied.action = visualization_msgs::Marker::ADD;
    marker_occupied.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker_occupied.scale.x = 1;
    marker_occupied.scale.y = 1;
    marker_occupied.scale.z = 1;

    marker_occupied.color.r = 0.0;
    marker_occupied.color.g = 1.0;
    marker_occupied.color.b = 0.0;
    marker_occupied.color.a = 1.0;

    //
    visualization_msgs::Marker marker_target;
    marker_target.header.frame_id = "world"; // 设置坐标系
    marker_target.header.stamp = ros::Time::now();
    marker_target.ns = "search_map_4";
    marker_target.id = 2;
    marker_target.type = visualization_msgs::Marker::POINTS;
    marker_target.action = visualization_msgs::Marker::ADD;
    marker_target.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker_target.scale.x = 5;
    marker_target.scale.y = 5;
    marker_target.scale.z = 5;

    marker_target.color.r = 1.0;
    marker_target.color.g = 1.0;
    marker_target.color.b = 0.0;
    marker_target.color.a = 1.0;

        //
    visualization_msgs::Marker marker_search;
    marker_search.header.frame_id = "world"; // 设置坐标系
    marker_search.header.stamp = ros::Time::now();
    marker_search.ns = "search_map_5";
    marker_search.id = 2;
    marker_search.type = visualization_msgs::Marker::POINTS;
    marker_search.action = visualization_msgs::Marker::ADD;
    marker_search.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker_search.scale.x = 2;
    marker_search.scale.y = 2;
    marker_search.scale.z = 2;

    marker_search.color.r = 0.5;
    marker_search.color.g = 0.5;
    marker_search.color.b = 0.2;
    marker_search.color.a = 1.0;

    // 发布 Marker 消息
    while (ros::ok()) {
        // 清空 Marker 消息的点
        marker_not_inspected.points.clear();
        marker_inspected.points.clear();
        marker_occupied.points.clear();
        marker_target.points.clear();
        marker_search.points.clear();

        inspector.get_occupied_points(marker_occupied);
        inspector.get_uninspected_points(marker_not_inspected);
        inspector.get_inspected_points(marker_inspected);
        inspector.get_search_points(marker_search);

        //显示target_point
        geometry_msgs::Point p;
        p.x=target_point(0);
        p.y=target_point(1);
        p.z=target_point(2);
        marker_target.points.push_back(p);

        // marker_pub.publish(marker_occupied);
        // marker_pub.publish(marker_not_inspected);
        // marker_pub.publish(marker_inspected);
        marker_pub.publish(marker_search);
        marker_pub.publish(marker_target);



        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    return 0;

}
