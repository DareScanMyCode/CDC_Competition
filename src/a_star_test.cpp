#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>
//include pcl
#include <pcl_conversions/pcl_conversions.h>

#include "../include/photo_taker.h"
#include "../include/inspector.h"

Inspector inspector;
void world_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    // ROS_INFO("world_octomap_callback");
    //根据获取的octomap更新我的grid_map
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

        if (octree) {
            for (octomap::OcTree::iterator it = octree->begin(); it != octree->end(); ++it) {
                if (octree->isNodeOccupied(*it)) {
                    Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());

                    // Check if the position is within your GridMap bounds
                    if (inspector.grid_map.inmap(position)) {
                        inspector.grid_map.set_occupancy_pos(position, true);
                    }
                }
                else{
                    Eigen::Vector3d position(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z());
                    if(inspector.grid_map.inmap(position)){
                        inspector.grid_map.set_occupancy_pos(position, false);
                    }
                }
            }
        inspector.grid_map.inflate_map();
        } 
        else {
            ROS_ERROR("Failed to cast the abstract tree to OcTree.");
        }
    } 
    else {
        ROS_ERROR("Received an unsupported octomap type.");
    }
}

Eigen::Vector3d uav_pos, taker1, taker2;
Eigen::Vector3d uav_vel, uav_ori, uav_w;
void uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    // ROS_INFO("uav_odom_callback");
    uav_pos.x() = msg->pose.pose.position.x;
    uav_pos.y() = msg->pose.pose.position.y;
    uav_pos.z() = msg->pose.pose.position.z;
    uav_vel.x() = msg->twist.twist.linear.x;
    uav_vel.y() = msg->twist.twist.linear.y;
    uav_vel.z() = msg->twist.twist.linear.z;
    // 将四元素转为姿态角
    Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    //四元数转为角度
    uav_ori=Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()).toRotationMatrix().eulerAngles(2,1,0);
    uav_w.x() = msg->twist.twist.angular.x;
    uav_w.y() = msg->twist.twist.angular.y;
    uav_w.z() = msg->twist.twist.angular.z;
}
void uav_odom_callback1(const nav_msgs::Odometry::ConstPtr &msg)
{
    taker1.x() = msg->pose.pose.position.x;
    taker1.y() = msg->pose.pose.position.y;
    taker1.z() = msg->pose.pose.position.z;
}
void uav_odom_callback2(const nav_msgs::Odometry::ConstPtr &msg)
{
    taker2.x() = msg->pose.pose.position.x;
    taker2.y() = msg->pose.pose.position.y;
    taker2.z() = msg->pose.pose.position.z;
}
Eigen::Vector3d gimbal_pos, gimbal_ori;
void gimbal_callback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    gimbal_pos.x() = msg->twist.linear.x;
    gimbal_pos.y() = msg->twist.linear.y;
    gimbal_pos.z() = msg->twist.linear.z;
    gimbal_ori.x() = msg->twist.angular.x;
    gimbal_ori.y() = msg->twist.angular.y;
    gimbal_ori.z() = msg->twist.angular.z;
}

std::vector<Eigen::Vector3d> interest_points;
std::vector<Eigen::Vector3d> target_points; // 要追踪的目标点们
size_t pre_interest_points_num=0;
void interest_points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // ROS_INFO("interest_points_callback");
    interest_points.clear();
    // 将sensor_msgs::PointCloud2的数据读入interest_points
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        Eigen::Vector3d point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        interest_points.push_back(point);
    }

    if ((interest_points.size()-pre_interest_points_num) > 300||(pre_interest_points_num==0 && interest_points.size()>10)) //&& inspector.grid_map != NULL)
    {
        pre_interest_points_num=interest_points.size();
        std::vector<std::vector<Eigen::Vector3d>> clusters = kMeansClustering(interest_points, 3, 0.01, 100);
        std::vector<Eigen::Vector3d> centroid = cal_centroid(clusters, 3);
        Eigen::Matrix3d dis;
        // 根据距离为每个聚类中心分配一个taker
        for (int i = 0; i < 3; i++)
        {
            dis(i, 0) = (centroid[i] - uav_pos).norm();
            dis(i, 1) = (centroid[i] - taker1).norm();
            dis(i, 2) = (centroid[i] - taker2).norm();
        }
        for (int n = 0; n < 3; n++)
        {
            // 找到所有dis中最小的值的二维索引
            int min = dis(0, 0);
            int min_i = 0;
            int min_j = 0;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (dis(i, j) < min)
                    {
                        min = dis(i, j);
                        min_i = i;
                        min_j = j;
                    }
                }
            }
            if (min_j == 0)
            {
                target_points = clusters[min_i];
                break;
            }
            else
            {
                // 第i行和第j列所有元素设为1000000
                for (int i = 0; i < 3; i++)
                {
                    dis(i, min_j) = INT_MAX;
                }
                for (int j = 0; j < 3; j++)
                {
                    dis(min_i, j) = INT_MAX;
                }
            }
        }
    }
}

//创建marker，用于rviz显示
void create_Marker(string frame_id,string ns,int scale,int r,int g, int b,visualization_msgs::Marker & marker){
    marker.header.frame_id = frame_id; // 设置坐标系
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    // 设置点的大小,三维点
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // 设置点的颜色
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
}

void fill_points(visualization_msgs::Marker & marker,std::vector<Eigen::Vector3d> points){
    for (size_t i = 0; i < points.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = points[i].x();
        p.y = points[i].y();
        p.z = points[i].z();
        marker.points.push_back(p);
    }
}


int main(int argc, char **argv)
{   

    // 定义两个 double 变量和一个存储 Eigen::Vector3i 的 vector
    double x_min,x_max,y_min,y_max,z_min,z_max,resolution;
    resolution=1.0;
    std::vector<Eigen::Vector3d> vector3d_face_data;
    std::vector<Eigen::Vector3d> vector3d_all_data;
    std::vector<Eigen::Vector3d> vector3d_search_data;


    // 打开文本文件
    std::ifstream input_file_face("/home/paobuliao/ws_caric/src/caric_competition_xmu/files/face_points.txt");
    std::ifstream input_file_all("/home/paobuliao/ws_caric/src/caric_competition_xmu/files/all_points.txt");
    std::ifstream input_file_search("/home/paobuliao/ws_caric/src/caric_competition_xmu/files/search_points.txt");

    // 检查文件是否成功打开
    if (!input_file_face.is_open()) {
        std::cerr << "无法打开文件" << std::endl;
        return 1;
    }
    input_file_face >> x_min>>x_max>>y_min>>y_max>>z_min>>z_max;
    x_min=x_min-40;
    x_max=x_max+40;
    y_min=y_min-40;
    y_max=y_max+40;
    z_min=z_min-20;
    z_max=z_max+20;
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

    ros::init(argc, argv, "photo_taker");
    ros::NodeHandle nh("~");
    
    // 读取yaml文件中的uav_name
    std::string uav_name="changi";

    nh.getParam("uav_name", uav_name);
    // nh.param("uav_name", uav_name, std::string("uav1"));
    //声明一个taker_controller类
    taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 订阅ocotomap,由gcs发送，其实也可以来自其他无人机
    ros::Subscriber world_octomap_sub = nh.subscribe("/world/octomap", 1, world_octomap_callback);
    // 订阅兴趣点，来自gcs(全局的)，消息类型是sensor_msgs/PointCloud，也可以来自其他无人机
    ros::Subscriber interest_points_sub = nh.subscribe("/interest_cloud", 1, interest_points_callback); // 在这里启动聚类
    // 订阅其他两个taker无人机的位置，包括所有无人机的位置，消息类型是nav_msgs/Odometry
    ros::Subscriber uav_odom_sub = nh.subscribe("/changi/ground_truth/odometry", 1, uav_odom_callback); // taker1从yaml文件中读取
    ros::Subscriber uav_odom_sub1 = nh.subscribe("/sentosa/ground_truth/odometry", 1, uav_odom_callback1);
    ros::Subscriber uav_odom_sub2 = nh.subscribe("/nanyang/ground_truth/odometry", 1, uav_odom_callback2);
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe("/changi/gimbal", 1, gimbal_callback);

    // 创建一个发布者来发布 Marker 消息
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("interest_points", 10);


    // 创建一个 Marker 消息
    visualization_msgs::Marker marker_start_point;
    create_Marker("world","start_pos",2,1,0,0,marker_start_point);
    visualization_msgs::Marker marker_end_point;
    create_Marker("world","end_point",2,0,1,0,marker_end_point);
    visualization_msgs::Marker marker_travel_point;
    create_Marker("world","travel_point",1,0,0,1,marker_travel_point);

    
    visualization_msgs::Marker marker_open_list;    
    create_Marker("world","open_list",1,0.5,0.5,0.5,marker_open_list);
    visualization_msgs::Marker marker_now_point;
    create_Marker("world","now_point",3,1,1,0,marker_now_point);

    Eigen::Vector3d start_pos;
    Eigen::Vector3d end_pos;
    std::vector<Eigen::Vector3d> travel_list;//经过A*修改后应去的路径
    while (ros::ok())
    {   
        //输入start_pos和end_pos
        std::cout<<"input start_pos"<<std::endl;
        std::cin>>start_pos(0)>>start_pos(1)>>start_pos(2);
        std::cout<<"input end_pos"<<std::endl;
        std::cin>>end_pos(0)>>end_pos(1)>>end_pos(2);
        
        travel_list.clear();//是二级聚类再避障后需要走的点


       
        // A*搜索
        std::vector<Eigen::Vector3d> pos_list;
        AStar a_star;
        a_star.map = inspector.grid_map;
        bool a_star_success = a_star.search(pos_list, start_pos, end_pos);
        // if (a_star_success && pos_list.size() < 1000)
        // { // a*路径不能太长
            // 将poslist除首尾外的点插入到travel_list中；
            //因为是回溯路径，所以要倒着插入
            for (int j = pos_list.size() - 2; j > 0; j--)//这里是pos_list.size()-2，因为pos_list的最后一个点是end_pos，不需要插入
            {
                travel_list.push_back(pos_list[j]);
            }
        // }
        std::cout<<"a_star_success"<<a_star_success<<std::endl;

        //发布marker
        fill_points(marker_start_point,std::vector<Eigen::Vector3d>{start_pos});
        fill_points(marker_end_point,std::vector<Eigen::Vector3d>{end_pos});
        fill_points(marker_travel_point,travel_list);
        marker_pub.publish(marker_start_point);
        marker_pub.publish(marker_end_point);
        marker_pub.publish(marker_travel_point);
        
        //清空
        marker_start_point.points.clear();
        marker_end_point.points.clear();
        marker_travel_point.points.clear();


        ros::spinOnce();
        ros::Rate(10).sleep();
    }
}

