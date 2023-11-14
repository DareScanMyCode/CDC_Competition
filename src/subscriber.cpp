#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

//include pcl
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "../include/photo_taker.h"
#include "../include/inspector.h"


Inspector inspector;
//计时，时间太短不更新
size_t pre_size=0;
void world_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{

    ROS_INFO("world_octomap_callback");
    if(msg->data.size()-pre_size<100)
        return;
    //根据获取的octomap更新我的grid_map
    octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
        pre_size=msg->data.size();
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
    ROS_INFO("uav_odom_callback");
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
//消息类型是sensor_msgs/PointCloud
bool first_interest_points=true;

void interest_points_callback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    ROS_INFO("interest_points_callback");
    if(msg->points.size()-interest_points.size()<60){
        if(first_interest_points && interest_points.size()>20){
            first_interest_points=false;
        }
        else{
            return;
        }
    }
    ROS_INFO("update_interest_points");
    interest_points.clear();
    // 将sensor_msgs::PointCloud的数据读入interest_points
    for (size_t i = 0; i < msg->points.size(); i++)
    {
        Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
        interest_points.push_back(point);
    }
  
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

int cluster_index = 0;    // 指示当前在检查第几个类
int point_index = 0;      // 指示当前在检查类中的第几个点
bool need_arrange = true; // 指示当前的巡检还没被安排
void check_callback(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_INFO("check_callback");
    if (msg->data == true)
    {
        // 检查完了，就要重新聚类
        need_arrange = true;
        cluster_index++;
        point_index = 0;
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
    // taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 订阅ocotomap,由gcs发送，其实也可以来自其他无人机
    ros::Subscriber world_octomap_sub = nh.subscribe("/world/octomap", 1, world_octomap_callback);
    // 订阅兴趣点，来自gcs(全局的)，消息类型是sensor_msgs/PointCloud，也可以来自其他无人机
    ros::Subscriber interest_points_sub = nh.subscribe("/gcs/score", 1, interest_points_callback); // 在这里启动聚类
    // 订阅其他两个taker无人机的位置，包括所有无人机的位置，消息类型是nav_msgs/Odometry
    ros::Subscriber uav_odom_sub = nh.subscribe("/changi/ground_truth/odometry", 1, uav_odom_callback); // taker1从yaml文件中读取
    ros::Subscriber uav_odom_sub1 = nh.subscribe("/sentosa/ground_truth/odometry", 1, uav_odom_callback1);
    ros::Subscriber uav_odom_sub2 = nh.subscribe("/nanyang/ground_truth/odometry", 1, uav_odom_callback2);
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe("/changi/gimbal", 1, gimbal_callback);
    //订阅一个bool量，这个类检查完没有
    ros::Subscriber check_sub = nh.subscribe("/changi/check", 1, check_callback);

    // 创建一个发布者来发布 Marker 消息
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("interest_points", 10);
    //创建一个发布者来发布一个std::vector<Eigen::Vector3d>消息，记录的是travel_list
    ros::Publisher travel_list_pub = nh.advertise<std_msgs::Float64MultiArray>("/changi/travel_list", 10);

    // 创建一个 Marker 消息
    visualization_msgs::Marker marker_interest_point;
    create_Marker("world","interest_point",resolution,1,0,0,marker_interest_point);
    visualization_msgs::Marker marker_target_point;
    create_Marker("world","target_point",resolution,0,0,1,marker_target_point);
    visualization_msgs::Marker marker_spec_point;
    create_Marker("world","specified_point",resolution,0,1,0,marker_spec_point);
    visualization_msgs::Marker marker_cluster_cluster_points;
    create_Marker("world","cluster_cluster_points",resolution,1,1,0,marker_cluster_cluster_points);
    visualization_msgs::Marker marker_points_near_free;
    create_Marker("world","points_near_free",resolution,1,0,1,marker_points_near_free);
    visualization_msgs::Marker marker_uav_pos;
    create_Marker("world","uav_pos",3,0,1,1,marker_uav_pos);
    visualization_msgs::Marker marker_occupied_points;
    create_Marker("world","occupied_points",resolution,0.5,0.5,0.5,marker_occupied_points);

    int num_for_each_cluster = 7;

    double check_resolution = 0.5;
    std::vector<Eigen::Vector3d> travel_list;//经过A*修改后应去的路径
    std::vector<std::vector<Eigen::Vector3d>> target_clusters;//经过了二级聚类后的点
    std::vector<Eigen::Vector3d> centroid;
    std::vector<int> arrange_cluster_index;
    std::vector<Eigen::Vector3d> points_cluster_cluster,points_near_free;
    std::vector<int> arrange_point_index;//经过二级聚类后，目前要的检查的类中的点的安排顺序
    //获取当前时间
    ros::Time now_time = ros::Time::now();
    // ros::Time ori_time = ros::Time::now();
    ros::Time pre_time = ros::Time::now();
    bool first_arrange=true;
    bool get_predefined_height=false;
    while (ros::ok())
    {   
        //rviz显示interest_point
        marker_interest_point.points.clear();
        marker_target_point.points.clear();
        marker_spec_point.points.clear();
        marker_cluster_cluster_points.points.clear();
        marker_points_near_free.points.clear();
        marker_occupied_points.points.clear();
        fill_points(marker_interest_point,interest_points);
        fill_points(marker_target_point,target_points);
        fill_points(marker_spec_point,travel_list);
        fill_points(marker_cluster_cluster_points,points_cluster_cluster);
        fill_points(marker_points_near_free,points_near_free);
        inspector.get_inflated_points(marker_occupied_points);
        geometry_msgs::Point uav_point;
        uav_point.x=uav_pos.x();
        uav_point.y=uav_pos.y();
        uav_point.z=uav_pos.z();
        marker_uav_pos.points.push_back(uav_point);
        marker_pub.publish(marker_interest_point);
        marker_pub.publish(marker_target_point);
        marker_pub.publish(marker_spec_point);
        marker_pub.publish(marker_cluster_cluster_points);
        marker_pub.publish(marker_points_near_free);
        marker_pub.publish(marker_uav_pos);
        marker_pub.publish(marker_occupied_points);

        // 这个判断要不要重新初始化
        now_time = ros::Time::now();
        //判断时间间隔是否大于5s
        auto duration = now_time.toSec()-pre_time.toSec();
        //1. target_points是这家飞机所有要检查的点，它不应该过少
        if ((duration >= 40||first_arrange) && target_points.size() > 10)
        // if(target_points.size()>10)
        { // 这个时候加入时间间隔限制，5s或者10s，太短的话，可能会导致无人机频繁调整航线
            first_arrange=false;
            pre_time = now_time;
            int num_smaller_clusters = std::max(1,int(target_points.size() / num_for_each_cluster));
            // std::vector<double> max_time_alloc;
            target_clusters = kMeansClustering(target_points, num_smaller_clusters, 0.01, 100);
            //滤去那些点数为0的类
            for(size_t i=0;i<target_clusters.size();i++){
                if(target_clusters[i].size()==0){
                    target_clusters.erase(target_clusters.begin()+i);
                    i--;
                }
            }
            centroid = cal_centroid(target_clusters, int(target_clusters.size()));
            arrange_cluster_index = arrange_points(uav_pos, centroid);
            arrange_cluster_index.erase(arrange_cluster_index.begin(),arrange_cluster_index.begin()+1);
            // 重置检查的类和点
            cluster_index = 0;
            point_index = 0;
        }
        //2. 现在处理的是二次聚类后的一部分点
        // 这个判断当前检查的类路径有没有被规划
    
        if (need_arrange && arrange_cluster_index.size() > 1) // 这有个终止条件我还没加，注意
        {   
            // cluster_index++;
            // 重置travel_list
            travel_list.clear();//是二级聚类再避障后需要走的点
            points_cluster_cluster = target_clusters[arrange_cluster_index[cluster_index]-1];//是二级聚类后，目前要检查的类
            arrange_point_index = arrange_points(uav_pos, points_cluster_cluster);//points_cluster_cluster经过处理后前面回多出来uav_pos
            // 对每一个兴趣点，我都在他附近搜索一个在自由空间的点，这个基础上再进行规划
            //  to do
            std::cout<<"before replace"<<std::endl;
            for(auto &t_point:points_cluster_cluster){
                //替换每一个点
                std::cout<<t_point.transpose()<<std::endl;
            }
            for(auto &t_point:points_cluster_cluster){
                //替换每一个点
                // Eigen::Vector3d new_point;
                inspector.grid_map.search_near_free_point(t_point,false);
            }
            std::cout<<"after replace"<<std::endl;
            points_near_free=points_cluster_cluster;
            for(auto &t_point:points_near_free){
                //替换每一个点
                std::cout<<t_point.transpose()<<std::endl;
            }
            //to do
            //我发现这样处理后，可能会存在重复的点，但是直接剔除不一定合适，因为我要拍那么多目标 

            // check是否没有障碍物(两两)
            size_t i = 0;
            size_t max_astar_points = 100;
            while (i < arrange_point_index.size() - 1)
            {
                // 如果travel_list最后一个元素与points[arrange_point_index[i]]相等，无需添加
                if (travel_list.size() == 0 || (travel_list[travel_list.size() - 1] - points_near_free[arrange_point_index[i]]).norm() > 0.0001)
                {   
                    travel_list.push_back(points_near_free[arrange_point_index[i]]);
                }
                if (!check_collision_free(points_near_free[arrange_point_index[i]], points_near_free[arrange_point_index[i + 1]], inspector.grid_map, check_resolution))
                {
                    // A*搜索
                    std::vector<Eigen::Vector3d> pos_list;
                    Eigen::Vector3d start_pos = points_near_free[arrange_point_index[i]];
                    Eigen::Vector3d end_pos = points_near_free[arrange_point_index[i+1]];
                    AStar a_star;
                    a_star.map = inspector.grid_map;
                    bool a_star_success = a_star.search(pos_list, start_pos, end_pos);
                    if (a_star_success && pos_list.size() < max_astar_points)
                    { // a*路径不能太长
                        // 将poslist除首尾外的点插入到travel_list中；
                        //因为是回溯路径，所以要倒着插入
                        for (int j = pos_list.size() - 2; j > 0; j--)//这里是pos_list.size()-2，因为pos_list的最后一个点是end_pos，不需要插入
                        {
                            travel_list.push_back(pos_list[j]);
                        }
                    }
                    else
                    {
                        // 搜索不到，抛弃这个点
                        points_near_free.erase(points_near_free.begin() + i + 1);
                        arrange_point_index.erase(arrange_point_index.begin() + i + 1);
                        // i不变
                        continue;
                    }
                }
                i++;
            }
            // 就尽可能多遍历，
            need_arrange = false;
            
            //发布travel_list
            std_msgs::Float64MultiArray travel_list_msg;
            travel_list_msg.data.clear();
            for(auto &t_point:travel_list){
                travel_list_msg.data.push_back(t_point.x());
                travel_list_msg.data.push_back(t_point.y());
                travel_list_msg.data.push_back(t_point.z());
            }
            travel_list_pub.publish(travel_list_msg);

        }
        // 这个就检查当前的类，更近cluster_index和point_index

        //to do
        //还需要判断是不是所有的点都检查完了，如果检查完了，就要重新聚类
        ros::spinOnce();
        ros::Rate(60).sleep();
    }
}

