#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

#include "../include/photo_taker.h"


octomap::OcTree *world_octomap = NULL;
void world_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    if (world_octomap != NULL)
    {
        delete world_octomap;
    }
    world_octomap = (octomap::OcTree *)octomap_msgs::binaryMsgToMap(*msg);
}

GridMap grid_map;

Eigen::Vector3d uav_pos, taker1, taker2;
Eigen::Vector3d uav_vel, uav_ori, uav_w;
void uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
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
void interest_points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    interest_points.clear();
    // 将sensor_msgs::PointCloud2的数据读入interest_points
    for(int i=0;i<msg->width;i++){
        Eigen::Vector3d point;
        point.x()=msg->data[i*msg->point_step+msg->fields[0].offset];
        point.y()=msg->data[i*msg->point_step+msg->fields[1].offset];
        point.z()=msg->data[i*msg->point_step+msg->fields[2].offset];
        interest_points.push_back(point);
    }

    if (interest_points.size() > 0 && world_octomap != NULL)
    {
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "photo taker");
    ros::NodeHandle nh("~");
    
    // 读取yaml文件中的uav_name
    std::string uav_name;

    nh.getParam("uav_name", uav_name);
    // nh.param("uav_name", uav_name, std::string("uav1"));
    //声明一个taker_controller类
    taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 订阅ocotomap,由gcs发送，其实也可以来自其他无人机
    ros::Subscriber world_octomap_sub = nh.subscribe("all/octomap", 1, world_octomap_callback);
    // 订阅兴趣点，来自gcs(全局的)，消息类型是sensor_msgs/PointCloud，也可以来自其他无人机
    ros::Subscriber interest_points_sub = nh.subscribe("/all/detected_interest_points", 1, interest_points_callback); // 在这里启动聚类
    // 订阅其他两个taker无人机的位置，包括所有无人机的位置，消息类型是nav_msgs/Odometry
    ros::Subscriber uav_odom_sub = nh.subscribe(uav_name, 1, uav_odom_callback); // taker1从yaml文件中读取
    ros::Subscriber uav_odom_sub1 = nh.subscribe(uav_name, 1, uav_odom_callback1);
    ros::Subscriber uav_odom_sub2 = nh.subscribe(uav_name, 1, uav_odom_callback2);
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe(uav_name + "/gimbal", 1, gimbal_callback);

    int num_for_each_cluster = 7;
    // 计时，单位是秒
    //pre_time取now_time前5s
    ros::Time now_time = ros::Time::now();
    ros::Time pre_time = now_time - ros::Duration(5);
    int cluster_index = 0;    // 指示当前在检查第几个类
    int point_index = 0;      // 指示当前在检查类中的第几个点
    bool need_arrange = true; // 指示当前的巡检还没被安排
    double check_resolution = 0.5;
    std::vector<Eigen::Vector3d> travel_list;//经过A*修改后应去的路径
    std::vector<std::vector<Eigen::Vector3d>> point_clusters;
    std::vector<Eigen::Vector3d> centroid;
    std::vector<int> arrange_cluser_index;
    std::vector<Eigen::Vector3d> points;
    std::vector<int> arrange_point_index;
    while (ros::ok())
    {   
        my_controller.update(uav_pos,uav_vel,uav_ori,uav_w,gimbal_pos,gimbal_ori);
        // 这个判断要不要重新初始化
        now_time = ros::Time::now();
        //判断时间间隔是否大于5s
        auto duration = (now_time - pre_time).toSec();
        if (duration >= 5 && target_points.size() > 0)
        { // 这个时候加入时间间隔限制，5s或者10s，太短的话，可能会导致无人机频繁调整航线
            pre_time = now_time;
            int num_smaller_clusters = target_points.size() / num_for_each_cluster;
            // std::vector<double> max_time_alloc;
            point_clusters = kMeansClustering(interest_points, num_smaller_clusters, 0.01, 100);
            centroid = cal_centroid(point_clusters, num_smaller_clusters);
            arrange_cluser_index = arrange_points(uav_pos, centroid);
            // 重置检查的类和点
            cluster_index = 0;
            point_index = 0;
        }
        // 这个判断当前检查的类路径有没有被规划
        if (need_arrange) // 说明这个类还没开始检查
        {
            // 重置travel_list
            travel_list.clear();
            points = point_clusters[arrange_cluser_index[cluster_index]];
            arrange_point_index = arrange_points(uav_pos, points);
            // 对每一个兴趣点，我都在他附近搜索一个在自由空间的点，这个基础上再进行规划
            //  to do

            // check是否没有障碍物(两两)
            size_t i = 0;
            int max_astar_points = 100;
            while (i < arrange_point_index.size() - 1)
            {
                // 如果travel_list最优一个元素与points[arrange_point_index[i]]相等，无需添加
                if (travel_list.size() == 0 || (travel_list[travel_list.size() - 1] - points[arrange_point_index[i]]).norm() > 0.0001)
                {
                    travel_list.push_back(points[arrange_point_index[i]]);
                }
                if (!check_collision_free(points[arrange_point_index[i]], points[arrange_point_index[i + 1]], world_octomap, check_resolution))
                {
                    // A*搜索
                    std::vector<Eigen::Vector3d> pos_list;
                    Eigen::Vector3d start_pos = uav_pos;
                    Eigen::Vector3d end_pos = points[arrange_point_index[0]];
                    AStar a_star;
                    bool a_star_success = a_star.search(pos_list, start_pos, end_pos);
                    if (a_star_success && pos_list.size() < max_astar_points)
                    { // a*路径不能太长
                        // 将poslist除首尾外的点插入到travel_list中；
                        for (int j = 1; j < pos_list.size() - 1; j++)
                        {
                            travel_list.push_back(pos_list[j]);
                        }
                    }
                    else
                    {
                        // 搜索不到，抛弃这个点
                        points.erase(points.begin() + i + 1);
                        // i不变
                        continue;
                    }
                }
                i++;
            }
            // 就尽可能多遍历，
            need_arrange = false;
            // if(i==arrange_point_index.size()-1){
            //     //没有障碍物，可以安排这个类
            //     need_arrange=false;
            // }
            // else{//如果最后还是到达不了最后的点，后面的点都不管了
            //     need_arrange=false;
            // }
        }
        // 这个就检查当前的类，更近cluster_index和point_index
        else
        {                                                                          // 不需要安排，就根据当前位置和姿态发送控制指令，包括本身运动和云台控制
            my_controller.uav_controller(travel_list, need_arrange, cluster_index, point_index); // 这几个参数都是引用，当所有都检查完后，need_arrange会变成true,cluster_index+1,进入下一个类，point_index=0
            my_controller.gimbal_controller(travel_list, point_index);
        }
    }
}

