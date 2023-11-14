#include <iostream>
#include <algorithm>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>


#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "../include/photo_taker.h"
#include "../include/inspector.h"



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


int cluster_index = 0;    // 指示当前在检查第几个类
int point_index = 0;      // 指示当前在检查类中的第几个点
bool need_arrange = true; // 指示当前的巡检还没被安排
std::vector<Eigen::Vector3d> travel_list;
void travel_list_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    travel_list.clear();
    for (int i = 0; i < msg->data.size(); i += 3)
    {
        Eigen::Vector3d point;
        point.x() = msg->data[i];
        point.y() = msg->data[i + 1];
        point.z() = msg->data[i + 2];
        travel_list.push_back(point);
    }
    point_index=0;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh("~");
    
    // 读取yaml文件中的uav_name
    std::string uav_name="changi";

    nh.getParam("uav_name", uav_name);

    //声明一个taker_controller类
    taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 订阅无人机的位置,消息类型是nav_msgs/Odometry
    ros::Subscriber uav_odom_sub = nh.subscribe("/changi/ground_truth/odometry", 1, uav_odom_callback); 
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe("/changi/gimbal", 1, gimbal_callback);
    //订阅travel_list
    ros::Subscriber travel_list_sub=nh.subscribe("/changi/travel_list",1,travel_list_callback);

    //发布这个是否检查完成
    ros::Publisher check_finish_pub = nh.advertise<std_msgs::Bool>("/changi/check", 1);

    while (ros::ok())
    {   
        my_controller.update(uav_pos,uav_vel,uav_ori,uav_w,gimbal_pos,gimbal_ori);
        if(point_index==travel_list.size()){
            std_msgs::Bool check_finish;
            check_finish.data=true;
            check_finish_pub.publish(check_finish);
        }
        if(travel_list.size()>0 && point_index!=travel_list.size()){    //不需要安排，就根据当前位置和姿态发送控制指令，包括本身运动和云台控制
            ROS_INFO("CONTROL");
            ROS_INFO("NOW %d - %d",point_index,travel_list.size());
            my_controller.uav_controller(travel_list, need_arrange, cluster_index, point_index,1.5,false);
            // my_controller.uav_controller(travel_list, need_arrange, cluster_index, point_index,3,true); // 这几个参数都是引用，当所有都检查完后，need_arrange会变成true,cluster_index+1,进入下一个类，point_index=0
            my_controller.gimbal_controller(travel_list, point_index);
        }
        ros::spinOnce();
        ros::Rate(60).sleep();
    }   
    
}

