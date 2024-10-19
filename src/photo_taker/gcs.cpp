#include <iostream>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "../include/photo_taker/photo_taker.h"
#include "../include/photo_taker/inspector_mini.h"

#include "../include/photo_taker/communication.h"

void gcs_score_callback(const sensor_msgs::PointCloud::ConstPtr & msg){
    //打印位置以及分數
    // for(size_t i=0;i<msg->points.size();i++){
    //     ROS_INFO("x:%f y:%f z:%f score:%f",msg->points[i].x,msg->points[i].y,msg->points[i].z,msg->channels[3].values[i]);
    // }
    caric_competition_xmu::gcs_score_pub.publish(*msg);
    // ROS_INFO("gcs_score_callback");
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gcs");
    ros::NodeHandle nh("~");
    std::string node_name;
    nh.getParam("node_name", node_name);
    node_name = "gcs";
    caric_competition_xmu::init_Communicator(nh, node_name, false);
    // added begin
    ros::Publisher ping_msg_pub;
    caric_mission::CreatePPComTopic srv;
    srv.request.source = "gcs";
    srv.request.targets = {"all"};
    srv.request.topic_name = "/gcs_ping_msg";
    srv.request.package_name = "std_msgs";
    srv.request.message_type = "Bool";
    ros::ServiceClient create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("/create_ppcom_topic");
    create_ppcom_topic.call(srv);
    ROS_INFO("[gcs ] : ping msg communicator gened, result: %s", srv.response.result.c_str());
    ping_msg_pub = nh.advertise<std_msgs::Bool>(srv.request.topic_name,1);
    // done

    ros::Subscriber gcs_score_sub = nh.subscribe("/gcs/score", 1, gcs_score_callback);
    ros::Rate rate(10);
    std_msgs::Bool live_msg;
    live_msg.data = 1;
    while (ros::ok()){
        ping_msg_pub.publish(live_msg);
        ros::spinOnce();
        rate.sleep();
    }
}