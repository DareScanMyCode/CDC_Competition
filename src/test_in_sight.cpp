#include <Eigen/Dense>
#include <tf/tf.h>
#include <cmath>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/Odometry.h"
// #include <tf/tf.h>
#include "geometry_msgs/TwistStamped.h"
// 假设 ori 和 gimbal_ori 已经被定义在全局或者传入函数中
Eigen::Vector3d ori(0.0, 0.0, 0.0); // 示例数据，需要根据实际情况修改
Eigen::Vector3d gimbal_ori(0.0, 0.0, 0.0); // 示例数据，需要根据实际情况修改
Eigen::Vector3d pos(0.0, 0.0, 0.0); // 示例数据，需要根据实际情况修改


nav_msgs::Odometry my_odom = nav_msgs::Odometry();
void odom_cb(const nav_msgs::OdometryConstPtr & msg){
    my_odom = *msg;
    pos = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
    );
    // 从odom中获取四元数
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    ori = Eigen::Vector3d(
        -roll, -pitch, -yaw
    );
}

geometry_msgs::TwistStamped my_gimbal = geometry_msgs::TwistStamped();
void gimbal_cb(const geometry_msgs::TwistStamped::ConstPtr & msg){
    my_gimbal = *msg;
    gimbal_ori = Eigen::Vector3d(
        -msg->twist.linear.x,
        -msg->twist.linear.y,
        -msg->twist.linear.z
    );
}

// 坐标系？相对坐标？
bool in_sight(Eigen::Vector3d photo_point){
    tf::Matrix3x3 world2uav, uav2camera;
    tf::Vector3 target_point(photo_point[0],photo_point[1],photo_point[2]);

    world2uav.setRPY(ori[0], ori[1], ori[2]);
    uav2camera.setRPY(gimbal_ori[0], gimbal_ori[1], gimbal_ori[2]);
    tf::Vector3 tf_target_dir =  uav2camera * world2uav * target_point;
    
    Eigen::Vector3d target_dir(tf_target_dir[0],tf_target_dir[1],tf_target_dir[2]);
    Eigen::Vector3d curr_dir(1,0,0); 
    double angle = acos((target_dir.dot(curr_dir))/(target_dir.norm()*curr_dir.norm()));
    return abs(angle/M_PI*180) < 45;
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "test_in_sight");
    ros::NodeHandle nh("~");
    ros::Rate rate(5);

    std::string node_name;
    nh.getParam("node_name", node_name);

    ros::Subscriber sub_odom = nh.subscribe("/"+node_name+"/ground_truth/odometry", 1, odom_cb);
    ros::Subscriber sub_gimbal = nh.subscribe("/"+node_name+"/gimbal", 1, gimbal_cb);
    ros::Publisher pub_in = nh.advertise<visualization_msgs::Marker>("/"+node_name+"/in_sight_marker", 10);
    ros::Publisher pub_out = nh.advertise<visualization_msgs::Marker>("/"+node_name+"/out_sight_marker", 10);
    while(ros::ok()){
        // ROS_INFO("111");
        visualization_msgs::Marker in_marker;
        visualization_msgs::Marker out_marker;
        in_marker.header.frame_id = "world";
        in_marker.header.stamp = ros::Time::now();
        in_marker.ns = "ns1";
        in_marker.id = 0;
        in_marker.type = visualization_msgs::Marker::POINTS;
        in_marker.action = visualization_msgs::Marker::ADD;
        in_marker.pose.orientation.w = 1;


        // 设置点的大小,三维点
        in_marker.scale.x = 1;
        in_marker.scale.y = 1;
        in_marker.scale.z = 1;

        // 设置点的颜色
        in_marker.color.r = 1;
        in_marker.color.g = 0;
        in_marker.color.b = 1;
        in_marker.color.a = 0.45;

        out_marker.header.frame_id = "world";
        out_marker.header.stamp = ros::Time::now();
        out_marker.ns = "ns1";
        out_marker.id = 0;
        out_marker.type = visualization_msgs::Marker::POINTS;
        out_marker.action = visualization_msgs::Marker::ADD;
        out_marker.pose.orientation.w = 1;
        out_marker.scale.x = 1;
        out_marker.scale.y = 1;
        out_marker.scale.z = 1;
        out_marker.color.r = 0;
        out_marker.color.g = 1;
        out_marker.color.b = 0;
        out_marker.color.a = 1;
        int num = 8;
        for(int i = -num; i<num; i++){
            for(int j = -num; j<num; j++){
                for(int k=-num; k<num; k++){
                    if(i==0&&j==0&&k==0) continue;
                    Eigen::Vector3d test_point(  i, //pos.x() +  
                                                 j, //pos.y() + 
                                                 k); //pos.z() + ;
                    bool result = in_sight(test_point);
                    geometry_msgs::Point p;
                    p.x = pos.x() + test_point.x();
                    p.y = pos.y() + test_point.y();
                    p.z = pos.z() + test_point.z();
                    // 输出结果
                    if(result) {
                        in_marker.points.push_back(p);
                    } else {
                        out_marker.points.push_back(p);
                    }
                }
            }
        }
        pub_in.publish(in_marker);
        pub_out.publish(out_marker);
        ros::spinOnce();
        rate.sleep();
    }
    // 创建一个测试点
    // Eigen::Vector3d test_point(1.0, 1.0, 1.0);

    // // 调用 in_sight 函数
    // bool result = in_sight(test_point);

    // // 输出结果
    // if(result) {
    //     std::cout << "Point is in sight." << std::endl;
    // } else {
    //     std::cout << "Point is not in sight." << std::endl;
    // }

    return 0;
}
