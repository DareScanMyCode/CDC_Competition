#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "keyboard_input_node");
    ros::NodeHandle nh;
    ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>("/world/is_occupied", 10);

    while (ros::ok()) {
        double x, y, z;
        nav_msgs::Odometry odom_msg;

        // 从键盘读入三个数值
        std::cout << "Enter x, y, z: ";
        std::cin >> x >> y >> z;

        // 填充Odometry消息的位置字段
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = z;

        // 发布消息到话题
        odom_publisher.publish(odom_msg);

        ros::spinOnce();
    }

    return 0;
}
