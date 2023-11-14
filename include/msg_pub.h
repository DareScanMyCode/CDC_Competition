#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <string>
/*
* 发布轨迹话题发布者
* #include "uav_traj_msg_pub.h"
* TrajPuber vel_puber = TrajPuber(nh);
* vel_puber.pub_vel(***);
* 或者发布目标位置或目标加速度，通过：pub_point() and pub_acc
*/
class TrajPuber
{
public:
    TrajPuber(ros::NodeHandle &nh,string topic_name){
        topic_name = "/"+topic_name+"/command/trajectory";
        trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(topic_name, 1);
    }

    int pub_vel(double vx, double vy, double vz, double yaw){
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajset_msg.header.stamp = ros::Time::now();
        trajset_msg.header.frame_id = "world";
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;
        transform_msg.translation.x = 0.0;
        transform_msg.translation.y = 0.0;
        transform_msg.translation.z = 0.0;
        transform_msg.rotation.x = 0.0;
        transform_msg.rotation.y = 0.0;
        transform_msg.rotation.z = sinf(yaw/180.0*M_PI*0.5);
        transform_msg.rotation.w = cosf(yaw/180.0*M_PI*0.5);
        trajpt_msg.transforms.push_back(transform_msg);
        vel_msg.linear.x = vx;
        vel_msg.linear.y = vy;
        vel_msg.linear.z = vz;
        accel_msg.linear.x = 0.0;
        accel_msg.linear.x = 0.0;
        accel_msg.linear.x = 0.0;
        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajectory_pub.publish(trajset_msg);
        return 0;
    }

    int pub_acc(double ax, double ay, double az, double yaw){
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajset_msg.header.stamp = ros::Time::now();
        trajset_msg.header.frame_id = "world";
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;
        transform_msg.translation.x = 0.0;
        transform_msg.translation.y = 0.0;
        transform_msg.translation.z = 0.0;
        transform_msg.rotation.x = 0.0;
        transform_msg.rotation.y = 0.0;
        transform_msg.rotation.z = sinf(yaw/180.0*M_PI*0.5);
        transform_msg.rotation.w = cosf(yaw/180.0*M_PI*0.5);
        trajpt_msg.transforms.push_back(transform_msg);
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        accel_msg.linear.x = ax;
        accel_msg.linear.x = ay;
        accel_msg.linear.x = az;
        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajectory_pub.publish(trajset_msg);
        return 0;
    }

    int pub_point(double px, double py, double pz, double yaw, double rx, double ry){
        trajectory_msgs::MultiDOFJointTrajectory trajset_msg;
        trajset_msg.header.stamp = ros::Time::now();
        trajset_msg.header.frame_id = "world";
        trajectory_msgs::MultiDOFJointTrajectoryPoint trajpt_msg;
        geometry_msgs::Transform transform_msg;
        geometry_msgs::Twist accel_msg, vel_msg;
        transform_msg.translation.x = px;
        transform_msg.translation.y = py;
        transform_msg.translation.z = pz;
        transform_msg.rotation.x = rx;
        transform_msg.rotation.y = ry;
        transform_msg.rotation.z = sinf(yaw/180.0*M_PI*0.5);
        transform_msg.rotation.w = cosf(yaw/180.0*M_PI*0.5);
        trajpt_msg.transforms.push_back(transform_msg);
        vel_msg.linear.x = 0.0;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;
        accel_msg.linear.x = 0.0;
        accel_msg.linear.x = 0.0;
        accel_msg.linear.x = 0.0;
        trajpt_msg.velocities.push_back(vel_msg);
        trajpt_msg.accelerations.push_back(accel_msg);
        trajset_msg.points.push_back(trajpt_msg);

        trajectory_pub.publish(trajset_msg);
        return 0;
    }

private:
    ros::Publisher trajectory_pub;
};


class GimbalPuber
{
    public:
        GimbalPuber(ros::NodeHandle &nh,string topic_name){
            topic_name=topic_name+"/command/gimbal";
            gimbal_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
        }
    int pub_gimbal_command(double target_pitch_rate,double target_yaw_rate){
        geometry_msgs::Twist gimbal_msg;
        gimbal_msg.linear.x  = -1.0;  //setting linear.x to -1.0 enables velocity control mode.
        gimbal_msg.linear.y  =  0.0;  //if linear.x set to 1.0, linear,y and linear.z are the 
        gimbal_msg.linear.z  =  0.0;  //target pitch and yaw angle, respectively.
        gimbal_msg.angular.x =  0.0; 
        gimbal_msg.angular.y = target_pitch_rate; //in velocity control mode, this is the target pitch velocity
        gimbal_msg.angular.z = target_yaw_rate;   //in velocity control mode, this is the target yaw velocity
        gimbal_pub.publish(gimbal_msg);
        return 0;
    }
    private:
    ros::Publisher gimbal_pub;
};