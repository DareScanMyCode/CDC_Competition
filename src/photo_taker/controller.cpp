#include <iostream>
#include <algorithm>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>


#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "../include/photo_taker/photo_taker.h"
#include "../include/photo_taker/inspector_mini.h"
#include "../include/photo_taker/communication.h"
std::string uav_name;

int uav_id,uav_id1,uav_id2;
bool should_send_check=false;//check msg should only be sended once
bool use_taker1=false;
bool use_taker2=false;
bool use_inspector1=false;
bool use_inspector2=false;
Eigen::Vector3d uav_pos, taker1, taker2,inspector1,inspector2;
Eigen::Vector3d uav_vel, uav_ori, uav_w;

bool has_odom=false;
void uav_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{   
    has_odom=true;
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

int force_sleep=0;
double force_z=0;
std::vector<nav_msgs::Odometry> swarm_odom(5);
void swarm_odom_cb(const caric_competition_xmu::OdometryArrayConstPtr& msg){
    // ROS_INFO("CONTROLLER SWARM CALLBACK");
    // if(5 != new_odom_msg.swarm_size) {result = -1; ROS_WARN("两个OdometryArray数组大小不同");}
    for(int i = 0; i < msg->swarm_size.data; i++){
        // if(msg->odometry_array[i].header.stamp.nsec > swarm_odom[i].header.stamp.nsec){
        if(msg->odometry_array[i].header.stamp.sec > swarm_odom[i].header.stamp.sec ||
            (msg->odometry_array[i].header.stamp.sec == swarm_odom[i].header.stamp.sec &&
            msg->odometry_array[i].header.stamp.nsec > swarm_odom[i].header.stamp.nsec)){
            swarm_odom[i] = msg->odometry_array[i];
            // if(i==uav_id){
            //     uav_pos.x() = swarm_odom[i].pose.pose.position.x;
            //     uav_pos.y() = swarm_odom[i].pose.pose.position.y;
            //     uav_pos.z() = swarm_odom[i].pose.pose.position.z;
            // }
            // if(i==0){
            //     inspector1.x() = swarm_odom[i].pose.pose.position.x;
            //     inspector1.y() = swarm_odom[i].pose.pose.position.y;
            //     inspector1.z() = swarm_odom[i].pose.pose.position.z;
            // }
            // if(i==1){
            //     inspector2.x() = swarm_odom[i].pose.pose.position.x;
            //     inspector2.y() = swarm_odom[i].pose.pose.position.y;
            //     inspector2.z() = swarm_odom[i].pose.pose.position.z;
            // }
            if(i==uav_id1){
                taker1.x() = swarm_odom[i].pose.pose.position.x;
                taker1.y() = swarm_odom[i].pose.pose.position.y;
                taker1.z() = swarm_odom[i].pose.pose.position.z;
            }
            else if(i==uav_id2){
                taker2.x() = swarm_odom[i].pose.pose.position.x;
                taker2.y() = swarm_odom[i].pose.pose.position.y;
                taker2.z() = swarm_odom[i].pose.pose.position.z;
            }
        }
    }
    if(inspector1.x()!=0 || inspector1.y()!=0 || inspector1.z()!=0){
        use_inspector1=true;
    }
    if(inspector2.x()!=0 || inspector2.y()!=0 || inspector2.z()!=0){
        use_inspector2=true;
    }
    if(taker1.x()!=0 || taker1.y()!=0 || taker1.z()!=0){
        use_taker1=true;
    }
    if(taker2.x()!=0 || taker2.y()!=0 || taker2.z()!=0){
        use_taker2=true;
    }
    double disi1=abs(inspector1.x()-uav_pos.x())+abs(inspector1.y()-uav_pos.y())+abs(inspector1.z()-uav_pos.z());
    double disi2=abs(inspector2.x()-uav_pos.x())+abs(inspector2.y()-uav_pos.y())+abs(inspector2.z()-uav_pos.z());
    double dis1=abs(taker1.x()-uav_pos.x())+abs(taker1.y()-uav_pos.y())+abs(taker1.z()-uav_pos.z());
    double dis2=abs(taker2.x()-uav_pos.x())+abs(taker2.y()-uav_pos.y())+abs(taker2.z()-uav_pos.z());
    if(disi1<5 && use_inspector1){
        force_sleep=5;
        // ori_pos=uav_pos;
        if(uav_pos.z()>inspector1.z()){
            force_z=uav_pos.z()+0.75;
        }
        else{
            force_z=uav_pos.z()-0.75;
        }
        // ROS_INFO("[%s] FORCE SLEEP %f",uav_name.data(),disi1);
    }
    if(disi2<5 && use_inspector2){
        force_sleep=5;
        // ori_pos=uav_pos;
        if(uav_pos.z()>inspector2.z()){
            force_z=uav_pos.z()+0.75;
        }
        else{
            force_z=uav_pos.z()-0.75;
        }
        // ROS_INFO("[%s] FORCE SLEEP %f",uav_name.data(),disi2);
    }
    // ROS_INFO("[%s] POS1 x=%f, y=%f, z=%f",uav_name.data(),taker1.x(),taker1.y(),taker1.z());
    // ROS_INFO("[%s] POS2 x=%f, y=%f, z=%f",uav_name.data(),taker2.x(),taker2.y(),taker2.z());
    // ROS_INFO("[%s],DIS1=%f, DIS2=%f",uav_name.data(),dis1,dis2);
    if(dis1<8 && use_taker1){
        if (uav_id>=uav_id1){
            force_sleep=5;
            // ori_pos=uav_pos;
            // ROS_INFO("[%s] FORCE SLEEP %f",uav_name.data(),dis1);
            if(dis1<5){
                if(uav_pos.z()>taker1.z()){
                    force_z=uav_pos.z()+0.75;
                }
                else{
                    force_z=uav_pos.z()-0.75;
                }
            }
            else{
                force_z=uav_pos.z();
            }
        }
    }
    else if(dis2< 8&& use_taker2){
        if (uav_id>=uav_id2){
            // ROS_INFO("[%s] FORCE SLEEP %f",uav_name.data(),dis2);
            force_sleep=5;
            // ori_pos=uav_pos;
            if(dis2<5){
                if(uav_pos.z()>taker2.z()){
                    force_z=uav_pos.z()+0.75;
                }
                else{
                    force_z=uav_pos.z()-0.75;
                }
            }
            else{
                force_z=uav_pos.z();
            }
        }
    }
    if(force_z<0.75){
        force_z=uav_pos.z();
    }
}


int cluster_index = 0;    // 指示当前在检查第几个类
int point_index = 0;      // 指示当前在检查类中的第几个点
bool need_arrange = true; // 指示当前的巡检还没被安排
std::vector<Eigen::Vector3d> travel_list;
std::vector<Eigen::Vector3d> travel_list_gimbal;
void travel_list_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    should_send_check=true;
    // ROS_INFO("[%s] has received travel list from subscriber, length: %d", uav_name.data(), msg->data.size()/2/3);

    travel_list.clear();
    travel_list_gimbal.clear();
    //msg的前后两段是不同的消息
    size_t middle=msg->data.size()/2;
    for (size_t i = 0; i < middle; i += 3)
    {
        Eigen::Vector3d point(msg->data[i], msg->data[i + 1], msg->data[i + 2]);
        travel_list.push_back(point);
    }
    for (size_t i = middle; i < msg->data.size(); i += 3)
    {
        Eigen::Vector3d point(msg->data[i], msg->data[i + 1], msg->data[i + 2]);
        travel_list_gimbal.push_back(point);
    }
    std::cout<<uav_name<<" travel_list.size:"<<travel_list.size()<<std::endl;
    for(size_t i=0;i<travel_list.size();i++){
        std::cout<<travel_list[i].transpose()<<std::endl;
    }
    std::cout<<std::endl;
    point_index=0;
}

Eigen::Vector3d next_pos,next_target;
bool receive_target=false;
void target_callback(const std_msgs::Float64MultiArray::ConstPtr &msg){
    
    // ROS_INFO("[%s] has received target from subscriber, length: %d", uav_name.data(), msg->data.size()/2/3);
    receive_target=true;
    next_pos=Eigen::Vector3d(msg->data[0],msg->data[1],msg->data[2]);
    next_target=Eigen::Vector3d(msg->data[3],msg->data[4],msg->data[5]);
    // ROS_INFO("[%s] target_pos: %f, %f, %f", uav_name.data(), next_pos.x(), next_pos.y(), next_pos.z());
}

bool need_takeoff=false;
void takeoff_callback(const std_msgs::Bool::ConstPtr &msg){
    ROS_INFO("NEED TAKEOFF");
    need_takeoff=true;
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh("~");
    
    
    // 读取yaml文件中的uav_name
    // std::string uav_name="changi";
    nh.getParam("node_name", uav_name);

    //1202 1504
    nh.getParam("uav_id", uav_id);
    ROS_INFO("[%s] uav_id:%d",uav_name.data(),uav_id);
    if(uav_id==2){
        uav_id1=3;
        uav_id2=4;
    }
    else if(uav_id==3){
        uav_id1=2;
        uav_id2=4;
    }
    else if(uav_id==4){
        uav_id1=2;
        uav_id2=3;
    }
    else{
        ROS_INFO("uav_id error");
    }


    double takeoff_height  = 0.75,
           hovering_height = 5.0;
    nh.getParam("takeoff_height", takeoff_height);
    nh.getParam("hovering_height", hovering_height);
    std::cout<< "[" << uav_name << "] " << "takeoff height: " << takeoff_height << " hovering height: " << hovering_height << std::endl;


    //声明一个taker_controller类
    taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 订阅无人机的位置,消息类型是nav_msgs/Odometry
    ros::Subscriber uav_odom_sub = nh.subscribe("/"+uav_name+"/ground_truth/odometry", 1, uav_odom_callback); 
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe("/"+uav_name+"/gimbal", 1, gimbal_callback);
    //订阅travel_list
    ros::Subscriber travel_list_sub=nh.subscribe("/"+uav_name+"/travel_list",1,travel_list_callback);
    ros::Subscriber target_point_sub=nh.subscribe("/"+uav_name+"/target_point",1,target_callback);
    ros::Subscriber takeroff_sub=nh.subscribe("/"+uav_name+"/takeoff",1,takeoff_callback);



    std::string topic_name;
    topic_name = "/swarm_odometry/"+uav_name;
    ros::Subscriber sub_swarm_odom = nh.subscribe(topic_name.c_str(), 1, swarm_odom_cb);

    

    ROS_INFO("[%s] controller node has started !", uav_name.data());

    //发布这个是否检查完成
    ros::Publisher check_finish_pub = nh.advertise<std_msgs::Bool>("/"+uav_name+"/check", 1);
    bool taking_off=false;
    bool get_ori_pos=false;
    Eigen::Vector3d ori_pos;
    // while (ros::ok())
    // {   
    //     // force_sleep--;
    //     if(uav_pos.z() - takeoff_height >= -0.2){
    //         taking_off = true;
    //     }
        
    //     my_controller.update(uav_pos,uav_vel,uav_ori,uav_w,gimbal_pos,gimbal_ori);
    //     if(point_index==travel_list.size() && should_send_check){//告诉别人我检查完了
    //         should_send_check=false;
    //         std_msgs::Bool check_finish;
    //         check_finish.data=true;
    //         check_finish_pub.publish(check_finish);
    //         ROS_INFO("[%s] FINISH ONE PART", uav_name.data());
    //     }
    //     else{
    //         if(travel_list.size()>0 && point_index!=travel_list.size()){    //不需要安排，就根据当前位置和姿态发送控制指令，包括本身运动和云台控制
    //             if(!taking_off){
    //                 if(uav_pos.z()<takeoff_height){
    //                     ROS_INFO("[%s] TAKING OFF", uav_name.data());
    //                     Eigen::Vector3d target_pos(uav_pos.x(),uav_pos.y(), takeoff_height);
    //                     my_controller.uav_controller(target_pos, 0.5, true, true);
    //                 }
    //             }
    //             else{
    //                 if(force_sleep>=1){
    //                     Eigen::Vector3d target_pos(uav_pos.x(),uav_pos.y(), force_z);
    //                     my_controller.uav_controller(target_pos, 0.0, true, true);
    //                     // ROS_INFO("[%s] REDUCE SLEEP %d", uav_name.data(),force_sleep);
    //                     my_controller.gimbal_controller(travel_list_gimbal, point_index);
    //                     force_sleep--;
    //                 }
    //                 else{
    //                     // ROS_INFO("[%s] CONTROL", uav_name.data());
    //                     // ROS_INFO("[%s] NOW %d - %d", uav_name.data(), point_index, travel_list.size());
    //                     my_controller.uav_controller(travel_list, travel_list_gimbal,need_arrange, cluster_index, point_index ,0.5, true, true);
    //                     // my_controller.uav_controller(travel_list, need_arrange, cluster_index, point_index,3,true); // 这几个参数都是引用，当所有都检查完后，need_arrange会变成true,cluster_index+1,进入下一个类，point_index=0
    //                     my_controller.gimbal_controller(travel_list_gimbal, point_index);
    //                 }
    //             }
    //         }
    //         else{
    //             if(taking_off){
    //                 // Eigen::Vector3d target_pos(uav_pos.x(),uav_pos.y(), uav_pos.z());
    //                 // ROS_INFO("[%s] HOVERING", uav_name.data());
    //                 // std_msgs::Bool check_finish;
    //                 // check_finish.data=true;
    //                 // check_finish_pub.publish(check_finish);
    //                 // my_controller.uav_controller(target_pos, 0, true, true);
    //             }
    //         }
    //     }
    //     ros::spinOnce();
    //     ros::Rate(10).sleep();          // Added 60 -> 5
    // }   
    
    while (ros::ok())
    {   
        if(has_odom){
            // force_sleep--;
            if(uav_pos.z() - takeoff_height >= -0.2){
                taking_off = true;
            }
            
            my_controller.update(uav_pos,uav_vel,uav_ori,uav_w,gimbal_pos,gimbal_ori);

            // if(receive_target){    //不需要安排，就根据当前位置和姿态发送控制指令，包括本身运动和云台控制
            if(!taking_off){
                if(!get_ori_pos){
                    ori_pos=uav_pos;
                    get_ori_pos=true;
                }
                if(need_takeoff){
                    if(uav_pos.z()<takeoff_height){
                        // ROS_INFO("[%s] TAKING OFF", uav_name.data());
                        Eigen::Vector3d target_pos(ori_pos.x(),ori_pos.y(), takeoff_height);
                        my_controller.uav_controller(target_pos, 0.0, false, true);
                    }
                }
            }
            else{
                if(force_sleep>=1){
                    Eigen::Vector3d target_pos(uav_pos.x(),uav_pos.y(), force_z);
                    my_controller.uav_controller(target_pos, 0.0, true, true);
                    force_sleep--;
                    if(receive_target){
                        my_controller.gimbal_controller(next_target);
                    }
                }
                else{
                    if(receive_target){
                        my_controller.uav_controller(next_pos,next_target);
                        my_controller.gimbal_controller(next_target);
                    }
                    else{
                        // ROS_INFO("[%s] HOVERING",uav_name.data());
                        Eigen::Vector3d hover_pos=uav_pos;
                        my_controller.uav_controller(hover_pos,hover_pos);
                        my_controller.gimbal_controller(hover_pos);
                    }
                }
            }
        }
            

        // }
        ros::spinOnce();
        ros::Rate(10).sleep();          // Added 60 -> 5
    }   

}

