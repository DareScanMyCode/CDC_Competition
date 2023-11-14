#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include "k_means_cluster.h"
#include "a_star.h"
#include "inspector.h"
#include "msg_pub.h"

#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

double cal_dis(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

Eigen::MatrixXd cal_dis_mat(std::vector<Eigen::Vector3d> target_points)
{
    Eigen::MatrixXd dis_mat;
    dis_mat.resize(target_points.size(), target_points.size());
    for (size_t i = 0; i < target_points.size(); i++)
    {
        for (size_t j = 0; j <= i; j++)
        {
            if (i == j)
            {
                dis_mat(i, j) = INT_MAX;
                continue;
            }
            dis_mat(i, j) = cal_dis(target_points[i], target_points[j]);
            dis_mat(j, i) = dis_mat(i, j);
        }
    }
    return dis_mat;
}

std::vector<int> arrange_points(Eigen::Vector3d uav_pos, std::vector<Eigen::Vector3d>& target_pos)
{   
    //虽然插入了uav_pos，但是返回的travel_index不包含它，因为当前就在uav_pos
    // 在target_pos最前面插入uav_pos
    target_pos.insert(target_pos.begin(), uav_pos);
    auto dis_mat = cal_dis_mat(target_pos);
    // std::cout<<dis_mat<<std::endl;
    size_t inserted = 0;
    std::vector<int> travel_index;
    travel_index.push_back(0);
    while (inserted < target_pos.size())
    {
        if (inserted == 0)
        {
            // 求dis_mat第一行中最小值的索引
            double min = dis_mat(0, 1);
            int min_index = 1;
            for (size_t i = 1; i < target_pos.size(); i++)
            { // 第一个没必要管
                if (dis_mat(0, i) < min)
                {
                    min = dis_mat(0, i);
                    min_index = i;
                }
            }
            travel_index.push_back(min_index);
            inserted++;
            // 将dis_mat的第一行和第一列设为无穷,防止由其他点走向它
            for (size_t i = 0; i < target_pos.size(); i++)
            {
                dis_mat(0, i) = INT_MAX;
                dis_mat(i, 0) = INT_MAX;
            }
            // std::cout<<dis_mat<<std::endl;
        }
        else
        {
            int now_index = travel_index[travel_index.size() - 1];
            double min = dis_mat(now_index, 1); // 都是从1开始应该没问题吧
            int min_index = 1;
            for (size_t i = 1; i < target_pos.size(); i++)
            { // 第一个没必要管
                if (dis_mat(now_index, i) < min)
                {
                    min = dis_mat(now_index, i);
                    min_index = i;
                }
            }
            travel_index.push_back(min_index);
            inserted++;
            for (size_t i = 0; i < target_pos.size(); i++)
            {
                dis_mat(now_index, i) = INT_MAX;
                dis_mat(i, now_index) = INT_MAX;
            }
            // std::cout<<dis_mat<<std::endl;
        }
    }
    return travel_index;
}

bool check_collision_free(Eigen::Vector3d pos1, Eigen::Vector3d pos2, octomap::OcTree *world_octomap, double resolution)
{
    // 检查两点之间是否有障碍物
    octomap::point3d start(pos1.x(), pos1.y(), pos1.z());
    octomap::point3d end(pos2.x(), pos2.y(), pos2.z());
    octomap::point3d direction = end - start;
    double length = direction.norm();
    direction = direction.normalized();
    octomap::point3d cur = start;
    for (double i = 0; i < length; i += resolution)
    {
        cur += direction * i;
        octomap::OcTreeNode *node = world_octomap->search(cur);
        if (node != NULL)
        {
            if (world_octomap->isNodeOccupied(node))
            {
                return false;
            }
        }
    }
    return true;
}

bool check_collision_free(Eigen::Vector3d pos1, Eigen::Vector3d pos2, GridMap &grid_map, double resolution)
{
    // 检查两点之间是否有障碍物
    Eigen::Vector3d start = pos1;
    Eigen::Vector3d end = pos2;
    Eigen::Vector3d direction = end - start;
    double length = direction.norm();
    direction = direction.normalized();
    Eigen::Vector3d cur = start;
    for (double i = 0; i < length; i += resolution)
    {
        cur += direction * i;
        if (grid_map.get_occupancy_pos(cur,false))//no_inflate=false
        {
            return false;
        }
    }
    return true;
}

class taker_controller
{
public:
    taker_controller(string name, ros::NodeHandle &nh, double max_uav_vel = 2, double max_uav_w = 1, double max_gimbal_w = 1) : vel_puber(nh, name), gimbal_puber(nh, name)
    {
        this->name = name;
        this->nh = nh;
        this->max_uav_vel = max_uav_vel;
        this->max_uav_w = max_uav_w;
        this->max_gimbal_w = max_gimbal_w;
        // this->vel_puber=TrajPuber(nh,name);
        // this->gimbal_puber=GimbalPuber(nh,name);
    }
    void update(Eigen::Vector3d uav_pos, Eigen::Vector3d uav_vel, Eigen::Vector3d uav_ori, Eigen::Vector3d uav_w,
                Eigen::Vector3d gimbal_ori, Eigen::Vector3d gimbal_w)
    {
        this->uav_pos = uav_pos;
        this->uav_vel = uav_vel;
        this->uav_ori = uav_ori;
        this->uav_w = uav_w;
        this->gimbal_ori = gimbal_ori;
        this->gimbal_w = gimbal_w;
    }
    void uav_controller(std::vector<Eigen::Vector3d> travel_list, bool &need_arrange, int &cluster_index, int &point_index, double tolerence = 3,bool vel_ctrl=false);
    void uav_controller(Eigen::Vector3d target_pos,double tolerence=3,bool vel_ctrl=false);
    void gimbal_controller(std::vector<Eigen::Vector3d> travel_list, int point_index);

private:
    string name;
    ros::NodeHandle nh;
    Eigen::Vector3d uav_pos;
    Eigen::Vector3d uav_vel;
    Eigen::Vector3d uav_ori;
    Eigen::Vector3d uav_w;
    Eigen::Vector3d gimbal_ori;
    Eigen::Vector3d gimbal_w;
    double max_uav_vel;
    double max_uav_w;
    double max_gimbal_w;
    TrajPuber vel_puber;
    GimbalPuber gimbal_puber;
};

void taker_controller::uav_controller(std::vector<Eigen::Vector3d> travel_list, bool &need_arrange, int &cluster_index, int &point_index, double tolerence,bool vel_ctrl)
{
    if(!vel_ctrl){
        if ((this->uav_pos - travel_list[point_index]).norm() < tolerence)
        {
            // 到达了这个点
            point_index++;
            if (size_t(point_index) == travel_list.size())
            {
                // 到达了这个类的最后一个点
                need_arrange = true;
                cluster_index++;
                // point_index = 0;
            }
        }
        else
        {
            // 没到达这个点
            // Eigen::Vector3d target_vel = travel_list[point_index] - this->uav_pos;
            // target_vel = target_vel.normalized() * max_uav_vel; // 简单的控制率，一直以很大的速度运动，对准目标交给gimbal了
            // Eigen::Vector3d target_w(0, 0, 0);
            // std::cout<<"target_vel"<<target_vel.transpose()<<std::endl;
            //改成到达期望位置
            Eigen::Vector3d target_pos=travel_list[point_index];
            if(target_pos.z()<0.5)
                target_pos.z()=0.5;
            // vel_puber.pub_vel(target_vel(0), target_vel(1), target_vel(2), target_w(2));
            vel_puber.pub_point(target_pos(0),target_pos(1),target_pos(2),0,0,0);
            ROS_INFO("UAV_pos:%f,%f,%f",this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
            ROS_INFO("target_pos:%f,%f,%f",target_pos(0),target_pos(1),target_pos(2));
        }
    }
    else{
        Eigen::Vector3d target_pos=travel_list[point_index];
        if(target_pos.z()<0.5)
            target_pos.z()=0.5;
        Eigen::Vector3d direct=target_pos-this->uav_pos;
        direct.normalize();
        direct=direct*max_uav_vel;
        vel_puber.pub_vel(direct(0),direct(1),direct(2),0);
        ROS_INFO("UAV_pos:%f,%f,%f",this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
        ROS_INFO("target_pos:%f,%f,%f",target_pos(0),target_pos(1),target_pos(2));
        ROS_INFO("VEL:%f,%f,%f",direct(0),direct(1),direct(2));
    }
};

void taker_controller::uav_controller(Eigen::Vector3d target_pos,double tolerence,bool vel_ctrl){
    if(!vel_ctrl){
        ROS_INFO("UAV_pos:%f,%f,%f",this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
        ROS_INFO("target_pos:%f,%f,%f",target_pos(0),target_pos(1),target_pos(2));
        if ((this->uav_pos - target_pos).norm() > tolerence){
            vel_puber.pub_point(target_pos(0),target_pos(1),target_pos(2),0,0,0);
        }
    }
    else{
        Eigen::Vector3d direct=target_pos-this->uav_pos;
        direct.norm();
        direct=direct*max_uav_vel;
        vel_puber.pub_vel(direct(0),direct(1),direct(2),0);
        ROS_INFO("UAV_pos:%f,%f,%f",this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
        ROS_INFO("target_pos:%f,%f,%f",target_pos(0),target_pos(1),target_pos(2));
        ROS_INFO("VEL:%f,%f,%f",direct(0),direct(1),direct(2));
    }
};



void taker_controller::gimbal_controller(std::vector<Eigen::Vector3d> travel_list, int point_index)
{
    // 目标是云台指向target_point
    // 云台相对无人机仅可以改变pitch，yaw
    // 先计算无人机到目标点的方向
    Eigen::Vector3d target_point = travel_list[point_index];
    Eigen::Vector3d target_dir = target_point - this->uav_pos;
    // 计算无人机旋转矩阵
    Eigen::Matrix3d R;
    R << cos(this->uav_ori.z()), -sin(this->uav_ori.z()), 0,
        sin(this->uav_ori.z()), cos(this->uav_ori.z()), 0,
        0, 0, 1;
    // 计算无人机到目标点的方向在无人机坐标系下的表示
    Eigen::Vector3d target_dir_uav = R.transpose() * target_dir;
    // 确定pitch和yaw
    double pitch = atan2(target_dir_uav.y(), target_dir_uav.z()); // 这个可能大概率是错的。。。
    double yaw = atan2(target_dir_uav.x(), target_dir_uav.z());
    // 计算角速度
    double pitch_w = (pitch - this->gimbal_ori.x()) * max_gimbal_w; // 这个可能太大
    double yaw_w = (yaw - this->gimbal_ori.y()) * max_gimbal_w;
    // 计算并发送控制指令
    gimbal_puber.pub_gimbal_command(pitch_w, yaw_w);
}