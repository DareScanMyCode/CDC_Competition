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
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <set>
#include <functional>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include "../include/photo_taker/photo_taker.h"
#include "../include/photo_taker/inspector_mini.h"
#include <std_msgs/Duration.h>

#include "../include/photo_taker/communication.h"
#include <unistd.h>
// #include "communication.h"


#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"
#define RESET "\033[0m"
std::string uav_name,uav_name1,uav_name2,node_name;

Inspector_mini inspector;
//计时，时间太短不更新
size_t pre_size=0;

bool enforce_cluster=false;

bool get_map=false;
void grid_map_callback(const caric_competition_xmu::GridMapMsg::ConstPtr &msg)
{
    // ROS_INFO("[%s] grid_map_callback", uav_name);
    //根据获取的grid_map更新我的grid_map
    
    inspector.grid_map = small_msg2map(*msg);//直接整个替换了
    // inspector.grid_map.inflate_map();
    get_map=true;
}


bool taking_off=false;
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
    // swarm_odom[uav_id].child_frame_id = msg->child_frame_id;
    // swarm_odom[uav_id].header = msg->header;
    // swarm_odom[uav_id].pose = msg->pose;
    // swarm_odom[uav_id].twist = msg->twist;
    // caric_competition_xmu::swarm_odom_pub.publish(caric_competition_xmu::swarm_odom2msg(swarm_odom));
    // my_odom_cb(msg);
}

// added zbw
// added zbw
int uav_id,uav_id1,uav_id2;
bool use_taker1=false;
bool use_taker2=false;
// done zbw
std::vector<nav_msgs::Odometry> swarm_odom(5);
void swarm_odom_cb(const caric_competition_xmu::OdometryArrayConstPtr& msg){
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
    if(taker1.x()!=0 || taker1.y()!=0 || taker1.z()!=0){
        use_taker1=true;
    }
    if(taker2.x()!=0 || taker2.y()!=0 || taker2.z()!=0){
        use_taker2=true;
    }
    //if uav is too close, re_cluster
    double dis1=abs(taker1.x()-uav_pos.x())+abs(taker1.y()-uav_pos.y())+abs(taker1.z()-uav_pos.z());
    double dis2=abs(taker2.x()-uav_pos.x())+abs(taker2.y()-uav_pos.y())+abs(taker2.z()-uav_pos.z());
    if(dis1<10 || dis2< 10)
        enforce_cluster=true;
}

void my_odom_cb(const nav_msgs::OdometryConstPtr & msg){
    // ROS_INFO("[%s] uav_odom_update_pub", uav_name.data());
    // ROS_WARN("[%s] uav_id: [---%d---]",uav_name, uav_id);
    // 在自己的odom_cb中添加如下代码
    // swarm_odom[5] = msg; // 是否可以？
    swarm_odom[uav_id].child_frame_id = msg->child_frame_id;
    swarm_odom[uav_id].header = msg->header;
    swarm_odom[uav_id].pose = msg->pose;
    swarm_odom[uav_id].twist = msg->twist;
    // ROS_INFO("[%s] SWARM_PUB x=%f, y=%f, z=%f",uav_name.data(),msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z);
    caric_competition_xmu::swarm_odom_pub.publish(caric_competition_xmu::swarm_odom2msg(swarm_odom));
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


// struct Vector3dHash {
//     std::size_t operator()(const Eigen::Vector3d& vec) const {
//         std::size_t h1 = std::hash<double>()(vec.x());
//         std::size_t h2 = std::hash<double>()(vec.y());
//         std::size_t h3 = std::hash<double>()(vec.z());
//         return h1 ^ (h2 << 1) ^ (h3 << 2); // 使用 XOR 来组合哈希值
//     }
// };

// // 自定义比较函数
// struct Vector3dEqual {
//     bool operator()(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2) const {
//         return vec1 == vec2;
//     }
// };

std::vector<Eigen::Vector3d> interest_points;
// std::unordered_map<Eigen::Vector3d,bool> all_interest_points;
std::unordered_set<Eigen::Vector3d> all_interest_points;
//记录已经检测过的兴趣点，他说使用的数据结构应该便于查找，使用unordered_map
// std::unordered_map<Eigen::Vector3d, bool> checked_points;
// std::unordered_map<Eigen::Vector3d, bool, Vector3dHash, Vector3dEqual> checked_points;
// std::unordered_set<Eigen::Vector3d,Vector3dHash> checked_points;
std::unordered_set<Eigen::Vector3d> checked_points;
// std::set<Eigen::Vector3d,VectorCompare> checked_points;

std::vector<Eigen::Vector3d> target_points; // 要追踪的目标点们
//消息类型是sensor_msgs/PointCloud
bool first_interest_points=true;
int new_added_points=0;
void interest_points_callback(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    caric_competition_xmu::gcs_score_pub.publish(*msg);
    //ROS_INFO("[%s] interest_points_callback", uav_name.data());
    // if(msg->points.size()-interest_points.size()<60){
    //     if(first_interest_points && interest_points.size()>20){
    //         first_interest_points=false;
    //     }
    //     else{
    //         return;
    //     }
    // }
    //1118 兴趣点只增加新增量
    // ROS_INFO("update_interest_points");
    // interest_points.clear();
    // 将sensor_msgs::PointCloud的数据读入interest_points
    //只增加新增的
    for(size_t i=0;i<msg->points.size();i++){
        Eigen::Vector3d point(msg->points[i].x, msg->points[i].y, msg->points[i].z);
        if(all_interest_points.find(point)==all_interest_points.end()){
            interest_points.push_back(point);
            // all_interest_points[point]=true;
            all_interest_points.insert(point);
            new_added_points++;
        }
    }
    for(auto begin=interest_points.begin();begin!=interest_points.end();){
        if(checked_points.find(*begin)!=checked_points.end()){
            begin=interest_points.erase(begin);
        }
        else{
            begin++;
        }
    }
    std::vector<Eigen::Vector3d> downsample_interest_points;
    int downsample_ratio=5;
    while(interest_points.size()<downsample_ratio*checked_points.size()&&downsample_ratio>1){
        downsample_ratio--;
    }
    // ROS_INFO("downsample_ratio:%d",downsample_ratio);
    for(size_t i=0;i<interest_points.size();i++){
        if(i%downsample_ratio==0){
            downsample_interest_points.push_back(interest_points[i]);
        }
    }
    if((enforce_cluster==true || new_added_points>10)&&interest_points.size()>10){
        enforce_cluster=false;
        // ROS_INFO("[%s] RECLUSTERING", uav_name.data());
        // ROS_INFO("[%s] add new interest points", uav_name.data());
        new_added_points=0;
        //11.18修改，初始聚类中心为三架飞机的位置
        std::vector<Eigen::Vector3d> ori_center;
        ori_center.push_back(uav_pos);
        // ROS_INFO("[%s] INTEREST_POINTS SIZE:%d", uav_name.data(),interest_points.size());
        if(use_taker1 && use_taker2){
            // ROS_INFO("USED_UAV_NUM 3");
            ori_center.push_back(taker1);
            ori_center.push_back(taker2);
            //做到记录历史的划分数据
            std::vector<std::vector<Eigen::Vector3d>> clusters = kMeansClustering(downsample_interest_points, 3, 0.01, 100,ori_center);
            // std::vector<std::vector<Eigen::Vector3d>> clusters = kMeansClustering(interest_points, 3, 0.01, 100);
            //打印clusters每个类的大小
            // for(size_t i=0;i<clusters.size();i++){
            //     // ROS_INFO("[%s] cluster %d size:%d", uav_name.data(),i,clusters[i].size());
            // }
            std::vector<Eigen::Vector3d> centroid = cal_centroid(clusters, 3);
            Eigen::Matrix3d dis;
            // 根据距离为每个聚类中心分配一个taker
            for (int i = 0; i < 3; i++)
            {
                dis(i, 0) = (centroid[i] - uav_pos).norm();
                dis(i, 1) = (centroid[i] - taker1).norm();
                dis(i, 2) = (centroid[i] - taker2).norm();
            }
            // target_points = clusters[uav_id-2];
            for (int n = 0; n < 3; n++)
            {   
                // ROS_INFO("[%s] IN CHOOSE", uav_name.data());
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
                    // ROS_INFO("[%s] target_points.size:%d", uav_name.data(), target_points.size());
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
        else if(use_taker1||use_taker2){
            // ROS_INFO("USED_UAV_NUM 2");
            if(use_taker1){
                ori_center.push_back(taker1);
            }
            if(use_taker2){
                ori_center.push_back(taker2);
            }
            std::vector<std::vector<Eigen::Vector3d>> clusters = kMeansClustering(downsample_interest_points, 2, 0.01, 100,ori_center);
            std::vector<Eigen::Vector3d> centroid = cal_centroid(clusters, 2);
            Eigen::Matrix2d dis;
            if(use_taker1){
                for(int i=0;i<2;i++){
                    dis(i,0)=(centroid[i]-uav_pos).norm();
                    dis(i,1)=(centroid[i]-taker1).norm();
                }
                // ori_center.push_back(taker1);
                if(uav_id<uav_id1){
                    //可以先选
                    if(dis(0,0)<dis(1,0)){
                        target_points = clusters[0];
                    }
                    else{
                        target_points=clusters[1];
                    }
                }
                else{
                    if(dis(0,1)<dis(1,1)){
                        //别人先选了0
                        target_points = clusters[1];
                    }
                    else{
                        target_points=clusters[0];
                    }
                }
            }
            if(use_taker2){
                for(int i=0;i<2;i++){
                    dis(i,0)=(centroid[i]-uav_pos).norm();
                    dis(i,1)=(centroid[i]-taker2).norm();
                }
                // ori_center.push_back(taker2);
                if(uav_id<uav_id2){
                    if(dis(0,0)<dis(1,0)){
                        target_points = clusters[0];
                    }
                    else{
                        target_points=clusters[1];
                    }
                }
                else{
                    if(dis(0,1)<dis(1,1)){
                        target_points = clusters[1];
                    }
                    else{
                        target_points=clusters[0];
                    }
                }
            }
            //打印clusters每个类的大小
            // for(size_t i=0;i<clusters.size();i++){
            //     ROS_INFO("[%s] cluster %d size:%d", uav_name.data(),i,clusters[i].size());
            // }
            // std::vector<Eigen::Vector3d> centroid = cal_centroid(clusters, 2);
            // Eigen::Matrix3d dis;
            // // 根据距离为每个聚类中心分配一个taker
            // for (int i = 0; i < 2; i++)
            // {
            //     dis(i, 0) = (centroid[i] - uav_pos).norm();
            //     dis(i, 1) = (centroid[i] - taker1).norm();
            // }
            // for (int n = 0; n < 2; n++)
            // {   
            //     // ROS_INFO("[%s] IN CHOOSE", uav_name.data());
            //     // 找到所有dis中最小的值的二维索引
            //     int min = dis(0, 0);
            //     int min_i = 0;
            //     int min_j = 0;
            //     for (int i = 0; i < 2; i++)
            //     {
            //         for (int j = 0; j < 2; j++)
            //         {
            //             if (dis(i, j) < min)
            //             {
            //                 min = dis(i, j);
            //                 min_i = i;
            //                 min_j = j;
            //             }
            //         }
            //     }
            //     if (min_j == 0)
            //     {
            //         target_points = clusters[min_i];
            //         // ROS_INFO("[%s] target_points.size:%d", uav_name.data(), target_points.size());
            //         break;
            //     }
            //     else
            //     {
            //         // 第i行和第j列所有元素设为1000000
            //         for (int i = 0; i < 2; i++)
            //         {
            //             dis(i, min_j) = INT_MAX;
            //         }
            //         for (int j = 0; j < 2; j++)
            //         {
            //             dis(min_i, j) = INT_MAX;
            //         }
            //     }
            // }
        }
        else{
            // ROS_INFO("USED_UAV_NUM 1");
            target_points=downsample_interest_points;
        }
    }
    //打印target_points.size
}


//这个处理自己发现的(地面站确定的)
void detedted_points_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    //ROS_INFO("[%s] detected_points_callback", uav_name.data());
    //处理和interest_points_callback一样
    //不过我的消息类型是sensor_msgs/PointCloud2
    //先转为pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    for(size_t i=0;i<cloud.points.size();i++){
        // double score=cloud.points[i].intensity;
        Eigen::Vector3d point(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        if(all_interest_points.find(point)==all_interest_points.end()){
            interest_points.push_back(point);
            // all_interest_points[point]=true;
            all_interest_points.insert(point);
            new_added_points++;
        }
    }
    for(auto begin=interest_points.begin();begin!=interest_points.end();){
        if(checked_points.find(*begin)!=checked_points.end()){
            begin=interest_points.erase(begin);
        }
        else{
            begin++;
        }
    }
}

//这个处理别人发过来的处理过的兴趣点
void checked_interest_points_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    //ROS_INFO("[%s] checked_interest_points_callback", uav_name.data());
    //posarray转为vector
    for(size_t i=0;i<msg->poses.size();i++){
        Eigen::Vector3d point(msg->poses[i].position.x, msg->poses[i].position.y, msg->poses[i].position.z);
        if(checked_points.find(point)==checked_points.end()){
            checked_points.insert(point);
        }
    }
    for(auto begin=interest_points.begin();begin!=interest_points.end();){
        if(checked_points.find(*begin)!=checked_points.end()){
            begin=interest_points.erase(begin);
        }
        else{
            begin++;
        }
    }
}


int cluster_index = 0;    // 指示当前在检查第几个类
int point_index = 0;      // 指示当前在检查类中的第几个点
bool need_arrange = true; // 指示当前的巡检还没被安排
void check_callback(const std_msgs::Bool::ConstPtr &msg)
{
    // ROS_INFO("[%s] check_callback", uav_name.data());
    if (msg->data == true)
    {
        // 检查完了，就要重新聚类
        need_arrange = true;
        cluster_index++;
        point_index = 0;
    }
    // for(auto point:points_cluster_cluster){
    //     checked_points[point]=true;
    // }
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

// 记录任务时间
double mission_duration_remained = 9999.0;
void mis_dura_cb(const std_msgs::Duration::ConstPtr &msg){
    mission_duration_remained = msg->data.toSec();
}


struct rollbackNode{
    ros::Time vist_time;
    Eigen::Vector3d pos;
    bool gcs_connected=false;
};

std::stack<rollbackNode> posRollBackStack;

bool has_roll_back=false;
void rollback2gcs(double max_rollback_time, TrajPuber tj_pb){
    has_roll_back = true;
    ros::Time rollback_start_time = ros::Time::now();
    ros::Time rollback_time_now = ros::Time::now();
    // 依次取出posRollBackStack中的点
    while(1){
        if (posRollBackStack.size() == 0) {
            break;
        }
        if ((ros::Time::now() - rollback_start_time).toSec() > max_rollback_time) {
            ROS_INFO(KGRN"[%s] rollback2gcs timeout"RESET, uav_name.c_str());
            break;
        }

        rollbackNode node = posRollBackStack.top();
        posRollBackStack.pop();
        tj_pb.pub_point(node.pos[0], node.pos[1], node.pos[2], 0., 0., 0.);
        // 等待一段时间

        if (posRollBackStack.size() == 0) {
            break;
        }
        double time_to_sleep = std::min(
            0.80 * (rollback_time_now - node.vist_time).toSec(),
            (node.pos - posRollBackStack.top().pos).norm() / 2.2
        );
        rollback_time_now = node.vist_time;
        ros::Duration(time_to_sleep).sleep();
    }
}

nav_msgs::Odometry last_gcs_connected_pos;
ros::Time last_gcs_connected_time;
bool is_gcs_connected;
bool rollback_init = false;
void gcs_ping_cb(const std_msgs::Bool::ConstPtr &msg){
    is_gcs_connected = true;
    last_gcs_connected_time = ros::Time::now();
    // if (odometryQueue.size() >= 20) {
    //         odometryQueue.pop(); // 如果队列满了，移除最旧的元素
    //     }
    // odometryQueue.push(inspector_pointer->pos);
    // posStack.push(inspector_pointer->pos);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "photo_taker");
    ros::NodeHandle nh("~");
    
    // 读取yaml文件中的uav_name
    // std::string uav_name="changi";
    nh.getParam("node_name", uav_name);        // nh.getParam("node_name", uav_name);
    nh.getParam("node_name", node_name); 
    nh.getParam("uav_name1", uav_name1);
    nh.getParam("uav_name2", uav_name2);
    // nh.param("uav_name", uav_name, std::string("uav1"));
    //声明一个taker_controller类
    // taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚
    std::cout<<"it's "<<uav_name << ", my friend: " << uav_name1 << ", " << uav_name2 << std::endl;

    bool is_debug;
    // nh.getParam("node_name", node_name);  // zbw added
    if(!nh.getParam("is_debug", is_debug)){
        is_debug = false;
    }
    
    nh.getParam("uav_id", uav_id);
    ROS_INFO("uav_id:%d",uav_id);
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

    taker_controller my_controller(uav_name, nh);//应该不存在没定义，因为他使用的比较晚

    // 定义两个 double 变量和一个存储 Eigen::Vector3i 的 vector
    double x_min,x_max,y_min,y_max,z_min,z_max,resolution;
    resolution=2.0;
    std::vector<Eigen::Vector3d> vector3d_face_data;
    std::vector<Eigen::Vector3d> vector3d_all_data;
    std::vector<Eigen::Vector3d> vector3d_search_data;

    double x,y,z,roll,pitch,yaw;
    x=0;y=0;z=0;roll=0;pitch=0;yaw=0;
    inspector.init(x,y,z,roll,pitch,yaw,x_min,x_max,y_min,y_max,z_min,z_max,resolution,vector3d_search_data);


    // std::cout << node_name.c_str() << "---------";
    caric_competition_xmu::init_Communicator(nh, node_name.c_str(), is_debug);

    // init communication
    // caric_competition_xmu::init_Communicator(nh, node_name, true);  // zbw added
    // 我的消息发到了pprom，他回根据指定的uav_name发送topic_name/uav_name
    std::string topic_name("/checked_interested_points/");
    topic_name += node_name;  // 结点名称 jurong等
    ros::Subscriber checked_interested_points_sub = nh.subscribe(topic_name.c_str(), 1, checked_interest_points_callback);

    // 订阅消息
    // nh.getParam("node_name", node_name); // zbw added
    topic_name = std::string("/detected_interested_points/");
    topic_name += node_name;  // 结点名称 jurong等
    ros::Subscriber detected_interested_points_topic_name_sub = nh.subscribe(topic_name.c_str(), 1, interest_points_callback);
    ros::Subscriber detected_interested_points_sub = nh.subscribe("/"+node_name+"/detected_interest_points", 1, detedted_points_callback);

    ros::Subscriber gcs_ping_sub        = nh.subscribe<std_msgs::Bool>("/gcs_ping_msg/"+uav_name, 1, gcs_ping_cb);
    topic_name = std::string("/grid_map/");
    topic_name += node_name;  // 结点名称 jurong等
    ros::Subscriber map_sub = nh.subscribe(topic_name.c_str(), 1, grid_map_callback);//这个名称是不是之后改一下

    ros::Subscriber gcs_score_sub = nh.subscribe("/gcs_score/"+node_name, 1, interest_points_callback);
    // 发布消息
    // caric_competition_xmu::map_msg_pub(caric_competition_xmu::map2msg(inspector.grid_map));

    // zbw added
    // std::string node_name;

    // caric_competition_xmu;
    topic_name = "/swarm_odometry/"+node_name;
    ros::Subscriber sub_swarm_odom = nh.subscribe(topic_name.c_str(), 1, swarm_odom_cb);

    topic_name = "/" + node_name + "/ground_truth/odometry";
    ros::Subscriber sub_my_odom = nh.subscribe(topic_name, 1, my_odom_cb);

    ros::Subscriber uav_odom_sub = nh.subscribe("/"+uav_name+"/ground_truth/odometry", 1, uav_odom_callback); // taker1从yaml文件中读取
    // 订阅gimbal
    ros::Subscriber gimbal_sub = nh.subscribe("/"+uav_name+"/gimbal", 1, gimbal_callback);
    //订阅一个bool量，这个类检查完没有
    ros::Subscriber check_sub = nh.subscribe("/"+uav_name+"/check", 1, check_callback);
    ros::Subscriber mis_dura_sub        = nh.subscribe<std_msgs::Duration>("/"+uav_name+"/mission_duration_remained", 1, mis_dura_cb);

    // 创建一个发布者来发布 Marker 消息
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("interest_points", 10);
    //创建一个发布者来发布一个std::vector<Eigen::Vector3d>消息，记录的是travel_list
    ros::Publisher travel_list_pub = nh.advertise<std_msgs::Float64MultiArray>("/"+uav_name+"/travel_list", 1);     // added 10 -> 1
    ros::Publisher target_point_pub = nh.advertise<std_msgs::Float64MultiArray>("/"+uav_name+"/target_point", 1);     // added 10 -> 1
    ros::Publisher takeoff_pub = nh.advertise<std_msgs::Bool>("/"+uav_name+"/takeoff", 1);

    // 创建一个 Marker 消息
    visualization_msgs::Marker marker_interest_point;
    create_Marker("world","interest_point",1,1,0,0,marker_interest_point);
    visualization_msgs::Marker marker_target_point;
    create_Marker("world","target_point",1,0,0,1,marker_target_point);
    visualization_msgs::Marker marker_spec_point;
    create_Marker("world","specified_point",1,0,1,0,marker_spec_point);
    visualization_msgs::Marker marker_cluster_cluster_points;
    create_Marker("world","cluster_cluster_points",1,1,1,0,marker_cluster_cluster_points);
    visualization_msgs::Marker marker_points_near_free;
    create_Marker("world","points_near_free",1,1,0,1,marker_points_near_free);
    visualization_msgs::Marker marker_uav_pos;
    create_Marker("world","uav_pos",3,0,1,1,marker_uav_pos);
    visualization_msgs::Marker marker_occupied_points;
    create_Marker("world","occupied_points",1,0.5,0.5,0.5,marker_occupied_points);

    int num_for_each_cluster = 7;

    double check_resolution = 0.5;
    std::vector<Eigen::Vector3d> travel_list;//经过A*修改后应去的路径
    std::vector<Eigen::Vector3d> travel_list_gimbal;//经过A*修改后的每个路点对应的目标点
    std::vector<bool> sign_simplify;//标记travel_list中的点是否是视点，是否能否简化 true表示是视点，不能简化
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
    bool neighbor_checked=false;

    ros::Time get_map_time = ros::Time::now();
    int count_info = 0;
    double taking_off_height = 0;
    ros::Time pre_take_off_time = ros::Time::now();
    while (ros::ok())
    {   
        try{
            rollbackNode node;
            node.pos = Eigen::Vector3d(inspector.pos[0],inspector.pos[1],inspector.pos[2]);
            node.vist_time = ros::Time::now();
            posRollBackStack.push(node);
        }catch(...){
            std::cout<<"rollback add fail";

        }

        if(!has_roll_back && mission_duration_remained < 19.0 && mission_duration_remained <( ros::Time::now() - last_gcs_connected_time).toSec() * 0.8 && mission_duration_remained >0){
            ROS_WARN("[%s]: NO MORE TIME --------%.2lf----------", uav_name.c_str(), mission_duration_remained);
            rollback2gcs( (ros::Time::now() - last_gcs_connected_time).toSec(), my_controller.vel_puber);
        }
        if(!get_map){
            // ROS_INFO("[%s] map not get", uav_name.data());
            ros::spinOnce();
            ros::Rate(60).sleep(); 
            get_map_time=ros::Time::now();
            continue; 
        }
        now_time=ros::Time::now();
        if(now_time.toSec()-get_map_time.toSec()<5){
            if(count_info++ > 15){
                // ROS_INFO("[%s] map not good enough", uav_name.data());
                count_info = 0;
            }
            
            ros::spinOnce();
            ros::Rate(60).sleep(); 
            continue; 
        }
        
        // my_controller.update(uav_pos,uav_vel,uav_ori,uav_w,gimbal_pos,gimbal_ori);

        //rviz显示interest_point
        if(0){
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
            // ROS_WARN("NEAR_FREE_SIZE:%d",points_cluster_cluster.size());
            // ROS_WARN("NEAR_FREE_SIZE:%d",points_near_free.size());
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
        }
        //检查地图，如果脚底下一定区域都是占据
        //说明他附近被照过了
        //他才配执行任务
        //如果这个效果还不好，就等20s再起飞
        // if(!neighbor_checked) {
        //     double check_x,check_y;
        //     check_x=uav_pos.x();
        //     check_y=uav_pos.y();
        //     Eigen::Vector3d check_pos(check_x,check_y,0);
        //     Eigen::Vector3i check_index;
        //     inspector.grid_map.pos2index(check_pos,check_index);
        //     //检查xy平面5半径有没有被occupied
        //     bool not_neighbor_occupied=false;
        //     for(int i=-5;i<=5;i++){
        //         for(int j=-5;j<=5;j++){
        //             int check_index_x=max(0,check_index(0)+i);
        //             check_index_x=min(check_index_x,inspector.grid_map.max_index[0]);
        //             int check_index_y=max(0,check_index(1)+j);
        //             check_index_y=min(check_index_y,inspector.grid_map.max_index[1]);
        //             if(inspector.grid_map.grid_map[check_index(0)+i][check_index(1)+j][check_index(2)].is_occupied==0){
        //                 not_neighbor_occupied=false;
        //                 break;
        //             }
        //         }
        //         if(!not_neighbor_occupied){
        //             break;
        //         }
        //     }
        //     if(!not_neighbor_occupied){
        //         neighbor_checked=true;
        //     }
        //     else{
        //         ROS_INFO("[%s] neighbor_not_checked_yet", uav_name.data());
        //         ros::spinOnce();
        //         ros::Rate(60).sleep(); 
        //         continue; 
        //     }
        // }
        // 这个判断要不要重新初始化
        now_time = ros::Time::now();
        //判断时间间隔是否大于5s
        auto duration = now_time.toSec()-pre_time.toSec();
        //1. target_points是这家飞机所有要检查的点，它不应该过少
        if ((duration >= 20||first_arrange) && target_points.size() > 3)
        // if(target_points.size()>10)
        { // 这个时候加入时间间隔限制，太短的话，可能会导致无人机频繁调整航线
            // ROS_INFO("[%s] ONE CHECK", uav_name.data());
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
            // ROS_INFO("[%s] target_clusters.size:%d", uav_name.data(), target_clusters.size());
            arrange_cluster_index = arrange_points(uav_pos, centroid);
            arrange_cluster_index.erase(arrange_cluster_index.begin(),arrange_cluster_index.begin()+1);
            // 重置检查的类和点
            cluster_index = 0;
            point_index = 0;
        }
        //2. 现在处理的是二次聚类后的一部分点
        // 这个判断当前检查的类路径有没有被规划
        if (need_arrange && arrange_cluster_index.size() >=1) // 这有个终止条件我还没加，注意
        {   
            int target_index=0;
            if(!sign_simplify.empty()){
                for(int i=0;i<=inspector.curr_seg;i++){
                    if(sign_simplify[i]){
                        target_index++;
                    }
                }
            }
            if(cluster_index>=0 && points_cluster_cluster.size()>0){
                // ROS_WARN("[%s] points_cluster_cluster.size:%d", uav_name.data(), points_cluster_cluster.size());
                for(int i=0;i<min(target_index-1,int(points_cluster_cluster.size()));i++){
                    Eigen::Vector3d& t_point=points_cluster_cluster[i]; 
                    if(checked_points.find(t_point)==checked_points.end()){
                        checked_points.insert(t_point);
                    }
                }
                // for(auto &t_point:points_cluster_cluster){
                //     if(checked_points.find(t_point)==checked_points.end()){
                //         checked_points[t_point]=true;
                //     }
                // }
                // ROS_WARN("[%s] checked_points_num:%d", uav_name.data(), checked_points.size());
                // ROS_WARN("[%s] Interest_points_num:%d", uav_name.data(), interest_points.size());
                // ROS_WARN("[%s] ALL Interest_points_num:%d", uav_name.data(), all_interest_points.size());
            }
            //ROS_INFO("[%s] TWO CHECK", uav_name.data());
            //在再一次arrange之前，checked_points增加
            //ROS_INFO("[%s] checked_points_num:%d", uav_name.data(), checked_points.size());
            //12012123
            // for(auto &t_point:points_cluster_cluster){
            //     checked_points[t_point]=true;
            // }
            //ROS_INFO("[%s] checked_points_num:%d", uav_name.data(), checked_points.size());

            //zbw发布check_points
            caric_competition_xmu::checked_interest_point_pub.publish(caric_competition_xmu::checked_point2msg(checked_points));


            // cluster_index++;
            // 重置travel_list
            travel_list.clear();//是二级聚类再避障后需要走的点
            travel_list_gimbal.clear();
            sign_simplify.clear();
            if(cluster_index>=arrange_cluster_index.size()){
                // ROS_INFO("[%s] cluster_index >= arrange_cluster_index.size()", uav_name.data());
                enforce_cluster=true;
                cluster_index=0;
                continue;
            }
            points_cluster_cluster = target_clusters[arrange_cluster_index[cluster_index]-1];//是二级聚类后，目前要检查的类
            cluster_index++;
            arrange_point_index = arrange_points(uav_pos, points_cluster_cluster);//points_cluster_cluster经过处理后前面回多出来uav_pos
            // 对每一个兴趣点，我都在他附近搜索一个在自由空间的点，这个基础上再进行规划
            //  to do
            // std::cout << "["<< uav_name << "] " << "before replace" <<std::endl;
            for(auto &t_point:points_cluster_cluster){
                //替换每一个点
                // std::cout<<t_point.transpose()<<std::endl;
            }
            points_near_free=points_cluster_cluster;
            // ROS_WARN("ori NEAR_FREE_SIZE:%d",points_near_free.size());
            for(auto &t_point:points_near_free){
                //替换每一个点
                // Eigen::Vector3d new_point;
                // ROS_INFO("GRID MAP RES %f",inspector.grid_map.resolution);
                inspector.grid_map.search_near_free_point(t_point,false);
            }
            // std::cout << "["<< uav_name << "] " << "after replace" << std::endl;
            
            for(auto &t_point:points_near_free){
                // std::cout<<t_point.transpose()<<std::endl;
            }
            //to do
            //我发现这样处理后，可能会存在重复的点，但是直接剔除不一定合适，因为我要拍那么多目标 


            // std::cout << "["<< uav_name << "] " << "checking obstacles in travel lists ..." << std::endl;
            // check是否没有障碍物(两两)
            size_t i = 0;
            size_t max_astar_points = 1000;
            // ROS_INFO("[%s] points_near_free.size:%d", uav_name.data(), points_near_free.size());
            while (i < arrange_point_index.size()-2)
            {
                // 如果travel_list最后一个元素与points[arrange_point_index[i]]相等，无需添加
                // if (travel_list.size() == 0 || (travel_list[travel_list.size() - 1] - points_near_free[arrange_point_index[i]]).norm() > 0.0001)
                // {   
                //     travel_list.push_back(points_near_free[arrange_point_index[i]]);
                //     travel_list_gimbal.push_back(points_cluster_cluster[arrange_point_index[i]]);
                //     sign_simplify.push_back(true);
                // }
                travel_list.push_back(points_near_free[arrange_point_index[i]]);
                travel_list_gimbal.push_back(points_cluster_cluster[arrange_point_index[i]]);
                sign_simplify.push_back(true);
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
                            travel_list_gimbal.push_back(points_cluster_cluster[arrange_point_index[i+1]]);//它反正一直照向期望的
                            sign_simplify.push_back(false);
                        }
                        if(i==arrange_point_index.size()-3){
                            travel_list.push_back(points_near_free[arrange_point_index[i+1]]);
                            travel_list_gimbal.push_back(points_cluster_cluster[arrange_point_index[i+1]]);
                            sign_simplify.push_back(true);
                            break;
                        }
                    }
                    else
                    {
                        // 搜索不到，抛弃这个点
                        points_near_free.erase(points_near_free.begin() + i + 1);
                        arrange_point_index.erase(arrange_point_index.begin() + i + 1);
                        points_cluster_cluster.erase(points_cluster_cluster.begin() + i + 1);
                        // travel_list.pop_back();//这个为什么要pop_back？
                        // i不变
                        continue;
                    }
                }

                i++;

            }
            // 就尽可能多遍历，
            need_arrange = false;
            
            // std::cout << "["<< uav_name << "] " << "attempting to publish travel list, size: " << travel_list.size() << " travel_list_gimbal size: " << travel_list_gimbal.size() << " sign_simplify size: " << sign_simplify.size() << std::endl;

            //到这一步，可以试着转为 inspector里面的minimal_snap
            //转为waypoints_list
            std::vector<waypoint> waypoints_list;//我需要做一些调整，获取各个时间段对应的目标点
            std::vector<waypoint*> waypoints_list_ptr;
            for(auto &t_point:travel_list){
                waypoint temp_wp;
                Eigen::Vector3i temp_index;
                inspector.grid_map.pos2index(t_point,temp_index);
                if(!waypoints_list.empty()){
                    if(temp_index==waypoints_list.back().idx){
                        // ROS_ERROR("[%s] temp_index==waypoints_list.back().idx", uav_name.data());
                        continue;
                    }
                }
                temp_wp.idx=temp_index;
                waypoints_list.emplace_back(temp_wp);
            }
            for(size_t i=0;i<waypoints_list.size();i++){
                waypoints_list_ptr.push_back(&waypoints_list[i]);
            }
            bool success_traj=inspector.generate_trajetory(waypoints_list_ptr,sign_simplify,travel_list_gimbal);
            // ROS_ERROR("[%s] success_traj:%d", uav_name.data(),success_traj);

            if(success_traj){
                inspector.curr_seg=0;
                // inspector.is_start=true;
                inspector.is_new_tarjectory_generate=true;
                inspector.is_last_trajectory_finish=false;
                inspector.start_t=ros::Time::now();
                inspector.generate_t=ros::Time::now();
                // ROS_INFO("[%s] traj size:%d", uav_name.data(),inspector._polyTime.size());
            }
            else{
                // ROS_ERROR("[%s] traj failed", uav_name.data());
                need_arrange = true;
                cluster_index++;
                inspector.curr_seg=0;
                inspector.is_new_tarjectory_generate=false;
                inspector.is_last_trajectory_finish=true;
            }

            //发布travel_list
            // std_msgs::Float64MultiArray travel_list_msg;
            // travel_list_msg.data.clear();
            // for(auto &t_point:travel_list){
            //     travel_list_msg.data.push_back(t_point.x());
            //     travel_list_msg.data.push_back(t_point.y());
            //     travel_list_msg.data.push_back(t_point.z());
            // }
            // for(auto &t_point:travel_list_gimbal){
            //     travel_list_msg.data.push_back(t_point.x());
            //     travel_list_msg.data.push_back(t_point.y());
            //     travel_list_msg.data.push_back(t_point.z());
            // }
            // travel_list_pub.publish(travel_list_msg);


            
        }
        
        //上面已经规划好轨迹了，现在由我执行控制，并进行gimbal的调整
        if(inspector.is_new_tarjectory_generate&&!inspector.is_last_trajectory_finish){
            if(!taking_off){
                std_msgs::Bool takeoff_msg;
                takeoff_msg.data=true;
                takeoff_pub.publish(takeoff_msg);
                taking_off=true;
            }
            // if((ros::Time::now()-inspector.generate_t).toSec()>5){
            //     need_arrange=true;
            // }
            if(((ros::Time::now()-inspector.start_t).toSec())>inspector._polyTime(inspector.curr_seg)){
                inspector.start_t+=ros::Duration(inspector._polyTime(inspector.curr_seg));
                inspector.curr_seg++;

                if(inspector.curr_seg>=inspector._polyTime.size()){
                    need_arrange=true;
                    // inspector.is_start=false;
                    inspector.is_last_trajectory_finish = true;
                    // ROS_INFO("[%s] curr traj finished!",inspector.name.c_str());
                    double hovering_height=uav_pos.z()>=0.5?uav_pos.z():0.5;
                    //发布控制率
                }
            }
            else{
                //期望位置
                Eigen::Vector3d target_pos=getPosPoly(inspector._polyCoeff,inspector.curr_seg,(ros::Time::now()-inspector.start_t).toSec());
                
                if((target_pos.z()>1.0 && (inspector.grid_map.get_occupancy_pos(target_pos) || inspector.grid_map.get_occupancy_pos(target_pos,true)))||std::isnan(target_pos.norm())||target_pos.norm()>1000){
                    need_arrange=true;
                    // ROS_ERROR("[%s] target_pos is occupied", uav_name.data());
                }
                else{
                    
                    //再检查
                    bool recheck=true;
                    for(int i=0;i<4;i++){
                        Eigen::Vector3d tmp_point=(target_pos*(i+1)+uav_pos*(4-i))/5;
                        // middle_check_points.push_back(tmp_points);
                        if(tmp_point.z()>1.0 && inspector.grid_map.get_occupancy_pos(tmp_point)){
                            need_arrange=true;
                            recheck=false;
                            // ROS_WARN("RECHECK FALSE");
                            break;
                        }
                    }
                    // if(checked_points.size()<20){
                    //     recheck=true;
                    // }
                    //轨迹的不同seg需要对应到不同的目标点，由此计算yaw
                    //发布控制率
                    if(recheck){
                        int target_index=0;
                        for(int i=0;i<=inspector.curr_seg;i++){
                            if(sign_simplify[i]){
                                target_index++;
                            }
                        }
                        Eigen::Vector3d target_gimbal=points_cluster_cluster[target_index];
                        //
                        std_msgs::Float64MultiArray target_point_msg;
                        target_point_msg.data.clear();
                        target_point_msg.data.push_back(target_pos.x());
                        target_point_msg.data.push_back(target_pos.y());
                        target_point_msg.data.push_back(target_pos.z());
                        target_point_msg.data.push_back(target_gimbal.x());
                        target_point_msg.data.push_back(target_gimbal.y());
                        target_point_msg.data.push_back(target_gimbal.z());
                        target_point_pub.publish(target_point_msg);
                    }
                }   
            }
        }

        // else{
        //     ROS_WARN("[%s] has no target_points or need_arrange", uav_name.data());
        // }
        // 这个就检查当前的类，更近cluster_index和point_index


        //to do
        //还需要判断是不是所有的点都检查完了，如果检查完了，就要重新聚类
        ros::spinOnce();
        ros::Rate(60).sleep();
    }
}

