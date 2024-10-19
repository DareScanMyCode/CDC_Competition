#include <Eigen/Dense>
#include <ros/ros.h>
#include <string>
#include "k_means_cluster.h"
#include "a_star.h"
#include "inspector_mini.h"
#include "msg_pub.h"

#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>


double cal_dis(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return sqrt(pow(p1.x() - p2.x(), 2) + pow(p1.y() - p2.y(), 2) + pow(p1.z() - p2.z(), 2));
}

inline double cal_dis_w_alt(Eigen::Vector3d pos_1, Eigen::Vector3d pos_2) {
        double cost_factor_xy = 1;
        double cost_factor_z  = 0.25;           // 可以调整

        if (pos_1[2] <= pos_2[2]) {
            cost_factor_z = 0.85;                  // default 0.25 可以是负的, 但是设计成负值可能导致一直往高了飞
        } else {                                   //               这和inspector_plus.h用的函数有点不一样, 注意上面的<=号, inspector是反过来的
            cost_factor_z = 0.85;                  //         1.25
        }

        double dx = pos_1[0] - pos_2[0];
        double dy = pos_1[1] - pos_2[1];
        double dz = pos_1[2] - pos_2[2];

        return std::sqrt(cost_factor_xy * (dx * dx + dy * dy) + cost_factor_z * dz * dz);    
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
            //dis_mat(i, j) = cal_dis(target_points[i], target_points[j]);
            //dis_mat(j, i) = dis_mat(i, j);
            dis_mat(i, j) = cal_dis_w_alt(target_points[i], target_points[j]);
            dis_mat(j, i) = cal_dis_w_alt(target_points[j], target_points[i]);      // Added, 这里改过了, 此时邻接矩阵是不对称的
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

bool check_collision_free(Eigen::Vector3d pos1, Eigen::Vector3d pos2, GridMap_mini &grid_map, double resolution)
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

double saturation(double num,double max,double min){
    if(num>max)
        num=num-max;
    else if(num<min)
        num=num-min;
    return num;
}

class taker_controller
{
public:
    taker_controller(string name, ros::NodeHandle &nh, double max_uav_vel = 1, double max_uav_w = 2, double max_gimbal_w = 1) : vel_puber(nh, name), gimbal_puber(nh, name)
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
    void uav_controller(std::vector<Eigen::Vector3d> travel_list,std::vector<Eigen::Vector3d> travel_list_gimbal, bool &need_arrange, int &cluster_index, int &point_index, double tolerence = 3,bool vel_ctrl=false, bool is_debug=false);
    void uav_controller(Eigen::Vector3d target_pos,double tolerence=3,bool vel_ctrl=false, bool is_debug=false);
    void uav_controller(Eigen::Vector3d next_pos, Eigen::Vector3d next_target);
    void send_control_target(Eigen::Vector3d target_pos, Eigen::Vector3d target_target_pos);
    void gimbal_controller(std::vector<Eigen::Vector3d> travel_list, int point_index);
    double cal_desired_yaw(std::vector<Eigen::Vector3d> travel_list, int point_index);
    double cal_desired_pitch(std::vector<Eigen::Vector3d> travel_list, int point_index);
    void gimbal_controller(Eigen::Vector3d next_target);
    double cal_desired_yaw(Eigen::Vector3d next_target);
    double cal_desired_pitch(Eigen::Vector3d next_target);
    TrajPuber vel_puber;
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
    GimbalPuber gimbal_puber;
};

double taker_controller::cal_desired_yaw(std::vector<Eigen::Vector3d> travel_list, int point_index){
    Eigen::Vector3d target_point = travel_list[point_index];
    target_point<<target_point.x(),target_point.y(),this->uav_pos.z();//投影到这个平面
    //计算两个向量的yaw
    //1.判断目标相对无人机在哪个象限
    Eigen::Vector3d target_dir = target_point - this->uav_pos;
    
    double yaw_dir;
    int phase_index=0;
    if(target_dir.x()>=0 && target_dir.y()>=0){
        yaw_dir=atan2(target_dir.y(),target_dir.x());
        phase_index=1;
        yaw_dir=yaw_dir;
    }
    else if(target_dir.x()<0 && target_dir.y()>0){
        yaw_dir=atan2(target_dir.y(),-target_dir.x());
        phase_index=2;
        yaw_dir=M_PI-yaw_dir;
    }   
    else if(target_dir.x()<=0 && target_dir.y()<=0){
        yaw_dir=atan2(-target_dir.y(),-target_dir.x());
        phase_index=3;
        yaw_dir=-M_PI+yaw_dir;
    }
    else{
        yaw_dir=atan2(-target_dir.y(),target_dir.x());
        phase_index=4;
        yaw_dir=-yaw_dir;
    }
    // yaw_dir=saturation(yaw_dir,M_PI,-M_PI);
    double uav_yaw=this->uav_ori.z();
    // int uav_phase_index=0;
    // if(uav_yaw>=0 && uav_yaw<=M_PI/2){
    //     uav_phase_index=1;
    // }
    // else if(uav_yaw>M_PI/2 && uav_yaw<=M_PI){
    //     uav_phase_index=2;
    // }
    // else if(uav_yaw>-M_PI && uav_yaw<=-M_PI/2){
    //     uav_phase_index=3;
    // }
    // else{
    //     uav_phase_index=4;
    // }

    double desired_yaw=yaw_dir;
    // double yaw_diff=uav_yaw-yaw_dir;
    // yaw_diff=saturation(yaw_diff,M_PI,-M_PI);
    // // if(saturation(uav_yaw+yaw_diff,M_PI/2,-M_PI/2)==yaw_dir){
    // //     desired_yaw=yaw_diff;
    // // }
    // // else{
    // //     desired_yaw=-yaw_diff;
    // // }
    // if(uav_yaw>yaw_dir){
    //     if(yaw_diff>M_PI){
    //         desired_yaw=2*M_PI-yaw_diff;
    //     }
    //     else{
    //         desired_yaw=-yaw_diff;
    //     }
    // }
    // else{
    //     if(abs(yaw_diff)>M_PI){
    //         desired_yaw=-2*M_PI+yaw_diff;
    //     }
    //     else{
    //         desired_yaw=-yaw_diff;
    //     }
    // }
    return desired_yaw;
}

double taker_controller::cal_desired_yaw(Eigen::Vector3d next_target){
    Eigen::Vector3d target_point = next_target;
    target_point<<target_point.x(),target_point.y(),this->uav_pos.z();//投影到这个平面
    //计算两个向量的yaw
    //1.判断目标相对无人机在哪个象限
    Eigen::Vector3d target_dir = target_point - this->uav_pos;
    
    double yaw_dir;
    int phase_index=0;
    if(target_dir.x()>=0 && target_dir.y()>=0){
        yaw_dir=atan2(target_dir.y(),target_dir.x());
        phase_index=1;
        yaw_dir=yaw_dir;
    }
    else if(target_dir.x()<0 && target_dir.y()>0){
        yaw_dir=atan2(target_dir.y(),-target_dir.x());
        phase_index=2;
        yaw_dir=M_PI-yaw_dir;
    }   
    else if(target_dir.x()<=0 && target_dir.y()<=0){
        yaw_dir=atan2(-target_dir.y(),-target_dir.x());
        phase_index=3;
        yaw_dir=-M_PI+yaw_dir;
    }
    else{
        yaw_dir=atan2(-target_dir.y(),target_dir.x());
        phase_index=4;
        yaw_dir=-yaw_dir;
    }
    double uav_yaw=this->uav_ori.z();
    double desired_yaw=yaw_dir;
    return desired_yaw;
}

double taker_controller::cal_desired_pitch(std::vector<Eigen::Vector3d> travel_list, int point_index){
    Eigen::Vector3d target_point = travel_list[point_index];
    Eigen::Vector3d pos_diff=target_point-this->uav_pos;
    double z_diff=pos_diff.z();
    double xy_diff=sqrt(pow(pos_diff.x(),2)+pow(pos_diff.y(),2));
    double desired_pitch;
    if(z_diff<0){//因为逆时针为正
        desired_pitch=atan2(-z_diff,xy_diff);
    }
    else{
        desired_pitch=-atan2(z_diff,xy_diff);
    }
    return desired_pitch;
}

double taker_controller::cal_desired_pitch(Eigen::Vector3d next_target){
    Eigen::Vector3d target_point = next_target;
    Eigen::Vector3d pos_diff=target_point-this->uav_pos;
    double z_diff=pos_diff.z();
    double xy_diff=sqrt(pow(pos_diff.x(),2)+pow(pos_diff.y(),2));
    double desired_pitch;
    if(z_diff<0){//因为逆时针为正
        desired_pitch=atan2(-z_diff,xy_diff);
    }
    else{
        desired_pitch=-atan2(z_diff,xy_diff);
    }
    return desired_pitch;
}


void taker_controller::uav_controller(std::vector<Eigen::Vector3d> travel_list,std::vector<Eigen::Vector3d> travel_list_gimbal, bool &need_arrange, int &cluster_index, int &point_index, double tolerence,bool vel_ctrl, bool is_debug)
{
    double desired_yaw=cal_desired_yaw(travel_list_gimbal,point_index)+this->uav_ori.z();
    if (is_debug) {
        ROS_INFO("[%s] desired_yaw: %f", this->name.data(), desired_yaw);
        ROS_INFO("[%s] uav_yaw:     %f", this->name.data(), this->uav_ori.z());
    }

    if(!vel_ctrl){
        if ((this->uav_pos - travel_list[point_index]).norm() < tolerence)
        {
            // 到达了这个点
            point_index++;
            ROS_INFO("[%s] point_index: %d", this->name.data(), point_index);
            if (size_t(point_index) == travel_list.size())
            {
                ROS_INFO("[%s] has finished one cluster", this->name.data());
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

            // vel_puber.pub_point(target_pos(0),target_pos(1),target_pos(2),desired_yaw,0,0);
            vel_puber.pub_point(this->uav_pos.x(), this->uav_pos.y(), this->uav_pos.z(),
                                target_pos(0),     target_pos(1),     target_pos(2), desired_yaw, 0, 0, true);        // Added

            if (is_debug) {
                // ROS_INFO("[%s] UAV_pos:   \t%f, %f, %f", this->name.data(), this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
                // ROS_INFO("[%s] target_pos:\t%f, %f, %f", this->name.data(), target_pos(0),target_pos(1),target_pos(2));
            }
        }
    }
    else{
        
        if ((this->uav_pos - travel_list[point_index]).norm() < tolerence)
        {
            // 到达了这个点
            point_index++;
            ROS_INFO("[%s] point_index: %d", this->name.data(), point_index);
            if (size_t(point_index) == travel_list.size())
            {
                ROS_INFO("[%s] has finished one cluster", this->name.data());
                // 到达了这个类的最后一个点
                need_arrange = true;
                cluster_index++;
                // point_index = 0;
            }
        }

        else{
            Eigen::Vector3d target_pos=travel_list[point_index];
            if(target_pos.z() < 0.5)
                target_pos.z() = 0.5;
            Eigen::Vector3d direct=target_pos-this->uav_pos,dir_vel;
            double dis=direct.norm();
            //direct变为单位向量
            direct=direct/dis;
            dir_vel=direct*max_uav_vel;

            if(dis<2){
                ROS_INFO("[%s] dis: %f", this->name.data(), dis);
                if(point_index<travel_list.size()-1){
                    Eigen::Vector3d next_pos=travel_list[point_index+1];
                    Eigen::Vector3d next_direct=next_pos-this->uav_pos;
                    next_direct=next_direct/next_direct.norm();

                    ROS_INFO("direct: %f, %f, %f", direct(0), direct(1), direct(2));
                    ROS_INFO("next_direct: %f, %f, %f", next_direct(0), next_direct(1), next_direct(2));
                    double angle = acos((direct/direct.norm()).dot(next_direct/next_direct.norm()));
                    Eigen::Vector3d cross = direct.cross(next_direct);
                    // 如果dir2在dir1的逆时针方向，cross.z>0
                    if (cross.z() < 0)
                    {
                        angle = -angle;
                    }
                    ROS_INFO("[%s] angle: %f", this->name.data(), angle);
                    if(cos(angle)>0 ){
                        dir_vel=dir_vel*cos(angle);
                    }
                    else{
                        dir_vel=dir_vel*dis/2;
                    }
                }
                else{
                    dir_vel=dir_vel*dis/2;
                }
            }

            vel_puber.pub_vel(dir_vel(0),dir_vel(1),dir_vel(2),desired_yaw);

            if (is_debug) {
                // ROS_WARN("[%s] dis: %f", this->name.data(), dis);
                // ROS_INFO("[%s] UAV_pos:    %f,\t%f,\t%f", this->name.data(), this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
                // ROS_INFO("[%s] target_pos: %f,\t%f,\t%f", this->name.data(), target_pos(0),target_pos(1),target_pos(2));
                // ROS_INFO("[%s] VEL:        %f,\t%f,\t%f", this->name.data(), dir_vel(0),dir_vel(1),dir_vel(2));
            }
        }
    }
};

void taker_controller::uav_controller(Eigen::Vector3d target_pos,double tolerence,bool vel_ctrl, bool is_debug){
    double kp=max_uav_vel;
    if(!vel_ctrl){
        if (is_debug) {
            // ROS_INFO("[%s] UAV_pos:   %f,%f,%f", this->name.data(), this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
            // ROS_INFO("[%s] target_pos:%f,%f,%f", this->name.data(), target_pos(0),target_pos(1),target_pos(2));
        }

        if ((this->uav_pos - target_pos).norm() > tolerence){
            vel_puber.pub_point(uav_pos(0),    uav_pos(1),    uav_pos(2),
                                target_pos(0), target_pos(1), target_pos(2),0,0,0, true);
            //vel_puber.pub_point(target_pos(0),target_pos(1),target_pos(2),0,0,0);
        }
    }
    else{
        Eigen::Vector3d direct=target_pos-this->uav_pos;
        // double err=0;
        // for(int i=0;i<3;i++){
        //     err+=direct[i]*direct[i];
        // }
        double dis=direct.norm();
        direct=direct*max_uav_vel*dis*0.5;
    
        vel_puber.pub_vel(direct(0),direct(1),direct(2),0);
        if (is_debug) {
            // ROS_WARN("[%s] dis: %f", this->name.data(), dis);
            // ROS_INFO("[%s] UAV_pos:    %f,\t%f,\t%f", this->name.data(), this->uav_pos.x(),this->uav_pos.y(),this->uav_pos.z());
            // ROS_INFO("[%s] target_pos: %f,\t%f,\t%f", this->name.data(), target_pos(0),target_pos(1),target_pos(2));
            // ROS_INFO("[%s] VEL:        %f,\t%f,\t%f", this->name.data(), direct(0),direct(1),direct(2));
        }
    }
};

void taker_controller::uav_controller(Eigen::Vector3d next_pos, Eigen::Vector3d next_target){
    double desired_yaw=cal_desired_yaw(next_target)+this->uav_ori.z();

    Eigen::Vector3d target_pos=next_pos;
    if(target_pos.z()<0.5)
        target_pos.z()=0.5;
    // vel_puber.pub_vel(target_vel(0), target_vel(1), target_vel(2), target_w(2));

    // vel_puber.pub_point(target_pos(0),target_pos(1),target_pos(2),desired_yaw,0,0);
    vel_puber.pub_point(this->uav_pos.x(), this->uav_pos.y(), this->uav_pos.z(),
                        target_pos(0),     target_pos(1),     target_pos(2), desired_yaw, 0, 0, true);        // Added
}


void taker_controller::gimbal_controller(std::vector<Eigen::Vector3d> travel_list, int point_index)
{
    // // 目标是云台指向target_point
    // // 云台相对无人机仅可以改变pitch，yaw
    // // 先计算无人机到目标点的方向
    // Eigen::Vector3d target_point = travel_list[point_index];
    // Eigen::Vector3d target_dir = target_point - this->uav_pos;
    // // 计算无人机旋转矩阵
    // Eigen::Matrix3d R;
    // R << cos(this->uav_ori.z()), -sin(this->uav_ori.z()), 0,
    //     sin(this->uav_ori.z()), cos(this->uav_ori.z()), 0,
    //     0, 0, 1;
    // // 计算无人机到目标点的方向在无人机坐标系下的表示
    // Eigen::Vector3d target_dir_uav = R.transpose() * target_dir;
    // // 确定pitch和yaw
    // double pitch = atan2(target_dir_uav.y(), target_dir_uav.z()); // 这个可能大概率是错的。。。
    // double yaw = atan2(target_dir_uav.x(), target_dir_uav.z());
    // // 计算角速度
    // double pitch_w = (pitch - this->gimbal_ori.x()) * max_gimbal_w; // 这个可能太大
    // double yaw_w = (yaw - this->gimbal_ori.y()) * max_gimbal_w;
    // // 计算并发送控制指令
    // gimbal_puber.pub_gimbal_command(pitch_w, yaw_w);
    double desired_yaw=this->cal_desired_yaw(travel_list,point_index);
    double desired_pitch=this->cal_desired_pitch(travel_list,point_index);
    gimbal_puber.pub_angle_control(desired_pitch,desired_yaw);
}

void taker_controller::gimbal_controller(Eigen::Vector3d next_target){
    double desired_yaw=cal_desired_yaw(next_target);
    double desired_pitch=cal_desired_pitch(next_target);
    double dis=(this->uav_pos-next_target).norm();
    if(dis>5){
        if(desired_pitch<-M_PI/4){
            desired_pitch=-M_PI/4;
            // std::cout<<"pitch too small"<<std::endl;
        }
        if(desired_pitch>M_PI/4){
            desired_pitch=M_PI/4;
            // std::cout<<"pitch too big"<<std::endl;
        }
    }
    else{
        // std::cout<<"close to target, pitch is: "<<desired_pitch<<std::endl;
    }
    gimbal_puber.pub_angle_control(desired_pitch,desired_yaw);
}