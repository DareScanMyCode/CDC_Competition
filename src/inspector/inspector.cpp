#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <string>
#include "inspector/inspector_plus.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <ctime>
#include <functional>
#include <algorithm>
#include <cmath>
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
// #include <map>
#include <unordered_map>
#include <queue>
#include <std_msgs/Duration.h>
Inspector* inspector_pointer;
// Inspector* inspector_pointer_for_inbox;
Inspector* just_for_check_inbox;

static std::string uav_name;                // Added 这里是一个名字, 如果是小飞机那要两个名字
                                            //       https://ntu-aris.github.io/caric/#64-communication-network
                                            //       For each target specified, 
                                            //       a new topic with the original name appended with the TARGET node ID will be created.
                                            //       要理解这里的TARGET (啊那其实没错)
int resolution=2.0;
int x_min,x_max,y_min,y_max,z_min,z_max;
int expand_size = 5;
int x_length,y_length,z_length;
std::vector<Eigen::Vector3d> vector3d_face_data;
std::vector<Eigen::Vector3d> vector3d_all_data;
std::vector<Eigen::Vector3d> vector3d_search_data;
bool face_points_is_get = false, all_points_is_get = false,search_points_is_get = false;
std::unordered_map<Eigen::Vector3i,bool> all_points_index_map;//chen added for in bounding box check
std::mutex mtx,mtx1;

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

struct rollbackList{
    std::vector<rollbackNode> list;
    void push_back(rollbackNode node){
        list.push_back(node);
    }
    void pop_back(){
        list.pop_back();
    }
    void clear(){
        list.clear();
    }
    int size(){
        return list.size();
    }
    rollbackNode back(){
        return list.back();
    }
    void erase(int index){
        list.erase(list.begin()+index);
    }
    void erase(rollbackNode node){
        for(int i=0;i<list.size();i++){
            if(list[i].pos==node.pos){
                list.erase(list.begin()+i);
                return;
            }
        }
    }
    bool find(rollbackNode node){
        for(int i=0;i<list.size();i++){
            if(list[i].pos==node.pos){
                return true;
            }
        }
        return false;
    }
}rollback_list;


// 记录最后和地面站通讯的位置
std::queue<Eigen::Vector3d> odometryQueue;
std::stack<Eigen::Vector3d> posStack;
std::stack<rollbackNode> posRollBackStack;
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
        if (posRollBackStack.size() == 0) {
            break;
        }
        // 等待一段时间
        // double time_to_sleep = 0.80 * (rollback_time_now - node.vist_time).toSec();
        double time_to_sleep = std::min(
            0.80 * (rollback_time_now - node.vist_time).toSec(),
            (node.pos - posRollBackStack.top().pos).norm() / 2.0
        );
        rollback_time_now = node.vist_time;
        ros::Duration(time_to_sleep).sleep();
    }
}

void mantain_gcs_conn(){
    if(is_gcs_connected){
        if((ros::Time::now()-last_gcs_connected_time).toSec()>0.5){
            is_gcs_connected = false;
        }
    }
    if(!is_gcs_connected){
        // 只要断了联系，就这样
        if(posStack.size()>2 && !rollback_init){
            // 取出前两个点（保险起见）
            rollbackNode new_rb_node1({ros::Time::now(),
                                posStack.top()});
            posStack.pop();
            rollbackNode new_rb_node2({ros::Time::now(),
                                posStack.top()});
            posStack.push(new_rb_node1.pos);  // 恢复队列

            // 加入回滚列表
            posRollBackStack.push(new_rb_node1);
            posRollBackStack.push(new_rb_node2);

            
            rollback_init = true;
        }
        posRollBackStack.push(
                rollbackNode(
                    {ros::Time::now(),
                    inspector_pointer->pos}
                )
        );
        
    }

}




Eigen::Vector3d get_gcs_connected_pos(){
    // TODO 逻辑？
    Eigen::Vector3d p1, p2, p3;
    if(odometryQueue.empty()){
        return inspector_pointer->pos;
        ROS_WARN("[%s]: empty pos queue", uav_name);
    }
    for(int i=0; i<15; i++){
        if(i==0){
            p1=odometryQueue.front();
            odometryQueue.pop();
            if(odometryQueue.empty()){
                return p1;
                ROS_WARN("[%s]: empty pos queue: code-1", uav_name);
            }
            p2=odometryQueue.front();
            odometryQueue.pop();
            if(odometryQueue.empty()){
                return p2;
                ROS_WARN("[%s]: empty pos queue: code-2", uav_name);
            }
            p3=odometryQueue.front();
            odometryQueue.pop();
        }else
        {
            if(odometryQueue.empty()){
                return p2;
                ROS_WARN("[%s]: empty pos queue: code-2", uav_name);
            }
            p1=p2; p2=p3;
            p3=odometryQueue.front();
            odometryQueue.pop();
        }
        if(((p1-p2).norm()+(p2-p3).norm()) < 0.5){
            return p2;
        }
    }
    return odometryQueue.front();
}

void grid_map_cb_t(const caric_competition_xmu::GridMapMsg::ConstPtr &msg){

    std::lock_guard<std::mutex> lock(mtx); //线程互斥锁
    try {
        // ROS_INFO("[%s] grid_callback",inspector_pointer->name.c_str());
        //std::cout<<inspector_pointer->grid_map.grid_map[0][0][0].pos<<std::endl;

        // if (!inspector_pointer->is_start) {
        //     // std::cout << "[" + uav_name + "]" << "RECEIVED in grid_map_callback, but WAITING FOR start!" << std::endl;
        //     return;
        // }

        // Added CRITICAL
        // 实测现在是这里会Bug
        // 不加的话能跑
        // 截止 2023-11-22 02:03这个问题好像解决了?
        // ROS_INFO("[%s] grid_callback, frame_id: %s",inspector_pointer->name.c_str(),msg->header.frame_id.c_str()
        // if (msg->header.frame_id.size() && msg->header.frame_id != uav_name) {
            // ROS_INFO("[%s] grid_callback, but frame_id is not %s",inspector_pointer->name.c_str(),uav_name.c_str());
            //chen comment: merge grid_map 加一个参数，all_points_index_map
            // caric_competition_xmu::merge_grid_map(inspector_pointer->grid_map, caric_competition_xmu::msg2map(*msg));
            caric_competition_xmu::merge_grid_map_inbox(inspector_pointer->grid_map, caric_competition_xmu::msg2map(*msg),just_for_check_inbox->grid_map);
            // caric_competition_xmu::merge_grid_map_inbox(inspector_pointer_for_inbox->grid_map, caric_competition_xmu::msg2map(*msg),just_for_check_inbox->grid_map);
            // std::cout << "[" + uav_name + "]" << "MERGING grid_map in callback from: " << msg->header.frame_id << " " <<
                        // inspector_pointer->grid_map.grid_map[0][0][0].pos(0) << "\t" << 
                        // inspector_pointer->grid_map.grid_map[0][0][0].pos(1) << "\t" <<
                        // inspector_pointer->grid_map.grid_map[0][0][0].pos(2) << std::endl;        
        // }
        // ROS_INFO("[%s] grid_callback done",inspector_pointer->name.c_str());
        /*
        std::cout << "[" + uav_name + "]" << "RECEIVED in grid_map_callback from: " << msg->header.frame_id << " " <<
                    inspector_pointer->grid_map.grid_map[0][0][0].pos(0) << "\t" << 
                    inspector_pointer->grid_map.grid_map[0][0][0].pos(1) << "\t" <<
                    inspector_pointer->grid_map.grid_map[0][0][0].pos(2) << std::endl;
        */
    }
    catch(...){
        // ROS_INFO("[%s] grid_callback error",inspector_pointer->name.c_str());
    }
    if(is_debuging) ROS_INFO("grid_map_cb_t done==========");
}   

void grid_map_callback(const caric_competition_xmu::GridMapMsg::ConstPtr &msg) {
    if(is_debuging) ROS_INFO("grid_map+cb");

    std::thread t_grid_map(grid_map_cb_t,msg);
    t_grid_map.detach();
    if(is_debuging) ROS_INFO("grid_map+cb done==============");

}

void search_points_cb_t(const caric_competition_xmu::SearchPointArray::ConstPtr & msg){
    if(is_debuging) ROS_INFO("search points cb t");

    std::lock_guard<std::mutex> lock(mtx1); //线程互斥锁
    try {
        // 转换
        //ROS_INFO("[%s] search_points_callback",inspector_pointer->name.c_str());
        std::vector<std::vector<std::vector<SearchPoint>>> tmp_search_points_array;
        tmp_search_points_array = caric_competition_xmu::msg2search_point_array3(*msg);

        volatile int32_t total_size_recv = 0;
        // TODO 合并
        // 此处是改变一个全局的变量！
        for(int i = 0; i < msg->max_index_x; i++){
                for(int j = 0; j < msg->max_index_y; j++){
                    for(int k = 0; k < msg->max_index_z; k++){
                        caric_competition_xmu::SearchPointMsg node;
                        inspector_pointer->search_points_array[i][j][k].is_visited      = inspector_pointer->search_points_array[i][j][k].is_visited        || tmp_search_points_array[i][j][k].is_visited;
                        // inspector_pointer->search_points_array[i][j][k].is_bad          = inspector_pointer->search_points_array[i][j][k].is_bad            || tmp_search_points_array[i][j][k].is_bad;
                        inspector_pointer->search_points_array[i][j][k].is_in_frontier  = inspector_pointer->search_points_array[i][j][k].is_in_frontier    || tmp_search_points_array[i][j][k].is_in_frontier;
                        // TODO 与或？
                        inspector_pointer->search_points_array[i][j][k].is_obstacle     = inspector_pointer->search_points_array[i][j][k].is_obstacle       || tmp_search_points_array[i][j][k].is_obstacle;
                        // global_search_points_array[i][j][k].heuristic       = global_search_points_array[i][j][k].heuristic         || tmp_search_points_array[i][j][k].is_visited;

                        total_size_recv++;
                    }
                }
            }

        // std::cout << "[" + uav_name + "]" << "RECEIVED in search_points_callback: " << total_size_recv << std::endl;
        //ROS_INFO("[%s] search_points_callback done",inspector_pointer->name.c_str());
    }
    catch(...){
        ROS_INFO("[%s] search_points_callback error",inspector_pointer->name.c_str());
    }
    if(is_debuging) ROS_INFO("search points cb t done ==========");

}

void search_points_callback(const caric_competition_xmu::SearchPointArray::ConstPtr & msg){
    if(is_debuging) ROS_INFO("search point cb");
    std::thread t_search_points(search_points_cb_t,msg);
    t_search_points.detach();
    if(is_debuging) ROS_INFO("search points cb done ===========");
    
}


void gcs_score_callback(const sensor_msgs::PointCloud::ConstPtr & msg){
    caric_competition_xmu::gcs_score_pub.publish(*msg);
    // ROS_INFO("gcs_score_callback");
}

void frontier_points_callback(const caric_competition_xmu::SearchPointArray::ConstPtr & msg){
    // 转换
    std::vector<SearchPoint> search_points_array;
    search_points_array = caric_competition_xmu::msg2search_point_array(*msg);
    // ROS_WARN("[%s] get another uav ,%d",uav_name.c_str(),search_points_array.size());
    // TODO 合并
    // std::cout << "[" + uav_name + "]" << "RECEIVED in frontier_points_callback "  << search_points_array.size() << std::endl;
    // caric_competition_xmu::SearchPoint
    // ROS_WARN("%d",msg->max_index_x);
    for(int i = 0; i < msg->max_index_x; i++){
        SearchPoint new_point_msg = search_points_array[i];
        bool is_new = true;
        for(int j=0; j< inspector_pointer->frontier_points.size();j++){
            SearchPoint old_node;
            old_node = inspector_pointer->frontier_points[j];
            if(new_point_msg.x == old_node.x && new_point_msg.y == old_node.y && new_point_msg.z == old_node.z){
                is_new = false;
                break;
            }
        }
        if(is_new){
            SearchPoint new_point;
            new_point.heuristic         = new_point_msg.heuristic     ;
            new_point.is_bad            = new_point_msg.is_bad        ;
            new_point.is_in_frontier    = new_point_msg.is_in_frontier;
            new_point.is_visited        = new_point_msg.is_visited    ;
            new_point.is_obstacle       = new_point_msg.is_obstacle   ;
            new_point.x                 = new_point_msg.x             ;
            new_point.y                 = new_point_msg.y             ;
            new_point.z                 = new_point_msg.z             ;
            new_point.pos = new_point_msg.pos;
            inspector_pointer->frontier_points.push_back(new_point);  // 
            // ROS_WARN("[%s] get a new frontier points", uav_name.c_str());
        }


    }
    // std::cout << "[" + uav_name + "]" << "RECEIVED in frontier_points_callback "  << inspector_pointer->frontier_points.size() << std::endl;

}

void swarm_odom_cb(const caric_competition_xmu::OdometryArrayConstPtr& msg){
    // ROS_INFO("--------------------------------------------------------------------------------------");

    try {
        // Added
        //ROS_INFO("[%s] swarm_odom_cb",inspector_pointer->name.c_str());
        for(int i = 0; i < msg->swarm_size.data; i++) {
            if(msg->odometry_array[i].header.stamp.sec > inspector_pointer->swarm_odom[i].header.stamp.sec ||
             (msg->odometry_array[i].header.stamp.sec == inspector_pointer->swarm_odom[i].header.stamp.sec &&
             msg->odometry_array[i].header.stamp.nsec > inspector_pointer->swarm_odom[i].header.stamp.nsec)){
                inspector_pointer->swarm_odom[i] = msg->odometry_array[i];
                if(i==(1 - inspector_pointer->uav_id)){
                    inspector_pointer->partner_pos=Eigen::Vector3d(inspector_pointer->swarm_odom[i].pose.pose.position.x,inspector_pointer->swarm_odom[i].pose.pose.position.y,inspector_pointer->swarm_odom[i].pose.pose.position.z);
                    //ROS打印伙伴位置
                    // ROS_WARN(" get partner [uav_id %d] pos %f%f%f",1-inspector_pointer->uav_id,inspector_pointer->partner_pos[0],inspector_pointer->partner_pos[1],inspector_pointer->partner_pos[2]);
                }
        }

        /* Added, for debugging */
        // std::cout << "[" + uav_name + "]" << "RECEIVED in swarm_odom_cb, cur_index: " << i << " pose: " << 
        //              inspector_pointer->swarm_odom[i].pose.pose.position.x << "\t" <<
        //              inspector_pointer->swarm_odom[i].pose.pose.position.y << "\t" << 
        //              inspector_pointer->swarm_odom[i].pose.pose.position.z << "\t" <<
        // std::endl;
        }
        //ROS_INFO("[%s] swarm_odom_cb",inspector_pointer->name.c_str());
    }
    catch(...){
        ROS_INFO("[%s] swarm_odom_cb error",inspector_pointer->name.c_str());
    }
    if(is_debuging) ROS_INFO("swarm odom cb done========");
}

void map_pub_thread_func(){
    try{    ros::Time t = ros::Time::now();       
    // std::cout <<"before1"<< "[" + uav_name + "]" << "PUBLISHING grid_map" << std::endl;
    auto re = caric_competition_xmu::map2msg(inspector_pointer->grid_map);
    // std::cout<< "after1"<<"[" + uav_name + "]" << "PUBLISHING grid_map, time: " << (ros::Time::now()-t).toSec() << std::endl;
    // std::cout <<"before2"<< "[" + uav_name + "]" << "PUBLISHING grid_map" << std::endl;
    caric_competition_xmu::map_msg_pub.publish(re);
    // std::cout<< "after2"<<"[" + uav_name + "]" << "PUBLISHING grid_map, time: " << (ros::Time::now()-t).toSec() << std::endl;
    }
    catch(...){
        ROS_INFO("[%s] map_pub_thread_func error",inspector_pointer->name.c_str());
    }
}

// void inbox_map_pub_thread_func(){
//     try{    ros::Time t = ros::Time::now();       
//     // std::cout <<"before1"<< "[" + uav_name + "]" << "PUBLISHING inbox_grid_map" << std::endl;
//     auto re = caric_competition_xmu::map2msg(inspector_pointer_for_inbox->grid_map);
//     // std::cout<< "after1"<<"[" + uav_name + "]" << "PUBLISHING inbox_grid_map, time: " << (ros::Time::now()-t).toSec() << std::endl;
//     // std::cout <<"before2"<< "[" + uav_name + "]" << "PUBLISHING inbox_grid_map" << std::endl;
//     caric_competition_xmu::inbox_map_msg_pub.publish(re);
//     std::cout<< "after2"<<"[" + uav_name + "]" << "PUBLISHING inbox_grid_map, time: " << (ros::Time::now()-t).toSec() << std::endl;
//     }
//     catch(...){
//         ROS_INFO("[%s] map_pub_thread_func error",inspector_pointer->name.c_str());
//     }
// }

void face_points_cb(const geometry_msgs::PoseArrayConstPtr & msg){
    if(is_debuging) ROS_INFO("face_points_cb");
    if(face_points_is_get) return;
    x_min = msg->poses[0].position.x-expand_size;
    y_min = msg->poses[0].position.y-expand_size;
    z_min = msg->poses[0].position.z-expand_size;
    x_max = msg->poses[1].position.x+expand_size;
    y_max = msg->poses[1].position.y+expand_size;
    z_max = msg->poses[1].position.z+expand_size;
    for(int i=2;i<msg->poses.size();i++){
        Eigen::Vector3d tmp(msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
        vector3d_face_data.push_back(tmp);
    }
    face_points_is_get = true;
    if(is_debuging) ROS_INFO("face_points_cb done============");
    
}

void all_points_cb(const geometry_msgs::PoseArrayConstPtr & msg){
    if(is_debuging) ROS_INFO("all points cb");
    if(all_points_is_get) return;
    for(int i=0;i<msg->poses.size();i++){
        Eigen::Vector3d tmp(msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
        vector3d_all_data.push_back(tmp);
    }
    // 创建一个bouding box的index的map

    all_points_is_get = true;
    // if(is_debuging) ROS_INFO("all points cb done===========");

}

void search_points_cb(const geometry_msgs::PoseArrayConstPtr & msg){
    // if(is_debuging) ROS_INFO("search points cb");
    if(search_points_is_get) return;
    x_length = msg->poses[0].position.x;
    y_length = msg->poses[0].position.y;
    z_length = msg->poses[0].position.z;
    resolution = (int)msg->poses[1].position.x;
    // ROS_INFO("[%s]: received resolution:[%d]", uav_name, resolution);
    for(int i=2;i<msg->poses.size();i++){
        Eigen::Vector3d tmp(msg->poses[i].position.x,msg->poses[i].position.y,msg->poses[i].position.z);
        vector3d_search_data.push_back(tmp);
    }
    search_points_is_get = true;
    // if(is_debuging) ROS_INFO("search points cb done ==============");

}

void interest_points_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    inspector_pointer->interest_points.clear();
    // 将sensor_msgs::PointCloud2的数据读入interest_points
    for(int i=0;i<msg->width;i++){
        Eigen::Vector3d point;
        point.x()=msg->data[i*msg->point_step+msg->fields[0].offset];
        point.y()=msg->data[i*msg->point_step+msg->fields[1].offset];
        point.z()=msg->data[i*msg->point_step+msg->fields[2].offset];
        inspector_pointer->interest_points.push_back(point);
    }    
}


void update_isolate_occupy(){
    int count = 0;
    for(int i=1;i<inspector_pointer->grid_map.max_index[0]-1;i++){
        for(int j=1;j<inspector_pointer->grid_map.max_index[1]-1;j++){
            for(int k=1;k<inspector_pointer->grid_map.max_index[2]-1;k++){
                if(!inspector_pointer->grid_map.grid_map[i][j][k].is_occupied)continue;
                if((inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i, j, k+1)) ||
                    inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i, j, k-1))  ||
                    inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i, j+1, k))  ||
                    inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i, j-1, k))  ||
                    inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i+1, j, k))  ||
                    inspector_pointer->grid_map.get_occupancy_idx(Eigen::Vector3i(i-1, j, k)))){
                }
                else{
                    inspector_pointer->grid_map.grid_map[i][j][k].is_occupied=false;
                    inspector_pointer->grid_map.grid_map[i][j][k].need_to_inspect=false;
                    count++;
                    Eigen::Vector3i tmp_idx(i,j,k);
                    for (int dx = -inspector_pointer->range; dx <= inspector_pointer->range; dx++) {
                        for (int dy = -inspector_pointer->range; dy <= inspector_pointer->range; dy++) {
                            for (int dz = -inspector_pointer->range; dz <= inspector_pointer->range; dz++) {
                                int newX = tmp_idx[0] + dx;
                                int newY = tmp_idx[1] + dy;
                                int newZ = tmp_idx[2] + dz;

                                // 检查是否越界
                                if (newX >= 0 && newX < inspector_pointer->grid_map.max_index[0] && newY >= 0 && newY < inspector_pointer->grid_map.max_index[1] && newZ >= 0 && newZ < inspector_pointer->grid_map.max_index[2]) {
                                    inspector_pointer->grid_map.grid_map[newX][newY][newZ].in_flat=false;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // ROS_WARN("%d",count);
}
    
int main(int argc, char** argv) {
    ros::init(argc, argv, "inspector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    nh_local.getParam("uav_plus_name", uav_name);
    std::cout<<"it's "<<uav_name<<std::endl;

    ros::Subscriber sub_face_points = nh.subscribe<geometry_msgs::PoseArray>("/face_points/"+uav_name, 1, face_points_cb);
    ros::Subscriber sub_all_points = nh.subscribe<geometry_msgs::PoseArray>("/all_points/"+uav_name, 1, all_points_cb);    
    ros::Subscriber sub_search_points = nh.subscribe<geometry_msgs::PoseArray>("/search_points/"+uav_name, 1, search_points_cb);
    
    


    while(!face_points_is_get || !all_points_is_get || !search_points_is_get){
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    ROS_INFO("[%s] All points get",uav_name.c_str());
    



    caric_competition_xmu::init_Communicator(nh,uav_name,false);

    // 实例化对象
    Inspector inspector(uav_name,x_length,y_length,z_length);
    Inspector inspector_inbox(uav_name,x_length,y_length,z_length);
    Inspector check_inbox(uav_name,x_length,y_length,z_length);
    inspector_pointer = &inspector;
    // inspector_pointer_for_inbox = &inspector_inbox;
    just_for_check_inbox = &check_inbox;
    // 初始化地图 
    inspector.init(x_min,x_max,y_min,y_max,z_min,z_max,resolution,vector3d_search_data);
    inspector.init_map(vector3d_all_data);
    // inspector.set_inpect_grid(vector3d_face_data);
    //输出data——size
    // std::cout<<"all_data_size:"<<vector3d_all_data.size()<<std::endl;
    //inbox
    // inspector_inbox.init(x_min,x_max,y_min,y_max,z_min,z_max,resolution,vector3d_search_data);
    // inspector_inbox.init_map_inbox(vector3d_all_data);
    // inspector_inbox.set_inpect_grid(vector3d_face_data);
    //check inbox
    check_inbox.init(x_min,x_max,y_min,y_max,z_min,z_max,resolution,vector3d_search_data);
    check_inbox.init_map_inbox(vector3d_all_data);
    // check_inbox.set_inpect_grid(vector3d_face_data);
    

    //chen added
    //创建一个map记录vector3d_all_data所有点的index
    for(int i=0;i<vector3d_all_data.size();i++){
        Eigen::Vector3i tmp_index;
        inspector.grid_map.pos2index(vector3d_all_data[i],tmp_index);
        all_points_index_map[tmp_index]=true;
    }
    //added
    // ROS_INFO("[%s] ready to subscribe", uav_name.c_str());

    // Subscriber
    ros::Subscriber octomap_sub         = nh.subscribe<octomap_msgs::Octomap>("/world/octomap/"+uav_name,1,std::bind(&Inspector::world_octomap_callback,&inspector,std::placeholders::_1));
    ros::Subscriber gimbal_sub          = nh.subscribe<geometry_msgs::TwistStamped>("/"+uav_name+"/gimbal", 1,std::bind(&Inspector::gimbal_callback,&inspector,std::placeholders::_1));
    ros::Subscriber grid_map_sub        = nh.subscribe<caric_competition_xmu::GridMapMsg>("/grid_map/" + uav_name, 1, grid_map_callback);                                 // Added, 10->1, 切记不要缓存队列
    ros::Subscriber search_points_sub   = nh.subscribe<caric_competition_xmu::SearchPointArray>("/search_points_array/" + uav_name, 1, search_points_callback);
    // ros::Subscriber forntier_points_sub = nh.subscribe<caric_competition_xmu::SearchPointArray>("/frontier_points/" + uav_name, 1, frontier_points_callback);
    ros::Subscriber swarm_sub           = nh.subscribe<caric_competition_xmu::OdometryArray>("/swarm_odometry/" + uav_name, 1, swarm_odom_cb);
    
    ros::Subscriber gcs_score_sub = nh.subscribe("/gcs_score/"+uav_name, 1, gcs_score_callback);

    ros::Subscriber gcs_ping_sub        = nh.subscribe<std_msgs::Bool>("/gcs_ping_msg/"+uav_name, 1, gcs_ping_cb);
    ros::Subscriber mis_dura_sub        = nh.subscribe<std_msgs::Duration>("/"+uav_name+"/mission_duration_remained", 1, mis_dura_cb);

    ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("/"+uav_name+"/ground_truth/odometry", 1, std::bind(&Inspector::odometry_callback,&inspector,std::placeholders::_1));
    
    ros::Subscriber detected_interest_sub = nh.subscribe("/"+uav_name+"/detected_interest_points/", 1, interest_points_callback);
    

    // Publisher
    TrajPuber control_point_puber    = TrajPuber(nh,uav_name);   
    GimbalPuber gimbal_control_puber = GimbalPuber(nh,uav_name);
    ros::Publisher marker_pub        = nh.advertise<visualization_msgs::Marker>("/uav_plus/"+uav_name+"/grid_map_marker", 10);
    ros::Publisher uav_ori_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav_plus/"+uav_name+"/uav_ori",10);
    ros::Publisher gimbal_ori_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav_plus/"+uav_name+"/gimbal_ori",10);

    // 创建 Marker 消息
    visualization_msgs::Marker marker_not_inspected;
    inspector.create_marker(&marker_not_inspected,"world","not_inspected",0,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0,0,1,1);
    
    //
    visualization_msgs::Marker marker_inspected;
    inspector.create_marker(&marker_inspected,"world","inspected",1,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,1,0,0,1);

    //
    visualization_msgs::Marker marker_occupied;
    inspector.create_marker(&marker_occupied,"world","occupied",2,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,1,1,1,1);

    //
    visualization_msgs::Marker marker_target;
    inspector.create_marker(&marker_target,"world","target",2,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,2,2,2,1,1,0,1);

    //
    visualization_msgs::Marker marker_search;
    inspector.create_marker(&marker_search,"world","search",2,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,2,2,2,0.5,0.5,0.2,1);

    //
    visualization_msgs::Marker marker_waypoint;
    inspector.create_marker(&marker_waypoint,"world","waypoint",2,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0,0,1,1);
   
    visualization_msgs::Marker marker_risk_waypoint;
    inspector.create_marker(&marker_risk_waypoint,"world","risk_waypoint",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,1,0,0,1);

    // visualization_msgs::Marker marker_trajectory;
    // inspector.create_marker(&marker_trajectory,"world","trajectory",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0.5,0.5,0.5,1);

    visualization_msgs::Marker marker_frontier_points;
    inspector.create_marker(&marker_frontier_points,"world","frontier_points",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0.7,0.2,0.5,1);

    visualization_msgs::Marker marker_gimbal_target;
    inspector.create_marker(&marker_gimbal_target,"world","gimbal_target",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1.5,1.5,1.5,0.3,0.3,0.3,1);

    visualization_msgs::Marker marker_bbox;
    inspector.create_marker(&marker_bbox,"world","bbox",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0,0,0,1);

    visualization_msgs::Marker marker_in_sight;
    inspector.create_marker(&marker_in_sight,"world","in_sight",3,visualization_msgs::Marker::POINTS,visualization_msgs::Marker::ADD,1,1,1,1,0.2,0.3,0.5,1);

    std::vector<visualization_msgs::Marker *> marker_list;
    marker_list.push_back(&marker_occupied);
    marker_list.push_back(&marker_target);
    marker_list.push_back(&marker_waypoint);
    marker_list.push_back(&marker_risk_waypoint);
    // marker_list.push_back(&marker_trajectory);
    marker_list.push_back(&marker_search);
    // marker_list.push_back(&marker_inspected);
    marker_list.push_back(&marker_not_inspected);
    marker_list.push_back(&marker_frontier_points);
    marker_list.push_back(&marker_gimbal_target);
    marker_list.push_back(&marker_bbox);
    marker_list.push_back(&marker_in_sight);
    
    ROS_INFO("[%s] \t wait for lidar", uav_name.data());
    // ----------------------------------------------------------------------------------
    // 起飞
    double takeoff_height  = 8.0, hovering_height = 5.0;
    nh_local.getParam("takeoff_height", takeoff_height);
    nh_local.getParam("hovering_height", hovering_height);
    std::cout<< "[" << uav_name << "] " << "takeoff height: " << takeoff_height << " hovering height: " << hovering_height << std::endl;
    bool is_takeoff=false;
    double take_off_yaw = 0;
    double take_off_yaw_correct;
    double take_off_speed = 1.5;
    //chen
    //start_time
    ros::Time start_time=ros::Time::now();
    ros::Time wait_t=ros::Time::now();
    while(wait_t.toSec()-start_time.toSec()<5){
        wait_t=ros::Time::now();
        ros::spinOnce();
        ros::Rate(10).sleep();
    }
    
    //chen
    ROS_INFO("[%s] \t prepare to take off", uav_name.data());
    start_time=ros::Time::now();
    while (!is_takeoff) {
        // try{
        //     std::thread t_marker(std::bind(&Inspector::publish_marker,&inspector,marker_list,marker_pub));
        //     t_marker.detach();
        // }
        // catch(...){
        //     std::cout<<"marker fail111";
        // }
        double delta_t = (ros::Time::now() - start_time).toSec();
        // if ((inspector.grid_map.get_occupancy_pos(Eigen::Vector3d(inspector.pos[0],inspector.pos[1], inspector.pos[2] + 0.8))) || (inspector.grid_map.get_flat_pos(Eigen::Vector3d(inspector.pos[0],inspector.pos[1], inspector.pos[2] + 0.8)))) {
        
        if (inspector.grid_map.get_face_pos(Eigen::Vector3d(inspector.pos[0],inspector.pos[1], inspector.pos[2] + 0.8))) {
            ROS_WARN( KMAG "[%s] \t Obstacle over head while taking off! @ %.3f" RESET, uav_name.data(),  inspector.pos[2]);
            if ( inspector.pos[2] < 1.5){

            }else{
                is_takeoff = true;

            }
            // gimbal_control_puber.pub_angle_control(0, 0.);
        }
        if (inspector.pos[2]>takeoff_height){
            is_takeoff = true;
            // gimbal_control_puber.pub_angle_control(0, 0.);            
        }
        // take_off_yaw=360.0*(delta_t/(takeoff_height/take_off_speed));
        control_point_puber.pub_vel(0, 0, take_off_speed, take_off_yaw);                           
        // gimbal_control_puber.pub_angle_control(-M_PI_2 + M_PI*(delta_t/(takeoff_height/take_off_speed)), 0.);                            // Added 我是真的服了, 在地上转云台会把飞机卡进土里

        ros::spinOnce();
        ros::Rate(10).sleep(); 

    }
    inspector.is_takeoff=true;
    ROS_INFO("[%s] \t take off completed!", uav_name.data());


    Eigen::Vector3d points_center;
    double uav_yaw=0.;
    ros::Time uav_yaw_t=ros::Time::now();
    const int pub_map_wait = 100; 
    int epoch = 0;
    double desired_yaw = 0;
    double desired_pitch;
    int gimbal_pub_wait=3;
    int count_gimbal_wait;

    // double desired_pitch_dir = -1;
    // int tt = 0;
    // while (ros::ok()){
    //     try{
    //         std::thread t_marker(std::bind(&Inspector::publish_marker,&inspector,marker_list,marker_pub));
    //         t_marker.detach();
    //     }
    //     catch(...){
    //         std::cout<<"marker fail";
    //     }
    //     // 起飞后测试insight]
    //     // ROS_INFO("uav ori R: %6.2f, P:%6.2f, Yaw: %6.2f, Gimbal Pitch %6.2f, Yaw %6.2f", inspector.ori[2], inspector.ori[1], inspector.ori[0], inspector.gimbal_ori[1], inspector.gimbal_ori[2]);
    //     uav_yaw += 0.095;

    //     // desired_pitch += 0.05 * desired_pitch_dir;
    //     // desired_pitch_dir = desired_pitch > 1.55 ? -1 : desired_pitch_dir;
    //     // desired_pitch_dir = desired_pitch < -1.55 ? 1 : desired_pitch_dir;
    //     if(tt++ % 100 == 0){
    //         desired_pitch_dir = -desired_pitch_dir;
    //     }
    //     desired_pitch = 1.57 *desired_pitch_dir ;
    //     control_point_puber.pub_point(inspector.pos[0], inspector.pos[1], inspector.pos[2], uav_yaw,0,0);     // Added
    //     gimbal_control_puber.pub_angle_control(desired_pitch,0);
    //     ros::spinOnce();
    //     ros::Rate(10).sleep();
    // }
    if(uav_name=="raffles") ros::Rate(0.2).sleep();
    ros::spinOnce();
    while (ros::ok()) {
        try{
            rollbackNode node;
            node.pos = Eigen::Vector3d(inspector.pos[0],inspector.pos[1],inspector.pos[2]);
            node.vist_time = ros::Time::now();
            posRollBackStack.push(node);
        }catch(...){
            std::cout<<"rollback add fail";

        }
        // try{
        //     std::thread t_marker(std::bind(&Inspector::publish_marker,&inspector,marker_list,marker_pub));
        //     t_marker.detach();
        // }
        // catch(...){
        //     std::cout<<"marker fail";
        // }
        
        if(!has_roll_back && mission_duration_remained < 12. && mission_duration_remained <( ros::Time::now() - last_gcs_connected_time).toSec() && mission_duration_remained >0){
            ROS_WARN("[%s]: NO MORE TIME --------%.2lf----------", uav_name.c_str(), mission_duration_remained);
            rollback2gcs( (ros::Time::now() - last_gcs_connected_time).toSec(), control_point_puber);
        }

    
        try {
            //  publish control command
            if(inspector.is_new_tarjectory_generate && inspector.is_last_trajectory_finish){
            // if (inspector.new_traj_gened){
                inspector.curr_seg=0;
                inspector.is_start=true;
                inspector.is_new_tarjectory_generate=false;
                // inspector.new_traj_gened = false;
                inspector.is_last_trajectory_finish=false;
                inspector.start_t=ros::Time::now();

                //std::cout << "[" + uav_name + "]" << "main thread, code: 1" << std::endl;
            }
        }
        catch (...) {
            std::cout << "[" + uav_name + "]" << "main thread error, code: 1" << std::endl;
        }

        // 补偿
        if(inspector.is_init && inspector.is_last_trajectory_finish && !inspector.is_new_tarjectory_generate && !inspector.is_collision_detect){
            Eigen::Vector3d err_vel;
            try {
                for(int j=0;j<3;j++){
                    err_vel[j] = 0.5 * (inspector.target_point.pos[j]-inspector.pos[j]);
                } 
                // gjr add 补偿时yaw遵循之前的 
                control_point_puber.pub_vel(err_vel[0],err_vel[1],err_vel[2], uav_yaw);
                // ROS_INFO("[%s] err_vel:%f %f %f","jurong",err_vel[0],err_vel[1],err_vel[2]);
                // inspector.co(ntrol_point_puber.pub_point(inspector.target_point.pos,0); 

                //std::cout << "[" + uav_name + "]" << "main thread, code: 2" << std::endl;
            }
            catch (...) {
                std::cout << "[" + uav_name + "]" << "main thread error, code: 2" << std::endl;
            }
        }
        if(inspector.is_start) {
            try {
                // 每段轨迹完成 (即每两个路点之间的轨迹)
                // if(inspector.new_traj_gened){
                //     // 开始新的路线
                //     // new_traj_gened = false;

                // }
                if(((ros::Time::now()-inspector.start_t).toSec())>inspector._polyTime(inspector.curr_seg)){

                    inspector.start_t+=ros::Duration(inspector._polyTime(inspector.curr_seg));
                    inspector.curr_seg++;

                    if(inspector.curr_seg>=inspector._polyTime.size()){
                        inspector.is_start=false;
                        if(!inspector.is_last_trajectory_finish) ROS_INFO("[%s] curr traj finished!",inspector.name.c_str());  // 只输出一次
                        
                        inspector.is_last_trajectory_finish = true;

                        // Added
                        double target_hovering_pos = inspector.pos[2] >= hovering_height ? inspector.pos[2] : hovering_height;
                        //control_point_puber.pub_point(inspector.pos[0],inspector.pos[1],target_hovering_pos, uav_yaw, 0, 0);       // Added
                        control_point_puber.pub_point(inspector.pos[0], inspector.pos[1], inspector.pos[2],
                                                      inspector.pos[0], inspector.pos[1], target_hovering_pos, uav_yaw, 0, 0);

                        continue;
                    }
                }
                // 实时避障 判断之后几段的Trajectory点附近是否安全
                for(int i=inspector.curr_seg;i<inspector._polyTime.size() && !inspector.is_collision_detect;i++){
                    for(double t=0;t<=inspector._polyTime[i] && !inspector.is_collision_detect;t+=0.1){
                        Eigen::Vector3d tmp = getPosPoly(inspector._polyCoeff,i,t);
                        for (int i = -0.5; i <= 0.5 && !inspector.is_collision_detect; i++) {
                            for (int j = -0.5; j <= 0.5 && !inspector.is_collision_detect; j++) {
                                for (int k = -0.5; k <= 0.5 && !inspector.is_collision_detect; k++) {
                                    Eigen::Vector3d pos = tmp + Eigen::Vector3d(i, j, k);
                                    if ((inspector.grid_map.get_flat_pos(pos) || inspector.grid_map.get_occupancy_pos(pos)) && pos[2]>2) {                 
                                        inspector.is_collision_detect = true;
                                        inspector.is_start = false;
                                        inspector.is_last_trajectory_finish = true;
                                    }
                                }
                            }
                        }
                        // bool tmp1 = inspector.grid_map.get_occupancy_pos(tmp) || inspector.grid_map.get_flat_pos(tmp);
                        // if(tmp1){
                        //     // geometry_msgs::Point p;
                        //     // p.x=tmp[0];
                        //     // p.y=tmp[1];
                        //     // p.z=tmp[2];
                        //     // marker_risk_waypoint.points.push_back(p);
                        //     // marker_pub.publish(marker_risk_waypoint);
                        //     inspector.is_collision_detect = true;
                        //     inspector.is_start = false;
                        //     inspector.is_last_trajectory_finish = true;
                        //     break;
                        // }
                    }
                }
                // 实时避障 判断自身附近障碍物
                // for (int i = -1; i <= 1 && !inspector.is_collision_detect; i++) {
                //     for (int j = -1; j <= 1 && !inspector.is_collision_detect; j++) {
                //         for (int k = -1; k <= 1 && !inspector.is_collision_detect; k++) {
                //             Eigen::Vector3d pos = inspector.pos + Eigen::Vector3d(i, j, k);
                //             if ((inspector.grid_map.get_flat_pos(pos) || inspector.grid_map.get_occupancy_pos(pos)) && pos[2]>2) {                 
                //                 inspector.is_collision_detect = true;
                //                 inspector.is_start = false;
                //                 inspector.is_last_trajectory_finish = true;
                //                 break;
                //             }
                //         }
                //     }
                // }
                // if ((inspector.grid_map.get_flat_pos(inspector.pos) || inspector.grid_map.get_occupancy_pos(inspector.pos)) && inspector.pos[2]>2) {                 
                //     inspector.is_collision_detect = true;
                //     inspector.is_start = false;
                //     inspector.is_last_trajectory_finish = true;
                // }
                if(inspector.is_collision_detect){
                    control_point_puber.pub_point(inspector.pos[0], inspector.pos[1], inspector.pos[2], uav_yaw,0,0);  
                    ROS_INFO("[%s] Hold",inspector.name.c_str());
                    continue;                     
                }
                // std::cout<<"timuav_namee past: "<<(ros::Time::now()-ju_start_t).toSec()<<std::endl;

                //std::cout << "[" + uav_name + "]" << "main thread, code: 3" << std::endl;
            }
            catch (...) {
                std::cout << "[" + uav_name + "]" << "main thread error, code: 3" << std::endl;
                return -1;
            }

            try {         
                Eigen::Vector3d photo_point =  inspector.find_inspect_points_center();
                // 朝向带检测点的质心方向，如果没有检测点则朝向上一次的质心方向
                if(photo_point!=Eigen::Vector3d(-1,-1,-1)){
                    desired_yaw=inspector.cal_desired_yaw(photo_point);
                    desired_pitch=inspector.cal_desired_pitch(photo_point);
                    // 设定云台角度
                    gimbal_control_puber.pub_angle_control(desired_pitch,0);
                    double delta_t = (ros::Time::now()-uav_yaw_t).toSec();
                    double yaw_rate = (desired_yaw-uav_yaw)/delta_t;
                    double max_rate=M_PI/4;
                    if(yaw_rate>max_rate) yaw_rate = max_rate;
                    else if(yaw_rate<-max_rate) yaw_rate=-max_rate;
                    // 飞机偏航慢慢转
                    uav_yaw += delta_t*yaw_rate;
                }
                else{
                    // uav_yaw 保持就行 不应等于desired_yaw
                    // uav_yaw = desired_yaw;
                }
                
                // ROS_INFO("[%s] uav_yaw:%f",uav_name.c_str(),desired_yaw*180/M_PI);

                //std::cout << "[" + uav_name + "]" << "main thread, code: 4" << std::endl;
                Eigen::Vector3d control_value = getPosPoly(inspector._polyCoeff,inspector.curr_seg,(ros::Time::now()-inspector.start_t).toSec());

                // geometry_msgs::PoseStamped uav_desired_ori;
                // uav_desired_ori.header.frame_id="world";
                // uav_desired_ori.pose.position.x=inspector.pos[0];
                // uav_desired_ori.pose.position.y=inspector.pos[1];
                // uav_desired_ori.pose.position.z=inspector.pos[2];
                // tf::Quaternion uav_ori_q;
                // uav_ori_q.setRPY(0,0,uav_yaw);
                // uav_desired_ori.pose.orientation.w=uav_ori_q.w();
                // uav_desired_ori.pose.orientation.x=uav_ori_q.x();
                // uav_desired_ori.pose.orientation.y=uav_ori_q.y();
                // uav_desired_ori.pose.orientation.z=uav_ori_q.z();
                // uav_ori_pub.publish(uav_desired_ori);

                // geometry_msgs::PoseStamped gimbal_desired_ori;
                // gimbal_desired_ori.header.frame_id="world";
                // gimbal_desired_ori.pose.position.x=inspector.pos[0];
                // gimbal_desired_ori.pose.position.y=inspector.pos[1];
                // gimbal_desired_ori.pose.position.z=inspector.pos[2];
                // uav_ori_q.setRPY(0,desired_pitch,desired_yaw);
                // gimbal_desired_ori.pose.orientation.w=uav_ori_q.w();
                // gimbal_desired_ori.pose.orientation.x=uav_ori_q.x();
                // gimbal_desired_ori.pose.orientation.y=uav_ori_q.y();
                // gimbal_desired_ori.pose.orientation.z=uav_ori_q.z();
                // gimbal_ori_pub.publish(gimbal_desired_ori);

                
                uav_yaw_t=ros::Time::now();
                // if (inspector.grid_map.get_occupancy_pos(Eigen::Vector3d(control_value[0],control_value[1],control_value[2])) || inspector.grid_map.get_flat_pos(Eigen::Vector3d(control_value[0],control_value[1],control_value[2]))) {
                //     ROS_WARN("[%s] \t Obstacle ahead! @ %.3f", uav_name.data(),  inspector.pos[2]);
                //     // 在附近找一个安全的点去
                //     bool find_new = false;
                //     for (int i = -1; i <= 1 && !find_new; i++) {
                //         for (int j = -1; j <= 1 && !find_new; j++) {
                //             for (int k = -1; k <= 1 && !find_new; k++) {
                //                 Eigen::Vector3d pos = Eigen::Vector3d(control_value[0],control_value[1],control_value[2]) + Eigen::Vector3d(i, j, k);
                //                 if (!inspector.grid_map.get_occupancy_pos(pos) && !inspector.grid_map.get_flat_pos(pos)) {
                //                     control_value = pos;
                //                     find_new = true;
                //                     break;
                //                 }
                //             }
                //         }
                //     }  
                // }
                control_point_puber.pub_point(inspector.pos[0], inspector.pos[1], inspector.pos[2], control_value[0],control_value[1],control_value[2],uav_yaw,0,0);     // Added
                

            }
            catch (...) {
                std::cout << "[" + uav_name + "]" << "main thread error, code: 4 or 5" << std::endl;

            }
        }
        
        try {
            // Added
            if(epoch++%pub_map_wait==0){
                std::thread t_map_pub(map_pub_thread_func);
                t_map_pub.detach();
                // std::thread t_inbox_map_pub(inbox_map_pub_thread_func);
                // t_inbox_map_pub.detach();
            }
            caric_competition_xmu::search_points_array_pub.publish(caric_competition_xmu::search_point_array3_2msg(inspector.search_points_array));
            // caric_competition_xmu::frontier_points_pub.publish(caric_competition_xmu::search_point_array1_2msg(inspector.frontier_points));
            
            // std::cout << "[" + uav_name + "]" << "main thread, code: 6" << std::endl;
        }
        catch (...) {
            std::cout << "[" + uav_name + "]" << "main thread error, code: 6" << std::endl;
        }
        // pub frontier points
        // auto re = caric_competition_xmu::search_point_array1_2msg(inspector_pointer->frontier_points);
        // caric_competition_xmu::frontier_points_pub.publish(re);

        // ROS_WARN("[%s] : %d",uav_name.c_str(),inspector_pointer->frontier_points.size());
        // added zbw
        if((ros::Time::now() - last_gcs_connected_time).toSec() > 0.2){
            is_gcs_connected = false;
        }
        ros::spinOnce();
        // rate太低可能会轨迹跟不上 
        ros::Rate(50).sleep();          // Added 50 -> 15
    }

    return 0;

}
