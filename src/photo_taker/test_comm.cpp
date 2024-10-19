#include "ros/ros.h"
#include "std_msgs/String.h"
#include "caric_mission/CreatePPComTopicRequest.h"
// #include "reatePPComTopicRequest.h"
// #include "CreatePPComTopic.h"
#include "octomap_msgs/Octomap.h"
// #include "../../caric_mission/srv/CreatePPComTopic.srv"
#include "caric_mission/CreatePPComTopic.h"
#include <octomap/octomap.h>
#include "caric_competition_xmu/GridNodeMsg.h"
#include "caric_competition_xmu/GridMapMsg.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <chrono>
// #include "inspector_plus.h"
// void
/*
    #include "CreatePPComTopic.h"
    ros::NodeHandle nh;

    caric_mission::CreatePPComTopic srv;
    srv.request.source = "nanyang";
    srv.request.targets = {"all"};
    srv.request.topic_name = "/Octomap";
    srv.request.package_name = "octomap_msgs";
    srv.request.message_type = "Octomap";

    Comunicater c(nh, srv);
    c.msg_pub.publish(msg);
*/
// 定义 grid_node 类，包括位置、索引、是否被占据、是否被检测过
class GridNode {
public:
    // 构造函数
    GridNode(double x, double y, double z, int idx) : pos(x, y, z){}
    // new
    GridNode(){
        GridNode(0,0,0,0);
    }

    Eigen::Vector3d pos; // 中心点的位置         // 根据地图原点和改点的位置确定的索引
    bool is_occupied{false};   // 是否被占据
    bool is_inspected{false};  // 是否被检测过
    bool need_to_inspect{false}; // 是否需要被检测
};

// 定义 search_point 类，包括位置、是否被visit、是否为frontier ponit
class SearchPoint {
public:
    // 构造函数
    SearchPoint(){};
    SearchPoint(Eigen::Vector3d pos, int x, int y, int z, bool is_obstacle):pos(pos),x(x),y(y),z(z),is_obstacle(is_obstacle){}

    Eigen::Vector3d pos; 
    // idx in saerch array
    int x,y,z;
    bool is_obstacle;   
    bool is_visited{false}; 
    bool is_in_frontier{false}; 
    double heuristic;
};


// 定义 grid_map 类，包括地图原点、分辨率、最大索引、栅格点信息等
class GridMap {
public:
    // 构造函数
    GridMap() {}
    GridMap(double origin_x, double origin_y, double origin_z, double x_width, double y_width, double z_width, double res) 
        : map_origin(origin_x, origin_y, origin_z), resolution(res) {
            x_min = origin_x - x_width / 2;
            x_max = origin_x + x_width / 2;
            y_min = origin_y - y_width / 2;
            y_max = origin_y + y_width / 2;
            z_min = origin_z - z_width / 2;
            z_max = origin_z + z_width / 2;
            max_index <<(int)(x_max-x_min)/res,(int)(y_max-y_min)/res,(int)(z_max-z_min)/res;//这种取整是四舍五入
            //先x，后y， 再z
            for (int i = 0; i < max_index[0]; i++) {
                std::vector<std::vector<GridNode>> grid_map_i;
                for (int j = 0; j < max_index[1]; j++) {
                    std::vector<GridNode> grid_map_ij;
                    for (int k = 0; k < max_index[2]; k++) {
                        grid_map_ij.push_back(GridNode(x_min + i * res, y_min + j * res, z_min + k * res, i * max_index[1] * max_index[2] + j * max_index[2] + k));
                    }
                    grid_map_i.push_back(grid_map_ij);
                }
                grid_map.push_back(grid_map_i);
            }
        }

    Eigen::Vector3d map_origin; // 原点
    double x_min, x_max, y_min, y_max, z_min, z_max; 
    double resolution;          // 分辨率 (m/格)
    Eigen::Vector3i max_index;   // 最大的索引
    //grid是一个三维的vector，每个元素是一个grid_node
    std::vector<std::vector<std::vector<GridNode>>> grid_map;

    // 将坐标转换为索引
    bool pos2index(const Eigen::Vector3d& position,Eigen::Vector3i& idx) {
        bool flag=true;
        double x,y,z;
        x=position[0];
        y=position[1];
        z=position[2];
        int id_x=(int)(x-x_min)/resolution;
        int id_y=(int)(y-y_min)/resolution;
        int id_z=(int)(z-z_min)/resolution;

        idx<<id_x,id_y,id_z;
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            // std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        if(idx[0]<0||idx[1]<0||idx[2]<0)
        {
            // std::cout<<"error:the position is out of the map"<<std::endl;
            // idx<<-1,-1,-1;
            flag=false;
        }
        return flag;
    }

    // 将索引转换为坐标
    Eigen::Vector3d index2pos(const Eigen::Vector3i& idx) {
        double x,y,z;
        x=x_min+idx[0]*resolution;
        y=y_min+idx[1]*resolution;
        z=z_min+idx[2]*resolution;
        Eigen::Vector3d pos(x,y,z);
        return pos;
    }

    // 判断栅格是否被占据
    bool get_occupancy_pos(const Eigen::Vector3d& position) {
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(idx[0]<=-1||idx[1]<=-1||idx[2]<=-1)//如果索引不合法
        {
            std::cout<<"error:index smaller tham 0"<<std::endl;
            return false;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            std::cout<<"error:the position is out of the map, code: 1"<<std::endl;
            return false;
        }
        if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool get_occupancy_idx(const Eigen::Vector3i idx) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]==-1||idx[1]==-1||idx[2]==-1)//如果索引不合法,false(默认空)
        {
            std::cout<<"error:the position is out of the map, code: 2"<<std::endl;
            return false;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            std::cout<<"error:the position is out of the map, code: 3"<<std::endl;
            return false;
        }
        if(grid_map[idx[0]][idx[1]][idx[2]].is_occupied==true)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    // 设置栅格占据情况
    void set_occupancy_pos(const Eigen::Vector3d& position, bool occupied) {
        // 根据位置找到相应的栅格并设置占据情况
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_occupied=occupied;
    }

    void set_occupancy_idx(const Eigen::Vector3d idx, bool occupied) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            std::cout<<"error:the position is out of the map, code: 4"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            std::cout<<"error:the position is out of the map, code: 5"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_occupied=occupied;
    }

    // 更新检测情况
    void update_state_pos(const Eigen::Vector3d& position, bool inspected) {
        // 根据位置找到相应的栅格并更新检测状态
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_inspected=inspected;
    }

    void update_state_idx(const Eigen::Vector3d idx, bool inspected) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            std::cout<<"error:the position is out of the map, code: 6"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            std::cout<<"error:the position is out of the map, code: 7"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].is_inspected=inspected;
    }

    void set_need_to_inspect_pos(const Eigen::Vector3d& position, bool need_to_inspect) {
        // 根据位置找到相应的栅格并设置是否需要被检测
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        if(!re)//如果索引不合法,false(默认空)
        {
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=need_to_inspect;
    }

    void set_need_to_inspect_idx(const Eigen::Vector3d idx, bool need_to_inspect) {
        // 根据位置找到相应的栅格并判断是否被占据
        if(idx[0]<0||idx[1]<0||idx[2]<0)//如果索引不合法,false(默认空)
        {
            std::cout<<"error:the position is out of the map, code: 8"<<std::endl;
            return ;
        }
        if(idx[0]>=max_index[0]||idx[1]>=max_index[1]||idx[2]>=max_index[2])
        {
            std::cout<<"error:the position is out of the map, code: 9"<<std::endl;
            return;
        }
        grid_map[idx[0]][idx[1]][idx[2]].need_to_inspect=need_to_inspect;
    }

    bool get_surrounding_pos(const Eigen::Vector3d& position,int range){
        Eigen::Vector3i idx;
        auto re=pos2index(position,idx);
        for(int i=std::max(idx[0]-range,0);i<std::min(max_index[0],idx[0]+range);i++){
            for(int j=std::max(idx[1]-range,0);j<std::min(max_index[1],idx[1]+range);j++){
                for(int k=std::max(idx[2]-range,0);k<std::min(max_index[2],idx[2]+range);k++){
                    if(grid_map[i][j][k].is_occupied==true)
                    {
                        return false;
                    }
                }
            }
        }
        return true; 
    }

    bool get_surrounding_idx(const Eigen::Vector3i idx,int range){
        for(int i=std::max(idx[0]-range,0);i<std::min(max_index[0],idx[0]+range);i++){
            for(int j=std::max(idx[1]-range,0);j<std::min(max_index[1],idx[1]+range);j++){
                for(int k=std::max(idx[2]-range,0);k<std::min(max_index[2],idx[2]+range);k++){
                    if(grid_map[i][j][k].is_occupied==true)
                    {
                        return false;
                    }
                }
            }
        }
    }
};

class Comunicater{
public:
    ros::Publisher msg_pub;
    Comunicater(ros::NodeHandle nh, caric_mission::CreatePPComTopic srv, int if_gen_publisher){
        ROS_INFO("Wait for service to appear");
        ros::service::waitForService("create_ppcom_topic");  // 可以在之前就wait
        ROS_INFO("Got Service");
        ros::ServiceClient create_ppcom_topic = nh.serviceClient<caric_mission::CreatePPComTopic>("create_ppcom_topic");

        create_ppcom_topic.call(srv);
        
        

        std::cout << "response(talker): " << srv.response.result << std::endl;
        // Publisher 可以单独创建
        if(if_gen_publisher){
            ROS_INFO("auto pub");

            if(srv.request.message_type == "Octomap"){
                msg_pub = nh.advertise<octomap_msgs::Octomap>(srv.request.topic_name,1);
            }else if (srv.request.message_type == "GridNode")
            {
                // 注意包名！！
                msg_pub = nh.advertise<caric_competition_xmu::GridNodeMsg>(srv.request.topic_name,1);
            }
        }
        
        
        
        msg_pub = nh.advertise<octomap_msgs::Octomap>("/Octomap",1);

        std::cout << "publisher generated" << std::endl;
    }

    Comunicater(ros::NodeHandle nh, caric_mission::CreatePPComTopic srv){
        Comunicater(nh, srv, 0);
    }

};


int merge_OCtree(octomap::OcTree old_tree, octomap::OcTree new_tree){
    return 0;
}

int merge_position(){
    return 0;
}

int merge_interest_points(){
    return 0;
}

int merge_grid_map(caric_competition_xmu::GridMapMsg old_map, caric_competition_xmu::GridMapMsg new_map){

}

caric_competition_xmu::GridMapMsg map2msg(GridMap map){
    caric_competition_xmu::GridMapMsg msg;

    msg.origin_x = map.map_origin.x();
    msg.origin_y = map.map_origin.y();
    msg.origin_z = map.map_origin.z();

    msg.x_min = map.x_min;
    msg.y_min = map.y_min;
    msg.z_min = map.z_min;
    msg.x_max = map.x_max;
    msg.y_max = map.y_max;
    msg.z_max = map.z_max;

    msg.max_index_x = map.max_index.x();
    msg.max_index_y = map.max_index.y();
    msg.max_index_z = map.max_index.z();

    msg.resolution = map.resolution;

    for(int i = 0; i < map.max_index.x(); i++){
        for(int j = 0; j < map.max_index.y(); j++){
            for(int k = 0; k < map.max_index.z(); k++){
                caric_competition_xmu::GridNodeMsg node;
                node.x = map.grid_map[i][j][k].pos.x();
                node.y = map.grid_map[i][j][k].pos.y();
                node.z = map.grid_map[i][j][k].pos.z();
                node.is_inspected = map.grid_map[i][j][k].is_inspected;
                node.is_occupied = map.grid_map[i][j][k].is_occupied;
                node.need_to_inspect = map.grid_map[i][j][k].need_to_inspect;

                msg.grid_nodes.push_back(node);
            }
        }
    }
    return msg;
}


GridMap msg2map(caric_competition_xmu::GridMapMsg msg) {
    GridMap map = GridMap();

    // Set map origin
    map.map_origin = Eigen::Vector3d(msg.origin_x, msg.origin_y, msg.origin_z);

    // Set min and max values
    map.x_min = msg.x_min;
    map.y_min = msg.y_min;
    map.z_min = msg.z_min;
    map.x_max = msg.x_max;
    map.y_max = msg.y_max;
    map.z_max = msg.z_max;

    // Set max index
    map.max_index = Eigen::Vector3i(msg.max_index_x, msg.max_index_y, msg.max_index_z);

    // Set resolution
    map.resolution = msg.resolution;

    // Resize and initialize the grid map based on max_index
    map.grid_map.resize(map.max_index.x());
    for (int i = 0; i < map.max_index.x(); i++) {
        map.grid_map[i].resize(map.max_index.y());
        for (int j = 0; j < map.max_index.y(); j++) {
            map.grid_map[i][j].resize(map.max_index.z());
        }
    }

    // Iterate over grid_nodes and fill the grid map
    for (const auto& node : msg.grid_nodes) {
        int i = static_cast<int>(round((node.x - map.x_min) / map.resolution));
        int j = static_cast<int>(round((node.y - map.y_min) / map.resolution));
        int k = static_cast<int>(round((node.z - map.z_min) / map.resolution));

        if (i >= 0 && i < map.max_index.x() &&
            j >= 0 && j < map.max_index.y() &&
            k >= 0 && k < map.max_index.z()) {
            map.grid_map[i][j][k].pos = Eigen::Vector3d(node.x, node.y, node.z);
            map.grid_map[i][j][k].is_inspected = node.is_inspected;
            map.grid_map[i][j][k].is_occupied = node.is_occupied;
            map.grid_map[i][j][k].need_to_inspect = node.need_to_inspect;
        }
    }

    return map;
}

void test_communication(ros::NodeHandle nh){
    caric_mission::CreatePPComTopic srv;
    // srv.request.source = "nanyang";
    // srv.request.targets = {"all"};
    // srv.request.topic_name = "/Octomap";
    // srv.request.package_name = "octomap_msgs";
    // srv.request.message_type = "Octomap";
    srv.request.source = "nanyang";
    srv.request.targets = {"all"};
    srv.request.topic_name = "/gridnode";
    srv.request.package_name = "caric_competition_xmu";
    srv.request.message_type = "GridNode";

    Comunicater c(nh, srv, 0);

    ros::Publisher msg_pub = nh.advertise<caric_competition_xmu::GridNodeMsg>(srv.request.topic_name,1);

    ros::Rate rate(1);

    while (ros::ok())
    {
        octomap_msgs::Octomap map;
        caric_competition_xmu::GridNodeMsg node;
        node.x=1;
        std::cout << "11" << std::endl;
        // 
        // msg_pub.publish(map);
        msg_pub.publish(node);
        ros::spinOnce();
        rate.sleep();
    }
}

void print_node(GridNode node){
    std::cout << "Node info:" << std::endl;
    std::cout << "\t" << "x\t" << "y\t" << "z\t" << "is_inspected\t" << "is_occupied\t" << "need_to_inspec\t" << std::endl; 
    std::cout << "\t" << node.pos.x() << "\t" << node.pos.y() << "\t" << node.pos.z() << "\t" <<
     node.is_inspected << "\t" << node.is_occupied << "\t" << node.need_to_inspect << std::endl;
}

double compareGridMaps(const GridMap& map1, const GridMap& map2) {
    double result = 0.0; 
    // 比较原点坐标
    if (map1.map_origin != map2.map_origin) result+=0.1;

    // 比较最小/最大值
    if (map1.x_min != map2.x_min || map1.y_min != map2.y_min || map1.z_min != map2.z_min ||
        map1.x_max != map2.x_max || map1.y_max != map2.y_max || map1.z_max != map2.z_max) {
        return result+=0.01;
    }

    // 比较分辨率
    if (map1.resolution != map2.resolution) result+=0.001;

    // 比较 max_index
    if (map1.max_index != map2.max_index) result+=0.0001;

    // 比较 grid_map 中的每个元素
    for (int i = 0; i < map1.max_index.x(); i++) {
        for (int j = 0; j < map1.max_index.y(); j++) {
            for (int k = 0; k < map1.max_index.z(); k++) {
                auto& node1 = map1.grid_map[i][j][k];
                auto& node2 = map2.grid_map[i][j][k];
                // std::cout << "node1" << std::endl;
                // print_node(node1);
                // std::cout << "node2" << std::endl;
                // print_node(node2);
                // std::string input;
                // std::getline(std::cin, input);  // 程序在这里暂停，等待用户输入

                if (node1.pos.x() != node2.pos.x() ||
                    node1.pos.y() != node2.pos.y() ||
                    node1.pos.z() != node2.pos.z() ||
                    node1.is_inspected != node2.is_inspected ||
                    node1.is_occupied != node2.is_occupied ||
                    node1.need_to_inspect != node2.need_to_inspect) {
                    result+=1.0;
                }
            }
        }
    }

    return result;
}


int test_transform(){
// 假设 GridMap 和 caric_competition_xmu::GridMapMsg 已经被定义
// 假设 map2msg 和 msg2map 函数也已经实现

    // 步骤 1: 创建一个测试 GridMap 对象
    GridMap originalMap(0,0,0,100,100,100,1);
    // ... 初始化 originalMap 的各个属性 ...

    // 步骤 2: 将 GridMap 转换为 GridMapMsg
    auto start = std::chrono::high_resolution_clock::now();
    caric_competition_xmu::GridMapMsg msg = map2msg(originalMap);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "map2msg 耗时: " << elapsed.count() << " 秒" << std::endl;

    // 步骤 3: 将 GridMapMsg 转换回 GridMap
    start = std::chrono::high_resolution_clock::now();
    GridMap convertedMap = msg2map(msg);
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;
    std::cout << "msg2map 耗时: " << elapsed.count() << " 秒" << std::endl;


    // 步骤 4: 比较原始和转换后的 GridMap 对象
    // ... 比较 originalMap 和 convertedMap 的各个属性 ...
    double result = compareGridMaps(originalMap, convertedMap);
    std::cout << "比较结果：" << result;
    return 0;
}

GridMap myMap(0,0,0,100,100,100,1);

void map_msg_cb(caric_competition_xmu::GridMapMsg msg){
    GridMap newMap = msg2map(msg);

    double result = compareGridMaps(myMap, newMap);

    std::cout << "result: " << result << std::endl;

}

void test_map_router(){

    // big: jurong raffles
    // small : changi sentosa nanyang
    
}


int main(int argc, char ** argv){
    ros::init(argc, argv, "communication");
    ros::NodeHandle nh;

    // test_transform();
    ros::Publisher pub_map_msg = nh.advertise<caric_competition_xmu::GridMapMsg>("test_map", 1);
    ros::Subscriber sub_map_msg = nh.subscribe("test_map", 1, map_msg_cb);

    ros::Rate rate(1);

    while(1){
        myMap.set_occupancy_pos(Eigen::Vector3d(2.,3.,4.), 1);
        pub_map_msg.publish(map2msg(myMap));
        std::cout << "pubed map msg" << std::endl;
        ros::spinOnce();
        rate.sleep();
        // std::string input;
        // std::cin >> input;
        // if(input == "q"){
        //     break;
        // }
        // else{
        //     continue;
        // }
    }


    



    // ros::ServiceClient client = nh.serviceClient<your_package::CreatePPComTopic>("create_ppcom_topic");
}