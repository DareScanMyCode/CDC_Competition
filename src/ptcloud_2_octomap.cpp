#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <octomap/octomap.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

#include <nav_msgs/Odometry.h>

//#include <pcl/point_cloud.h>
//#include <pcl_conversions.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

using namespace std;

static string uav_name;
static double map_resolution_1 = 1, map_resolution_2 = 0.5, map_resolution_3 = 0.1;

// blog.csdn.net/wang073081/article/details/106134144
// https://blog.csdn.net/LOVE1055259415/article/details/79911653
sensor_msgs::PointCloud2 filtered_points;
volatile bool is_pt_cloud_updated = false;
void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) {

    // cout << "[" << uav_name << "]" << " pt cloud updated ..." << endl;

    filtered_points.header.frame_id = msg->header.frame_id;
    filtered_points.header.stamp    = msg->header.stamp;
    filtered_points.data            = msg->data;

    filtered_points.width           = msg->width;
    filtered_points.height          = msg->height;
    filtered_points.row_step        = msg->row_step;    

    filtered_points.is_bigendian = msg->is_bigendian;
    filtered_points.is_dense     = msg->is_dense;

    filtered_points.fields       = msg->fields;
    filtered_points.point_step   = msg->point_step;

    is_pt_cloud_updated = true;
}

nav_msgs::Odometry uav_pos;
volatile bool is_uav_odom_updated = false;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {

    uav_pos.header.frame_id = msg->header.frame_id;
    uav_pos.header.stamp    = msg->header.stamp;

    uav_pos.pose.pose.position.x = msg->pose.pose.position.x;
    uav_pos.pose.pose.position.y = msg->pose.pose.position.y;
    uav_pos.pose.pose.position.z = msg->pose.pose.position.z;

    uav_pos.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    uav_pos.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    uav_pos.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    uav_pos.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    uav_pos.twist.twist.linear.x = msg->twist.twist.linear.x;
    uav_pos.twist.twist.linear.y = msg->twist.twist.linear.y;
    uav_pos.twist.twist.linear.z = msg->twist.twist.linear.z;

    uav_pos.twist.twist.angular.x = msg->twist.twist.angular.x;
    uav_pos.twist.twist.angular.y = msg->twist.twist.angular.y;
    uav_pos.twist.twist.angular.z = msg->twist.twist.angular.z;    

    is_uav_odom_updated = true;
}

octomap::OcTree octo_map_all(map_resolution_1);//创建地图


octomap_msgs::Octomap octomap_world_msg;
volatile bool is_world_octomap_updated = false;
volatile bool is_first_update_world_world_octomap = true;               // 第一次用local数据代替global数据, 防止死锁在第一步
void world_octomap_callback(const octomap_msgs::Octomap::ConstPtr &msg) {

    octomap_world_msg.header.frame_id = msg->header.frame_id;
    octomap_world_msg.header.stamp    = msg->header.stamp;

    octomap_world_msg.id     = msg->id;
    octomap_world_msg.binary = msg->binary;
    octomap_world_msg.data   = msg->data;
    octomap_world_msg.resolution = msg->resolution;
    

    is_world_octomap_updated = true;
}

void traverseOctree(const octomap::OcTree* octree, const octomap::point3d& min_xyz, const octomap::point3d& max_xyz) {
    for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min_xyz, max_xyz), end = octree->end_leafs_bbx(); it != end; ++it) {
        // 获取当前叶子节点的坐标
        octomap::point3d coordinates = it.getCoordinate();

        // 检查当前叶子节点是否被占用
        if (octree->isNodeOccupied(*it)) {
            std::cout << "Occupied Coordinate: " << coordinates << std::endl;
        } else {
            std::cout << "Free Coordinate: " << coordinates << std::endl;
        }
    }
}

void is_occupied_callback (const nav_msgs::Odometry::ConstPtr &msg){
    
    // 遍历八叉树，我不知道怎么确定一个坐标来测试，来了个死办法。而且我发现用他的迭代器遍历，坐标取值为分辨率间隔，建议判断是否占用时候，输入形如(20.5 10.5 35.5)的形式
    octomap::point3d min_xyz(-20, -20, -20);
    octomap::point3d max_xyz(40, 40, 40);

    // 遍历八叉树并检查占用状态
    traverseOctree(&octo_map_all, min_xyz, max_xyz);

    double x,y,z;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    octomap::point3d point(x, y, z);

    octomap::OcTreeNode* node = octo_map_all.search(point);

    if (node != NULL) {
        // 在这里处理获取到的数据，例如获取占用概率
        double occupancy = node->getOccupancy();
        ROS_INFO("Occupancy at (%f, %f, %f): %f", x, y, z, occupancy);
    } else {
        ROS_WARN("Coordinate not found in the octree");
    }
    
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ptcloud_2_octomap_w_tf");
    ros::NodeHandle nh;

    // TODO
    // use uav_name to dynamically subscribe messages
    nh.getParam("uav_name", uav_name);

    nh.getParam("/map_resolution_1", map_resolution_1);
    nh.getParam("/map_resolution_2", map_resolution_2);
    nh.getParam("/map_resolution_3", map_resolution_3);

    cout << "[" << uav_name << "]" << " uav iniltiazed !" << endl;
    cout << "[" << uav_name << "]" << " uav map_resolution_1: " << map_resolution_1 << endl;
    cout << "[" << uav_name << "]" << " uav map_resolution_2: " << map_resolution_2 << endl;
    cout << "[" << uav_name << "]" << " uav map_resolution_3: " << map_resolution_3 << endl;

    // http://wiki.ros.org/ROSNodeTutorialC%2B%2B
    ros::Subscriber sub_pt_cloud = nh.subscribe("cloud_inW", 1, point_cloud_callback);
    ros::Subscriber sub_uav_odom = nh.subscribe("ground_truth/odometry", 1, odom_callback);
    ros::Subscriber sub_octomap_world = nh.subscribe("/world/octomap", 1, world_octomap_callback);
    ros::Subscriber sub_inoccupied = nh.subscribe("/world/is_occupied", 1, is_occupied_callback);//给世界坐标判断是否被占据，消息类型用nav_msgs::Odometry


    ros::Publisher _octomap3D_all_pub   = nh.advertise<octomap_msgs::Octomap>("/world/octomap", 1);
    ros::Publisher _octomap3D_local_pub = nh.advertise<octomap_msgs::Octomap>("octo3Dmap_world", 1);        // /jurong/octo3Dmap_all
    //octomap_msgs::Octomap octomap_fullmsg;


    // update TF
    static tf::TransformBroadcaster br;

    ros::Rate loop_rate(1);    

    cout << "[" << uav_name << "]" << " process started ..." << endl;

    while (ros::ok()) {

        if (is_uav_odom_updated) {

            // publish TF
            // https://blog.csdn.net/weixin_45024226/article/details/125114237

            tf::Transform transform;

            transform.setOrigin(tf::Vector3(uav_pos.pose.pose.position.x, uav_pos.pose.pose.position.y, uav_pos.pose.pose.position.z));
            transform.setRotation(tf::Quaternion(uav_pos.pose.pose.orientation.x, uav_pos.pose.pose.orientation.y, uav_pos.pose.pose.orientation.z, uav_pos.pose.pose.orientation.w));
            //"robot" 是父坐标，"world"是子坐标
            //转换关系是从robot坐标系转到world坐标系
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", uav_name + "/base_link"));

            // cout << "[" << uav_name << "]" << " tf broadcasted ...!" << endl;

            // DO NOT UNCOMMENT!
            //is_uav_odom_updated = false;
        }

        if (is_pt_cloud_updated && is_uav_odom_updated) {//好像有个filter可以做时间同步

            octomap::Pointcloud octo_cloud;  
            octomap::pointCloud2ToOctomap(filtered_points, octo_cloud);

            if (filtered_points.data.size()) {

                octo_map_all.insertPointCloud(octo_cloud,   octomap::point3d(uav_pos.pose.pose.position.x, uav_pos.pose.pose.position.y, uav_pos.pose.pose.position.z));
            
                //
                octomap_msgs::Octomap octomap_fullmsg_all;
                octomap_msgs::fullMapToMsg(octo_map_all, octomap_fullmsg_all);
                octomap_fullmsg_all.header.frame_id = "world";
                octomap_fullmsg_all.header.stamp = ros::Time::now();
                _octomap3D_local_pub.publish(octomap_fullmsg_all);

                //
                if (is_first_update_world_world_octomap) {

                    _octomap3D_all_pub.publish(octomap_fullmsg_all);

                    is_first_update_world_world_octomap = false;
                }

                // cout << "[" << uav_name << "]" << " local octomap updated ...!" << endl;
            }

            if (is_world_octomap_updated) {
                
                octomap::AbstractOcTree *octomap_world_t = octomap_msgs::fullMsgToMap(octomap_world_msg);
                octomap::OcTree         *octomap_world   = new octomap::OcTree(octomap_world_msg.resolution);

                octomap_world = dynamic_cast<octomap::OcTree*>(octomap_world_t);

                octomap_world->insertPointCloud(octo_cloud, octomap::point3d(uav_pos.pose.pose.position.x, uav_pos.pose.pose.position.y, uav_pos.pose.pose.position.z));

                octomap_msgs::fullMapToMsg(*octomap_world, octomap_world_msg);
                octomap_world_msg.header.frame_id = "world";
                octomap_world_msg.header.stamp = ros::Time::now();
                _octomap3D_all_pub.publish(octomap_world_msg);

                is_world_octomap_updated = false;

                // cout << "[" << uav_name << "]" << " world octomap updated !" << endl;
            }

            is_pt_cloud_updated = false;
            is_uav_odom_updated = false;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }


}