#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "inspector/trajectory_generator.h"

using namespace std;
using namespace Eigen;
#define T_RELAX_RATIO 1.10
double MAX_VEL = 3.85;
double MAX_ACCEL = 1.5;
int _dev_order=3, _min_order=1;
int _poly_num1D = 6; // 2*_dev_order


// ros related
ros::Subscriber _way_pts_sub;
ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;



Vector3d _startPos  = Vector3d::Zero();
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();


void trajGeneration(Eigen::MatrixXd path,Eigen::MatrixXd& _polyCoeff,Eigen::VectorXd& _polyTime)
{
    ros::Time time_start = ros::Time::now();

    // TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;

    TrajectoryGeneratorTool trajectoryGeneratorWaypoint;
    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;

    // use "trapezoidal velocity" time allocation
    _polyTime  = timeAllocation(path);

    // generate a minimum-jerk/snap piecewise monomial polynomial-based trajectory
    _polyCoeff = trajectoryGeneratorWaypoint.SolveQPClosedForm(_dev_order, path, vel, acc, _polyTime);

    for(int i=0;i<_polyCoeff.rows();i++){
        // cout<<_polyCoeff.row(i)<<std::endl;
        std::vector<double> tmp;
        for(int j=0;j<_polyCoeff.cols();j++){  
            tmp.push_back(_polyCoeff(i,j));         
            if((j+1)%(_polyCoeff.cols()/3)==0){
                for(int k=0;k<_polyCoeff.cols()/3;k++){
                    _polyCoeff(i,j-k)=tmp[k];
                }
                tmp.clear();
            }        
        }
        // cout<<_polyCoeff.row(i)<<std::endl;
    }

    ros::Time time_end = ros::Time::now();
    // ROS_INFO("Time consumed in trajectory generation is %f ms", (time_end - time_start).toSec() * 1000.0);

    // visWayPointPath(path);    // visulize path
    // visWayPointTraj( _polyCoeff, _polyTime);    // visulize trajectory
}


// 求导
Eigen::MatrixXd calculate_polycoeff_de(Eigen::MatrixXd polyCoeff){
    Eigen::MatrixXd polyCoeff_de = Eigen::MatrixXd::Zero(polyCoeff.rows(),polyCoeff.cols() - 3);
    for(int i=0;i<polyCoeff.rows();i++){
        int k=0;
        for(int j=0;j<polyCoeff.cols();j++){
            if((j+1)%(polyCoeff.cols()/3)==1) k+=1;
            else{
                polyCoeff_de(i,j-k) = polyCoeff(i,j) * (j%(polyCoeff.cols()/3));
            }
        }
    } 
    return polyCoeff_de;
}


// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "traj_node");
//     ros::NodeHandle nh("~");

//     nh.param("planning/vel", _Vel, 1.0);
//     nh.param("planning/acc", _Acc, 1.0);
//     nh.param("planning/dev_order", _dev_order, 3);  // the order of derivative, _dev_order = 3->minimum jerk, _dev_order = 4->minimum snap
//     nh.param("planning/min_order", _min_order, 3);
//     nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

//     //_poly_numID is the maximum order of polynomial
//     _poly_num1D = 2 * _dev_order;

//     _way_pts_sub     = nh.subscribe( "waypoints", 1, rcvWaypointsCallBack );

//     _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
//     _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

//     ros::Rate rate(100);
//     bool status = ros::ok();
//     while(status) 
//     {
//         ros::spinOnce();
//         status = ros::ok();
//         rate.sleep();
//     }
//     return 0;
// }


VectorXd timeAllocation(MatrixXd Path) {
    VectorXd times(Path.rows() - 1);

//#define USE_FIXED_TIME
#ifdef USE_FIXED_TIME
    times.setOnes();
#else
    const double t = MAX_VEL / MAX_ACCEL;
    const double dist_threshold_1 = MAX_ACCEL * t * t * 0.5;

    double segment_t;
    double distance_travel = 0.0;
    double distance_remain = 0.0;
    for (unsigned int i = 1; i < Path.rows(); ++i) {
        double delta_dist = (Path.row(i) - Path.row(i - 1)).norm();
        distance_remain += delta_dist;
    }

    for (unsigned int i = 1; i < Path.rows(); ++i) {
        double delta_dist = (Path.row(i) - Path.row(i - 1)).norm();
        distance_travel += delta_dist;
        distance_remain -= delta_dist;
        // if (distance_travel > dist_threshold_1){
        //     segment_t = (t * 2 + distance_remain / MAX_VEL) * T_RELAX_RATIO;
        // } else {
        //     segment_t = std::sqrt(delta_dist / MAX_ACCEL) * T_RELAX_RATIO;
        // }


        // if (delta_dist > dist_threshold_1) {
        //     segment_t = (t * 2 + (delta_dist - dist_threshold_1) / MAX_VEL) * T_RELAX_RATIO;
        // } else {
        //     segment_t = std::sqrt(delta_dist / MAX_ACCEL)  * T_RELAX_RATIO ;
        // }
        if (i == 1 || i == Path.rows() - 1) {
            // 第一段和最后一段
            if (delta_dist > dist_threshold_1){
                // 比较远，超过了最大加速度达到最大速度的时候的距离
                segment_t = (t +  (delta_dist - dist_threshold_1) / MAX_VEL) * T_RELAX_RATIO;
            }
            else {
                // 比较近
                segment_t = std::sqrt(2 * delta_dist / MAX_ACCEL) * T_RELAX_RATIO;
            }
        } else {
            // 中间的直接按照最大加速度算
            // 其实应该考虑这里能达到的最大速度，但是这里简单处理
            if (distance_travel < dist_threshold_1) {
               // 还没有走过最大加速度所能走过的距离
                segment_t = (delta_dist / MAX_VEL) * T_RELAX_RATIO * 1.25;

            }
            else{
                segment_t = (delta_dist / MAX_VEL) * T_RELAX_RATIO;
            }
        }

        times[i - 1] = segment_t;
    }
#endif

    return times;
}

// VectorXd timeAllocation( MatrixXd Path)
// { 
//     VectorXd time(Path.rows() - 1);

//     // The time allocation is many relative timelines but not one common timeline
//     for(int i = 0; i < time.rows(); i++)
//     {
//         double distance = (Path.row(i+1) - Path.row(i)).norm();    // or .lpNorm<2>()
//         double x1 = _Vel * _Vel / (2 * _Acc); 
//         double x2 = distance - 2 * x1;
//         double t1 = _Vel / _Acc;
//         double t2 = x2 / _Vel;
//         time(i) = 2 * t1 + t2;
//     }
//     // cout << time << endl;

//     return time;
// }



Vector3d getPosPoly( MatrixXd polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }


    return ret;
}

Vector3d getVelPoly( MatrixXd polyCoeff_vel, int k, double t )
{
    Vector3d ret;

     for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff_vel.row(k)).segment( dim * (_poly_num1D-1), _poly_num1D - 1 );
        VectorXd time  = VectorXd::Zero( _poly_num1D - 1 );
        
        for(int j = 0; j < _poly_num1D - 1; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        // cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

Vector3d getAccPoly( MatrixXd polyCoeff_acc, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff_acc.row(k)).segment( dim * (_poly_num1D-2), _poly_num1D - 2 );
        VectorXd time  = VectorXd::Zero( _poly_num1D - 2 );
        
        for(int j = 0; j < _poly_num1D - 2; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}