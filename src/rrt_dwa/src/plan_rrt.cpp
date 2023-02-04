#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include "KDTree.hpp"
#include "rrt.h"
#include "dwa.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// **************** test in BARN ****************** //
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
//#include <gazebo_msgs/GetModelState.h>
// ******************************************** //

using namespace std;

using point_t = std::vector< double >;
using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< std::vector< double >, size_t >;
using pointIndexArr = typename std::vector< pointIndex >;
using pointVec = std::vector< point_t >;

ros::Subscriber poseVel_sub;
ros::Subscriber dstPoint_sub;
ros::Subscriber laser_sub; // obst added
ros::Publisher rviz_laser_pub; // ground obst tested
ros::Publisher vel_pub, path_pub, traj_pub, midpose_pub;

int got_obst;

// -------------- self pose, vel and destination --------------
int RobotId;
tf::StampedTransform base_in_odom;
// **************** test in BARN ****************** //
ros::ServiceClient state_client;
// ******************************************** //
point_t self_pose, dst_pose;
double self_w = 0;
bool is_forward = true;
int ori_sign;
double v_in_yaml, expected_v, expected_w;
double slow_down_dist, stop_dist;
double dwa_predict_time;
// -------------- plan --------------
B_RRT global_planner;
DWA   local_planner;
pointVec  obstacle;
pointVec  path, temp_path, traj;
bool find_path;
point_t mid_goal;
pair<double, vector<vector<double>>>  wAndTraj;
double rrt_avoid_dist, rrt_boundary_expand_dist;


void CB_publishCycle(const ros::TimerEvent& e);
// void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg);
// void CB_dstPoint(const geometry_msgs::Pose::ConstPtr& pose_msg);
void CB_dstPoint(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan);
void display_PathandTraj(pointVec path, ros::Publisher visual_pub);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRT_DWA_plan");
    ros::NodeHandle n;
    string laser_topic, vel_pub_topic;
    ros::param::get("RobotId", RobotId);
    ros::param::get("expected_v", v_in_yaml);
    ros::param::get("predict_time", dwa_predict_time);
    ros::param::get("avoid_dist", rrt_avoid_dist);
    ros::param::get("boundary_expand_dist", rrt_boundary_expand_dist);
    ros::param::get("slow_down_dist", slow_down_dist);
    ros::param::get("stop_dist", stop_dist);
    ros::param::get("laser_topic", laser_topic);
    // ros::param::get("vel_pub_topic", vel_pub_topic);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
    global_planner.load_param();
    local_planner.load_param();

    // **************** test in BARN ****************** //
    // state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    // ******************************************** //
    // dstPoint_sub = n.subscribe("/dst_point", 1, CB_dstPoint);  // aero
    dstPoint_sub = n.subscribe("/move_base_simple/goal", 1, CB_dstPoint);  // aero
    // laser_sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10, CB_laser);
    // laser_sub = n.subscribe("/ouster/points", 10, CB_laser);
    // laser_sub = n.subscribe("/robot_2/pointcloud", 10, CB_laser);
    laser_sub = n.subscribe(laser_topic, 10, CB_laser);
    // rviz_laser_pub = n.advertise<sensor_msgs::LaserScan>(laser_pub_topic, 10);

    // // vel_pub = n.advertise<geometry_msgs::Twist>("target_vel", 10);
    // vel_pub = n.advertise<geometry_msgs::Twist>(vel_pub_topic, 10);
    path_pub = n.advertise<nav_msgs::Path>("rrt_path", 10);
    traj_pub = n.advertise<nav_msgs::Path>("dwa_traj", 10);
    midpose_pub = n.advertise<geometry_msgs::PoseStamped>("/course_agv/mid_goal", 10);

    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        // cout << "obst_size: " << obstacle.size() << endl;
        chrono::time_point<chrono::system_clock> start_time = chrono::system_clock::now();
        /// 获取定位 ///
        
        static tf::TransformListener tf_listener;
        try{
            tf_listener.lookupTransform("robot_"+to_string(RobotId)+"/odom", "robot_"+to_string(RobotId)+"/base", ros::Time(0), base_in_odom);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s, in CB_main.", ex.what());
            continue;
        }
        geometry_msgs::Quaternion orientation;
        tf::quaternionTFToMsg(base_in_odom.getRotation(), orientation);
        double yaw_now = tf::getYaw(orientation);
        self_pose = {base_in_odom.getOrigin().x(), base_in_odom.getOrigin().y(), yaw_now};

        chrono::time_point<chrono::system_clock> end_time = chrono::system_clock::now();
        // cout << "main_cycle: " << chrono::duration_cast<chrono::milliseconds>(end_time-start_time).count() / 1000.0 << "s" << endl;

        r.sleep();
    }
}


void CB_publishCycle(const ros::TimerEvent& e)
{
    if (path.size())
        display_PathandTraj(path, path_pub);
}

// void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg)  // 目标点
// void CB_dstPoint(const geometry_msgs::Pose::ConstPtr& pose_msg)  // 目标点
void CB_dstPoint(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)  // 目标点
{
    cout << "get dst_point!" << endl;
    // double theta = tf::getYaw(odom_msg->pose.pose.orientation);
    // dst_pose = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, theta};
    double theta = tf::getYaw(pose_msg->pose.orientation);
    dst_pose = {pose_msg->pose.position.x, pose_msg->pose.position.y, theta};
    // dst_pose = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0};
    // expected_v = odom_msg->twist.twist.linear.x;
    double dstPoint_ori = atan2(dst_pose[1]-self_pose[1], dst_pose[0]-self_pose[0]) - self_pose[2]; // 夹角
    if (dstPoint_ori > M_PI)
        dstPoint_ori -= 2*M_PI;
    else if (dstPoint_ori < -M_PI)
        dstPoint_ori += 2*M_PI;
    if(abs(dstPoint_ori) > M_PI/2)
        is_forward = false;
    else is_forward = true;

    if(is_forward) ori_sign = 1.0;
    else ori_sign = -1.0;

    expected_v = ori_sign * v_in_yaml;
    local_planner.set_v(expected_v);  // DWA 线速度设为定值
    
    ///   RRT 规划全局路径   ///
    find_path = false;
    // cout << "obst_size: " << obstacle.size() << endl;
    global_planner.set_boundary(min(self_pose[0], dst_pose[0])-rrt_boundary_expand_dist,   min(self_pose[1], dst_pose[1])-rrt_boundary_expand_dist,
                                                                    max(self_pose[0], dst_pose[0])+rrt_boundary_expand_dist,   max(self_pose[1], dst_pose[1])+rrt_boundary_expand_dist);
    double len, shortest_len = 100;
    ros::param::get("avoid_dist", rrt_avoid_dist);
    while (!find_path && rrt_avoid_dist > 0.1){  // 如果找不到路径，减小避碰距离继续搜索
        rrt_avoid_dist -= 0.05;
        global_planner.set_avoid_dist(rrt_avoid_dist);
        
        for (int j=0; j<3; j++){
            temp_path = global_planner.plan(obstacle, {self_pose[0], self_pose[1]}, {dst_pose[0], dst_pose[1]});
            if (temp_path.size() == 0){
                continue;
            }
            len = 0;
            for (int k=0; k<temp_path.size()-1; k++)
                len += hypot(temp_path[k+1][0]-temp_path[k][0], temp_path[k+1][1]-temp_path[k][1]);
            if (len < shortest_len){
                shortest_len = len;
                path = temp_path;
                find_path = true;
            }
        }
    }
    
    if (!find_path){
        ROS_WARN("Path searching failed in the condition avoid_dist >= 0.1! Please assign destination point again!");
        // cout << "Path searching failed in the condition avoid_dist >= 0.1! Please assign destination point again!" << endl;
    }
    
    // display_PathandTraj(path, path_pub);
}



// **************** test in BARN ****************** //
void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan_msg)  // 激光
{
    static tf::TransformListener tf_listener;
    double obst_dist = 5.0;

    obstacle.clear();

    int scan_size = scan_msg->ranges.size();
    // cout << "angle_min: " << scan_msg->angle_min << endl;
    // cout << "angle_max: " << scan_msg->angle_max << endl;
    // cout << "angle_increment: " << scan_msg->angle_increment << endl;
    // cout << "range_min: " << scan_msg->range_min << endl;
    // cout << "range_max: " << scan_msg->range_max << endl;

    int last_i;
    for(int i = 0; i < scan_size; i++)  // for one point
    // for(int i = 0; i < scan_size; i+=5)  // for one point
    {
        double range = scan_msg->ranges[i];

        // cout << "range " << i << ": " << range << ", ";

        if( isinf(range) || isnan(range) )
        {
            continue;
        }
        else if( range < obst_dist )  // in the inflation range
        {
            // only consider obst in front of car
            // if(scan_msg->angle_min + i*scan_msg->angle_increment < -M_PI/2  ||  scan_msg->angle_min + i*scan_msg->angle_increment > M_PI/2)
            //     continue;
            
            double angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
            double x = range * cos(angle);
            double y = range * sin(angle);

            geometry_msgs::PointStamped obst_in_laser;
            geometry_msgs::PointStamped obst_in_world;

            obst_in_laser.header.stamp = ros::Time();
            // obst_in_laser.header.frame_id = scan_msg->header.frame_id;  // laser
            // obst_in_laser.header.frame_id = "/base_link";  // laser
            obst_in_laser.header.frame_id = "robot_"+to_string(RobotId)+"/base";  // laser
            obst_in_laser.point.x = x;
            obst_in_laser.point.y = y;
            obst_in_laser.point.z = 0;
            try{
                // tf_listener.transformPoint("/map", obst_in_laser, obst_in_world);
                tf_listener.transformPoint("robot_"+to_string(RobotId)+"/odom", obst_in_laser, obst_in_world);
            }
            catch (tf::TransformException &ex){
                ROS_ERROR("%s, in CB_laser.", ex.what());
                return;
            }

            obstacle.push_back({obst_in_world.point.x, obst_in_world.point.y});
        }
        else
        {
            continue;
        }
    }

    // got_obst = 1;
}

// void CB_laser(const sensor_msgs::PointCloud2::ConstPtr& scan_msg)  // 激光
// {
//     static tf::TransformListener tf_listener;
//     // vector<vector<double>> scan_points;

//     pcl::PointCloud<pcl::PointXYZ> temp_pcl;
//     pcl::fromROSMsg(*scan_msg, temp_pcl);
//     double obst_dist = 5.0, blind = 0.01;
//     int point_filter_num = 5;

//     obstacle.clear();

//     for (int i = 0; i < temp_pcl.points.size(); i++)
//     {
//         if (i % point_filter_num != 0) continue;
//         // cout << "obst size: " << temp_pcl.points.size() << endl;
//         // cout << "obst x,y,z: " << temp_pcl.points[i].x << ", " << temp_pcl.points[i].y << ", " << temp_pcl.points[i].z << endl;

//         double range = temp_pcl.points[i].x * temp_pcl.points[i].x + temp_pcl.points[i].y * temp_pcl.points[i].y + temp_pcl.points[i].z * temp_pcl.points[i].z;
        
//         // only consider range < 5.0m
//         if (range < (blind * blind) || range > (obst_dist * obst_dist) || temp_pcl.points[i].z < -0.4 || temp_pcl.points[i].z > 0.1)
//             continue;
        
//         // PointType added_pt;
//         // added_pt.x = temp_pcl.points[i].x;
//         // added_pt.y = temp_pcl.points[i].y;
//         // added_pt.z = temp_pcl.points[i].z;
//         // // added_pt.intensity = temp_pcl.points[i].intensity;
//         // // added_pt.normal_x = 0;
//         // // added_pt.normal_y = 0;
//         // // added_pt.normal_z = 0;
//         // // added_pt.curvature = temp_pcl.points[i].t * time_unit_scale; // curvature unit: ms

//         geometry_msgs::PointStamped temp_obst_xyz_in_laser;
//         geometry_msgs::PointStamped temp_obstPoint_in_odom;

//         temp_obst_xyz_in_laser.header.stamp = ros::Time();
//         // temp_obst_xyz_in_laser.header.frame_id = scan_msg->header.frame_id;  // laser
//         temp_obst_xyz_in_laser.header.frame_id = "robot_"+to_string(RobotId)+"/base";  // laser
//         // cout << "scan_frame: " << scan_msg->header.frame_id << endl;
//         temp_obst_xyz_in_laser.point.x = temp_pcl.points[i].x;
//         temp_obst_xyz_in_laser.point.y = temp_pcl.points[i].y;
//         temp_obst_xyz_in_laser.point.z = temp_pcl.points[i].z;
//         try{
//             tf_listener.transformPoint("robot_"+to_string(RobotId)+"/odom", temp_obst_xyz_in_laser, temp_obstPoint_in_odom);
//         }
//         catch (tf::TransformException &ex){
//             ROS_ERROR("%s, in CB_laser.", ex.what());
//             return;
//         }

//         // scan_points.push_back(p);  // all points, ( (x0,y0), (x1,y1), ... )
//         obstacle.push_back({temp_obstPoint_in_odom.point.x, temp_obstPoint_in_odom.point.y});
//     }

//     got_obst = 1;
// }


void display_PathandTraj(pointVec path, ros::Publisher visual_pub){  // 在 Rviz 中显示 RRT 与 DWA 路径
    nav_msgs::Path  visual_path;
    visual_path.header.stamp = ros::Time(0);
    // **************** test in BARN ****************** //
    visual_path.header.frame_id = "robot_"+to_string(RobotId)+"/odom";
    // visual_path.header.frame_id = "/map";
    for (int i = 0; i < path.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(0);
        pose.header.frame_id = "robot_"+to_string(RobotId)+"/odom";
        // pose.header.frame_id = "/map";
        pose.pose.position.x = path[i][0];
        pose.pose.position.y = path[i][1];
        pose.pose.position.z = 0.01;
        pose.pose.orientation.w = 1;
        visual_path.poses.push_back(pose);
    }
    visual_pub.publish(visual_path);
}

