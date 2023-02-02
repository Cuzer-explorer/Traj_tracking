#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "KDTree.hpp"
#include "rrt.h"
#include "dwa.h"

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

using namespace std;

using point_t = std::vector< double >;
using indexArr = std::vector< size_t >;
using pointIndex = typename std::pair< std::vector< double >, size_t >;
using pointIndexArr = typename std::vector< pointIndex >;
using pointVec = std::vector< point_t >;

ros::Subscriber poseVel_sub;
ros::Subscriber dstPoint_sub;
ros::Subscriber laser_sub; // obst added
ros::Publisher vel_pub, path_pub, traj_pub;

int got_obst;

// -------------- self pose and vel --------------
int RobotId;
tf::StampedTransform base_in_odom;
point_t self_pose;
double self_w = 0;
// -------------- destination --------------
point_t dst_pose;
double expected_v;
// -------------- plan --------------
B_RRT global_planner;
DWA   local_planner;
pointVec  obstacle;
pointVec  path, temp_path, traj;
pair<double, vector<vector<double>>>  wAndTraj;


void CB_publishCycle(const ros::TimerEvent& e);
void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg);
void CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);
void CB_laser(const sensor_msgs::PointCloud2::ConstPtr &scan_msg);
void display_PathandTraj(pointVec path, ros::Publisher visual_pub);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "RRT_DWA_plan");
    ros::NodeHandle n;
    string laser_topic, vel_pub_topic;
    ros::param::get("RobotId", RobotId);
    ros::param::get("laser_topic", laser_topic);
    ros::param::get("vel_pub_topic", vel_pub_topic);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.5), CB_publishCycle);
    global_planner.load_param();
    local_planner.load_param();

    poseVel_sub = n.subscribe("/velocity_info", 1, CB_update_vel);
    dstPoint_sub = n.subscribe("/dst_point", 1, CB_dstPoint);  // aero
    // laser_sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10, CB_laser);
    // laser_sub = n.subscribe("/ouster/points", 10, CB_laser);
    // laser_sub = n.subscribe("/robot_2/pointcloud", 10, CB_laser);
    laser_sub = n.subscribe(laser_topic, 10, CB_laser);

    // vel_pub = n.advertise<geometry_msgs::Twist>("target_vel", 10);
    vel_pub = n.advertise<geometry_msgs::Twist>(vel_pub_topic, 10);
    path_pub = n.advertise<nav_msgs::Path>("rrt_path", 10);
    traj_pub = n.advertise<nav_msgs::Path>("dwa_traj", 10);

    // ------------------------ debug ------------------------ //
    // self_pose = {0,0,0};
    // dst_pose = {5,5,0};
    self_pose = {0,0,0};
    dst_pose = {0,0,0};
    ///   RRT 规划全局路径   ///
    // global_planner.set_boundary(0, 0, 10, 10);
    // double len, shortest_len = 100;
    // for (int j=0; j<3; j++){
    //     temp_path = global_planner.plan(obstacle, {self_pose[0], self_pose[1]}, {dst_pose[0], dst_pose[1]});
    //     len = 0;
    //     for (int k=0; k<temp_path.size()-1; k++)
    //         len += hypot(temp_path[k+1][0]-temp_path[k][0], temp_path[k+1][1]-temp_path[k][1]);
    //     if (len < shortest_len){
    //         shortest_len = len;
    //         path = temp_path;
    //     }
    // }
    // display_PathandTraj(path, path_pub);
    local_planner.set_v(1.0);  // DWA 线速度设为定值
    // ------------------------------------------------------- //

    int count=0;
    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        cout << "obst_size: " << obstacle.size() << endl;
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
        
        // if(count%25==0){
        //     // global_planner.set_boundary(0, 0, 10, 10);
        //     global_planner.set_boundary(min(self_pose[0], dst_pose[0])-2, min(self_pose[1], dst_pose[1])-2,
        //                                 max(self_pose[0], dst_pose[0])+2, max(self_pose[1], dst_pose[1])+2);
        //     double len, shortest_len = 100;
        //     for (int j=0; j<3; j++){
        //         temp_path = global_planner.plan(obstacle, {self_pose[0], self_pose[1]}, {dst_pose[0], dst_pose[1]});
        //         if (temp_path.size() == 0){
        //             break;
        //         }
        //         len = 0;
        //         for (int k=0; k<temp_path.size()-1; k++)
        //             len += hypot(temp_path[k+1][0]-temp_path[k][0], temp_path[k+1][1]-temp_path[k][1]);
        //         if (len < shortest_len){
        //             shortest_len = len;
        //             path = temp_path;
        //         }
        //     }
        // }
        
        if (path.size() == 0){
            continue;
        }
        ///  DWA 选取最优角速度  ///
        // -----------------------------------------------------
        double dstPoint_dist = hypot(dst_pose[0]-self_pose[0], dst_pose[1]-self_pose[1]); // 距离
        if (dstPoint_dist < 1.0 && dstPoint_dist > 0.2){  // 减速
            expected_v = 1.0 * dstPoint_dist;
            local_planner.set_v(expected_v);
        }
        else if (dstPoint_dist <= 0.2){
            expected_v = 0.2;
            local_planner.set_v(expected_v);
        }
        double dstPoint_angle = atan2(dst_pose[1]-self_pose[1], dst_pose[0]-self_pose[0]) - self_pose[2]; // 夹角
        if (dstPoint_angle > M_PI)
            dstPoint_angle -= 2*M_PI;
        else if (dstPoint_angle < -M_PI)
            dstPoint_angle += 2*M_PI;
        if (abs(dstPoint_angle) > M_PI/2){  // 走过点，停车
            expected_v = 0;
            local_planner.set_v(expected_v);
        }
        // -----------------------------------------------------

        wAndTraj = local_planner.control_trajectory(self_pose, self_w, dst_pose, obstacle, path, 0);
        // self_w = wAndTraj.first;
        traj = wAndTraj.second;
        // local_planner.check_param();

        // 在 Rviz 中显示 DWA 路径
        // display_PathandTraj(traj, traj_pub);

        r.sleep();
    }
}


void CB_publishCycle(const ros::TimerEvent& e)
{
    if (path.size())
        display_PathandTraj(path, path_pub);
    if (traj.size()){
        display_PathandTraj(traj, traj_pub);

        geometry_msgs::Twist target_vel;
        target_vel.linear.x = expected_v;
        target_vel.angular.z = wAndTraj.first;
        vel_pub.publish(target_vel);
    }
}

void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg)  // 目标点
{
    double theta = tf::getYaw(odom_msg->pose.pose.orientation);
    dst_pose = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, theta};
    // dst_pose = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0};
    // expected_v = odom_msg->twist.twist.linear.x;
    expected_v = 1.0;
    local_planner.set_v(expected_v);  // DWA 线速度设为定值

    ///   RRT 规划全局路径   ///
    global_planner.set_boundary(min(self_pose[0], dst_pose[0])-2, min(self_pose[1], dst_pose[1])-2,
                                                                    max(self_pose[0], dst_pose[0])+2, max(self_pose[1], dst_pose[1])+2);
    double len, shortest_len = 100;
    for (int j=0; j<3; j++){
        temp_path = global_planner.plan(obstacle, {self_pose[0], self_pose[1]}, {dst_pose[0], dst_pose[1]});
        len = 0;
        for (int k=0; k<temp_path.size()-1; k++)
            len += hypot(temp_path[k+1][0]-temp_path[k][0], temp_path[k+1][1]-temp_path[k][1]);
        if (len < shortest_len){
            shortest_len = len;
            path = temp_path;
        }
    }
    display_PathandTraj(path, path_pub);
}


void  CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)  // 速度反馈
{
    self_w = vel_msg->twist.angular.z;
}


void CB_laser(const sensor_msgs::PointCloud2::ConstPtr& scan_msg)  // 激光
{
    static tf::TransformListener tf_listener;
    // vector<vector<double>> scan_points;

    pcl::PointCloud<pcl::PointXYZ> temp_pcl;
    pcl::fromROSMsg(*scan_msg, temp_pcl);
    double obst_dist = 5.0, blind = 0.01;
    int point_filter_num = 5;

    obstacle.clear();

    for (int i = 0; i < temp_pcl.points.size(); i++)
    {
        if (i % point_filter_num != 0) continue;
        // cout << "obst size: " << temp_pcl.points.size() << endl;
        // cout << "obst x,y,z: " << temp_pcl.points[i].x << ", " << temp_pcl.points[i].y << ", " << temp_pcl.points[i].z << endl;

        double range = temp_pcl.points[i].x * temp_pcl.points[i].x + temp_pcl.points[i].y * temp_pcl.points[i].y + temp_pcl.points[i].z * temp_pcl.points[i].z;
        
        // only consider range < 5.0m
        if (range < (blind * blind) || range > (obst_dist * obst_dist) || temp_pcl.points[i].z < -0.2)  continue;
        
        // PointType added_pt;
        // added_pt.x = temp_pcl.points[i].x;
        // added_pt.y = temp_pcl.points[i].y;
        // added_pt.z = temp_pcl.points[i].z;
        // // added_pt.intensity = temp_pcl.points[i].intensity;
        // // added_pt.normal_x = 0;
        // // added_pt.normal_y = 0;
        // // added_pt.normal_z = 0;
        // // added_pt.curvature = temp_pcl.points[i].t * time_unit_scale; // curvature unit: ms

        geometry_msgs::PointStamped temp_obst_xyz_in_laser;
        geometry_msgs::PointStamped temp_obstPoint_in_odom;

        temp_obst_xyz_in_laser.header.stamp = ros::Time();
        // temp_obst_xyz_in_laser.header.frame_id = scan_msg->header.frame_id;  // laser
        temp_obst_xyz_in_laser.header.frame_id = "robot_"+to_string(RobotId)+"/base";  // laser
        // cout << "scan_frame: " << scan_msg->header.frame_id << endl;
        temp_obst_xyz_in_laser.point.x = temp_pcl.points[i].x;
        temp_obst_xyz_in_laser.point.y = temp_pcl.points[i].y;
        temp_obst_xyz_in_laser.point.z = temp_pcl.points[i].z;
        try{
            tf_listener.transformPoint("robot_"+to_string(RobotId)+"/odom", temp_obst_xyz_in_laser, temp_obstPoint_in_odom);
        }
        catch (tf::TransformException &ex){
            ROS_ERROR("%s, in CB_laser.", ex.what());
            return;
        }

        // scan_points.push_back(p);  // all points, ( (x0,y0), (x1,y1), ... )
        obstacle.push_back({temp_obstPoint_in_odom.point.x, temp_obstPoint_in_odom.point.y});
    }

    got_obst = 1;
}


void display_PathandTraj(pointVec path, ros::Publisher visual_pub){  // 在 Rviz 中显示 RRT 与 DWA 路径
    nav_msgs::Path  visual_path;
    visual_path.header.stamp = ros::Time(0);
    visual_path.header.frame_id = "robot_"+to_string(RobotId)+"/odom";
    for (int i = 0; i < path.size(); i++){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time(0);
        pose.header.frame_id = "robot_"+to_string(RobotId)+"/odom";
        pose.pose.position.x = path[i][0];
        pose.pose.position.y = path[i][1];
        pose.pose.position.z = 0.01;
        pose.pose.orientation.w = 1;
        visual_path.poses.push_back(pose);
    }
    visual_pub.publish(visual_path);
}