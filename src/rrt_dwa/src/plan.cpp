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
double slow_down_dist, stop_dist, start_w, terminal_w;
double dwa_predict_time;
// -------------- plan --------------
B_RRT global_planner;
DWA   local_planner;
pointVec  obstacle;
pointVec  path, temp_path, traj;
bool find_path, arrive_dst;
int turn_at_start;
point_t mid_goal;
double mid_goal_dist_gain;
int goal_index;
pair<double, vector<vector<double>>>  wAndTraj;
double rrt_avoid_dist, rrt_boundary_expand_dist;


void CB_publishCycle(const ros::TimerEvent& e);
// void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg);
// void CB_dstPoint(const geometry_msgs::Pose::ConstPtr& pose_msg);
void CB_dstPoint(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
void CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);
// **************** test in BARN ****************** //
// void CB_laser(const sensor_msgs::PointCloud2::ConstPtr &scan_msg);
void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan);
// ******************************************** //
void get_mid_goal(const double &threshold);
void display_PathandTraj(pointVec path, ros::Publisher visual_pub);
void display_midGoal(point_t mid_goal, ros::Publisher visual_pub);


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
    ros::param::get("start_w", start_w);
    ros::param::get("terminal_w", terminal_w);
    ros::param::get("mid_goal_dist_gain", mid_goal_dist_gain);
    ros::param::get("laser_topic", laser_topic);
    ros::param::get("vel_pub_topic", vel_pub_topic);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
    global_planner.load_param();
    local_planner.load_param();

    // **************** test in BARN ****************** //
    // state_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    // ******************************************** //
    poseVel_sub = n.subscribe("/velocity_info", 1, CB_update_vel);
    // dstPoint_sub = n.subscribe("/dst_point", 1, CB_dstPoint);  // aero
    dstPoint_sub = n.subscribe("/move_base_simple/goal", 1, CB_dstPoint);  // aero
    // laser_sub = n.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 10, CB_laser);
    // laser_sub = n.subscribe("/ouster/points", 10, CB_laser);
    // laser_sub = n.subscribe("/robot_2/pointcloud", 10, CB_laser);
    laser_sub = n.subscribe(laser_topic, 10, CB_laser);
    // rviz_laser_pub = n.advertise<sensor_msgs::LaserScan>(laser_pub_topic, 10);

    // vel_pub = n.advertise<geometry_msgs::Twist>("target_vel", 10);
    vel_pub = n.advertise<geometry_msgs::Twist>(vel_pub_topic, 10);
    path_pub = n.advertise<nav_msgs::Path>("rrt_path", 10);
    traj_pub = n.advertise<nav_msgs::Path>("dwa_traj", 10);
    midpose_pub = n.advertise<geometry_msgs::PoseStamped>("/course_agv/mid_goal", 10);

    // ------------------------ debug ------------------------ //
    // self_pose = {0,0,0};
    // dst_pose = {5,5,0};
    self_pose = {0,0,0};
    dst_pose = {0,0,0};
    
    double threshold = mid_goal_dist_gain * v_in_yaml * dwa_predict_time; // 局部目标点距离
    local_planner.set_v(0.0);  // DWA 线速度设为定值, init to 0
    // ------------------------------------------------------- //

    int count=0;
    ros::Rate r(5);
    while(ros::ok()){
        ros::spinOnce();
        // cout << "obst_size: " << obstacle.size() << endl;
        chrono::time_point<chrono::system_clock> start_time = chrono::system_clock::now();
        /// 获取定位 ///
        // **************** test in BARN ****************** //
        // ros::service::waitForService("/gazebo/get_model_state");
        // gazebo_msgs::GetModelState model_state;
        // model_state.request.model_name = "jackal";
        // model_state.request.relative_entity_name = "world";
        // state_client.call(model_state);
        // if (model_state.response.success == 0){
        //     cout << "call model_state failed!" << endl;
        //     continue;
        // }
        // // cout << "time2: " << ros::Time::now() << endl;

        // self_pose[0] = model_state.response.pose.position.x;
        // self_pose[1] = model_state.response.pose.position.y;
        // self_pose[2] = tf::getYaw(model_state.response.pose.orientation);
        // // cout << "yaw: " << self_pose[2] << endl;
        // self_w = model_state.response.twist.angular.z;

        static tf::TransformListener tf_listener;
        try{
            tf_listener.lookupTransform("robot_"+to_string(RobotId)+"/odom", "robot_"+to_string(RobotId)+"/base", ros::Time(0), base_in_odom);
        }
        catch (tf::TransformException &ex){
            ROS_WARN("%s, in CB_main.", ex.what());
            continue;
        }
        geometry_msgs::Quaternion orientation;
        tf::quaternionTFToMsg(base_in_odom.getRotation(), orientation);
        double yaw_now = tf::getYaw(orientation);
        self_pose = {base_in_odom.getOrigin().x(), base_in_odom.getOrigin().y(), yaw_now};
        
        
        if (path.size() == 0){
            continue;
        }
        ///  DWA 选取最优角速度  ///
        // -----------------------------------------------------
        // 开始时转向，转到再执行后续
        if (turn_at_start){
            double startOri_angle = atan2(path[1][1]-path[0][1], path[1][0]-path[0][0]) - self_pose[2]; // 夹角
            if (startOri_angle > M_PI)
                startOri_angle -= 2*M_PI;
            else if (startOri_angle < -M_PI)
                startOri_angle += 2*M_PI;
            double startOri_threshold = M_PI/12; //, start_w = 0.8;
            if (turn_at_start == 1)  // 左转
                expected_w = start_w;
            else if (turn_at_start == 2)  // 右转
                expected_w = -1*start_w;
            if (abs(startOri_angle) < startOri_threshold || abs(startOri_angle) > M_PI-startOri_threshold){
                expected_w = 0;
                turn_at_start = 0;
            }
            continue;
        }

        double dstPoint_dist = hypot(dst_pose[0]-self_pose[0], dst_pose[1]-self_pose[1]); // 距离
        if (dstPoint_dist < slow_down_dist && dstPoint_dist > stop_dist){  // 减速
            expected_v = ori_sign * v_in_yaml * dstPoint_dist/slow_down_dist;
            if (abs(expected_v) < 0.2)
                expected_v = ori_sign*0.2;
            local_planner.set_v(expected_v);
        }

        // double dstPosition_angle = atan2(dst_pose[1]-self_pose[1], dst_pose[0]-self_pose[0]) - self_pose[2]; // 目标点到当前点与车头夹角
        // if (dstPosition_angle > M_PI)
        //     dstPosition_angle -= 2*M_PI;
        // else if (dstPosition_angle < -M_PI)
        //     dstPosition_angle += 2*M_PI;
        
        double dstOri_angle = dst_pose[2] - self_pose[2]; // 目标朝向与车头夹角
        if (dstOri_angle > M_PI)
            dstOri_angle -= 2*M_PI;
        else if (dstOri_angle < -M_PI)
            dstOri_angle += 2*M_PI;

        double dstOri_threshold = M_PI/16;  //, terminal_w = 0.8;
        if (dstPoint_dist < stop_dist){  // 停车距离内，停车
            arrive_dst = true;
        }
        // else if (is_forward && abs(dstPosition_angle) > M_PI/2){  // 走过点，停车
        //     expected_v = 0;
        //     local_planner.set_v(expected_v);
        //     // expected_w = 0;
        //     // 停车后原地转到目标朝向
        //     if (dstOri_angle > dstOri_threshold)
        //         expected_w = terminal_w;
        //     else if (dstOri_angle < -1*dstOri_threshold)
        //         expected_w = -1*terminal_w;
        //     else
        //         expected_w = 0;
        //     // cout << "run over!" << endl;
        // }
        // else if(not is_forward && abs(dstPosition_angle) < M_PI/2){
        //     expected_v = 0;
        //     local_planner.set_v(expected_v);
        //     // expected_w = 0;
        //     // 停车后原地转到目标朝向
        //     if (dstOri_angle > dstOri_threshold)
        //         expected_w = terminal_w;
        //     else if (dstOri_angle < -1*dstOri_threshold)
        //         expected_w = -1*terminal_w;
        //     else
        //         expected_w = 0;
        // }

        if (arrive_dst){  // 停车
            expected_v = 0;
            local_planner.set_v(expected_v);
            // expected_w = 0;
            // 停车后原地转到目标朝向
            if (dstOri_angle > dstOri_threshold)
                expected_w = terminal_w;
            else if (dstOri_angle < -1*dstOri_threshold)
                expected_w = -1*terminal_w;
            else
                expected_w = 0;
        }
        else{  // DWA规划
            if (!find_path)
                continue;
            get_mid_goal(threshold);
            wAndTraj = local_planner.control_trajectory(self_pose, self_w, mid_goal, obstacle, 0);
            // self_w = wAndTraj.first;
            expected_w = wAndTraj.first;
            traj = wAndTraj.second;
            // local_planner.check_param();
        }
        // -----------------------------------------------------

        // 在 Rviz 中显示 DWA 路径
        // display_PathandTraj(traj, traj_pub);

        chrono::time_point<chrono::system_clock> end_time = chrono::system_clock::now();
        // cout << "main_cycle: " << chrono::duration_cast<chrono::milliseconds>(end_time-start_time).count() / 1000.0 << "s" << endl;

        // r.sleep();
    }
}


void CB_publishCycle(const ros::TimerEvent& e)
{
    // if (path.size())
    //     display_PathandTraj(path, path_pub);
    if (mid_goal.size())
        display_midGoal(mid_goal, midpose_pub);

    if (traj.size()){
        display_PathandTraj(traj, traj_pub);
    }

    if (find_path){
        geometry_msgs::Twist target_vel;
        if (abs(expected_w) >= 0.5)  // 转弯时降低线速度
            target_vel.linear.x = 0.5*expected_v;
        else if (abs(expected_w) >= 0.2)
            target_vel.linear.x = 0.7*expected_v;
        else
            target_vel.linear.x = expected_v;
        target_vel.angular.z = expected_w;
        vel_pub.publish(target_vel);
    }
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
    

    // !!!!! 改为与第一段路径夹角 !!!!!
    // double dstPoint_ori = atan2(dst_pose[1]-self_pose[1], dst_pose[0]-self_pose[0]) - self_pose[2]; // 夹角
    // if (dstPoint_ori > M_PI)
    //     dstPoint_ori -= 2*M_PI;
    // else if (dstPoint_ori < -M_PI)
    //     dstPoint_ori += 2*M_PI;
    // if(abs(dstPoint_ori) > M_PI/2)
    //     is_forward = false;
    // else is_forward = true;

    // if(is_forward) ori_sign = 1.0;
    // else ori_sign = -1.0;

    // expected_v = ori_sign * v_in_yaml;
    // local_planner.set_v(expected_v);  // DWA 线速度设为定值
    

    ///   RRT 规划全局路径   ///
    find_path = false;  // reset
    arrive_dst = false;
    goal_index = 0;
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
        return;
    }

    // 判断前进还是后退
    // !!!!! 根据与第一段路径夹角 !!!!!
    double path_first_line_ori = atan2(path[1][1]-path[0][1], path[1][0]-path[0][0]) - self_pose[2]; // 夹角
    if (path_first_line_ori > M_PI)
        path_first_line_ori -= 2*M_PI;
    else if (path_first_line_ori < -M_PI)
        path_first_line_ori += 2*M_PI;
    if(abs(path_first_line_ori) > M_PI/2)
        is_forward = false;
    else is_forward = true;

    if(is_forward) ori_sign = 1.0;
    else ori_sign = -1.0;

    expected_v = ori_sign * v_in_yaml;
    local_planner.set_v(expected_v);  // DWA 线速度设为定值
    
    // 决定是否在运动前先转向
    turn_at_start = 0;  // 不转
    double startOri_threshold = M_PI/12;
    if (is_forward){ // 前进
        if (path_first_line_ori > startOri_threshold){
            turn_at_start = 1;  // 左转
        }
        else if (path_first_line_ori < -1*startOri_threshold){
            turn_at_start = 2;  // 右转
        }
    }
    else{ // 后退
        if (path_first_line_ori < -M_PI/2 && path_first_line_ori > -(M_PI-startOri_threshold)){
            turn_at_start = 1;  // 左转
        }
        else if (path_first_line_ori > M_PI/2 && path_first_line_ori < M_PI-startOri_threshold){
            turn_at_start = 2;  // 右转
        }
    }

    display_PathandTraj(path, path_pub);
}


void  CB_update_vel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)  // 速度反馈
{
    self_w = vel_msg->twist.angular.z;
}


void get_mid_goal(const double &threshold)
{
    // 动态选取局部目标点
    int idx = goal_index;
    goal_index = path.size() - 1;
    // ——————————————————————————————————————————————
    // cout << "goal_index: " << goal_index << endl;
    while (idx < path.size()){
        double dis = hypot(path[idx][0]-self_pose[0], path[idx][1]-self_pose[1]);
        if (dis >= threshold){
            goal_index = idx;
            break;
        }
        idx++;
    }

    mid_goal = path[goal_index];
    // if (goal_dis >= threshold)
    double step_forward = 0.1;
    double front_point_x = path[goal_index][0];
    double front_point_y = path[goal_index][1];
    double back_point_x = path[goal_index-1][0];
    double back_point_y = path[goal_index-1][1];
    // print("front_x: %.2f, front_y: %.2f" % (front_point_x, front_point_y))
    // print("back_x: %.2f, back_y: %.2f" % (back_point_x, back_point_y))
    double front_to_back_dist = hypot(front_point_x - back_point_x,
                                    front_point_y - back_point_y);
    double ratio, temp_x, temp_y;

    int temp_count = int(front_to_back_dist / step_forward);
    for (int i=0; i<temp_count; i++){  // 从远至近找局部目标点
        ratio = (i+1) * step_forward / front_to_back_dist;
        temp_x = front_point_x + (back_point_x-front_point_x) * ratio;
        temp_y = front_point_y + (back_point_y-front_point_y) * ratio;
        if (hypot(temp_x-self_pose[0], temp_y-self_pose[1]) <= threshold){
            break;
        }
    }
    mid_goal[0] = temp_x;
    mid_goal[1] = temp_y;
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

            if (x>-0.42 && x<0.17 && y>-0.24 && y<0.24) // 车体范围内
                continue;

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
                ROS_WARN("%s, in CB_laser.", ex.what());
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
//             ROS_WARN("%s, in CB_laser.", ex.what());
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

void display_midGoal(point_t mid_goal, ros::Publisher visual_pub){  // Rviz 中显示局部目标点
    geometry_msgs::PoseStamped  mid_goal_msg;
    mid_goal_msg.header.stamp = ros::Time(0);
    // **************** test in BARN ****************** //
    mid_goal_msg.header.frame_id = "robot_"+to_string(RobotId)+"/odom";
    // mid_goal_msg.header.frame_id = "/map";
    mid_goal_msg.pose.position.x = mid_goal[0];
    mid_goal_msg.pose.position.y = mid_goal[1];
    mid_goal_msg.pose.orientation.w = 1;

    visual_pub.publish(mid_goal_msg);
}
