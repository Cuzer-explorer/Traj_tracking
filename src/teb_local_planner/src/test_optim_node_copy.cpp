/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017.
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/************************************************/
// used to read .txt which is converted from .bag
// #include <unistd.h>
// #include <fstream>
#include <iostream>
// #include <sstream>
// #include <string>
// #include <cmath>
/****************  obst added ****************/
#include <sensor_msgs/LaserScan.h>
// #include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <teb_local_planner/MyObstPolygonsMsg.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <geometry_msgs/TwistStamped.h>
// #include <algorithm>
/**********************************************/

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

// ============= Global Variables ================
// Ok global variables are bad, but here we only have a simple testing node.
PlannerInterfacePtr planner;
TebVisualizationPtr visual;
std::vector<ObstaclePtr> obst_vector;
ViaPointContainer via_points;
TebConfig config;

boost::shared_ptr< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> > dynamic_recfg;
ros::Subscriber custom_obst_sub;
ros::Subscriber via_points_sub;
ros::Subscriber clicked_points_sub;
std::vector<ros::Subscriber> obst_vel_subs;
unsigned int no_fixed_obstacles;
/**********************************************/
ros::Subscriber poseVel_sub;
ros::Subscriber dstPoint_sub;
ros::Subscriber laser_sub; // obst added
ros::Publisher feedback_pub; // obst added
// ros::Subscriber polygon_sub; // obst added



nav_msgs::Odometry location_now;


/****************  obst added ****************/
std::vector <std::vector<double> > obstPoint_in_world, obstOrigin_in_world, obstPolygon_in_world;
std::vector <double> obstPolygon_Length;
teb_local_planner::MyObstPolygonsMsg  obst_polygons;

int got_obst, got_location;

int car_collision;
double brake_a;
// -------------------------------------------
/**********************************************/

// ------------------ aero ------------------
tf::StampedTransform base_in_odom;
tf::Vector3 dst_pose;
double expected_v;


// =========== Function declarations =============
std::vector <std::vector <double>> TransGlobalPathToMap();
void CB_mainCycle(const ros::TimerEvent& e);
void CB_publishCycle(const ros::TimerEvent& e);
void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level);
void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb);
void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);
void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id);


void CB_update_PoseVel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg);  // aero
void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg);  // aero

void CB_laser(const sensor_msgs::LaserScan::ConstPtr &scan_msg);  // obst added
// void CB_obstPolygon(const geometry_msgs::PolygonStamped::ConstPtr &polygon_msg);  // obst added
void CB_obstPolygon(const teb_local_planner::MyObstPolygonsMsg::ConstPtr &polygon_msg);  // obst added
// void publishVelocity(const double& vx_now, const double& brake_a, const int& car_collision);  // obst added
void publishVelocity(const int& car_collision);  // obst added


std::string location_topic;
// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc,argv,"test_optim_node");
  ros::NodeHandle n;
  

  // load ros parameters from node handle
  config.loadRosParamFromNodeHandle(n);
  // setup dynamic reconfigure
  dynamic_recfg = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(n);
  dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(CB_reconfigure, _1, _2);
  dynamic_recfg->setCallback(cb);
  
  // setup callback for custom obstacles
  custom_obst_sub = n.subscribe("obstacles", 1, CB_customObstacle);
  
  // setup callback for clicked points (in rviz) that are considered as via-points
  clicked_points_sub = n.subscribe("/clicked_point", 5, CB_clicked_points);
  
  // setup callback for via-points (callback overwrites previously set via-points)
  via_points_sub = n.subscribe("via_points", 1, CB_via_points);
  

  ros::Timer cycle_timer = n.createTimer(ros::Duration(0.025), CB_mainCycle);
  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), CB_publishCycle);
  

  /******************************* self sub and pub *******************************/
  poseVel_sub = n.subscribe("/velocity_info", 1, CB_update_PoseVel);
  dstPoint_sub = n.subscribe("/dst_point", 1, CB_dstPoint);  // aero
  // grid_obst_sub = n.subscribe("/map", 1, CB_gridObstacle); // obst added
  // polygon_sub = n.subscribe<teb_local_planner::MyObstPolygonsMsg>("/my_obst_polygons", 10, CB_obstPolygon); // obst added
  laser_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, CB_laser); // obst added
  feedback_pub = n.advertise<teb_local_planner::FeedbackMsg>("/teb_feedback", 10);  


  // interactive marker server for simulated dynamic obstacles
  interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

  // modified on 7.6
  // obst_vector.push_back( boost::make_shared<PointObstacle>(2,-2) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(3,-3) );
  // obst_vector.push_back( boost::make_shared<PointObstacle>(4,-4) );


  // Dynamic obstacles
  // Eigen::Vector2d vel (0.1, -0.3);
  // obst_vector.at(0)->setCentroidVelocity(vel);
  // vel = Eigen::Vector2d(-0.3, -0.2);
  // obst_vector.at(1)->setCentroidVelocity(vel);

  /*
  PolygonObstacle* polyobst = new PolygonObstacle;
  polyobst->pushBackVertex(1, -1);
  polyobst->pushBackVertex(0, 1);
  polyobst->pushBackVertex(1, 1);
  polyobst->pushBackVertex(2, 1);
 
  polyobst->finalizePolygon();
  obst_vector.emplace_back(polyobst);
  */
  
  for (unsigned int i=0; i<obst_vector.size(); ++i)
  {
    // setup callbacks for setting obstacle velocities
    std::string topic = "/test_optim_node/obstacle_" + std::to_string(i) + "/cmd_vel";
    obst_vel_subs.push_back(n.subscribe<geometry_msgs::Twist>(topic, 1, boost::bind(&CB_setObstacleVelocity, _1, i)));

    //CreateInteractiveMarker(obst_vector.at(i)[0],obst_vector.at(i)[1],i,&marker_server, &CB_obstacle_marker);  
    // Add interactive markers for all point obstacles
    boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(obst_vector.at(i));
    if (pobst)
    {
      CreateInteractiveMarker(pobst->x(),pobst->y(),i, config.map_frame, &marker_server, &CB_obstacle_marker);  
    }
  }
  marker_server.applyChanges();
  
  // Setup visualization
  visual = TebVisualizationPtr(new TebVisualization(n, config));
  
  // Setup robot shape model
  RobotFootprintModelPtr robot_model = TebLocalPlannerROS::getRobotFootprintFromParamServer(n, config);
  
  // Setup planner (homotopy class planning or just the local teb planner)
  if (config.hcp.enable_homotopy_class_planning)
    planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_model, visual, &via_points));
  else
    planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));
  
  // planner = PlannerInterfacePtr(new TebOptimalPlanner(config, &obst_vector, robot_model, visual, &via_points));

  no_fixed_obstacles = obst_vector.size();

  ros::spin();

  return 0;
}


// Planning loop
void CB_mainCycle(const ros::TimerEvent& e)
{
  // planner->plan(PoseSE2(-4,0,0), PoseSE2(4,0,0)); // hardcoded start and goal for testing purposes

  // ROS_INFO("mainCycle start!");
  static int count;

  // Wait for location and obstacle mode finishing init (Wait for tf being sent)
  // if (got_location == 0 || got_obst == 0)
  if (got_location == 0)
    return;


  // Get pose and vel
  double x_now = location_now.pose.pose.position.x;
  double y_now = location_now.pose.pose.position.y;
  double yaw_now = tf::getYaw(location_now.pose.pose.orientation);
  double vx_now = location_now.twist.twist.linear.x;
  double vy_now = location_now.twist.twist.linear.y;
  double wz_now = location_now.twist.twist.angular.z;



  /******************************  obst added  ******************************/
  // obst_vector.clear();
  // // for (int i=0; i<obstPoint_in_world.size(); i++){
  // for (int i=0; i<obstPoint_in_world.size(); i++){
  //   obst_vector.push_back( boost::make_shared<PointObstacle>(obstPoint_in_world[i][0],obstPoint_in_world[i][1]) );
  // }
  // std::cout << "obst point num: " << obst_vector.size() << std::endl;
  /**************************************************************************/

  config.robot.max_vel_x = expected_v;
  double terminal_threshold = 1.0, obst_inflation_dist = 1.8;
  
  car_collision = 0; 
  // if the car is near terminal
  double  terminal_dist = hypot(dst_pose.x()-x_now,  dst_pose.y()-y_now);
  if (terminal_dist <= terminal_threshold){
    car_collision = 1;
    brake_a = 1;
  }
  
  if (car_collision)  // don't plan, call brake function written by self
    return;
  

  // Get now_vel
  geometry_msgs::Twist* start_vel_ptr = NULL;
  geometry_msgs::Twist  start_vel;
  bool free_goal_vel = true;

  start_vel.linear.x  = vx_now;
  start_vel.linear.y  = vy_now;
  start_vel.angular.z  = wz_now;
  start_vel_ptr = &start_vel;
  

  //**************************** Plan ****************************//
  planner->plan(PoseSE2(x_now, y_now, yaw_now),
                PoseSE2(dst_pose.x(),  dst_pose.y(),  dst_pose.z()),
                start_vel_ptr, free_goal_vel);
  // debug
  // std::cout << "x_now: " << x_now << ", " << "y_now: " << y_now << ", " << "yaw_now: " <<  yaw_now << std::endl;
  /******************************************************************/


  count++;
  // ROS_INFO("mainCycle end!");
  
}


// Visualization loop
void CB_publishCycle(const ros::TimerEvent& e)
{
  if (car_collision){
    // publishVelocity(location_now.twist.twist.linear.x, brake_a, car_collision);
    publishVelocity(car_collision);
  }
  else
    planner->visualize();
  visual->publishObstacles(obst_vector);
}

void CB_reconfigure(TebLocalPlannerReconfigureConfig& reconfig, uint32_t level)
{
  config.reconfigure(reconfig);
}

void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame, interactive_markers::InteractiveMarkerServer* marker_server, interactive_markers::InteractiveMarkerServer::FeedbackCallback feedback_cb)
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker i_marker;
  i_marker.header.frame_id = frame;
  i_marker.header.stamp = ros::Time::now();
  std::ostringstream oss;
  //oss << "obstacle" << id;
  oss << id;
  i_marker.name = oss.str();
  i_marker.description = "Obstacle";
  i_marker.pose.position.x = init_x;
  i_marker.pose.position.y = init_y;
  i_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  box_marker.id = id;
  box_marker.scale.x = 0.2;
  box_marker.scale.y = 0.2;
  box_marker.scale.z = 0.2;
  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;
  box_marker.pose.orientation.w = 1.0f; // make quaternion normalized

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  i_marker.controls.push_back( box_control );

  // create a control which will move the box, rviz will insert 2 arrows
  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;


  // add the control to the interactive marker
  i_marker.controls.push_back(move_control);

  // add the interactive marker to our collection
  marker_server->insert(i_marker);
  marker_server->setCallback(i_marker.name,feedback_cb);
}

void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  std::stringstream ss(feedback->marker_name);
  unsigned int index;
  ss >> index;
  
  if (index>=no_fixed_obstacles) 
    return;
  PointObstacle* pobst = static_cast<PointObstacle*>(obst_vector.at(index).get());
  pobst->position() = Eigen::Vector2d(feedback->pose.position.x,feedback->pose.position.y);	  
}

void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  // resize such that the vector contains only the fixed obstacles specified inside the main function
  obst_vector.resize(no_fixed_obstacles);
  
  // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)  
  for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
  {
    if (obst_msg->obstacles.at(i).polygon.points.size() == 1 )
    {
      if (obst_msg->obstacles.at(i).radius == 0) 
      {
        obst_vector.push_back(ObstaclePtr(new PointObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                           obst_msg->obstacles.at(i).polygon.points.front().y )));
      }
      else
      {
        obst_vector.push_back(ObstaclePtr(new CircularObstacle( obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y,
                                                            obst_msg->obstacles.at(i).radius )));
      }
    }
    else if (obst_msg->obstacles.at(i).polygon.points.empty())
    {
      ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
      continue;
    }
    else
    {
      PolygonObstacle* polyobst = new PolygonObstacle;
      for (size_t j=0; j<obst_msg->obstacles.at(i).polygon.points.size(); ++j)
      {
        polyobst->pushBackVertex( obst_msg->obstacles.at(i).polygon.points[j].x,
                                  obst_msg->obstacles.at(i).polygon.points[j].y );
      }
      polyobst->finalizePolygon();
      obst_vector.push_back(ObstaclePtr(polyobst));
    }

    if(!obst_vector.empty())
      obst_vector.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
  }
}


void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
  // we assume for simplicity that the fixed frame is already the map/planning frame
  // consider clicked points as via-points
  via_points.push_back( Eigen::Vector2d(point_msg->point.x, point_msg->point.y) );
  ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
  if (config.optim.weight_viapoint<=0)
    ROS_WARN("Note, via-points are deactivated, since 'weight_via_point' <= 0");
}

void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  via_points.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
}


void CB_setObstacleVelocity(const geometry_msgs::TwistConstPtr& twist_msg, const unsigned int id)
{
  if (id >= obst_vector.size())
  {
    ROS_WARN("Cannot set velocity: unknown obstacle id.");
    return;
  }

  Eigen::Vector2d vel (twist_msg->linear.x, twist_msg->linear.y);
  obst_vector.at(id)->setCentroidVelocity(vel);
}



void CB_laser(const sensor_msgs::LaserScan::ConstPtr& scan_msg)  // obst added
{
  // cout << "range_min: " << scan_msg->range_min << endl;  // 0
  // cout << "range_max: " << scan_msg->range_max << endl;  // 100

  int scan_size = scan_msg->ranges.size();
  static tf::TransformListener tf_listener;
  // vector<vector<double>> scan_points;
  double obst_dist = 5.0;

  obstPoint_in_world.clear();
  int last_i;
  // for(int i = 0; i < scan_size; i++)  // for one point
  for(int i = 0; i < scan_size; i+=5)  // for one point
  {
    double range = scan_msg->ranges[i];

    // cout << "range " << i << ": " << range << ", ";

    if( isinf(range) || isnan(range) )
    {
      continue;
    }
    // else if( range < obst_dist )
    else if( range > 0.01 && range < obst_dist )  // in the inflation range
    {
      // only consider obst in front of car
      if(scan_msg->angle_min + i*scan_msg->angle_increment < -2*M_PI/3  ||  scan_msg->angle_min + i*scan_msg->angle_increment > 2*M_PI/3)
        continue;
      // continue the isolated point(abnormal)
      // if( i-last_i > 10){
      //   if( i+10 >= scan_size)  continue;
      //   int count = 0, flag = 0;
      //   for(int j = 1; j <= 10; j++){
      //     if( isinf(scan_msg->ranges[i+j]) || isnan(scan_msg->ranges[i+j]) || scan_msg->ranges[i+j] >= obst_dist)
      //       count++;
      //     if( count >= 5){
      //       flag = 1;
      //       break;
      //     }
      //   }
      //   if( flag )
      //     continue;
      // }
      double angle = scan_msg->angle_min + i * (scan_msg->angle_increment);
      double x = range * cos(angle);
      double y = range * sin(angle);


      geometry_msgs::PointStamped temp_obst_xyz_in_laser;
      geometry_msgs::PointStamped temp_obstPoint_in_world;

      temp_obst_xyz_in_laser.header.stamp = ros::Time();
      temp_obst_xyz_in_laser.header.frame_id = scan_msg->header.frame_id;  // laser
      temp_obst_xyz_in_laser.point.x = x;
      temp_obst_xyz_in_laser.point.y = y;
      temp_obst_xyz_in_laser.point.z = 0;
      try{
        tf_listener.transformPoint("world", temp_obst_xyz_in_laser, temp_obstPoint_in_world);
      }
      catch (tf::TransformException &ex){
        ROS_ERROR("%s, in CB_laser.", ex.what());
        return;
      }

      // scan_points.push_back(p);  // all points, ( (x0,y0), (x1,y1), ... )
      obstPoint_in_world.push_back({temp_obstPoint_in_world.point.x, temp_obstPoint_in_world.point.y});
      last_i = i;
    }
    else
    {
      continue;
    }
  }


  got_obst = 1;
}


// void CB_obstPolygon(const teb_local_planner::MyObstPolygonsMsg::ConstPtr& polygon_msg)  // obst added
// {
//   obst_polygons = *polygon_msg;
// }



// void publishVelocity(const double& vx_now, const double& brake_a, const int& car_collision)
void publishVelocity(const int& car_collision)
{
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  
  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  std::vector<TrajectoryPointMsg>& trajectory = msg.trajectories.front().trajectory;

  double publish_cycle = 0.1, vx_now = location_now.twist.twist.linear.x, curr_time = 0;

  std::cout << "car_collision: " << car_collision << std::endl;
  if (car_collision == 1){  // slow down
    std::cout << "brake_a: " << brake_a << ", traj_size: " << (int)(vx_now / (brake_a*publish_cycle)) + 2 << std::endl;
    
    trajectory.resize((int)(vx_now / (brake_a*publish_cycle)) + 2);
    for (int i=0; i<trajectory.size(); i++){
      trajectory[i].velocity.linear.x = vx_now - i * brake_a * publish_cycle;
      trajectory[i].velocity.linear.y = 0;
      trajectory[i].velocity.angular.z = 0;
      if (trajectory[i].velocity.linear.x * vx_now < 0)  // different sign
        trajectory[i].velocity.linear.x = 0;
      trajectory[i].time_from_start.fromSec(curr_time);
      curr_time += publish_cycle;
    }
  }
  else if (car_collision == 10){  // stop immediately
    trajectory.resize(10);
    for (int i=0; i<10; i++){
      trajectory[i].velocity.linear.x = 0;
      trajectory[i].velocity.linear.y = 0;
      trajectory[i].velocity.angular.z = 0;
      trajectory[i].time_from_start.fromSec(curr_time);
      curr_time += publish_cycle;
    }
  }
  feedback_pub.publish(msg);
}

void CB_dstPoint(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  dst_pose.setValue(0,0,0);
  expected_v = 1.0;
}


void  CB_update_PoseVel(const geometry_msgs::TwistStamped::ConstPtr& vel_msg)
{
  static tf::TransformListener tf_listener;
  try{
    tf_listener.lookupTransform("robot_1/odom", "robot_1/base", ros::Time(0), base_in_odom);
  }
  catch (tf::TransformException &ex){
    ROS_ERROR("%s, in CB_poseVel.", ex.what());
    return;
  }

  // Assign location_now
  location_now.header.stamp = vel_msg->header.stamp;
  // location_now.header.frame_id = vel_msg->header.frame_id;
  location_now.pose.pose.position.x = base_in_odom.getOrigin().x();
  location_now.pose.pose.position.y = base_in_odom.getOrigin().y();
  location_now.pose.pose.position.z = base_in_odom.getOrigin().z();
  tf::quaternionTFToMsg(base_in_odom.getRotation(), location_now.pose.pose.orientation);
  location_now.twist.twist.linear.x = vel_msg->twist.linear.x;
  location_now.twist.twist.linear.y = 0;
  location_now.twist.twist.angular.z = vel_msg->twist.angular.z;


  got_location = 1;
}

    
