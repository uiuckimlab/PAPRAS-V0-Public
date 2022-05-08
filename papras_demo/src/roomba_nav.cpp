/* Authors: Kazuki Shin, Sankalp Yamsani */

// ROS
#include <ros/ros.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <aruco_msgs/MarkerArray.h>
#include <aruco_msgs/Marker.h>

// eigen_conversions
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


// Other
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <cmath>        // std::abs
#include <vector>
#include <unordered_map>




enum state
{
    ST_INIT,
    ST_PAUSED,
    ST_ARM_TO_REST_START,
    ST_INIT_SIM_OBJ,
    ST_CAPTURE_OBJ,
    ST_PICK_OBJ,
    ST_ARM_TO_TRANSPORT,
    ST_PLACE_OBJ,
    ST_ARM_TO_HOME_END,
    ST_DONE
};

int goal_tag_id = 0;
aruco_msgs::MarkerArray found_markers;
std::unordered_map<int,int> marker_id_index;
int camera_upside_down = -1;
double tolerance = 0.22;

int aruco_tag_cb_cnt = -1;
bool finished_roomba_nav = false;
volatile bool start_roomba = false;

geometry_msgs::Pose get_position_from_marker(aruco_msgs::Marker marker){
  return  marker.pose.pose;
}

void aruco_tag_callback(const aruco_msgs::MarkerArray::ConstPtr& msgs){
  ROS_INFO_STREAM(*msgs);
  found_markers = *msgs;
  marker_id_index.erase(marker_id_index.begin(),marker_id_index.end());
  for(int i = 0; i < found_markers.markers.size(); i++){
    marker_id_index[(int)found_markers.markers[i].id] = i;
  }
  if (marker_id_index.find(goal_tag_id) != marker_id_index.end())
    aruco_tag_cb_cnt = 0;
}

void start_roomba_callback(const std_msgs::Bool data){
    start_roomba = data.data;
}

void find_tag(ros::Publisher pub,ros::NodeHandle nh ){
  geometry_msgs::Twist cmd_vel_msg;
  cmd_vel_msg.angular.z = 0.4;
  pub.publish(cmd_vel_msg);
}

void roomba_nav(ros::Publisher pub, int goal){
  ROS_INFO_STREAM("in roomba nav goal id: " << goal);
  if(marker_id_index.find(goal) == marker_id_index.end()){
    return;
  }
  geometry_msgs::Twist cmd_vel_msg;
  geometry_msgs::Pose p =  get_position_from_marker(found_markers.markers[marker_id_index[goal]]);

  if(p.position.x == NULL){
    return;
  }

  cmd_vel_msg.linear.x = 0.0;
  cmd_vel_msg.angular.z = 0.0;

  if(p.position.x > tolerance){
    cmd_vel_msg.angular.z = (-0.2 * camera_upside_down) * (1+(std::abs(p.position.x-tolerance)));
  }else if (p.position.x < - tolerance){
    cmd_vel_msg.angular.z = (0.2 * camera_upside_down) * (1+(std::abs(p.position.x-tolerance)));
  }

  if (goal_tag_id == 0)
  {
    if(p.position.z  > 2.0){
      cmd_vel_msg.linear.x = 0.05 * (1+(std::abs(p.position.z-2.0)));
    }else if(p.position.z > 1.25){
      cmd_vel_msg.linear.x = 0.01 * (1+(std::abs(p.position.z - 1.25)));
      tolerance = 0.025;
    }else if(p.position.z  < 1.0){
      cmd_vel_msg.linear.x = -0.01 * (1+(std::abs(p.position.z - 1.00)));
    }else{
      goal_tag_id = 10;
    }
  } else if (goal_tag_id == 10) {
    if(p.position.z  > 2.0){
    cmd_vel_msg.linear.x = 0.05 * (1+(std::abs(p.position.z-2)));
    }else if(p.position.z > 0.63){
      cmd_vel_msg.linear.x = 0.05 * (1+(std::abs(p.position.z - 0.63)));
      tolerance = 0.15;
    }else if(p.position.z  < 0.62){
      cmd_vel_msg.linear.x = -0.05 * (1+(std::abs(p.position.z -0.62)));
    }else{
      finished_roomba_nav = true;      
    }
  }

  // pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roomba_nav");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate r(10); 
  ros::NodeHandle nh;
  state task_state = ST_INIT;
  state paused_state = ST_INIT;

  std::string world_name;
  bool handover_planned;

  // Load rosparams
  ros::NodeHandle nh_priv("~");
  std::string param_path;

  moveit::planning_interface::MoveGroupInterface group("arm1");
  group.setPlanningTime(45.0);
  group.setPlannerId("RRTConnect");
  group.setMaxAccelerationScalingFactor(0.10);
  group.setMaxVelocityScalingFactor(0.10);

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/roomba/cmd_vel", 10);
  ros::Publisher start_kitchen = nh.advertise<std_msgs::Bool>("/roomba/start_kitchen", 10);

  ros::Subscriber marker_sub = nh.subscribe("/aruco_marker_publisher/markers", 10, aruco_tag_callback);
  ros::Subscriber handoff_done = nh.subscribe("/big_table/start_roomba",10, start_roomba_callback);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  ROS_INFO_STREAM("after init");
  goal_tag_id = 10;

  group.setStartStateToCurrentState();
  group.setNamedTarget("plate_big_table");
  moveit::core::MoveItErrorCode error_code = group.plan(plan);
  error_code = group.execute(plan);

  while(!start_roomba);

  group.setStartStateToCurrentState();
  group.setNamedTarget("plate_down");
  error_code = group.plan(plan);
  error_code = group.execute(plan);

  // geometry_msgs::Twist cmd_vel_msg;
  // for(int i = 0; i < 100 && aruco_tag_cb_cnt == -1; i++){
  //   cmd_vel_msg.linear.x = 0.40;
  //   cmd_vel_msg.angular.z = -0.08;
  //   cmd_vel_pub.publish(cmd_vel_msg);
  //   r.sleep();
  // }

  while(ros::ok() && !finished_roomba_nav){
    aruco_tag_cb_cnt+=1;
    // if (aruco_tag_cb_cnt > 10) 
    // {
    //   find_tag(cmd_vel_pub, nh);
    //   ros::Duration(1.0).sleep();
    // }
    roomba_nav(cmd_vel_pub,goal_tag_id);
    r.sleep();
  }

  group.setStartStateToCurrentState();
  group.setNamedTarget("plate_big_table");
  error_code = group.plan(plan);
  error_code = group.execute(plan);

  std_msgs::Bool finished_nav;
  finished_nav.data = true;
  start_kitchen.publish(finished_nav);

  return 0;
}