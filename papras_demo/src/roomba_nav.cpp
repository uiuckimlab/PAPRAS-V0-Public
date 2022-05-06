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

// Other
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>
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
double tolerance = 0.10;



geometry_msgs::Pose get_position_from_marker(aruco_msgs::Marker marker){
  return  marker.pose.pose;
}

void aruco_tag_callback(const aruco_msgs::MarkerArray::ConstPtr& msgs){
  found_markers = *msgs;
  for(int i = 0; i < found_markers.markers.size(); i++){
    marker_id_index[(int)found_markers.markers[i].id] = i;
  }
}

void find_tag(ros::Publisher pub, int goal,ros::NodeHandle nh ){
  geometry_msgs::Twist cmd_vel_msg;
  bool found_a_marker = false;
  do{
    const aruco_msgs::MarkerArray::ConstPtr detections = ros::topic::waitForMessage<aruco_msgs::MarkerArray>("/aruco_marker_publisher/markers", nh, ros::Duration(2.0));
    if (detections && marker_id_index.find(goal) != marker_id_index.end())
    {
        found_a_marker = true;
    }

    if(!found_a_marker){
      cmd_vel_msg.angular.z = 2.0;
      pub.publish(cmd_vel_msg);
    }
  }while(!found_a_marker);
}

void roomba_nav(ros::Publisher pub, int goal){
  if(marker_id_index.find(goal) == marker_id_index.end()){
    return;
  }
  geometry_msgs::Twist cmd_vel_msg;
  geometry_msgs::Pose p =  get_position_from_marker(found_markers.markers[marker_id_index[goal]]);

  if(p.position.x == NULL){
    return;
  }

  cmd_vel_msg.linear.x = 0.2;
  cmd_vel_msg.angular.z = 0;

  if(p.position.x > tolerance){
    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = (-0.15 * camera_upside_down) * (1+(std::abs(p.position.x-tolerance)));
  }else if (p.position.x < - tolerance){
    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = (0.15 * camera_upside_down) * (1+(std::abs(p.position.x-tolerance)));
  }

  if(p.position.z  > 2){
    cmd_vel_msg.linear.x = 0.2 * (1+(std::abs(p.position.z-2)));
  }else if(p.position.z > 0.8){
    cmd_vel_msg.linear.x = 0.1 * (1+(std::abs(p.position.z -0.8)));
    tolerance = 0.025;
  }else if(p.position.z  < 0.5){
    cmd_vel_msg.linear.x = -0.1 * (1+(std::abs(p.position.z -0.5)));
  }else{
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.angular.z = 0.5;
    goal_tag_id = 10;
  }


  pub.publish(cmd_vel_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roomba_nav");
  ros::AsyncSpinner spinner(1);
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
 

  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/roomba/cmd_vel", 1000);
  ros::Subscriber marker_sub = nh.subscribe("/aruco_marker_publisher/markers", 1000, aruco_tag_callback);
  
  goal_tag_id = 0;

  geometry_msgs::Twist cmd_vel_msg;
  for(int i = 0; i < 100; i++){
    cmd_vel_msg.linear.x = 0.20;
    cmd_vel_pub.publish(cmd_vel_msg);
    r.sleep();
  }

  while(ros::ok()){
    find_tag(cmd_vel_pub,goal_tag_id, nh);
    roomba_nav(cmd_vel_pub,goal_tag_id);
    r.sleep();
  }





  return 1;
}