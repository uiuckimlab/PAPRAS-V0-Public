/* Authors: Kazuki Shin */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <termios.h>
#include "../include/papras/command_operator.hpp"

Command_Operator::Command_Operator(int argc, char** argv)
   :init_argc(argc),
    init_argv(argv),
    node_handle_(""),
    priv_node_handle_("~"),
    open_manipulator_actuator_enabled_(false),
    open_manipulator_is_moving_(false),
    with_gripper_(false)
{
    arm_config_       = priv_node_handle_.param<std::string>("arm_config", "left_arm");
}

Command_Operator::~Command_Operator() {
   if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
}

bool Command_Operator::init() {
  // msg publisher
  open_manipulator_option_pub_ = node_handle_.advertise<std_msgs::String>("option", 10);
  // msg subscriber
  open_manipulator_states_sub_       = node_handle_.subscribe("states", 10, &Command_Operator::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &Command_Operator::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &Command_Operator::kinematicsPoseCallback, this);
  
  // service client
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_task_space_path_client_= node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  set_actuator_state_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");
  goal_drawing_trajectory_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("goal_drawing_trajectory");

  // ROS params
  with_gripper_ = priv_node_handle_.param<bool>("with_gripper", false);

	return true;
}

int Command_Operator::getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void Command_Operator::run(){
  ros::Rate loop_rate(10);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
  
    int c = getch();   // call your non-blocking input function
    if (c == 'q'){
      setActuatorState(false);
      ros::shutdown();
    }

	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
}

void Command_Operator::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;

  if(msg->open_manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    open_manipulator_actuator_enabled_ = true;
  else
    open_manipulator_actuator_enabled_ = false;
}

void Command_Operator::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL - 1);
  if (getWithGripperState()) temp_angle.resize(NUM_OF_JOINT_AND_TOOL);

  for(int i = 0; i < msg->name.size(); i ++)
  {
    if     (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
    
    if (getWithGripperState())
    {
      if(!msg->name.at(i).compare("gripper")) temp_angle.at(6) = (msg->position.at(i));      
    }
  }
  present_joint_angle_ = temp_angle;
}

void Command_Operator::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  Eigen::Quaterniond temp_orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

  present_kinematics_position_ = temp_position;
  present_kinematics_orientation_ = temp_orientation;

  kinematics_pose_.pose = msg->pose;
}

std::vector<double> Command_Operator::getPresentJointAngle()
{
  return present_joint_angle_;
}
std::vector<double> Command_Operator::getPresentKinematicsPosition()
{
  return present_kinematics_position_;
}
Eigen::Quaterniond Command_Operator::getPresentKinematicsOrientation()
{
  return present_kinematics_orientation_;
}
Eigen::Vector3d Command_Operator::getPresentKinematicsOrientationRPY()
{
  present_kinematics_orientation_rpy_ = robotis_manipulator::math::convertQuaternionToRPYVector(present_kinematics_orientation_);

  return present_kinematics_orientation_rpy_;
}
bool Command_Operator::getOpenManipulatorMovingState()
{
  return open_manipulator_is_moving_;
}
bool Command_Operator::getOpenManipulatorActuatorState()
{
  return open_manipulator_actuator_enabled_;
}
bool Command_Operator::getWithGripperState()
{
  return with_gripper_;
}

void Command_Operator::setOption(std::string opt)
{
  std_msgs::String msg;
  msg.data = opt;
  open_manipulator_option_pub_.publish(msg);
}

bool Command_Operator::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool Command_Operator::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kinematics_pose.at(3);
  srv.request.kinematics_pose.pose.orientation.x = kinematics_pose.at(4);
  srv.request.kinematics_pose.pose.orientation.y = kinematics_pose.at(5);
  srv.request.kinematics_pose.pose.orientation.z = kinematics_pose.at(6);

  srv.request.path_time = path_time;

  if(goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool Command_Operator::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool Command_Operator::setActuatorState(bool actuator_state)
{
  open_manipulator_msgs::SetActuatorState srv;
  srv.request.set_actuator_state = actuator_state;

  if(set_actuator_state_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void Command_Operator::enable_actuator(void){
  if(!setActuatorState(true))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }
  ROS_INFO("Send actuator state to enable");
}

void Command_Operator::disable_actuator(void){
  if(!setActuatorState(false))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }
  ROS_INFO("Send actuator state to disable");
}

void Command_Operator::move_arm(std::vector<double> command){
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = command[6];
  joint_name.push_back("joint1"); joint_angle.push_back(command[0]);
  joint_name.push_back("joint2"); joint_angle.push_back(command[1]);
  joint_name.push_back("joint3"); joint_angle.push_back(command[2]);
  joint_name.push_back("joint4"); joint_angle.push_back(command[3]);
  joint_name.push_back("joint5"); joint_angle.push_back(command[4]);
  joint_name.push_back("joint6"); joint_angle.push_back(command[5]);

  if(!setJointSpacePath(joint_name, joint_angle, path_time))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }

  ros::Duration(path_time+0.1).sleep();
  ROS_INFO("move_arm done.");
}

void Command_Operator::set_gripper(double joint_angle_val_){
  std::vector<double> joint_angle;
  joint_angle.push_back(joint_angle_val_);
  if(!setToolControl(joint_angle))
  {
    ROS_INFO("[ERR!!] Failed to send service");
    return;
  }
  ROS_INFO("Send gripper value");
  ros::Duration(1.1).sleep();
}

void Command_Operator::read_joint_angle(){
  std::vector<double> joint_angle = getPresentJointAngle();

  int joint_size = 6;
  if (getWithGripperState()) joint_size = 7;

  if(joint_angle.size() != joint_size)
    return;

  ROS_INFO("Joints positions for %s: %f, %f, %f, %f, %f, %f", arm_config_.c_str()
                                                      ,joint_angle.at(0)
                                                      ,joint_angle.at(1)
                                                      ,joint_angle.at(2)
                                                      ,joint_angle.at(3)
                                                      ,joint_angle.at(4)
                                                      ,joint_angle.at(5));
}

int main(int argc, char** argv)
{
  // Setup node and start AsyncSpinner
  const std::string node_name = "kitchen_script";
  ros::init(argc, argv, node_name);
  ros::NodeHandle node_handle("");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string arm_config = argv[1];
  Command_Operator cmd_op(argc, argv);
  cmd_op.init();

  //****************************************************************************
  /* Kitchen Demo
  *  Arm move to named joint angles (command):
  *    move_arm(command)
  * 
  *  Gripper move to generic angle:
  *    set_gripper (angle)
  */
  ROS_INFO("Starting Kitchen Demo");

  // Load list of moves from ROS Parameter Server
  std::vector<std::string> moves_list;
  node_handle.getParam("/kitchen_moves", moves_list);
  
  // Enable robot actuators
  cmd_op.setActuatorState(true);
  ros::Duration(1.0).sleep();

  for (std::vector<std::string>::iterator it = std::begin(moves_list); it != std::end(moves_list); ++it)
  {
    // String parsing
    std::string move_name = *it;
    std::string control_group = move_name.substr(0, move_name.find("/"));
    std::string pose_name = move_name.substr(move_name.find("/")+1);
    ROS_INFO("Move found: %s, control_group: %s, pose_name: %s", move_name.c_str(), control_group.c_str(), pose_name.c_str());

    // Get command joint angles and path time
    std::vector<double> command;
    node_handle.getParam("/"+move_name, command);

    // Do appropriate move
    if (control_group == "kitchen_arm") {
      cmd_op.move_arm(command);
    } else if (control_group == "kitchen_gripper") {
      cmd_op.set_gripper(command[0]);
    } else {} 
  }

  // Disable robot actuators
  cmd_op.setActuatorState(false);

  // Kill ROS node
  ROS_INFO("Done.");
  ros::shutdown();
  return 0;
}