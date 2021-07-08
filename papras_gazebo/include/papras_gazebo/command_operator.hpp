/* Authors: Kazuki Shin */

#ifndef COMMAND_OPERATOR
#define COMMAND_OPERATOR

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <eigen3/Eigen/Eigen>

#include "robotis_manipulator/robotis_manipulator.h"

#include "open_manipulator_msgs/OpenManipulatorState.h"
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"

#define NUM_OF_JOINT_AND_TOOL 7

/*****************************************************************************
** Interface [Command Operator]
*****************************************************************************/
class Command_Operator {

public:
	Command_Operator(int argc, char** argv);
	virtual ~Command_Operator();
  bool init();
  int getch();
  void run();

  // from qnode.hpp
  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg);
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPosition();
  Eigen::Quaterniond getPresentKinematicsOrientation();
  Eigen::Vector3d getPresentKinematicsOrientationRPY();
  bool getOpenManipulatorMovingState();
  bool getOpenManipulatorActuatorState();
  bool getWithGripperState();

  void setOption(std::string opt);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time);
  bool setDrawingTrajectory(std::string name, std::vector<double> arg, double path_time);
  bool setToolControl(std::vector<double> joint_angle);
  bool setActuatorState(bool actuator_state);

  // from main_window.hpp
  void enable_actuator(void);
  void disable_actuator(void);
  void move_arm(std::vector<double> waypoint_joint_angle_, double path_time_);
  void set_gripper(double joint_angle_val_);
  void read_joint_angle(void);

private:
  int init_argc;
	char** init_argv;

  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  bool with_gripper_;
  std::string arm_config_;

  // ROS Publishers
  ros::Publisher open_manipulator_option_pub_;

  // ROS Subscribers
  ros::Subscriber open_manipulator_states_sub_;
  ros::Subscriber open_manipulator_joint_states_sub_;
  ros::Subscriber open_manipulator_kinematics_pose_sub_;

  // ROS Service Clients
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_task_space_path_position_only_client_;
  ros::ServiceClient goal_task_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;
  ros::ServiceClient set_actuator_state_client_;
  ros::ServiceClient goal_drawing_trajectory_client_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematics_position_;
  Eigen::Quaterniond present_kinematics_orientation_;
  Eigen::Vector3d present_kinematics_orientation_rpy_;
  open_manipulator_msgs::KinematicsPose kinematics_pose_;

  bool open_manipulator_is_moving_;
  bool open_manipulator_actuator_enabled_;
};

#endif // COMMAND_OPERATOR
