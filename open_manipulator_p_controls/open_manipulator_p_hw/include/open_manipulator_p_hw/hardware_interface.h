/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#ifndef OPEN_MANIPULATOR_P_HARDWARE_INTERFACE_H
#define OPEN_MANIPULATOR_P_HARDWARE_INTERFACE_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

typedef struct _ItemValue
{
  std::string item_name;
  int32_t value;
} ItemValue;

typedef struct _WayPoint
{
  double position;
  double velocity;
  double acceleration;
} WayPoint;

typedef struct _Joint
{
  double position;
  double velocity;
  double current;
  double effort;
  double position_command;
  double velocity_command;
  double effort_command;
} Joint;

namespace open_manipulator_p_hw
{
class HardwareInterface : public hardware_interface::RobotHW
{
 public:
  HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~HardwareInterface() {}

  void read();
  void write();

 private:
  void registerActuatorInterfaces();
  void registerControlInterfaces();
  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);

  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  std::string port_name_;
  int32_t baud_rate_;
  std::string yaml_file_;
  std::string interface_;

  // Variables
  DynamixelWorkbench *dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  std::vector<Joint> joints_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
};

} // namespace open_manipulator_p_hw
#endif // OPEN_MANIPULATOR_P_HARDWARE_INTERFACE_H
