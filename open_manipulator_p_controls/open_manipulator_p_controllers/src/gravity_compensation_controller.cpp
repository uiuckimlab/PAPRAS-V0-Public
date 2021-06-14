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

#include "open_manipulator_p_controllers/gravity_compensation_controller.h"

namespace open_manipulator_p_controllers
{
bool GravityCompensationController::init(
    hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  // Joint interface
  effort_joint_interface_ =
      robot_hardware->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface_ == nullptr)
  {
    ROS_ERROR(
        "[GravityCompensationController] Could not get effort joint interface "
        "from hardware!");
    return false;
  }

  // Joint handle
  if (!node_handle.getParam("joint_names", joint_names_))
  {
    ROS_ERROR("[GravityCompensationController] Could not parse joint names");
    return false;
  }
  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    try
    {
      effort_joint_handles_.push_back(
          effort_joint_interface_->getHandle(joint_names_[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "[GravityCompensationController] Could not get joint handle: "
          << e.what());
      return false;
    }
  }

  // KDL
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("[GravityCompensationController] Could not parse urdf file");
    return false;
  }
  if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
  {
    ROS_ERROR("[GravityCompensationController] Could not construct kdl tree");
    return false;
  }
  if (!node_handle.getParam("root_link", root_name_))
  {
    ROS_ERROR("[GravityCompensationController] Could not find root link name");
    return false;
  }
  if (!node_handle.getParam("tip_link", tip_name_))
  {
    ROS_ERROR("[GravityCompensationController] Could not find tip link name");
    return false;
  }
  if (!kdl_tree_.getChain(root_name_, tip_name_, kdl_chain_))
  {
    ROS_ERROR(
        "[GravityCompensationController] Could not get KDL chain from tree");
    return false;
  }

  // Resize the variables
  q_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  G_.resize(kdl_chain_.getNrOfJoints());

  // Gravity torque
  KDL::Vector g(0.0, 0.0, -9.81);
  grav_ = g;
  MCG_solver_.reset(new KDL::ChainDynParam(kdl_chain_, grav_));

  return true;
}

void GravityCompensationController::starting(const ros::Time& time) {}

void GravityCompensationController::update(const ros::Time& time,
                                           const ros::Duration& period)
{
  // Get the current joint positions
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
  {
    q_(i) = effort_joint_handles_[i].getPosition();
  }

  // Compute the gravity torque
  MCG_solver_->JntToGravity(q_, G_);

  for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
  {
    tau_(i) = 0;
    tau_(i) += G_(i);
  }

  // Set effort command
  for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); i++)
  {
    effort_joint_handles_[i].setCommand(tau_(i));
  }
}

void GravityCompensationController::stopping(const ros::Time& time) {}

}  // namespace open_manipulator_p_controllers

PLUGINLIB_EXPORT_CLASS(
    open_manipulator_p_controllers::GravityCompensationController,
    controller_interface::ControllerBase)
