/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#ifndef ROBOTIS_MANIPULATOR_H_
#define ROBOTIS_MANIPULATOR_H_

#include "robotis_manipulator_common.h"
#include "robotis_manipulator_manager.h"
#include "robotis_manipulator_trajectory_generator.h"
#include "robotis_manipulator_math.h"
#include "robotis_manipulator_log.h"

#include <algorithm>

namespace robotis_manipulator
{

#define DYNAMICS_ALL_SOVING 0
#define DYNAMICS_GRAVITY_ONLY 1
#define DYNAMICS_NOT_SOVING 2

class RobotisManipulator
{
private:
  Manipulator manipulator_;
  Trajectory trajectory_;
  Kinematics *kinematics_;
  Dynamics *dynamics_;
  std::map<Name, JointActuator *> joint_actuator_;
  std::map<Name, ToolActuator *> tool_actuator_;

  bool trajectory_initialized_state_;
  bool moving_state_;
  bool moving_fail_flag_;
  bool step_moving_state_;

  bool joint_actuator_added_stete_;
  bool tool_actuator_added_stete_;
  bool kinematics_added_state_;
  bool dynamics_added_state_;

private:
  void startMoving();
  JointWaypoint getTrajectoryJointValue(double tick_time, int option=0);

public:
  RobotisManipulator();
  virtual ~RobotisManipulator();


  /*****************************************************************************
  ** Initialize Function
  *****************************************************************************/
  void addWorld(Name world_name,
                Name child_name,
                Eigen::Vector3d world_position = Eigen::Vector3d::Zero(),
                Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

  void addJoint(Name my_name,
                Name parent_name,
                Name child_name,
                Eigen::Vector3d relative_position,
                Eigen::Matrix3d relative_orientation,
                Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                int8_t joint_actuator_id = -1, 
                double max_position_limit = M_PI, 
                double min_position_limit = -M_PI,
                double coefficient = 1.0,
                double mass = 0.0,
                Eigen::Matrix3d inertia_tensor = Eigen::Matrix3d::Identity(),
                Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero(),
                double torque_coefficient = 1.0);

  void addTool(Name my_name,
               Name parent_name,
               Eigen::Vector3d relative_position,
               Eigen::Matrix3d relative_orientation,
               int8_t tool_id = -1, 
               double max_position_limit =M_PI, 
               double min_position_limit = -M_PI,
               double coefficient = 1.0,
               double object_mass = 0.0,
               Eigen::Matrix3d object_inertia_tensor = Eigen::Matrix3d::Identity(),
               Eigen::Vector3d object_center_of_mass = Eigen::Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void printManipulatorSetting();

  void addKinematics(Kinematics *kinematics);
  void addDynamics(Dynamics *dynamics);
  void addJointActuator(Name actuator_name, JointActuator *joint_actuator, std::vector<uint8_t> id_array, const void *arg);
  void addToolActuator(Name tool_name, ToolActuator *tool_actuator, uint8_t id, const void *arg);
  void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
  void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);

  /*****************************************************************************
  ** Manipulator Function
  *****************************************************************************/
  Manipulator *getManipulator();

  void setTorqueCoefficient(Name component_name, double torque_coefficient);

  JointValue getJointValue(Name joint_name);
  JointValue getToolValue(Name tool_name);
  std::vector<JointValue> getAllActiveJointValue();
  std::vector<JointValue> getAllJointValue();
  std::vector<double> getAllToolPosition();
  std::vector<JointValue> getAllToolValue();
  KinematicPose getKinematicPose(Name component_name);
  DynamicPose getDynamicPose(Name component_name);
  Pose getPose(Name component_name);

  /*****************************************************************************
  ** Kinematics Function (Including Virtual Function)
  *****************************************************************************/
  Kinematics *getKinematics();
  Eigen::MatrixXd jacobian(Name tool_name);
  void solveForwardKinematics();
  void solveForwardKinematics(std::vector<JointValue> *goal_joint_value);
  bool solveInverseKinematics(Name tool_name, Pose goal_pose, std::vector<JointValue> *goal_joint_value);
  void setKinematicsOption(const void* arg);

  /*****************************************************************************
  ** Dynamics Function (Including Virtual Function)
  *****************************************************************************/
  Dynamics *getDynamics();
  void solveForwardDynamics(std::map<Name, double> joint_torque);
  bool solveInverseDynamics(std::map<Name, double> *joint_torque);
  bool solveGravityTerm(std::map<Name, double> *joint_torque);
  void setDynamicsOption(STRING param_name, const void* arg);
  void setDynamicsEnvironments(STRING param_name, const void* arg);

  /*****************************************************************************
  ** Actuator Function (Including Virtual Function)
  *****************************************************************************/
  JointActuator *getJointActuator(Name actuator_name);
  ToolActuator *getToolActuator(Name actuator_name);
  void setJointActuatorMode(Name actuator_name, std::vector<uint8_t> id_array, const void *arg);
  void setToolActuatorMode(Name actuator_name, const void *arg);
  std::vector<uint8_t> getJointActuatorId(Name actuator_name);
  uint8_t getToolActuatorId(Name actuator_name);
  void enableActuator(Name actuator_name);
  void disableActuator(Name actuator_name);
  void enableAllJointActuator();
  void disableAllJointActuator();
  void enableAllToolActuator();
  void disableAllToolActuator();
  void enableAllActuator();
  void disableAllActuator();
  bool getActuatorEnabledState(Name actuator_name);

  bool sendJointActuatorValue(Name joint_component_name, JointValue value);
  bool sendMultipleJointActuatorValue(std::vector<Name> joint_component_name, std::vector<JointValue> value_vector);
  bool sendAllJointActuatorValue(std::vector<JointValue> value_vector);
  JointValue receiveJointActuatorValue(Name joint_component_name);
  std::vector<JointValue> receiveMultipleJointActuatorValue(std::vector<Name> joint_component_name);
  std::vector<JointValue> receiveAllJointActuatorValue();

  bool sendToolActuatorValue(Name tool_component_name, JointValue value);
  bool sendMultipleToolActuatorValue(std::vector<Name> tool_component_name, std::vector<JointValue> value_vector);
  bool sendAllToolActuatorValue(std::vector<JointValue> value_vector);
  JointValue receiveToolActuatorValue(Name tool_component_name);
  std::vector<JointValue> receiveMultipleToolActuatorValue(std::vector<Name> tool_component_name);
  std::vector<JointValue> receiveAllToolActuatorValue();

  /*****************************************************************************
  ** Time Function
  *****************************************************************************/
  double getTrajectoryMoveTime();
  bool getMovingState();

  /*****************************************************************************
  ** Check Joint Limit Function
  *****************************************************************************/
  bool checkJointLimit(Name component_name, double position);
  bool checkJointLimit(Name component_name, JointValue value);
  bool checkJointLimit(std::vector<Name> component_name, std::vector<double> position_vector);
  bool checkJointLimit(std::vector<Name> component_name, std::vector<JointValue> value_vector);

  /*****************************************************************************
  ** Trajectory Control Fuction
  *****************************************************************************/
  Trajectory *getTrajectory();
  /**
   * @brief makeJointTrajectoryFromPresentPosition
   * @param delta_goal_joint_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectoryFromPresentPosition(std::vector<double> delta_goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param goal_joint_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(std::vector<double> goal_joint_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param goal_joint_value
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(std::vector<JointValue> goal_joint_value, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_orientation
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeJointTrajectory
   * @param tool_name
   * @param goal_pose
   * @param move_time
   * @param present_joint_value
   */
  bool makeJointTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param position_meter
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Vector3d position_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param orientation_meter
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, Eigen::Matrix3d orientation_meter, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectoryFromPresentPose
   * @param tool_name
   * @param goal_pose_delta
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectoryFromPresentPose(Name tool_name, KinematicPose goal_pose_delta, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_position
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, Eigen::Vector3d goal_position, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_orientation
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, Eigen::Matrix3d goal_orientation, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeTaskTrajectory
   * @param tool_name
   * @param goal_pose
   * @param move_time
   * @param present_joint_value
   */
  bool makeTaskTrajectory(Name tool_name, KinematicPose goal_pose, double move_time, std::vector<JointValue> present_joint_value = {});

  /**
   * @brief setCustomTrajectoryOption
   * @param trajectory_name
   * @param arg
   */
  void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param tool_name
   * @param arg
   * @param move_time
   * @param present_joint_value
   */
  bool makeCustomTrajectory(Name trajectory_name, Name tool_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeCustomTrajectory
   * @param trajectory_name
   * @param arg
   * @param move_time
   * @param present_joint_value
   */
  bool makeCustomTrajectory(Name trajectory_name, const void *arg, double move_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief sleepTrajectory
   * @param wait_time
   * @param present_joint_value
   */
  bool sleepTrajectory(double wait_time, std::vector<JointValue> present_joint_value = {});
  /**
   * @brief makeToolTrajectory
   * @param tool_name
   * @param tool_goal_position
   */
  bool makeToolTrajectory(Name tool_name, double tool_goal_position);
  /**
   * @brief getJointGoalValueFromTrajectory
   * @param present_time
   * @return
   */
  std::vector<JointValue> getJointGoalValueFromTrajectory(double present_time, int option=DYNAMICS_ALL_SOVING);
  /**
   * @brief getToolGoalValue
   * @return
   */
  std::vector<JointValue> getToolGoalValue();
  std::vector<JointValue> getJointGoalValueFromTrajectoryTickTime(double tick_time);

  void stopMoving();
  bool getMovingFailState();
  void resetMovingFailState();
};
} // namespace ROBOTIS_MANIPULATOR

#endif
