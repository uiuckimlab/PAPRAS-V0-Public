/* Authors: Kazuki Shin */

#ifndef PAPRS_CONTROLS_H
#define PAPRS_CONTROLS_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <yaml-cpp/yaml.h>
#include <std_msgs/Int32.h>
#include <unordered_set>

#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

// Control table address
// Data Byte Length
#define LEN_INT 2
#define LEN_IND_WRITE 4
#define LEN_READ_POS 4
#define LEN_READ_VEL 4
#define LEN_READ_CUR 2
#define LEN_IND_READ (LEN_READ_POS + LEN_READ_CUR + LEN_READ_VEL)

// Indirect Address Parameters
#define ADDR_INDADDR_READ_PH54 198 // POS:198 VEL:206 CUR:214   
#define ADDR_INDADDR_READ_XM 608 // POS:608 VEL:616 CUR:624
#define ADDR_INDDATA_READ 649
// #define ADDR_INDADDR_WRITE 168
#define ADDR_INDADDR_WRITE_PH54 168
#define ADDR_INDADDR_WRITE_XM 578
#define ADDR_INDDATA_WRITE 634

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

typedef struct _ItemValue
{
  std::string item_name;
  int32_t value;
} ItemValue;

typedef struct _Joint
{
  double position;
  double velocity;
  double effort;
  double position_command;
} Joint;

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

  std::unordered_set<uint16_t> PH_model_numbers {2000, 2010, 2020};
  std::unordered_set<uint16_t> XM_model_numbers {1030, 1020};

  // Variables
  DynamixelWorkbench *dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, std::map<std::string, const ControlItem *>> control_items_;
  std::map<std::string, std::map<std::string, uint16_t>> indaddr_info_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  std::vector<Joint> joints_;
  
  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::PositionJointInterface position_joint_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;

};

#endif // PAPRAS_CONTROLS_H
