/* Authors: Kazuki Shin */

#ifndef PAPRS_CONTROLS_H
#define PAPRS_CONTROLS_H

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include "controller_manager_msgs/SwitchController.h"
#include "std_msgs/Int32.h"

#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"

// Control table address
// Control table address is different in Dynamixel model
#define ADDR_Baudrate 8
#define ADDR_Rtrndelay 9
#define ADDR_Drive_Mode 11
#define ADDR_Oper_Mode 11
#define ADDR_PWM_LIMIT 36
#define ADDR_CUR_LIMIT 38
#define ADDR_VEL_LIMIT 44
#define ADDR_ACC_LIMIT 40
#define ADDR_TORQUE_EN 512
#define ADDR_LED_R 513
#define ADDR_LED_G 514
#define ADDR_LED_B 515
#define ADDR_POS_D 528
#define ADDR_POS_I 530
#define ADDR_POS_P 532
#define ADDR_POS_FF1 538
#define ADDR_VEL_I 524
#define ADDR_VEL_P 526
#define ADDR_VEL_FF2 536
#define ADDR_GOAL_VEL 552
#define ADDR_PROFILE_ACCL 556
#define ADDR_PROFILE_VEL 560
#define ADDR_GOAL_POS 564
#define ADDR_READ_MOV 570
#define ADDR_READ_PWM 572
#define ADDR_READ_CUR 574
#define ADDR_READ_VEL 576
#define ADDR_READ_POS 580
#define ADDR_SHUTDOWN 63

// Data Byte Length
#define LEN_GOAL_POS 4
#define LEN_READ_POS 4
#define LEN_READ_VEL 4
#define LEN_READ_CUR 2
#define LEN_READ_MOV 1
#define LEN_INT 2
#define LEN_IND_READ (LEN_READ_POS + LEN_READ_CUR + LEN_READ_VEL + LEN_READ_MOV)

// Indirect Address Parameters
#define ADDR_INDADDR_READ_POS 168
#define ADDR_INDADDR_READ_VEL (ADDR_INDADDR_READ_POS + 2 * LEN_READ_POS)
#define ADDR_INDADDR_READ_CUR (ADDR_INDADDR_READ_VEL + 2 * LEN_READ_VEL)
#define ADDR_INDADDR_READ_MOV (ADDR_INDADDR_READ_CUR + 2 * LEN_READ_CUR)

#define ADDR_INDDATA_READ_POS 634
#define ADDR_INDDATA_READ_VEL (ADDR_INDDATA_READ_POS + LEN_READ_POS)
#define ADDR_INDDATA_READ_CUR (ADDR_INDDATA_READ_VEL + LEN_READ_VEL)
#define ADDR_INDDATA_READ_MOV (ADDR_INDDATA_READ_CUR + LEN_READ_CUR)

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

#define RECONFIG_WAIT 200
#define SWITCH_CONTROLLER_CNT RECONFIG_WAIT-10
#define NUM_JOINTS_PER_ARM 7


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
  bool isMoving;

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
    int controlLoopCnt;
    int droppedPackets;

  private:
    void registerActuatorInterfaces();
    void registerControlInterfaces();
    bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
    bool getDynamixelsInfo(const std::string yaml_file);
    bool loadDynamixels(void);
    bool initDynamixels(void);
    bool initControlItems(void);
    bool initSDKHandlers(void);

    bool checkMotorIDs();
    bool motorsStopped();

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
    std::map<std::string, const ControlItem *> control_items_;
    std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
    std::vector<Joint> joints_;
    std::vector<int> joint_ids_;

    
    bool isMotorsMissing;
    bool isTorqueOn;
    bool isMotorsStopped;

    // ROS Control interfaces
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    ros::Publisher pub_switch_controller_;
  };

} // namespace papras_controls
#endif // PAPRAS_CONTROLS_H