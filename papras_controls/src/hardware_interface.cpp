#include "papras_hw/hardware_interface.h"

namespace open_manipulator_p_hw
{
  HardwareInterface::HardwareInterface(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : node_handle_(nh),
        priv_node_handle_(private_nh)
  {
    /************************************************************
  ** Initialize ROS parameters
  ************************************************************/
    port_name_ = priv_node_handle_.param<std::string>("usb_port", "/dev/ttyUSB0");
    baud_rate_ = priv_node_handle_.param<int32_t>("baud_rate", 1000000);
    yaml_file_ = priv_node_handle_.param<std::string>("yaml_file", "");
    interface_ = priv_node_handle_.param<std::string>("interface", "position");

    pub_switch_controller_ = nh.advertise<std_msgs::Int32>("switch_controller",10);
    joint_ids_ = {};

    /************************************************************
  ** Register Interfaces
  ************************************************************/
    registerActuatorInterfaces();
    registerControlInterfaces();

    isMotorsMissing = false;
    isTorqueOn = false;
    controlLoopCnt = 0;
    droppedPackets = 0;
  }

  void HardwareInterface::registerActuatorInterfaces()
  {
    dxl_wb_ = new DynamixelWorkbench;
    bool result = false;

    result = initWorkbench(port_name_, baud_rate_);
    if (result == false)
    {
      ROS_ERROR("Please check USB port name");
      return;
    }

    result = getDynamixelsInfo(yaml_file_);
    if (result == false)
    {
      ROS_ERROR("Please check YAML file");
      return;
    }

    result = loadDynamixels();
    if (result == false)
    {
      ROS_ERROR("Please check Dynamixel ID or BaudRate");
      return;
    }

    result = initDynamixels();
    if (result == false)
    {
      ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
      return;
    }

    result = initControlItems();
    if (result == false)
    {
      ROS_ERROR("Please check control items");
      return;
    }

    result = initSDKHandlers();
    if (result == false)
    {
      ROS_ERROR("Failed to set Dynamixel SDK Handler");
      return;
    }
  }

  bool HardwareInterface::initWorkbench(const std::string port_name, const uint32_t baud_rate)
  {
    bool result = false;
    const char *log;

    result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    return result;
  }

  bool HardwareInterface::getDynamixelsInfo(const std::string yaml_file)
  {
    YAML::Node dynamixel;
    dynamixel = YAML::LoadFile(yaml_file.c_str());

    if (dynamixel == NULL)
      return false;

    for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
    {
      std::string name = it_file->first.as<std::string>();
      if (name.size() == 0)
      {
        continue;
      }

      YAML::Node item = dynamixel[name];
      for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
      {
        std::string item_name = it_item->first.as<std::string>();
        int32_t value = it_item->second.as<int32_t>();

        if (item_name == "ID"){
          dynamixel_[name] = value;
          joint_ids_.push_back((int)value);
        }
          

        ItemValue item_value = {item_name, value};
        std::pair<std::string, ItemValue> info(name, item_value);

        dynamixel_info_.push_back(info);
      }
    }

    return true;
  }

  bool HardwareInterface::loadDynamixels(void)
  {
    bool result = false;
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
      uint16_t model_number = 0;
      result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
        ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
        return result;
      }
      else
      {
        ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
      }
    }

    return result;
  }

  bool HardwareInterface::initDynamixels(void)
  {
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
      dxl_wb_->torqueOff((uint8_t)dxl.second);
      isTorqueOn = false;

      // Indirect Address Setup
      for (int j = 0; j < LEN_READ_POS; j++)
      {
        // write data (580) to address (168) 
        uint16_t data = ADDR_READ_POS + j;
        uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
        uint16_t address = ADDR_INDADDR_READ_POS + 2 * j;

        bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write ADDR_INDADDR_READ_POS for %s, ID : %d", dxl.first.c_str(), dxl.second);
          return false;
        }
      }
      for (int j = 0; j < LEN_READ_VEL; j++)
      {
        // write data () to address () 
        uint16_t data = ADDR_READ_VEL + j;
        uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
        uint16_t address = ADDR_INDADDR_READ_VEL + 2 * j;

        bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write ADDR_INDADDR_READ_VEL for %s, ID : %d", dxl.first.c_str(), dxl.second);
          return false;
        }
      }
      for (int j = 0; j < LEN_READ_CUR; j++)
      {
        // write data () to address () 
        uint16_t data = ADDR_READ_CUR + j;
        uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
        uint16_t address = ADDR_INDADDR_READ_CUR + 2 * j;

        bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write ADDR_INDADDR_READ_CUR for %s, ID : %d", dxl.first.c_str(), dxl.second);
          return false;
        }
      }
      for (int j = 0; j < LEN_READ_MOV; j++)
      {
        // write data () to address () 
        uint16_t data = ADDR_READ_MOV + j;
        uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
        uint16_t address = ADDR_INDADDR_READ_MOV + 2 * j;

        bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write ADDR_INDADDR_READ_MOV for %s, ID : %d", dxl.first.c_str(), dxl.second);
          return false;
        }
      }
      ROS_INFO("Indirect Address setup successful for %s, ID : %d", dxl.first.c_str(), dxl.second);

      for (auto const &info : dynamixel_info_)
      { 
        if (dxl.first == info.first)
        {
          if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
          {
            bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
            if (result == false)
            {
              ROS_ERROR("%s", log);
              ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
              return false;
            }
          }
        }
      }
    }

    // Torque On after setting up all servo
    for (auto const &dxl : dynamixel_)
      dxl_wb_->torqueOn((uint8_t)dxl.second);
    isTorqueOn = true;

    return true;
  }

  bool HardwareInterface::initControlItems(void)
  {
    bool result = false;
    const char *log = NULL;

    auto it = dynamixel_.begin();

    const ControlItem *goal_position = dxl_wb_->getItemInfo(it->second, "Goal_Position");
    if (goal_position == NULL)
      return false;

    const ControlItem *goal_velocity = dxl_wb_->getItemInfo(it->second, "Goal_Velocity");
    if (goal_velocity == NULL)
      goal_velocity = dxl_wb_->getItemInfo(it->second, "Moving_Speed");
    if (goal_velocity == NULL)
      return false;

    const ControlItem *goal_current = dxl_wb_->getItemInfo(it->second, "Goal_Current");
    if (goal_current == NULL)
      goal_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    if (goal_current == NULL)
      return false;

    const ControlItem *present_position = dxl_wb_->getItemInfo(it->second, "Present_Position");
    if (present_position == NULL)
      return false;

    const ControlItem *present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Velocity");
    if (present_velocity == NULL)
      present_velocity = dxl_wb_->getItemInfo(it->second, "Present_Speed");
    if (present_velocity == NULL)
      return false;

    const ControlItem *present_current = dxl_wb_->getItemInfo(it->second, "Present_Current");
    if (present_current == NULL)
      present_current = dxl_wb_->getItemInfo(it->second, "Present_Load");
    if (present_current == NULL)
      return false;

    control_items_["Goal_Position"] = goal_position;
    control_items_["Goal_Velocity"] = goal_velocity;
    control_items_["Goal_Current"] = goal_current;

    control_items_["Present_Position"] = present_position;
    control_items_["Present_Velocity"] = present_velocity;
    control_items_["Present_Current"] = present_current;

    return true;
  }

  bool HardwareInterface::initSDKHandlers(void)
  {
    bool result = false;
    const char *log = NULL;

    auto it = dynamixel_.begin();

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Position"]->address, control_items_["Goal_Position"]->data_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
    else
    {
      ROS_INFO("%s", log);
    }

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Velocity"]->address, control_items_["Goal_Velocity"]->data_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
    else
    {
      ROS_INFO("%s", log);
    }

    result = dxl_wb_->addSyncWriteHandler(control_items_["Goal_Current"]->address, control_items_["Goal_Current"]->data_length, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
    else
    {
      ROS_INFO("%s", log);
    }

    if (dxl_wb_->getProtocolVersion() == 2.0f)
    {
      uint16_t start_address = std::min(control_items_["Present_Position"]->address, control_items_["Present_Current"]->address);

      /* As some models have an empty space between Present_Velocity and Present Current, read_length is modified as below.*/
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length;
      // uint16_t read_length = control_items_["Present_Position"]->data_length + control_items_["Present_Velocity"]->data_length + control_items_["Present_Current"]->data_length + 2;

      result = dxl_wb_->addSyncReadHandler(ADDR_INDDATA_READ_POS,
                                           LEN_IND_READ,
                                           &log);
      if (result == false)
      {
        ROS_ERROR("%s", log);
        return result;
      }
    }

    return result;
  }

  void HardwareInterface::registerControlInterfaces()
  {
    // resize vector
    uint8_t joint_size = 0;
    for (auto const &dxl : dynamixel_)
    {
      if (joint_size < (uint8_t)dxl.second)
        joint_size = (uint8_t)dxl.second;
    }
    joints_.resize(joint_size);

    for (auto iter = dynamixel_.begin(); iter != dynamixel_.end(); iter++)
    {
      // initialize joint vector
      Joint joint;
      joints_[iter->second - 1] = joint;
      ROS_INFO("joint_name : %s, servo ID: %d", iter->first.c_str(), iter->second);

      // connect and register the joint state interface
      hardware_interface::JointStateHandle joint_state_handle(iter->first.c_str(),
                                                              &joints_[iter->second - 1].position,
                                                              &joints_[iter->second - 1].velocity,
                                                              &joints_[iter->second - 1].effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      // connect and register the joint position, velocity and effort interface
      hardware_interface::JointHandle position_joint_handle(joint_state_handle, &joints_[iter->second - 1].position_command);
      position_joint_interface_.registerHandle(position_joint_handle);
      hardware_interface::JointHandle velocity_joint_handle(joint_state_handle, &joints_[iter->second - 1].velocity_command);
      velocity_joint_interface_.registerHandle(velocity_joint_handle);
      hardware_interface::JointHandle effort_joint_handle(joint_state_handle, &joints_[iter->second - 1].effort_command);
      effort_joint_interface_.registerHandle(effort_joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
  }

  bool HardwareInterface::motorsStopped()
  {
    double position_command;
    double position_current;

    for (auto const &dxl : dynamixel_)
    {
      position_command = joints_[(uint8_t)dxl.second - 1].position_command;
      position_current = joints_[(uint8_t)dxl.second - 1].position;

      if (position_command != position_current)
        return false;
    }
    return true;
  }

  bool HardwareInterface::checkMotorIDs(void)
  {
    bool result = false;
    const char *log;

    for (auto const &dxl : dynamixel_)
    {
      uint16_t model_number = 0;
      result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
      if (result == false)
      {
        isMotorsMissing = true;
        isTorqueOn = false;
        return result;
      }
      else
      {
        isMotorsMissing = false;
      }
    }

    return result;
  }

  void HardwareInterface::read()
  {
    isMotorsStopped = motorsStopped();
    // every few sec, check motor ids if the motors are not moving
    if (controlLoopCnt % 10 == 0 && (isMotorsMissing || isMotorsStopped))
    {
      checkMotorIDs();
    }

    // exit if motor ids missing
    if (isMotorsMissing)
      return;

    ros::Time start_time = ros::Time::now();
    bool result = false;
    const char *log = NULL;

    int32_t get_position[dynamixel_.size()];
    int32_t get_velocity[dynamixel_.size()];
    int32_t get_current[dynamixel_.size()];
    int32_t get_moving[dynamixel_.size()];

    uint8_t id_array[dynamixel_.size()];
    std::string name_array[dynamixel_.size()];
    uint8_t id_cnt = 0;


    uint8_t sync_read_handler = 0; // 0 for present position, velocity, current
    for (auto const &dxl : dynamixel_)
    {
      name_array[id_cnt] = dxl.first.c_str();
      id_array[id_cnt++] = (uint8_t)dxl.second;
    }

    // Read Present State
    result = dxl_wb_->syncRead(sync_read_handler,
                               id_array,
                               dynamixel_.size(),
                               &log);

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(sync_read_handler,
                                      id_array,
                                      id_cnt,
                                      ADDR_INDDATA_READ_POS,
                                      LEN_READ_POS,
                                      get_position,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(sync_read_handler,
                                      id_array,
                                      id_cnt,
                                      ADDR_INDDATA_READ_VEL,
                                      LEN_READ_VEL,
                                      get_velocity,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(sync_read_handler,
                                      id_array,
                                      id_cnt,
                                      ADDR_INDDATA_READ_CUR,
                                      LEN_READ_CUR,
                                      get_current,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    result = dxl_wb_->getSyncReadData(sync_read_handler,
                                      id_array,
                                      id_cnt,
                                      ADDR_INDDATA_READ_MOV,
                                      LEN_READ_MOV,
                                      get_moving,
                                      &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }

    // Motors Missing after 5 consecutive failed reads
    droppedPackets = (result == false) ? droppedPackets+1 : 0;
    if (droppedPackets > 5) isMotorsMissing = true;


    for (uint8_t index = 0; index < id_cnt; index++)
    {
      // Position
      joints_[id_array[index] - 1].position = dxl_wb_->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]);

    
      // Velocity
      joints_[id_array[index] - 1].velocity = dxl_wb_->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]);


      // Effort
      if (strcmp(dxl_wb_->getModelName((uint8_t)id_array[index]), "XL-320") == 0)
        joints_[id_array[index] - 1].effort = dxl_wb_->convertValue2Load((int16_t)get_current[index]);
      else
        joints_[id_array[index] - 1].effort = dxl_wb_->convertValue2Current((int16_t)get_current[index]) * (1.78e-03);


      // isMoving
      joints_[id_array[index] - 1].isMoving = (bool) get_moving[index];

      joints_[id_array[index] - 1].position_command = joints_[id_array[index] - 1].position;
    }
  }

  void HardwareInterface::write()
  {

    bool result = false;
    const char *log = NULL;

    uint8_t id_array[dynamixel_.size()];
    uint8_t id_cnt = 0;

    int32_t dynamixel_position[dynamixel_.size()];
    int32_t dynamixel_velocity[dynamixel_.size()];
    int32_t dynamixel_effort[dynamixel_.size()];

    // reset cnt if arms missing, exit
    if(isMotorsMissing) {
      controlLoopCnt = 0;
      return;
    }

    if (controlLoopCnt == 11) {
      ROS_INFO_STREAM("arm plugged in");
    }

    if (controlLoopCnt == SWITCH_CONTROLLER_CNT){
      ROS_INFO_STREAM("switching controllers");
      std_msgs::Int32 switch_controller_cnt;
      switch_controller_cnt.data = joint_ids_.size()/NUM_JOINTS_PER_ARM;
      pub_switch_controller_.publish(switch_controller_cnt);
    }

    if (controlLoopCnt < RECONFIG_WAIT){
      return;
    }

    // torqueOn N secs after arm is plugged in
    if (controlLoopCnt == RECONFIG_WAIT){
      bool result = initDynamixels();
      if (result == false)
      {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return;
      }
    } 
    
    id_cnt = 0;

    if (strcmp(interface_.c_str(), "position") == 0)
    {
      for (auto const &dxl : dynamixel_)
      {
        id_array[id_cnt] = (uint8_t)dxl.second;

        // if torqueOn write pos command, if torqueOff write current pos val
        if (isTorqueOn){
          dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position_command);
        } else {
          dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position);
        }

        id_cnt++;
      }
      uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
      result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, dynamixel_position, 1, &log);

      // ROS_INFO_STREAM("position_command: " << joints_[43].position_command);
    }
    else if (strcmp(interface_.c_str(), "effort") == 0)
    {
      // if torqueOn write effort command, if torqueOff write current effort val
      if (isTorqueOn) {
          for (auto const &dxl : dynamixel_)
          {
            id_array[id_cnt] = (uint8_t)dxl.second;
            dynamixel_effort[id_cnt] = dxl_wb_->convertCurrent2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].effort_command / (1.78e-03));

            if (strcmp(dxl.first.c_str(), "gripper") == 0)
              dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position_command * 150.0);
            id_cnt++;
          }
      } else {
          for (auto const &dxl : dynamixel_)
          {
            id_array[id_cnt] = (uint8_t)dxl.second;
            dynamixel_effort[id_cnt] = dxl_wb_->convertCurrent2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].effort / (1.78e-03));

            if (strcmp(dxl.first.c_str(), "gripper") == 0)
              dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position * 150.0);
            id_cnt++;
          }
      }
      
      uint8_t sync_write_handler = 2; // 0: position, 1: velocity, 2: effort
      result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, dynamixel_effort, 1, &log);
    }

    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    
  
  }
} // namespace open_manipulator_p_hw