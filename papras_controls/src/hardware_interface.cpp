#include "papras_hw/hardware_interface.h"


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

    /************************************************************
     ** Register Interfaces
    ************************************************************/
    registerActuatorInterfaces();
    registerControlInterfaces();
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

    result = initControlItems();
    if (result == false)
    {
        ROS_ERROR("Please check control items");
        return;
    }

    result = initDynamixels();
    if (result == false)
    {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
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

        if (PH_model_numbers.find(model_number) != PH_model_numbers.end())
        {
            indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"] = ADDR_INDADDR_READ_PH54;
            indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_WRITE"] = ADDR_INDADDR_WRITE_PH54;
        }
        else if (XM_model_numbers.find(model_number) != XM_model_numbers.end()) 
        {
            indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"] = ADDR_INDADDR_READ_XM;
            indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_WRITE"] = ADDR_INDADDR_WRITE_XM;
        }
        else 
            std::cout << "Dxl model number not found" << std::endl;
    }

    return result;
}

bool HardwareInterface::initControlItems(void)
{
    bool result = false;
    const char *log = NULL;

    for (auto const &dxl : dynamixel_)
    {
        const ControlItem *goal_position = dxl_wb_->getItemInfo(dxl.second, "Goal_Position");
        if (goal_position == NULL)
            return false;

        const ControlItem *present_position = dxl_wb_->getItemInfo(dxl.second, "Present_Position");
        if (present_position == NULL)
            return false;

        const ControlItem *present_velocity = dxl_wb_->getItemInfo(dxl.second, "Present_Velocity");
        if (present_velocity == NULL)
            present_velocity = dxl_wb_->getItemInfo(dxl.second, "Present_Speed");
        if (present_velocity == NULL)
            return false;

        const ControlItem *present_current = dxl_wb_->getItemInfo(dxl.second, "Present_Current");
        if (present_current == NULL)
            present_current = dxl_wb_->getItemInfo(dxl.second, "Present_Load");
        if (present_current == NULL)
            return false;

        control_items_[std::to_string(dxl.second)]["Goal_Position"] = goal_position;
        control_items_[std::to_string(dxl.second)]["Present_Position"] = present_position;
        control_items_[std::to_string(dxl.second)]["Present_Velocity"] = present_velocity;
        control_items_[std::to_string(dxl.second)]["Present_Current"] = present_current;
        // ROS_INFO_STREAM("goal_position->address: " << std::to_string(goal_position->address));
        // ROS_INFO_STREAM("present_position->address: " << std::to_string(present_position->address));
        // ROS_INFO_STREAM("present_velocity->address: " << std::to_string(present_velocity->address));
        // ROS_INFO_STREAM("present_current->address: " << std::to_string(present_current->address));
    }

    return true;
}

bool HardwareInterface::initDynamixels(void)
{
    const char *log;

    /* Indirect address setup
        - Each address is 2 bytes or 16 bits (unint16_t)
        - 
    */
    for (auto const &dxl : dynamixel_)
    {
        dxl_wb_->torqueOff((uint8_t)dxl.second);
        ROS_INFO_STREAM("motor id: " << std::to_string(dxl.second));
        ROS_INFO_STREAM("indirect addr to read data: " << std::to_string(indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"]));
        ROS_INFO_STREAM("original present position addr: " << std::to_string(control_items_[std::to_string(dxl.second)]["Present_Position"]->address));
        ROS_INFO_STREAM("");

        // Indirect Address Setup
        for (int j = 0; j < LEN_READ_POS; j++)
        {
            uint16_t data = control_items_[std::to_string(dxl.second)]["Present_Position"]->address + j;
            uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
            uint16_t address = indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"] + 2*j;
            
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
            uint16_t data = control_items_[std::to_string(dxl.second)]["Present_Velocity"]->address + j;
            uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
            uint16_t address = indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"]  + 2*LEN_READ_POS + 2*j;

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
            uint16_t data = control_items_[std::to_string(dxl.second)]["Present_Current"]->address + j;
            uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
            uint16_t address = indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_READ"]  + 2*(LEN_READ_POS+LEN_READ_VEL) + 2*j;

            bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
            if (result == false)
            {
                ROS_ERROR("%s", log);
                ROS_ERROR("Failed to write ADDR_INDADDR_READ_CUR for %s, ID : %d", dxl.first.c_str(), dxl.second);
                return false;
            }
        }
        for (int j = 0; j < LEN_IND_WRITE; j++)
        {
            uint16_t data = control_items_[std::to_string(dxl.second)]["Goal_Position"]->address + j;
            uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
            uint16_t address = indaddr_info_[std::to_string(dxl.second)]["ADDR_INDADDR_WRITE"]  + 2*j;

            bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, address, 2, data_write, &log);
            if (result == false)
            {
                ROS_ERROR("%s", log);
                ROS_ERROR("Failed to write ADDR_INDADDR_WRITE for %s, ID : %d", dxl.first.c_str(), dxl.second);
                return false;
            }
        }

        for (auto const &info : dynamixel_info_)
        { 
            if (dxl.first == info.first && info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
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

    // Torque On after setting up all servo
    for (auto const &dxl : dynamixel_)
        dxl_wb_->torqueOn((uint8_t)dxl.second);

    return true;
}

bool HardwareInterface::initSDKHandlers(void)
{  
    bool result = false;
    const char *log = NULL;

    auto it = dynamixel_.begin();

    result = dxl_wb_->addSyncWriteHandler(ADDR_INDDATA_WRITE, 
                                            LEN_IND_WRITE, 
                                            &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        return result;
    }

    result = dxl_wb_->addSyncReadHandler(ADDR_INDDATA_READ,
                                            LEN_IND_READ,
                                            &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
        return result;
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
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void HardwareInterface::read()
{
    bool result = false;
    const char *log = NULL;

    int32_t get_position[dynamixel_.size()];
    int32_t get_velocity[dynamixel_.size()];
    int32_t get_current[dynamixel_.size()];

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
                                        ADDR_INDDATA_READ,
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
                                        ADDR_INDDATA_READ + LEN_READ_POS,
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
                                        ADDR_INDDATA_READ + LEN_READ_POS + LEN_READ_VEL,
                                        LEN_READ_CUR,
                                        get_current,
                                        &log);
    if (result == false)
    {
        ROS_ERROR("%s", log);
    }

    for (uint8_t index = 0; index < id_cnt; index++)
    {

        joints_[id_array[index] - 1].position = dxl_wb_->convertValue2Radian((uint8_t)id_array[index], (int32_t)get_position[index]);
        joints_[id_array[index] - 1].velocity = dxl_wb_->convertValue2Velocity((uint8_t)id_array[index], (int32_t)get_velocity[index]);
        joints_[id_array[index] - 1].effort = dxl_wb_->convertValue2Current((uint8_t)id_array[index], (int16_t)get_current[index]);
        
        // ROS_INFO_STREAM("get_position: " << (joints_[id_array[index] - 1].position) << " idx: " << (int)(id_array[index]));
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

    for (auto const &dxl : dynamixel_)
    {
        id_array[id_cnt] = (uint8_t)dxl.second;
        // joints_[(uint8_t)dxl.second - 1].position_command = (double) 1.57;
        // ROS_INFO_STREAM("pos cmd dxl val" << std::to_string(dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position_command)));
        dynamixel_position[id_cnt] = dxl_wb_->convertRadian2Value((uint8_t)dxl.second, joints_[(uint8_t)dxl.second - 1].position_command);
        id_cnt++;
    }

    uint8_t sync_write_handler = 0; // 0: position, 1: velocity, 2: effort
    result = dxl_wb_->syncWrite(sync_write_handler, id_array, id_cnt, dynamixel_position, 1, &log);

    if (result == false)
    {
        ROS_ERROR("%s", log);
    }

}
