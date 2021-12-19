#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import yaml

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 512               # Control table address is different in Dynamixel model, change to 
ADDR_PRO_GOAL_POSITION      = 564
ADDR_PRO_PRESENT_POSITION   = 580

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
# DEVICENAME                  = '/dev/ttyUSB2'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
DEVICE_NAMES                = ['/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2']

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -30           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 30            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

def scan_port(packetHandler,portHandler, device_name):
    valid_ids = []
    gripper_ids = [111,121,131]

    for dxl_id in range(1,19):
        if enable_dynamixel_torque(packetHandler,portHandler, dxl_id):
            valid_ids.append(dxl_id)
    
    for dxl_id in gripper_ids:
        if enable_dynamixel_torque(packetHandler,portHandler, dxl_id):
            valid_ids.append(dxl_id)

    print("Found ids: ")
    print(valid_ids)
    classify_arm_config(valid_ids, device_name)
    return valid_ids

def enable_dynamixel_torque(packetHandler,portHandler, dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return False
    else:
        print("Dynamixel has been successfully connected")
        disable_dynamixel_torque(packetHandler, portHandler, dxl_id)
        return True

def disable_dynamixel_torque(packetHandler,portHandler, dxl_id):
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler,dxl_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def classify_arm_config(valid_ids, device_name):
    if min(valid_ids) == 1 and max(valid_ids) == 6:
        print("Classified as arm1 without gripper")
    elif min(valid_ids) == 1 and max(valid_ids) == 111:
        print("Classified as arm1 with gripper")
    elif min(valid_ids) == 7 and max(valid_ids) == 12:
        print("Classified as arm2 without gripper")
    elif min(valid_ids) == 7 and max(valid_ids) == 121:
        print("Classified as arm2 with gripper")
    elif min(valid_ids) == 13 and max(valid_ids) == 18:
        print("Classified as arm3 without gripper")
    elif min(valid_ids) == 13 and max(valid_ids) == 131:
        print("Classified as arm3 with gripper")
    else:
        print("Could not classify joints")

    if device_name == '/dev/ttyUSB0':
        robot_name = 'robot1'
    elif device_name =='/dev/ttyUSB1':
        robot_name = 'robot2'
    else:
        robot_name = 'robot3'

    if len(valid_ids) == 6:
        file_name = 'hardware.yaml'
    elif len(valid_ids):
        file_name = 'hardware_with_gripper.yaml'
    else:
        print("Missing joints!")
        return 

    for joint_id in valid_ids:
        id_dict = {'ID' : joint_id}
        config_dict[robot_name+'/joint'+str(joint_id)] = id_dict
    
    return config_dict

def read_write():
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            break

        # Write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        while 1:
            # Read present position
            if getch() == chr(0x1b):
                break
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position))

            if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
                break

        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0

def init(device_name):
    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(device_name)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    return packetHandler,portHandler

all_ids = []
config_dict = {}

for device_name in DEVICE_NAMES:
    packetHandler, portHandler = init(device_name)
    all_ids += scan_port(packetHandler,portHandler, device_name)
    portHandler.closePort()

with open(r'hardware.yaml', 'w') as file:
    documents = yaml.dump(config_dict, file)

print(config_dict)
