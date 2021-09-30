import yaml
import os
# import ipdb

def generate(config, n_robots = 3, n_joints = 6):
    actuator_list = {}
    general_arm_info = {"ID": 1,
                        "Return_Delay_Time": config['arm']['Return_Delay_Time'],
                        "Operating_Mode" : config['arm']['Operating_Mode'],
                        "Acceleration_Limit" : config['arm']['Acceleration_Limit'],
                        "Velocity_Limit" : config['arm']['Velocity_Limit'],
                        "Profile_Acceleration" : config['arm']['Profile_Acceleration'],
                        "Profile_Velocity" : config['arm']['Profile_Velocity']
                        }

    general_gripper_info = {   "ID": 1,
                        "Return_Delay_Time": config['gripper']['Return_Delay_Time'],
                        "Acceleration_Limit" : config['gripper']['Acceleration_Limit'],
                        "Velocity_Limit" : config['gripper']['Velocity_Limit'],
                        "Profile_Acceleration" : config['gripper']['Profile_Acceleration'],
                        "Profile_Velocity" : config['gripper']['Profile_Velocity']
                    }

    for robot_id in range(1, n_robots+1):
        arm_id = config["robot"+str(robot_id)]
        for joint_id in range(1, n_joints+1):
            arm_info = general_arm_info.copy()
            arm_info["ID"] = (arm_id-1)*n_joints + joint_id

            if joint_id == 3:
                arm_info["Homing_Offset"] = config['arm']['Homing_Offset']
            
            actuator_list['robot'+str(robot_id)+'/joint'+str(joint_id)] = arm_info.copy()

        gripper_info = general_gripper_info.copy()
        gripper_info["ID"] = int('1' + str(arm_id) + '1')
        actuator_list['robot'+str(robot_id)+'/gripper'] = gripper_info

    return actuator_list

if __name__ == "__main__":
    demo = 'coffee'
    repo_path = os.path.expanduser('~/catkin_kaz/src/PAPRAS')

    if demo == 'cage':
        read_path = repo_path + '/papras/config/cage_arm_config.yaml'
        write_path = repo_path + '/open_manipulator_p_controls/open_manipulator_p_hw/config/hardware_cage_2gripper.yaml'
        n_robots = 4
    if demo == 'coffee':
        read_path = repo_path + '/papras/config/coffee_arm_config.yaml'
        write_path = repo_path + '/open_manipulator_p_controls/open_manipulator_p_hw/config/hardware_coffee_3arm_arm2_nogripper.yaml'
        n_robots = 3


    with open(read_path, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    actuator_list = generate(config, n_robots)

    with open(write_path, 'w') as file:
        yaml.dump(actuator_list, file, default_flow_style=False, sort_keys=False)

    print('Finished writing to file: ', write_path)

