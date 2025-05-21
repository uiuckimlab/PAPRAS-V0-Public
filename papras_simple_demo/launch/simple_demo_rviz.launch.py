# Author: Sankalp Yamsani, Kazuki Shin
# Description: Launch a simple papras URDF file using Rviz.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
 

def generate_launch_description():

  # Set the path to this package.
  pkg_share = get_package_share_directory('papras_simple_demo')

  # Set the path to the RViz configuration settings
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz_basic_settings.rviz')

  # Set the path to the URDF file
  default_urdf_model_path = os.path.join(pkg_share, 'urdf/simple_robot_gazebo.urdf.xacro')

  pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
  gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Pose where we want to spawn the robot
  spawn_x_val = '0.0'
  spawn_y_val = '0.0'
  spawn_z_val = '0.0'
  spawn_yaw_val = '0.00'
  world_file_name = 'papras_system.world'

  return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='papras_world',
        ),
        SetLaunchConfiguration(name='world_file', 
                               value=[LaunchConfiguration('world'), 
                                      TextSubstitution(text='.sdf')]),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[],
            remappings=[],
            output='screen'
        ),
    ])
