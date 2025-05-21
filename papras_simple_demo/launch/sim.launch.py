from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Get paths to packages and directories
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')
    papras_description_pkg = get_package_share_directory('papras_description')
    papras_pkg = get_package_share_directory('papras_simple_demo')

    robot_description_content = xacro.process_file(
                 os.path.join(papras_pkg, 'urdf', 'simple_robot.urdf.xacro')).toxml()

    # Launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('pipeline', default_value='ompl'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Gazebo args
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),

        # Model spawn locations
        DeclareLaunchArgument('robot_pos_x', default_value='0'),
        DeclareLaunchArgument('robot_pos_y', default_value='0'),
        DeclareLaunchArgument('robot_pos_z', default_value='0.5'),
        DeclareLaunchArgument('robot_pos_yaw', default_value=str(3.14159)),

        # Include Gazebo Ignition world launchos.path.join(get_package_share_directory('xacro')), ' ',
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [gazebo_ros_pkg, "/launch/gz_sim.launch.py"]
            ),
            launch_arguments={"gz_args": ["-r", "-v", "4", "empty_world.sdf"]}.items(),
            condition=IfCondition( LaunchConfiguration('gazebo_gui')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        
        ),

        # Spawn robot model in Gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-string",
                robot_description_content,
                "-name",
                "papras",
                "-allow_renaming",
                "true",
            ],
        ), 
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[get_package_share_directory('papras_controls')+"/config/joint_trajectory_controller_gazebo.yaml"],
            remappings=[
                ('~/robot_description', '/robot_description'),
            ]
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ), 
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper1_controller", "--controller-manager", "/controller_manager"],
        ), 
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm1_controller", "--controller-manager", "/controller_manager"],
        )

    
    ])