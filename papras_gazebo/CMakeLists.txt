################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(papras_gazebo)

add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencie
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    gazebo_ros
    gazebo_ros_control
    controller_manager
    urdf
    xacro
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    gazebo_ros
    gazebo_ros_control
    controller_manager
    urdf
    xacro
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(coffee_demo src/coffee_demo.cpp)
target_link_libraries(coffee_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS coffee_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(arm3_test src/arm3_test.cpp)
target_link_libraries(arm3_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS arm3_test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Install
################################################################################
install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
