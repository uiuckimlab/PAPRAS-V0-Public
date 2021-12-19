################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(papras_demo)

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
    sensor_msgs
    robotis_manipulator
    open_manipulator_msgs
    cmake_modules
)

find_package(gazebo REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Qt5Widgets REQUIRED)

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
    sensor_msgs
    robotis_manipulator
    open_manipulator_msgs
  DEPENDS
    EIGEN3
)

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${papras_gazebo_INCLUDE_DIRECTORIES})

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${papras_gazebo_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(coffee_demo src/coffee_demo.cpp)
target_link_libraries(coffee_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS coffee_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cage_tasks src/cage_tasks.cpp)
target_link_libraries(cage_tasks ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS cage_tasks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(stand_tasks src/stand_tasks.cpp)
target_link_libraries(stand_tasks ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS stand_tasks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(command_operator src/command_operator.cpp)
target_link_libraries(command_operator Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})
install(TARGETS command_operator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Install
################################################################################
install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
