################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.0.2)
project(papras_demo)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
################################################################################
# Find catkin packages and libraries for catkin and system dependencie
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roslib
    roscpp
    rospy
    std_msgs
    message_generation
    serial
    tf
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
    vision_msgs
    rosparam_shortcuts
    eigen_conversions
    # moveit_grasps
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

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Battery.msg
  Brushes.msg
  Bumper.msg
  Buttons.msg
  Diagnostic.msg
  DigitLeds.msg
  IRCharacter.msg
  Leds.msg
  Note.msg
  PlaySong.msg
  RoombaIR.msg
  RoombaSwitch.msg
  ScheduleLeds.msg
  Song.msg
  WheelDrop.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

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
set(CMAKE_CURRENT_BINARY_DIR ${papras_demo_INCLUDE_DIRECTORIES})

################################################################################
# Build
################################################################################
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
  ${papras_demo_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


add_executable(coffee_demo src/coffee_demo.cpp)
target_link_libraries(coffee_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS coffee_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(tea_demo src/tea_demo.cpp)
target_link_libraries(tea_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS tea_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(tea_chess_demo src/tea_chess_demo.cpp)
target_link_libraries(tea_chess_demo ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS tea_chess_demo DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(tea_chess_calibration src/tea_chess_calibration.cpp)
target_link_libraries(tea_chess_calibration ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS tea_chess_calibration DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(cage_tasks src/cage_tasks.cpp)
target_link_libraries(cage_tasks ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
install(TARGETS cage_tasks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(stand_tasks src/stand_tasks.cpp)
target_link_libraries(stand_tasks ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS stand_tasks DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(command_operator src/command_operator.cpp)
target_link_libraries(command_operator Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})
install(TARGETS command_operator DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(pick_place src/pick_place.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_place DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# add_executable(papras_irobotcreate2_lightweight src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/irobotcreate2/irobotcreate2_lightweight.cpp)
# add_dependencies(papras_irobotcreate2_lightweight papras_irobotcreate2_lightweight_generate_messages_cpp)
# target_link_libraries(papras_irobotcreate2_lightweight ${catkin_LIBRARIES})

# add_executable(papras_irobotcreate2 src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/irobotcreate2/irobotcreate2.cpp)
# add_dependencies(papras_irobotcreate2 papras_irobotcreate2_generate_messages_cpp)
# target_link_libraries(papras_irobotcreate2 ${catkin_LIBRARIES})

# add_executable(papras_joy_node src/irobotcreate2/joy_node.cpp)
# add_dependencies(papras_joy_node papras_irobotcreate2_generate_messages_cpp)
# target_link_libraries(papras_joy_node ${catkin_LIBRARIES})

# add_executable(papras_keyboard_node src/irobotcreate2/keyboard_node.cpp)
# add_dependencies(papras_keyboard_node irobotcreate2_generate_messages_cpp)
# target_link_libraries(papras_keyboard_node ${catkin_LIBRARIES})

# add_executable(papras_basic_commands src/irobotcreate2/OpenInterface.cpp src/irobotcreate2/odometry.cpp src/irobotcreate2/basic_commands.cpp)
# target_link_libraries(papras_basic_commands ${catkin_LIBRARIES})

# add_executable(plan_grasps_service src/plan_grasps_service.cpp)
# target_link_libraries(plan_grasps_service ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# install(TARGETS plan_grasps_service DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

################################################################################
# Install
################################################################################
install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
