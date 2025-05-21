cmake_minimum_required(VERSION 3.8)
project(papras_controls)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find and load build settings from external packages
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(std_srvs REQUIRED)
find_package(dynamixel_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml-cpp REQUIRED)

################################################################################
# Build
################################################################################
add_library(
  ${PROJECT_NAME}
  SHARED 
  src/papras_hardware_interface.cpp
  src/dynamixel/dynamixel_info.cpp
  src/dynamixel/dynamixel.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  dynamixel_sdk
  std_srvs
  dynamixel_interfaces
  realtime_tools
  yaml-cpp
  )
  target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})
  pluginlib_export_plugin_description_file(hardware_interface papras_hardware.xml)

  install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
  install(
    DIRECTORY include/
    DESTINATION include
  )
  
  install(
    DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
  )
  
  install(
  DIRECTORY param
  DESTINATION share/${PROJECT_NAME}
  )

  install(
  FILES 99-papras-cdc.rules
  DESTINATION share/${PROJECT_NAME}
  )
 

  # uncomment the following section in order to fill in
  # further dependencies manually.
  # find_package(<dependency> REQUIRED)
  
  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
  
  ament_export_include_directories(
    include
  )
  ament_export_libraries(
    papras_hardware_interface
  )
  ament_export_dependencies(
    rclcpp
    hardware_interface
    pluginlib
    dynamixel_sdk
    dynamixel_interfaces
    aimbot_log_interfaces
    yaml-cpp
  )
  
  
  ament_package()
  
