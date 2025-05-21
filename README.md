# PAPRAS-ROS2
- The hw interface is a modification of [dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface/tree/humble), to overwrite the motor id and other properties from yaml.

- Please install followings:

  ```
  cd ~/${WORKSPACE}/src
  git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
  git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
  ```
- Additional dependencies
  ```
  sudo apt-get remove ros-humble-dynamixel-sdk # remove if it's installed 
  sudo apt-get install ros-humble-realtime-tools ros-humble-moveit ros-humble-moveit-ros ros-humble-moveit-core ros-humble-moveit-configs-utils ros-humble-gazebo-ros ros-humble-gazebo-ros2-control ros-humble-moveit-ros-control-interface
  ```

- Build the package

  ```
  cd ~/${WORKSPACE}
  colcon build --symlink-install
  ```

- source your workspace

  ```
  source ~/${WORKSPACE}/install/setup.bash
  ```

- About gazebo, you might need to source:
  ```
  source /usr/share/gazebo/setup.bash
  ```

  - Since the new gazebo is kind of tricky to use, this repo will only use the old gazebo.

- After setting the env, you can run papras in gazebo with following command:

  ```
  ros2 launch papras_simple_demo gazebo.launch.py
  ```

- Also, you can run the actual 7dof papras hardware with following command:

  ```
  ros2 launch papras_simple_demo hw.launch.py
  ```

  - Both command will launch moveit



## TODO

### Gazebo

- [ ] Add mimic joint working 
- [ ] Add demo with the new gazebo 
