# PAPRAS

Project-NR: Developing Plug-And-Play Robotic Arm System (PAPRAS)

## How to Run

We use a ROS1 build environment. This project can easily be made to interface with ROS1 for visualization or for running on a real or simulated robot.

1. Manual Install
   Follow installation instructions for [ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
   The specific commands are listed below:

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

2. Install MoveIt! and other necessary packages

```
sudo apt install ros-noetic-moveit \
ros-noetic-rviz-visual-tools \
ros-noetic-moveit-visual-tools \
ros-noetic-moveit-resources-prbt-moveit-config \
ros-noetic-pilz-industrial-motion-planner \
ros-noetic-joint-trajectory-controller \
ros-noetic-effort-controllers \
ros-noetic-plotjuggler-ros \
ros-noetic-vision-msgs \
ros-noetic-rosparam-shortcuts \
ros-noetic-serial \
ros-noetic-libcreate \
ros-noetic-aruco-msgs \
libspnav-dev \
ros-noetic-trac-ik-kinematics-plugin \
ros-noetic-dynamixel-sdk \
ros-noetic-dynamixel-workbench \
ros-noetic-open-manipulator-msgs \
ros-noetic-open-manipulator-p-simulations \
ros-noetic-robotis-manipulator \
ros-noetic-realsense2-camera \
ros-noetic-aruco-ros \
ros-noetic-roboticsgroup-upatras-gazebo-plugins
```

3. Go to catkin_ws directory, build the package, and configure ROS
```
cd && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
git clone https://github.com/uiuckimlab/PAPRAS-V0-Public.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

3. Optional for grasping in gazebo to work with the position controllers, clone these two repositories in your catkin_ws
```
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
git clone https://github.com/JenniferBuehler/general-message-pkgs.git

```

4. Run ./create_udev_rules bash script located in PAPRAS/scripts
This script copies a udev rule to /etc to facilitate bringing up the PAPRAS usb connection.
```
cd ~/catkin_ws/src/PAPRAS/scripts
```
```
chmod +x create_udev_rules && ./create_udev_rules
```

4. Run the executable

```
 roslaunch papras_simple_demo simple_demo_sim.launch gazebo_gui:=true rviz:=true
```

Example output:

```
Loading 'pilz_industrial_motion_planner/MoveGroupSequenceAction'...
[ INFO] [1647025084.512747382, 0.959000000]: initialize move group sequence action
[ INFO] [1647025084.516892254, 0.963000000]: Reading limits from namespace /robot_description_planning
Loading 'pilz_industrial_motion_planner/MoveGroupSequenceService'...
[ INFO] [1647025084.526412752, 0.973000000]: Reading limits from namespace /robot_description_planning
[ INFO] [1647025084.535253190, 0.982000000]:
*
[ INFO] [1647025084.535305630, 0.982000000]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1647025084.535326920, 0.982000000]: MoveGroup context initialization complete

You can start planning now!
```

5. Plotting joint positon and torque using PlotJuggler

```
rosrun plotjuggler plotjuggler
```

Streaming Panel -> Select `ROS Topic Subscriber` -> Click Start \
Select all `/ft_sensor_topic` topics and click OK \
Drag & Drop any of the joint force/torque values onto plot.


# Credits

This code was written by Kazuki Shin, Dhruv Mathur and Sankalp Yamsani.
