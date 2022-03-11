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
ros-noetic-plotjuggler-ros
```

3. Go to catkin_ws directory, build the package, and configure ROS
```
cd && mkdir catkin_ws && cd catkin_ws && mkdir src && cd src
git clone repo
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

4. Run the executable
```
 roslaunch papras_demo _demo_kitchen.launch sim:=true gazebo_gui:=true rviz:=true
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
This code was written by Kazuki Shin, Dhruv Mathur and Sankalp Yamsani. The grasp detection code was adapted from 2020 PickNik Inc.
