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
ros-noetic-aruco-ros
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
This code was written by Kazuki Shin, Dhruv Mathur and Sankalp Yamsani. 


# IK Fast Plugin
docker run -v /home/kazuki/Workspace:/mnt -it personalrobotics/ros-openrave /bin/bash
http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html'
sudo apt-get install ros-indigo-collada-urdf

rosrun xacro xacro --inorder -o simple_robot.urdf simple_robot.urdf.xacro 
rosrun collada_urdf urdf_to_collada robot.urdf robot.dae

```
sudo apt-get install ros-indigo-moveit-resources
&& ros-indigo-shape-tools
&& ros-indigo-ompl
&& ros-indigo-interactive-markers
&& ros-indigo-warehouse-ros
&& ros-indigo-control-msgs
&& ros-indigo-controller-manager-msgs
&& ros-indigo-rviz
&& ros-indigo-manipulation-msgs
```

 `python ikfast.py --robot=simple_robot.dae --iktype=transform6d --baselink=0 --freeindex=6 --eelink=10 --savefile=ikfast61_arm1.cpp`

 name                     index parents                 
-------------------------------------------------------
world                    0                             
robot1/link1             1     world                   
robot1/link2             2     robot1/link1            
robot1/link3             3     robot1/link2            
robot1/link4             4     robot1/link3            
robot1/link5             5     robot1/link4            
robot1/link6             6     robot1/link5            
robot1/end_link          7     robot1/link6            
robot1/gripper_main_link 8     robot1/end_link         
robot1/camera_link       9     robot1/gripper_main_link
robot1/end_effector_link 10    robot1/gripper_main_link
robot1/gripper_link      11    robot1/gripper_main_link
robot1/gripper_sub_link  12    robot1/gripper_main_link
-------------------------------------------------------
name                     index parents

## OpenRave IKfast resources
- http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html
- https://moveit.picknik.ai/humble/doc/examples/ikfast/ikfast_tutorial.html?highlight=ik%20fast
- http://openrave.org/docs/latest_stable/openravepy/ikfast/#ik-types
- https://github.com/personalrobotics/docker-public-images
- https://github.com/roboticslab-uc3m/installation-guides/blob/master/install-openrave.md
