```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```

# Homecare Service Robot
## Tea Table PC NUC 3
```
ssh papras@nuc-3
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_tea_table.launch
```
## Big Table PC 
*remove arm 4 from tea table and move to big table*
```
sudo setserial /dev/ttyUSB0 low_latency
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_big_table.launch
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch dope camera.launch
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch dope dope.launch
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo papras_homecare_handoff_fsm.launch
```
## Kitchen PC NUC 5
```
ssh papras@nuc-5
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_kitchen.launch
```
**kitchen vision - dope detection**
```
ssh kazukis2@nv-lambda
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch dope camera.launch
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch dope dope.launch
```
## Roomba PC NUC 10
```
ssh papras@nuc-10
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
*ensure roomba is powered on (clean button should be flashing green)*
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_roomba.launch
```
** roomba vision - aruco marker detection**
```
ssh kimlab@jetson-xavier-nx
```
```
source ~/catkin_ws/devel_isolated/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch realsense2_camera rs_camera.launch
```
```
source ~/catkin_ws/devel_isolated/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch aruco_ros marker_publisher.launch 
```
## OPC
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_OPC.launch roomba_joy:=false
```
## Wall Mount PC NUC 2
```
ssh papras@nuc-2
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git pull && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras_demo _demo_homecare_hw_wall_mount.launch
```
## calibration
rosrun camera_calibration cameracalibrator.py --no-service-check --size 8x6 --square 0.026 image:=/camera/color/image_raw camera:=/camera
