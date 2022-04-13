```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```
```
Kimlab_2020
```

# Homecare Service Robot

## Kitchen PC NUC5
```
ssh papras@nuc-5
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras_demo _demo_homecare_hw_kitchen.launch
```
## Tea Table PC NUC3
```
ssh papras@nuc-3
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras_demo _demo_homecare_hw_tea_table.launch
```
## OPC
```
cd ~/catkin_ws/src/PAPRAS && git checkout homecare-robot && git pull && cd ~/catkin_ws && catkin_make
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras _demo_homecare_hw_OPC.launch
```
