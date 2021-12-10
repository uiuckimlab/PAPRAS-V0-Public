```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```
```
Kimlab_2020
```

# Stand Demo

## Stand 1 (1T)
```
ssh papras@nuc-3
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_stand.launch nuc_id:=3
```

## Stand 2 (1T)
```
ssh papras@nuc-2
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_stand.launch nuc_id:=2
```

## Stand 3 (1T)
```
ssh papras@nuc-4
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_stand.launch nuc_id:=4
```

## Stand 4 (1T)
```
ssh papras@nuc-6
```
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_stand.launch nuc_id:=6
```

## Lambda (3T)
T1
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras lambda_stand.launch
```

*click continue on rviz visual tool*

# Kitchen Demo
```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```
```
ssh papras@nuc-5
```
*on nuc*
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_kitchen.launch
```

*on lambda*
```
cd ~/catkin_ws && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras lambda_kitchen.launch
```

# Coffee Pour Demo 
*on lambda, no ssh needed* \
T1
```
sudo setserial /dev/ttyUSB0 low_latency
```
```
cd ~/catkin_ws && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras _demo_coffee_camera.launch
```
T2
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras lambda_coffee.launch
```

# Spot Demo
```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```
```
ssh papras@nuc-7
```
*on nuc* \
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_spot_mount.launch
```

*on lambda*
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras lambda_spot.launch
```

# Cage Demo 
```
export ROS_MASTER_URI=http://lambda-dual:11311/ && roscore
```
```
ssh papras@nuc-1
```
*on nuc* \
*enter pass for setting usb low latency*
```
cd ~/catkin_ws/src/PAPRAS && git checkout dryrun && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/
```
```
roslaunch papras _demo_cage.launch
```

*on lambda*
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://lambda-dual:11311/ && roslaunch papras lambda_cage.launch
```
