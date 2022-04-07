```
export ROS_MASTER_URI=http://athena:11311/ && roscore
```
```
Kimlab_2020
```
!!! Remove git pull commands for EOH since computers will not have internet access !!!

# Spot

## Nuc-7
```
ssh papras@nuc-7
```
```
cd ~/catkin_ws/src/PAPRAS && git checkout eoh-demo && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/
```
```
roslaunch papras_demo _demo_spot_mount.launch rviz:=false
```

## Athena
```
cd ~/catkin_ws/src/PAPRAS && git checkout eoh-demo && git pull && cd ~/catkin_ws && catkin_make
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/ && roslaunch papras_demo lambda_spot.launch
```

# Tea Table

## Nuc-8
```
ssh papras@nuc-8
```
```
cd ~/catkin_ws/src/PAPRAS && git checkout eoh-demo && git pull && cd ~/catkin_ws && catkin_make && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/
```
```
roslaunch papras_demo _demo_tea_table.launch rviz:=false
```

## Athena
```
cd ~/catkin_ws/src/PAPRAS && git checkout eoh-demo && git pull && cd ~/catkin_ws && catkin_make
```
```
source ~/catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://athena:11311 && roslaunch papras_demo lambda_tea_table.launch
```
