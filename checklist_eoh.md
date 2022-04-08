```
export ROS_MASTER_URI=http://athena:11311/ && roscore
```
```
Kimlab_2020
```

# Backpack

## Nuc-6
```
ssh papras@nuc-6
```
```
cd ~/catkin_ws && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/
```
```
roslaunch papras_demo _demo_backpack.launch
```

## Athena
```
export ROS_MASTER_URI=http://athena:11311/ && rosrun papras_demo backpack.py
```

# Spot

## Nuc-7
```
ssh papras@nuc-7
```
```
cd ~/catkin_ws && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/
```
```
roslaunch papras_demo _demo_spot_mount.launch rviz:=false
```

## Athena
```
export ROS_MASTER_URI=http://athena:11311/ && roslaunch papras_demo lambda_spot.launch
```

# Tea Table Coffee

## Nuc-3
```
ssh papras@nuc-3
```
```
cd ~/catkin_ws && sudo -s
```
```
source devel/setup.bash && export ROS_MASTER_URI=http://athena:11311/
```
```
roslaunch papras_demo _demo_tea_table.launch rviz:=false
```

## Athena
```
export ROS_MASTER_URI=http://athena:11311 && roslaunch papras_demo lambda_tea_table.launch
```
