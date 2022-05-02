#!/bin/bash
# Don't actually run this, but use it for reference

git clone https://github.com/IntelRealSense/realsense-ros.git
# https://github.com/IntelRealSense/realsense-ros

git clone https://github.com/pal-robotics/aruco_ros.git
# https://github.com/pal-robotics/aruco_ros

source ~/catkin_ws/devel/setup.bash

echo "Launching Homecare Demo"
gnome-terminal -- roslaunch papras_demo _demo_homecare_sim.launch

echo "Launching RealSense Camera"
gnome-terminal -- roslaunch realsense2_camera rs_camera.launch

echo "Launching Roomba Aruco Tag Following"
gnome-terminal -- roslaunch papras_demo roomba_nav.launch

echo "View Aruco Tag estimation output."
rosrun image_view image_view image:=/aruco_tracker/result
