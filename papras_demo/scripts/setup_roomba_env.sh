source /usr/share/gazebo/setup.sh
source ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find irobotcreate2)
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(rospack find irobotcreate2)
