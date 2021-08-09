FROM osrf/ros:noetic-desktop-full
# set enviroment    
# ENV ROS_MASTER_URI=http://<your master IP address>:11311
# ENV ROS_IP=<your node "static" IP addresse>
ENV ROS_WS=/opt/catkin_ws

# install ros dependencies packages
RUN apt update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool build-essential \ 
    git \
    xauth \
    setserial \
    ros-noetic-moveit \
    ros-noetic-rviz-visual-tools \
    ros-noetic-moveit-visual-tools \
    ros-noetic-moveit-resources-prbt-moveit-config \
    ros-noetic-pilz-industrial-motion-planner \
    ros-noetic-joint-trajectory-controller \
    ros-noetic-effort-controllers \
    ros-noetic-realsense2-camera \
    && apt-get clean 

    

# install papras dependencies packages
RUN mkdir -p /opt/catkin_ws/src && cd /opt/catkin_ws/src \
    && git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git \
    && git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench.git \
    && git clone https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs.git \
    && git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git \
    && git clone https://github.com/ROBOTIS-GIT/open_manipulator_p_simulations.git \
    && git clone https://github.com/ROBOTIS-GIT/robotis_manipulator.git \
    && git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git \
    && git clone https://github.com/ravijo/ros_openpose.git

RUN cd $ROS_WS/src/ros_openpose && touch CATKIN_IGNORE
# need to mount /dev first 
# RUN sudo setserial /dev/ttyUSB0 low_latency
RUN . "/opt/ros/$ROS_DISTRO/setup.sh" && cd $ROS_WS && catkin_make \
    && sed --in-place --expression \ 
     '$isource "$ROS_WS/devel/setup.bash"' \
      /ros_entrypoint.sh

# run ros package launch file
# CMD [ "roslaunch", "<your package>", "<your launch file>" ]

# RUN export DISPLAY=192.168.10.12:0
# https://unix.stackexchange.com/a/481647
# https://answers.ros.org/question/322212/robot-docker-ros/
# https://answers.ros.org/question/248771/is-it-a-good-idea-to-run-docker-images-on-a-robot/
# https://answers.ros.org/question/303171/connection-between-ros-in-docker-with-external-ros-master-another-pc-the-baxter-case/