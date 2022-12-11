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
    ros-noetic-aruco-ros \
    && apt-get clean

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