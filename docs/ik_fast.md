
# IK Fast Plugin

docker run -v /home/kazuki/Workspace:/mnt -it personalrobotics/ros-openrave /bin/bash \
http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/ikfast_tutorial.html' \
sudo apt-get install ros-indigo-collada-urdf \

rosrun xacro xacro --inorder -o simple_robot.urdf simple_robot.urdf.xacro \
rosrun collada_urdf urdf_to_collada robot.urdf robot.dae