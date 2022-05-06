#!/usr/bin/env python3

import roslaunch
import rospy

rospy.init_node('homecare_node', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

home_path = "/home/papras"

cli_args = [home_path+'/catkin_ws/src/PAPRAS/papras_demo/launch/roomba_nav.launch']
roslaunch_args = cli_args[1:]
# cli_args_handoff = [home_path+'/catkin_ws/src/PAPRAS/papras_demo/launch/papras_homecare_handoff_fsm.launch']
# roslaunch_args_handoff = cli_args_handoff[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)


launch.start()
rospy.loginfo("started")

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.shutdown()