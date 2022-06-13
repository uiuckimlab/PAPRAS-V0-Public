#!/usr/bin/env python3

import roslaunch
import rospy

rospy.init_node('homecare_node', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

home_path = '/home/kimlab'

cli_args1 = ['papras_demo', 'kitchen_sink_to_dishwasher.launch']
cli_args2 = ['papras_demo', 'table_handoff_fsm.launch']
cli_args3 = ['papras_demo', 'kitchen_roomba_handoff.launch']
cli_args4 = ['papras_demo', 'tea_tasks_chess.launch']

roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)[0]
roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)[0]
roslaunch_file3 = roslaunch.rlutil.resolve_launch_arguments(cli_args3)[0]
roslaunch_file4 = roslaunch.rlutil.resolve_launch_arguments(cli_args4)[0]

launch_files = [roslaunch_file1, roslaunch_file2, roslaunch_file3, roslaunch_file4]

parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
parent.start()

rospy.loginfo("started")

try:
  parent.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  parent.shutdown()