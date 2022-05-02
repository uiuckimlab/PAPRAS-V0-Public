import roslaunch
import rospy

rospy.init_node('en_Mapping', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

cli_args = ['/home/kazuki/catkin_ws/src/PAPRAS/papras_demo/launch/_demo_homecare_sim.launch','gazebo_gui:=false']
roslaunch_args = cli_args[1:]
roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]

launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
launch.start()
rospy.loginfo("started")

try:
  launch.spin()
finally:
  # After Ctrl+C, stop all nodes from running
  launch.shutdown()