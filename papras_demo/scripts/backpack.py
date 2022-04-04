#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import numpy as np

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint

class Backpack(object):
    def __init__(self):
        print("in init")

        self._arm_1_client = actionlib.SimpleActionClient("/arm1_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_1_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 1 Action Server Not Found")
            rospy.signal_shutdown("Arm 1 Action Server not found")
            sys.exit(1)
        print("Arm 1 Action Server Found")

        self._arm_2_client = actionlib.SimpleActionClient("/arm2_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_2_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 2 Action Server Not Found")
            rospy.signal_shutdown("Arm 2 Action Server not found")
            sys.exit(1)
        print("Arm 2 Action Server Found")

        self._arm_3_client = actionlib.SimpleActionClient("/arm3_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_3_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 3 Action Server Not Found")
            rospy.signal_shutdown("Arm 3 Action Server not found")
            sys.exit(1)
        print("Arm 3 Action Server Found")

        self._arm_4_client = actionlib.SimpleActionClient("/arm4_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_4_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 4 Action Server Not Found")
            rospy.signal_shutdown("Arm 4 Action Server not found")
            sys.exit(1)
        print("Arm 4 Action Server Found")

        self._gripper_1_client = actionlib.SimpleActionClient("/gripper1_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        if not self._gripper_1_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper 1 Action Server Not Found")
            rospy.signal_shutdown("Gripper 1 Action Server not found")
            sys.exit(1)
        print("Gripper 1 Action Server Found")

        self._gripper_2_client = actionlib.SimpleActionClient("/gripper2_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        if not self._gripper_2_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper 2 Action Server Not Found")
            rospy.signal_shutdown("Gripper 2 Action Server not found")
            sys.exit(1)
        print("Gripper 2 Action Server Found")

        self._gripper_3_client = actionlib.SimpleActionClient("/gripper3_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        if not self._gripper_3_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper 3 Action Server Not Found")
            rospy.signal_shutdown("Gripper 3 Action Server not found")
            sys.exit(1)
        print("Gripper 3 Action Server Found")

        self._gripper_4_client = actionlib.SimpleActionClient("/gripper4_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        if not self._gripper_4_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper 4 Action Server Not Found")
            rospy.signal_shutdown("Gripper 4 Action Server not found")
            sys.exit(1)
        print("Gripper 4 Action Server Found")

    def move_arm(self, arm_1_goal=None, arm_2_goal=None, arm_3_goal=None, arm_4_goal=None, goal_secs=5):
        print("in move arm")

        if arm_1_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot1/joint1", "robot1/joint2", "robot1/joint3",
                                           "robot1/joint4", "robot1/joint5", "robot1/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = arm_1_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("arm 1 goal: ", arm_1_goal)
            self._arm_1_client.send_goal(goal)
            print("arm 1 command sent")

        if arm_2_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot2/joint1", "robot2/joint2", "robot2/joint3",
                                           "robot2/joint4", "robot2/joint5", "robot2/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = arm_2_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("arm 2 goal: ", arm_2_goal)
            self._arm_2_client.send_goal(goal)
            print("arm 2 command sent")

        if arm_3_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot3/joint1", "robot3/joint2", "robot3/joint3",
                                           "robot3/joint4", "robot3/joint5", "robot3/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = arm_3_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("arm 3 goal: ", arm_3_goal)
            self._arm_3_client.send_goal(goal)
            print("arm 3 command sent")

        if arm_4_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot4/joint1", "robot4/joint2", "robot4/joint3",
                                           "robot4/joint4", "robot4/joint5", "robot4/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = arm_4_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("arm 4 goal: ", arm_4_goal)
            self._arm_4_client.send_goal(goal)
            print("arm 4 command sent")

        rospy.sleep(rospy.Duration(goal_secs + 0.1))

    def move_gripper(self, gripper_1_goal=None,gripper_2_goal=None,gripper_3_goal=None, gripper_4_goal=None, goal_secs=1):
        print("in move gripper")

        if gripper_1_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot1/gripper"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = gripper_1_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("gripper 1 goal: ", gripper_1_goal)
            self._gripper_1_client.send_goal(goal)
            print("gripper 1 command sent")

        if gripper_2_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot2/gripper"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = gripper_2_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("gripper 2 goal: ", gripper_2_goal)
            self._gripper_2_client.send_goal(goal)
            print("gripper 2 command sent")

        if gripper_3_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot3/gripper"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = gripper_3_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("gripper 3 goal: ", gripper_3_goal)
            self._gripper_3_client.send_goal(goal)
            print("gripper 3 command sent")

        if gripper_4_goal is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot4/gripper"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = gripper_4_goal
            joint_angle.time_from_start = rospy.Duration(goal_secs)
            goal.trajectory.points.append(joint_angle)
            print("gripper 4 goal: ", gripper_4_goal)
            self._gripper_4_client.send_goal(goal)
            print("gripper 4 command sent")

        rospy.sleep(rospy.Duration(goal_secs + 0.1))

def demo_reset():
    # Notch Arrangment:
    # Arms 1 and 2: Notch points back -> 0 position points right
    # Arm 3: Notch points up -> 0 position points forward
    # Arm 4: Notch points up -> 0 position points back
    rest1 = np.radians([90, -90, 85, 0, 20, 0])
    rest2 = np.radians([90, -90, 85, 0, 20, 0])
    rest3 = np.radians([0, -90, 85, 0, 20, 0])
    rest4 = np.radians([-180, -90, 85, 0, 20, 0]) # Joint 1 is maxed out!
    backpack.move_arm(rest1, rest2, rest3, rest4, 5)

def main():
    r = rospy.Rate(300)
    print("in main")

    rospy.on_shutdown(demo_reset)

    # Reset demo
    demo_reset()

    # Demo start
    # Forward
    input("Press enter to continue forward")
    forward1 = np.radians([90, 30, -30, -60, 0, 90])
    forward2 = np.radians([90, 30, -30, 60, 0, -90])
    forward3 = np.radians([0, 30, -30, 0, 0, 0])
    forward4 = np.radians([-180, 30, -30, 0, 0, 0]) # Joint 1 is maxed out!
    backpack.move_arm(forward1, forward2, forward3, forward4, 5)

    # Look left
    input("Press enter to continue look_left")
    left1 = np.radians([90, 30, -30, -60, -45, 90])
    left2 = np.radians([90, 30, -30, 60, 45, -90])
    left3 = np.radians([0, 30, -30, 0, -45, 0])
    left4 = np.radians([-180, 30, -30, 0, 45, 0]) # Joint 1 is maxed out!
    backpack.move_arm(left1, left2, left3, left4, 0.5)

    # Look right
    input("Press enter to continue look_right")
    right1 = np.radians([90, 30, -30, -60, 45, 90])
    right2 = np.radians([90, 30, -30, 60, -45, -90])
    right3 = np.radians([0, 30, -30, 0, 45, 0])
    right4 = np.radians([-180, 30, -30, 0, -45, 0]) # Joint 1 is maxed out!
    backpack.move_arm(right1, right2, right3, right4, 1)

    # Back to forward
    input("Press enter to continue look_forward")
    backpack.move_arm(forward1, forward2, forward3, forward4, 0.5)

    # Converge
    input("Press enter to continue converge")
    converge1 = np.radians([120, 45, 0, 45, -45, 0])
    converge2 = np.radians([60, 45, 0, -45, -45, 0])
    converge3 = np.radians([-30, 45, 0, -45, -45, 0])
    converge4 = np.radians([-150, 45, 0 ,45, -45, 0])
    backpack.move_arm(converge1, converge2, converge3, converge4, 5)

    # Back to rest
    input("Press enter to continue rest")
    rospy.signal_shutdown(None)

if __name__ == '__main__':
    rospy.init_node("backpack_demo")

    backpack = Backpack()

    main()