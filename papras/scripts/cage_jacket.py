#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import sys

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint


class JacketGrabber(object):
    def __init__(self):
        print("in init")
        self._right_arm_client = actionlib.SimpleActionClient("/arm3_controller/follow_joint_trajectory",
                                                              FollowJointTrajectoryAction)
        self._right_arm_client.wait_for_server(rospy.Duration(5.0))
        if not self._right_arm_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Right Arm Action Server Not Found")
            rospy.signal_shutdown("Right Arm Action Server not found")
            sys.exit(1)
        print("Right Arm Action Server Found")

        self._right_gripper_client = actionlib.SimpleActionClient("/gripper3_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        self._right_gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self._right_gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("right Gripper Action Server Not Found")
            rospy.signal_shutdown("right Gripper Action Server not found")
            sys.exit(1)
        print("right Gripper Action Server Found")

        self._left_arm_client = actionlib.SimpleActionClient("/arm4_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        self._left_arm_client.wait_for_server(rospy.Duration(5.0))
        if not self._left_arm_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Left Arm Action Server Not Found")
            rospy.signal_shutdown("Left Arm Action Server not found")
            sys.exit(1)
        print("Left Arm Action Server Found")

        self._left_gripper_client = actionlib.SimpleActionClient("/gripper4_controller/follow_joint_trajectory",
                                                             FollowJointTrajectoryAction)
        self._left_gripper_client.wait_for_server(rospy.Duration(5.0))
        if not self._left_gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Left Gripper Action Server Not Found")
            rospy.signal_shutdown("Left Gripper Action Server not found")
            sys.exit(1)
        print("Left Gripper Action Server Found")

    def set_angle(self, right_arm_goal, left_arm_goal, goal_secs=5):
        print("in set angle")
        # Right Arm Command
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot3/joint1", "robot3/joint2", "robot3/joint3",
                                       "robot3/joint4", "robot3/joint5", "robot3/joint6"]

        joint_angle = JointTrajectoryPoint()
        joint_angle.positions = right_arm_goal
        joint_angle.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(joint_angle)
        print(right_arm_goal)
        self._right_arm_client.send_goal(goal)
        print("right command sent")

        # Left Arm Command
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot4/joint1", "robot4/joint2", "robot4/joint3",
                                       "robot4/joint4", "robot4/joint5", "robot4/joint6"]

        joint_angle = JointTrajectoryPoint()
        joint_angle.positions = left_arm_goal
        joint_angle.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(joint_angle)

        print(left_arm_goal)
        self._left_arm_client.send_goal(goal)
        print("left command sent")

        # Wait for result
        self._right_arm_client.wait_for_result(rospy.Duration(0.1))
        self._left_arm_client.wait_for_result(rospy.Duration(0.1))

        rospy.sleep(rospy.Duration(goal_secs + 0.1))

    def set_gripper_angle(self, right_gripper_goal, left_gripper_goal, goal_secs = 1):
        print("in gripper set angle")
        # Right Gripper Command
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot3/gripper"]

        joint_angle = JointTrajectoryPoint()
        joint_angle.positions = right_gripper_goal
        joint_angle.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(joint_angle)
        print(right_gripper_goal)
        self._right_gripper_client.send_goal(goal)
        print("right gripper command sent")

        # Left Arm Command
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot4/gripper"]

        joint_angle = JointTrajectoryPoint()
        joint_angle.positions = left_gripper_goal
        joint_angle.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(joint_angle)

        print(left_gripper_goal)
        self._left_gripper_client.send_goal(goal)
        print("left gripper command sent")

        # Wait for result
        self._right_gripper_client.wait_for_result(rospy.Duration(0.1))
        self._left_gripper_client.wait_for_result(rospy.Duration(0.1))

        rospy.sleep(rospy.Duration(goal_secs + 0.1))

def demo_shutdown():
    rest_angle_right = [0, -1.5707, 1.5, 0, 0.4, 0]
    rest_angle_left = [0, -1.5707, 1.5, 0, 0.4, 0]
    jacket_grabber.set_angle(rest_angle_right, rest_angle_left, goal_secs=3)
    jacket_grabber.set_gripper_angle([1],[1])

def main():
    r = rospy.Rate(300)
    print("in cage jacket")

    rospy.on_shutdown(demo_shutdown)

    # move to jacket
    # rest_angle_right = [-1.99, -0.57, 0.31, -0.08, 0.38, -1.55]
    # rest_angle_left = [2.09, -1.06, 0.91, 1.24, -0.02, 0.19]
    # jacket_grabber.set_angle(rest_angle_right, rest_angle_left, goal_secs=5)
    jacket_grabber.set_gripper_angle([0],[0])

    # approach jacket
    rest_angle_right = [-2.07, -0.36, 0.24, -0.08, 0.19, -1.55]
    rest_angle_left = [2.06, -0.52, 0.56, 1.24, -0.02, 0.19]
    jacket_grabber.set_angle(rest_angle_right, rest_angle_left, goal_secs=1)

    # grab jacket
    jacket_grabber.set_gripper_angle([1.13],[1.13])

    # pull jacket
    rest_angle_right = [-1.22, 0.31, 0.52, -0.08, -0.52, -1.64]
    rest_angle_left = [1.27, -0.17, 0.80, 1.24, -0.37, 0.19]
    jacket_grabber.set_angle(rest_angle_right, rest_angle_left, goal_secs=2)

    # rotate wrist down jacket
    rest_angle_right = [-0.91, 0.58, 0.23, 1.24, -1.05, -2.60]
    rest_angle_left = [0.61, 0.00, 0.49, -1.95, -1.03, 3.00]
    jacket_grabber.set_angle(rest_angle_right, rest_angle_left, goal_secs=3)

    # drop jacket
    jacket_grabber.set_gripper_angle([0],[0])

    rospy.signal_shutdown(None)


if __name__ == '__main__':
    rospy.init_node("jacket_grab_demo")

    jacket_grabber = JacketGrabber()

    main()
