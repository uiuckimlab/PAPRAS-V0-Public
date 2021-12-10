#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import sys

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose

# for Stacker
import moveit_commander
import actionlib
from tf.transformations import quaternion_from_euler
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class JacketGrabber(object):
    _RIGHT_ARM = 1
    _LEFT_ARM = 2
    _BOTH_ARMS = 3

    _GRIPPER_OPEN = 0
    _GRIPPER_CLOSE = 1.13

    def __init__(self):
        self._right_arm = moveit_commander.MoveGroupCommander(
            "arm3")
        self._right_arm.set_max_velocity_scaling_factor(0.1)
        self._right_gripper = actionlib.SimpleActionClient(
            "/gripper3_controller/gripper_cmd", GripperCommandAction)
        self._right_gripper.wait_for_server()

        self._left_arm = moveit_commander.MoveGroupCommander(
            "arm4")
        self._left_arm.set_max_velocity_scaling_factor(0.1)
        self._left_gripper = actionlib.SimpleActionClient(
            "/gripper3_controller/gripper_cmd", GripperCommandAction)
        self._left_gripper.wait_for_server()

        self._both_arms = moveit_commander.MoveGroupCommander(
            "arm3_4")
        self._both_arms.set_max_velocity_scaling_factor(0.1)
        self._both_grippers = actionlib.SimpleActionClient(
            "/gripper3_4/gripper_cmd", GripperCommandAction)
        self._both_grippers.wait_for_server()

        self._gripper_goal = GripperCommandGoal()
        self._gripper_goal.command.max_effort = 2.0

        self.initialize_arms()

        self._current_arm = None

    def _move_arm_to_pose(self, current_arm, target_pose):
        if current_arm == self._RIGHT_ARM:
            q = quaternion_from_euler(3.14/2.0, 0.0, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            self._right_arm.set_pose_target(target_pose)
            return self._right_arm.go()
        elif current_arm == self._LEFT_ARM:
            q = quaternion_from_euler(-3.14/2.0, 0.0, 0.0)
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            self._left_arm.set_pose_target(target_pose)
            return self._left_arm.go()
        else:
            return False

    def _move_arm_to_joint(self, current_arm, joint_goal):
        if current_arm == self._RIGHT_ARM:
            return self._right_arm.go(joint_goal, wait=True)
        elif current_arm == self._LEFT_ARM:
            return self._left_arm.go(joint_goal, wait=True)
        elif current_arm == self._BOTH_ARMS:
            return self._both_arms.go(joint_goal, wait=True)
        else:
            return False

    def _open_gripper(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._gripper_goal.command.position = self._GRIPPER_OPEN
            self._right_gripper.send_goal(self._gripper_goal)
            return self._right_gripper.wait_for_result(rospy.Duration(1.0))
        elif current_arm == self._LEFT_ARM:
            self._gripper_goal.command.position = -self._GRIPPER_OPEN
            self._left_gripper.send_goal(self._gripper_goal)
            return self._left_gripper.wait_for_result(rospy.Duration(1.0))
        elif current_arm == self._BOTH_ARMS:
            self._gripper_goal.command.position = -self._GRIPPER_OPEN
            self._both_grippers.send_goal(self._gripper_goal)
            return self._both_grippers.wait_for_result(rospy.Duration(1.0))
        else:
            return False

    def _close_gripper(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._gripper_goal.command.position = self._GRIPPER_CLOSE
            self._right_gripper.send_goal(self._gripper_goal)
        elif current_arm == self._LEFT_ARM:
            self._gripper_goal.command.position = -self._GRIPPER_CLOSE
            self._left_gripper.send_goal(self._gripper_goal)
        elif current_arm == self._BOTH_ARMS:
            self._gripper_goal.command.position = -self._GRIPPER_CLOSE
            self._both_grippers.send_goal(self._gripper_goal)
        else:
            return False

    def initialize_arms(self):
        self._move_arm_to_init_pose(self._RIGHT_ARM)
        self._move_arm_to_init_pose(self._LEFT_ARM)
        self._open_gripper(self._RIGHT_ARM)
        self._open_gripper(self._LEFT_ARM)

    def _move_arm_to_init_pose(self, current_arm):
        if current_arm == self._RIGHT_ARM:
            self._right_arm.set_named_target("rest")
            return self._right_arm.go()
        elif current_arm == self._LEFT_ARM:
            self._left_arm.set_named_target("rest")
            return self._left_arm.go()
        elif current_arm == self._BOTH_ARMS:
            self._both_arms.set_named_target("rest")
            return self._both_arms.go()
        else:
            return False


def cage_shutdown():
    jacket_grabber.initialize_arms()


def main():
    r = rospy.Rate(60)

    rospy.on_shutdown(cage_shutdown)

    MOVE_TO_INIT = 0
    GRAB_JACKET = 1

    current_mode = MOVE_TO_INIT
    CHECK_RESULT = True

    while not rospy.is_shutdown():
        if current_mode == MOVE_TO_INIT:
            if jacket_grabber.pick_up(CHECK_RESULT) is False:
                rospy.logwarn("Move Init Failed")
            else:
                rospy.loginfo("Move Init Succeeded")
                current_mode = GRAB_JACKET
        elif current_mode == GRAB_JACKET:
            if jacket_grabber.place_on_highest_object(CHECK_RESULT) is False:
                rospy.logwarn("GRAB_JACKET Failed")
            else:
                rospy.loginfo("GRAB_JACKET Succeeded")

        r.sleep()


if __name__ == '__main__':
    rospy.init_node("jacket_grab_example")

    jacket_grabber = JacketGrabber()

    main()
