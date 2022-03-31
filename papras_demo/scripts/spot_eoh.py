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
from std_msgs.msg import String

import moveit_commander
import moveit_msgs.msg


class SpotMoveitCommander(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # arm move groups
        left_arm_group_name = "arm1"
        self.left_arm_move_group = moveit_commander.MoveGroupCommander(left_arm_group_name)
        right_arm_group_name = "arm2"
        self.right_arm_move_group = moveit_commander.MoveGroupCommander(right_arm_group_name)
        both_arm_group_name = "arm1_2"
        self.both_arm_move_group = moveit_commander.MoveGroupCommander(both_arm_group_name)

        # gripper move groups
        left_gripper_group_name = "gripper1"
        self.left_gripper_move_group = moveit_commander.MoveGroupCommander(left_gripper_group_name)
        right_gripper_group_name = "gripper2"
        self.right_gripper_move_group = moveit_commander.MoveGroupCommander(right_gripper_group_name)
        both_gripper_group_name = "gripper1_2"
        self.both_gripper_move_group = moveit_commander.MoveGroupCommander(both_gripper_group_name)
    
    def getMoveGroupFromButtonPress(self, buttonPress):
        buttonGroup = buttonPress[:buttonPress.rfind('_')]
        print("buttonGroup", buttonGroup)
        if buttonGroup == "arm1":
            return self.left_arm_move_group
        elif buttonGroup == "arm2":
            return self.right_arm_move_group
        elif buttonGroup == "arm1_2":
            return self.both_arm_move_group
        elif buttonGroup == "gripper1":
            return self.left_gripper_move_group
        elif buttonGroup == "gripper2":
            return self.right_gripper_move_group
        elif buttonGroup == "gripper1_2":
            return self.both_gripper_move_group
        else:
            return

    def getNamedTargetFromButtonPress(self, buttonPress):
        buttonTarget = buttonPress[buttonPress.rfind('_')+1:]
        print("buttonTarget", buttonTarget)
        return buttonTarget


def spotCommandCallback(msg):
    global is_executing_button_press
    rospy.loginfo("spotCommandCallback heard %s",msg.data)
    rospy.loginfo("is_executing_button_press %s", is_executing_button_press)

    if msg.data == "stop":
        print("stop")
        spot_eoh.left_arm_move_group.stop()
        spot_eoh.right_arm_move_group.stop()
        spot_eoh.both_arm_move_group.stop()
        spot_eoh.left_gripper_move_group.stop()
        spot_eoh.right_gripper_move_group.stop()
        spot_eoh.both_gripper_move_group.stop()
        return 

    if not is_executing_button_press:
        is_executing_button_press = True
        move_group = spot_eoh.getMoveGroupFromButtonPress(msg.data)
        named_target = spot_eoh.getNamedTargetFromButtonPress(msg.data)
        move_group.set_start_state_to_current_state()
        move_group.set_max_acceleration_scaling_factor(0.1)
        move_group.set_max_velocity_scaling_factor(0.1)
        move_group.set_planning_time(0.3)
        move_group.set_num_planning_attempts(30)
        move_group.set_named_target(named_target)
        move_group.plan()
        move_group.go()
        is_executing_button_press = False

def main():
    r = rospy.Rate(300)
    rospy.Subscriber("rviz_visual_tools_gui_papras", String, spotCommandCallback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("spot_eoh_demo")
    is_executing_button_press = False

    spot_eoh = SpotMoveitCommander()
    main()
