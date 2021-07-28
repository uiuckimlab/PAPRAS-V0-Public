#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ros_openpose.msg import Frame
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import ipdb

# calc angle of vertex pt1 using law of cosines
def left_calc_joint_angle(inner, middle, out):
    res = np.arctan2(out.y - middle.y, out.x - middle.x) - np.arctan2(middle.y - inner.y, middle.x - inner.x)
    return res

def right_calc_joint_angle(inner, middle, out):
    res = np.arctan2(inner.y - middle.y, inner.x - middle.x) - np.arctan2(middle.y - out.y, middle.x - out.x)
    return res

def callback(msg):
    # text = [bodyPart.pixel for person in msg.persons for bodyPart in person.bodyParts]
    # rospy.loginfo('%s\n' % text)
    if not msg.persons:
        return 

    ####
    # Left arm3_controller
    ###
    body_parts = msg.persons[0].bodyParts
    l_core = body_parts[1].pixel
    l_shoulder = body_parts[5].pixel
    l_elbow = body_parts[6].pixel
    l_wrist = body_parts[7].pixel
    l_core.y = l_shoulder.y # Core in line with shoulder

    l_joint1 = np.pi
    l_joint2 = min(max(left_calc_joint_angle(l_core, l_shoulder, l_elbow), -2.0), 2.0)
    l_joint3 = min(max(left_calc_joint_angle(l_shoulder, l_elbow, l_wrist) - np.pi/2, -np.pi), np.pi/2)
    l_joint4 = 0
    l_joint5 = 0
    l_joint6 = 0

    left_goal = FollowJointTrajectoryActionGoal()
    left_goal.header.stamp = rospy.Time.now()  + rospy.Duration(0.5)
    left_goal.goal_id.stamp = left_goal.header.stamp
    left_goal.goal.trajectory.joint_names = ["robotleft/joint1", "robotleft/joint2", "robotleft/joint3", "robotleft/joint4", "robotleft/joint5", "robotleft/joint6"]

    traj_point = JointTrajectoryPoint()
    traj_point.positions = [l_joint1, l_joint2, l_joint3, l_joint4, l_joint5, l_joint6]
    traj_point.time_from_start = rospy.Duration(0.08)
    points = left_goal.goal.trajectory.points 
    points.append(traj_point)
    left_goal.goal.trajectory.points = points
    left_joint_state_pub.publish(left_goal)

    ####
    # Right arm3_controller
    ###
    body_parts = msg.persons[0].bodyParts
    r_core = body_parts[1].pixel
    r_shoulder = body_parts[2].pixel
    r_elbow = body_parts[3].pixel
    r_wrist = body_parts[4].pixel
    r_core.y = r_shoulder.y # Core in line with shoulder

    r_joint1 = -1*np.pi
    r_joint2 = min(max(right_calc_joint_angle(r_core, r_shoulder, r_elbow), -2.0), 2.0)
    r_joint3 = min(max(right_calc_joint_angle(r_shoulder, r_elbow, r_wrist) - np.pi/2, -np.pi), np.pi/2)
    r_joint4 = 0
    r_joint5 = 0
    r_joint6 = 0

    right_goal = FollowJointTrajectoryActionGoal()
    right_goal.header.stamp = rospy.Time.now()  + rospy.Duration(0.5)
    right_goal.goal_id.stamp = right_goal.header.stamp
    right_goal.goal.trajectory.joint_names = ["robotright/joint1", "robotright/joint2", "robotright/joint3", "robotright/joint4", "robotright/joint5", "robotright/joint6"]

    traj_point2 = JointTrajectoryPoint()
    traj_point2.positions = [r_joint1, r_joint2, r_joint3, r_joint4, r_joint5, r_joint6]
    traj_point2.time_from_start = rospy.Duration(0.08)
    points = right_goal.goal.trajectory.points 
    points.append(traj_point2)
    right_goal.goal.trajectory.points = points
    right_joint_state_pub.publish(right_goal)

# from collections import namedtuple
# Point = namedtuple("Point","x y")
# core = Point(10, 2)
# shoulder = Point(9, 2)
# elbow = Point(8, 2)
# wrist = Point(9, 3)

# print(right_calc_joint_angle(shoulder, elbow, wrist) - np.pi/2)

rospy.init_node('openpose2papras', anonymous=False)

# read the parameter from ROS parameter server
frame_topic = rospy.get_param('~pub_topic')

rospy.Subscriber(frame_topic, Frame, callback)
# left_joint_state_pub = rospy.Publisher('/arm3_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=1)
left_joint_state_pub = rospy.Publisher('/left_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
right_joint_state_pub = rospy.Publisher('/right_arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)

rospy.spin()