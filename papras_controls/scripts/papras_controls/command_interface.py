#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import moveit_commander

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
import sys

from moveit_msgs.srv import GetPositionIK
import moveit_msgs
import os

import moveit_msgs.msg
import shape_msgs.msg
import geometry_msgs.msg
import rospy


'''
http://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
'''
class SingleArmCommandInterface:
    def __init__(self, prefix='robot1', vel_scale=0.8, accel_scale=0.8):    
        self.prefix = prefix
        self.robot_num = int(prefix[-1])
        self.arm_group_name = "arm" + str(self.robot_num)
        self.gripper_group_name = "gripper" + str(self.robot_num)

        # set planning parameters
        self.NUM_PLANNING_ATTEMPTS = 30
        self.PLANNING_TIME = 20.0
        self.MAX_VELOCITY_SCALING_FACTOR = self.vel_scale
        self.MAX_ACCELERATION_SCALING_FACTOR = self.accel_scale
        
        self.marker_pub = rospy.Publisher(prefix + '/grasp_pose', PoseStamped, queue_size=1)
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.init_follow_joint_trajectory_client()
        self.init_moveit()

    def init_follow_joint_trajectory_client(self):
        print("in init_follow_joint_trajectory_client")

        self._arm_client = actionlib.SimpleActionClient("/arm"+self.robot_num+"_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm {} Action Server Not Found".format(self.robot_num))
            rospy.signal_shutSingleArmCommandInterfacedown("Arm 1 Action Server not found")
            sys.exit(1)
        print("Arm {} Action Server Found".format(self.robot_num))

        self._gripper_client = actionlib.SimpleActionClient("/gripper"+self.robot_num+"_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper {} Action Server Not Found".format(self.robot_num))
            rospy.signal_shutdown("Gripper {} Action Server not found".format(self.robot_num))
            sys.exit(1)
        print("Gripper {} Action Server Found".format(self.robot_num))

    def init_moveit(self):
        # moveit groups
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name)

        # planning scene  
        self.planning_scene = moveit_commander.PlanningSceneInterface()

        # set planning parameters
        self.arm_group.set_num_planning_attempts(self.NUM_PLANNING_ATTEMPTS)
        self.arm_group.set_planning_time(self.PLANNING_TIME)
        self.arm_group.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)
        self.arm_group.set_max_acceleration_scaling_factor(self.MAX_ACCELERATION_SCALING_FACTOR)
        self.gripper_group.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)
        self.gripper_group.set_max_acceleration_scaling_factor(self.MAX_ACCELERATION_SCALING_FACTOR)

        # set end effector frame
        self.robot_eef_frame = self.arm_group.get_end_effector_link()

    def clear_collision_objects(self):
        '''
        Removes all collision objects from the planning scene
        '''
        self.planning_scene.remove_world_object()

    def close_gripper(self):
        self.gripper_group1.set_named_target("close")
        self.gripper_group1.go(wait=True)
        self.gripper_group1.stop()
        self.gripper_group1.clear_pose_targets()

    def open_gripper(self):
        self.gripper_group1.set_named_target("open")
        self.gripper_group1.go(wait=True)
        self.gripper_group1.stop()
        self.gripper_group1.clear_pose_targets()

    def close_gripper_direct(self, pos=1.08, time=1):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot1/gripper"]
        point = JointTrajectoryPoint()
        point.positions = [pos]
        point.time_from_start = rospy.Duration(time)
        goal.trajectory.points.append(point)
        self._gripper_1_client.send_goal_and_wait(goal)
        
        if not self._gripper_1_client.wait_for_result(rospy.Duration(5.0)):
            res = self._gripper_1_client.get_result()
            print(res)

    def open_gripper_direct(self, pos=0.77, time=1):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["robot1/gripper"]
        point = JointTrajectoryPoint()
        point.positions = [pos]
        point.time_from_start = rospy.Duration(time)
        goal.trajectory.points.append(point)
        self._gripper_1_client.send_goal_and_wait(goal)
        
        if not self._gripper_1_client.wait_for_result(rospy.Duration(5.0)):
            self._gripper_1_client.get_result()

    def move_arm_to_cartesian_pose(self, goal_pose,):
        self.arm_group1.set_start_state_to_current_state()
        self.arm_group1.set_pose_target(goal_pose)
        success = self.arm_group1.go(wait=True)
        self.arm_group1.stop()
        self.arm_group1.clear_pose_targets()

    def move_arm_to_named_pose(self, pose_name):
        self.arm_group1.set_start_state_to_current_state()
        self.arm_group1.set_named_target(pose_name)
        success = self.arm_group1.go(wait=True)
        self.arm_group1.stop()
        self.arm_group1.clear_pose_targets()

    def move_arm_to_joint_pose(self, goal_pose):
        self.arm_group1.set_start_state_to_current_state()
        self.arm_group1.set_joint_value_target(goal_pose)
        success = self.arm_group1.go(wait=True)
        self.arm_group1.stop()
        self.arm_group1.clear_pose_targets()

    def move_arm_to_joint_pose_direct(self, goal_pose, time=3):
        if goal_pose is not None:
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = [self.prefix+"/joint1", self.prefix+"/joint2", self.prefix+"/joint3",
                                            self.prefix+"/joint4", self.prefix+"/joint5", self.prefix+"/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = goal_pose
            joint_angle.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(joint_angle)
            self._arm_client.send_goal(goal)

            if self._arm_client.wait_for_result(rospy.Duration(10.0)):
                return self._arm_client.get_result()

    def get_ik_request(self, pose):
        
        ik_request = moveit_msgs.msg.PositionIKRequest()
        ik_request.group_name = self.arm_group_name
        ik_request.ik_link_name = self.robot_eef_frame
        ik_request.pose_stamped = pose
        ik_request.timeout = rospy.Duration(5.0)
        ik_request.attempts = 20

        # send ik request to moveit server
        try:
            resp = self.ik_service(ik_request)
        except rospy.ServiceException:
            rospy.logerr("Service call failed: %s" )
            return None

        # check if ik solution is found
        if resp.error_code.val != resp.error_code.SUCCESS:
            rospy.logerr("IK solution not found")
            return None

class DualArmCommandInterface(object):
    def __init__(self, prefix1, prefix2):
        self.prefix1 = prefix1
        self.robot1_num = int(prefix1[-1])
        self.prefix2 = prefix2
        self.robot2_num = int(prefix2[-1])

        # moveit groups names
        self.arm1_2_group_name = "arm" + str(self.robot1_num) + "_arm" + str(self.robot2_num)
        self.gripper1_2_group_name = "gripper" + str(self.robot1_num) + "_gripper" + str(self.robot2_num)

        # set planning parameters
        self.NUM_PLANNING_ATTEMPTS = 30
        self.PLANNING_TIME = 20.0
        self.MAX_VELOCITY_SCALING_FACTOR = self.vel_scale
        self.MAX_ACCELERATION_SCALING_FACTOR = self.accel_scale
        
        self.arm1_grasp_pub = rospy.Publisher(prefix1 + '/grasp_pose', PoseStamped, queue_size=1)
        self.arm2_grasp_pub = rospy.Publisher(prefix2 + '/grasp_pose', PoseStamped, queue_size=1)
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.init_follow_joint_trajectory_client()
        self.init_moveit()

    def init_follow_joint_trajectory_client(self):
        self._arm1_2_client = actionlib.SimpleActionClient(self.prefix1 + '/arm1_2_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self._arm1_2_client.wait_for_server()

    def init_moveit(self):
        # moveit planning scene
        self.planning_scene = moveit_commander.PlanningSceneInterface()

        # moveit groups
        self.arm1_2_group = moveit_commander.MoveGroupCommander(self.arm1_2_group_name)
        self.gripper1_2_group = moveit_commander.MoveGroupCommander(self.gripper1_2_group_name)

        # set planning parameters
        self.arm1_2_group.set_num_planning_attempts(self.NUM_PLANNING_ATTEMPTS)
        self.arm1_2_group.set_planning_time(self.PLANNING_TIME)
        self.arm1_2_group.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)
        self.arm1_2_group.set_max_acceleration_scaling_factor(self.MAX_ACCELERATION_SCALING_FACTOR)
        self.gripper1_2_group.set_max_velocity_scaling_factor(self.MAX_VELOCITY_SCALING_FACTOR)
        self.gripper1_2_group.set_max_acceleration_scaling_factor(self.MAX_ACCELERATION_SCALING_FACTOR)

    def move_arm_to_cartesian_pose(self, goal_pose,):
        pass

    def move_arm_to_named_pose(self, pose_name):
        pass

    def move_arm_to_joint_pose(self, goal_pose):
        pass

    def move_arm_to_joint_pose_direct(self, goal_pose, time=3):
        pass

