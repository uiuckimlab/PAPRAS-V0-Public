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

'''
http://docs.ros.org/en/noetic/api/moveit_commander/html/move__group_8py_source.html
'''
class SingleArmCommandInterface:
    def __init__(self):        
        self.robot1_eef_frame = "robot1/end_effector_link"
        self.marker_pub = rospy.Publisher('grasp_pose', PoseStamped, queue_size=1)
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

        # self.init_follow_joint_trajectory_client()
        self.init_moveit()

    def init_follow_joint_trajectory_client(self):
        print("in init")

        self._arm_1_client = actionlib.SimpleActionClient("/arm1_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_1_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm 1 Action Server Not Found")
            rospy.signal_shutdown("Arm 1 Action Server not found")
            sys.exit(1)
        print("Arm 1 Action Server Found")

        self._gripper_1_client = actionlib.SimpleActionClient("/gripper1_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._gripper_1_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Gripper 1 Action Server Not Found")
            rospy.signal_shutdown("Gripper 1 Action Server not found")
            sys.exit(1)
        print("Gripper 1 Action Server Found")

    def init_moveit(self):
        # moveit groups
        self.arm_group1 = moveit_commander.MoveGroupCommander("arm1")
        self.gripper_group1 = moveit_commander.MoveGroupCommander("gripper1")

        # planning scene  
        self.planning_scene = moveit_commander.PlanningSceneInterface()

        # set planning parameters
        NUM_PLANNING_ATTEMPTS = 30
        PLANNING_TIME = 20.0
        MAX_VELOCITY_SCALING_FACTOR = 0.8
        MAX_ACCELERATION_SCALING_FACTOR = 0.8

        self.arm_group1.set_num_planning_attempts(NUM_PLANNING_ATTEMPTS)
        self.arm_group1.set_planning_time(PLANNING_TIME)
        self.arm_group1.set_max_velocity_scaling_factor(MAX_VELOCITY_SCALING_FACTOR)
        self.arm_group1.set_max_acceleration_scaling_factor(MAX_ACCELERATION_SCALING_FACTOR)
        self.gripper_group1.set_max_velocity_scaling_factor(MAX_VELOCITY_SCALING_FACTOR)
        self.gripper_group1.set_max_acceleration_scaling_factor(MAX_ACCELERATION_SCALING_FACTOR)

    def clear_collision_objects(self):
        # list all collision objects
        collision_objects = self.planning_scene.get_known_object_names()
        # remove all collision objects
        if len(collision_objects) > 0:
            self.planning_scene.remove_world_object(collision_objects)

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
            goal.trajectory.joint_names = ["robot1/joint1", "robot1/joint2", "robot1/joint3",
                                            "robot1/joint4", "robot1/joint5", "robot1/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = goal_pose
            joint_angle.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(joint_angle)
            self._arm_1_client.send_goal(goal)

            if self._arm_1_client.wait_for_result(rospy.Duration(10.0)):
                return self._arm_1_client.get_result()

    def get_ik_request(self, pose):
        
        ik_request = moveit_msgs.msg.PositionIKRequest()
        ik_request.group_name = "arm1"
        ik_request.ik_link_name = "robot1/end_effector_link"
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
