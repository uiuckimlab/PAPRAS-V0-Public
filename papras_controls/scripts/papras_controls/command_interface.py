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
    def __init__(self, prefix='robot1', vel_scale=0.2, accel_scale=0.2):    
        self.prefix = prefix
        self.robot_num = int(prefix[-1])
        self.arm_group_name = "arm" + str(self.robot_num)
        self.gripper_group_name = "gripper" + str(self.robot_num)

        # set planning parameters
        self.NUM_PLANNING_ATTEMPTS = 30
        self.PLANNING_TIME = 20.0
        self.MAX_VELOCITY_SCALING_FACTOR = vel_scale
        self.MAX_ACCELERATION_SCALING_FACTOR = accel_scale
        
        self.marker_pub = rospy.Publisher(prefix + '/grasp_pose', PoseStamped, queue_size=1)
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

        self.init_follow_joint_trajectory_client()
        self.init_moveit()

    def init_follow_joint_trajectory_client(self):
        print("in init_follow_joint_trajectory_client")

        self._arm_client = actionlib.SimpleActionClient("/arm"+str(self.robot_num)+"_controller/follow_joint_trajectory",
                                                          FollowJointTrajectoryAction)
        if not self._arm_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Arm {} Action Server Not Found".format(self.robot_num))
            rospy.signal_shutSingleArmCommandInterfacedown("Arm 1 Action Server not found")
            sys.exit(1)
        print("Arm {} Action Server Found".format(self.robot_num))

        self._gripper_client = actionlib.SimpleActionClient("/gripper"+str(self.robot_num)+"_controller/follow_joint_trajectory",
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
        self.gripper_group.set_named_target("close")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()

    def open_gripper(self):
        self.gripper_group.set_named_target("open")
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()

    def move_gripper_direct(self, pos=1.08, time=1):
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

    def move_arm_to_cartesian_pose(self, goal_pose):
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_pose_target(goal_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def move_arm_to_named_pose(self, pose_name):
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_named_target(pose_name)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

    def move_arm_to_joint_pose(self, goal_pose):
        self.arm_group.set_start_state_to_current_state()
        self.arm_group.set_joint_value_target(goal_pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()

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
    '''
    Class for interfacing with the dual arm robot
    
    Parameters
    ----------
    prefix1 : str
        prefix for the first robot
    prefix2 : str
        prefix for the second robot
        
    Attributes
    ----------
    prefix1 : str
        prefix for the first robot
    robot1_num : int
        number of the first robot
    prefix2 : str
        prefix for the second robot
    robot2_num : int
        number of the second robot
    arm1_2_group_name : str
        moveit group name for the first arm
    gripper1_2_group_name : str
        moveit group name for the first gripper
    NUM_PLANNING_ATTEMPTS : int
        number of planning attempts
    PLANNING_TIME : float
        planning time
    MAX_VELOCITY_SCALING_FACTOR : float
        maximum velocity scaling factor
    MAX_ACCELERATION_SCALING_FACTOR : float
        maximum acceleration scaling factor

    Methods
    -------
    move_arm_to_cartesian_pose(self, goal_pose)
        Move the arm to a cartesian pose
    move_arm_to_named_pose(self, pose_name)
        Move the arm to a named pose
    move_arm_to_joint_pose(self, goal_pose)
        Move the arm to a joint pose
    move_arm_to_joint_pose_direct(self, goal_pose, time=3)
        Move the arm to a joint pose directly
    close_gripper(self)
        Close the gripper to a predefined position using moveit (collision checking enabled)
    open_gripper(self)
        Open the gripper to a predefined position using moveit (collision checking enabled)
    move_gripper_direct(self, pos=0.77, time=1)
        Move the gripper directly to a specified position (no collision checking)
        '''
    def __init__(self, prefix1='robot1', prefix2='robot2', vel_scale=0.2, accel_scale=0.2):
        self.prefix1 = prefix1
        self.robot1_num = int(prefix1[-1])
        self.prefix2 = prefix2
        self.robot2_num = int(prefix2[-1])

        # set planning parameters
        self.NUM_PLANNING_ATTEMPTS = 30
        self.PLANNING_TIME = 20.0
        self.MAX_VELOCITY_SCALING_FACTOR = vel_scale
        self.MAX_ACCELERATION_SCALING_FACTOR = accel_scale

        self.robot1 = SingleArmCommandInterface(prefix1, vel_scale, accel_scale)
        self.robot2 = SingleArmCommandInterface(prefix2, vel_scale, accel_scale)

        # moveit groups names
        self.arm1_group = self.robot1.arm_group
        self.arm2_group = self.robot2.arm_group
        self.gripper1_group = self.robot1.gripper_group
        self.gripper2_group = self.robot2.gripper_group
        self.arm1_2_group_name = "arm" + str(self.robot1_num) + "_" + str(self.robot2_num)
        self.gripper1_2_group_name = "gripper" + str(self.robot1_num) + "_" + str(self.robot2_num)
        
        self.arm1_grasp_pub = rospy.Publisher(prefix1 + '/grasp_pose', PoseStamped, queue_size=1)
        self.arm2_grasp_pub = rospy.Publisher(prefix2 + '/grasp_pose', PoseStamped, queue_size=1)
        self.ik_service = rospy.ServiceProxy('compute_ik', GetPositionIK)

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
        self.gripper1_2_group.set_max_acceleration_scaling_factor(0.3)

    def move_arm_to_cartesian_pose(self, goal_pose, move_group):
        '''
        Move arm to cartesian pose
        :param goal_pose: geometry_msgs.msg.PoseStamped (pose of the end effector)
        :param move_group: moveit_commander.MoveGroupCommander (moveit group)
        :return: None
        '''
        if goal_pose is not None and move_group is not None:
            assert(goal_pose.header.frame_id == move_group.get_planning_frame())
            move_group.set_pose_target(goal_pose)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

    def move_arm_to_named_pose(self, pose_name, move_group):
        '''
        Move arm to named pose
        :param pose_name: string (name of the pose)
        :param move_group: moveit_commander.MoveGroupCommander (moveit group)
        :return: None
        '''
        if pose_name is not None and move_group is not None:
            if pose_name not in move_group.get_named_targets():
                rospy.logerr("Named pose " + pose_name + " does not exist in movegroup " + move_group.get_name())
                return
            move_group.set_named_target(pose_name)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

    def move_arm_to_joint_pose(self, goal_pose, move_group):
        '''
        Move arm to joint pose
        :param goal_pose: list of floats (joint values)
        :param move_group: moveit_commander.MoveGroupCommander (moveit group)
        :return: None
        '''
        if goal_pose is not None and move_group is not None:
            if len(goal_pose) != len(move_group.get_current_joint_values()):
                rospy.logerr("Joint pose dimension does not match movegroup " + move_group.get_name())
                return
            move_group.set_joint_value_target(goal_pose)
            success = move_group.go(wait=True)
            move_group.stop()
            move_group.clear_pose_targets()

    def move_arm_to_joint_pose_direct(self, goal_pose, robot_num=-1, time=3):
        '''
        Move arm to joint pose
        :param goal_pose: list of floats (joint values, dim=12)
            for both arm ->  joint order: 
            [robot1/joint1, robot1/joint2, robot1/joint3, robot1/joint4, 
                robot1/joint5, robot1/joint6, robot2/joint1, robot2/joint2, 
                robot2/joint3, robot2/joint4, robot2/joint5, robot2/joint6]
        :param robot_num: int (1 or 2 or -1) where -1 means both arms
        :param time: float (time to reach goal pose)
        :return: None
        '''
        if goal_pose is not None and robot_num ==1:
            self.robot1.move_arm_to_joint_pose_direct(goal_pose, time)
        elif goal_pose is not None and robot_num ==2:
            self.robot2.move_arm_to_joint_pose_direct(goal_pose, time)
        elif goal_pose is not None and robot_num != 1 and robot_num != 2:
            robot1_goal_pose = goal_pose[:6]
            robot2_goal_pose = goal_pose[6:]
            arm1_client = self.robot1._arm_client
            arm2_client = self.robot2._arm_client
            
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = [self.prefix1+"/joint1", self.prefix1+"/joint2", self.prefix1+"/joint3",
                                            self.prefix1+"/joint4", self.prefix1+"/joint5", self.prefix1+"/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = robot1_goal_pose
            joint_angle.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(joint_angle)
            arm1_client.send_goal(goal)

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = [self.prefix2+"/joint1", self.prefix2+"/joint2", self.prefix2+"/joint3",
                                            self.prefix2+"/joint4", self.prefix2+"/joint5", self.prefix2+"/joint6"]
            joint_angle = JointTrajectoryPoint()
            joint_angle.positions = robot2_goal_pose
            joint_angle.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(joint_angle)
            arm2_client.send_goal(goal)

            if arm1_client.wait_for_result(rospy.Duration(10.0)) and arm2_client.wait_for_result(rospy.Duration(10.0)):
                rospy.loginfo("Both arms reached goal pose")
                return arm1_client.get_result(), arm2_client.get_result()
            else:
                rospy.logerr("Both arms failed to reach goal pose")
                arm1_client.cancel_goal()
                arm2_client.cancel_goal()
                return None, None

    def close_gripper(self, gripper_move_group):
        '''
        Close gripper
        :param gripper_move_group: moveit_commander.MoveGroupCommander (moveit group)
        :return: None
        '''
        if gripper_move_group is not None:
            gripper_move_group.set_named_target('close')
            success = gripper_move_group.go(wait=True)
            if success:
                rospy.loginfo("Gripper closed")
            else:
                rospy.logerr("Gripper failed to close")
            gripper_move_group.stop()
            gripper_move_group.clear_pose_targets()

    def open_gripper(self, gripper_move_group):
        '''
        Open gripper
        :param gripper_move_group: moveit_commander.MoveGroupCommander (moveit group)
        :return: None
        '''
        if gripper_move_group is not None:
            gripper_move_group.set_named_target('open')
            success = gripper_move_group.go(wait=True)
            if success:
                rospy.loginfo("Gripper opened")
            else:
                rospy.logerr("Gripper failed to open")
            gripper_move_group.stop()
            gripper_move_group.clear_pose_targets()

    def move_gripper_direct(self, pos=1.08, robot_num=-1, time=1):
        '''
        Move gripper directly to specified position (without planning or collision checking)
        :param pos: float (0.0~1.08) where 0.0 is fully open and 1.08 is fully closed.
        :param robot_num: int (1 or 2 or -1) where -1 means both grippers
        :param time: float
        :return: None
        '''
        if robot_num == 1:
            self.robot1.move_gripper_direct(pos, time)
        elif robot_num == 2:
            self.robot2.move_gripper_direct(pos, time)
        elif robot_num != 1 and robot_num != 2:
            gripper1_client = self.robot1._gripper_client
            gripper2_client = self.robot2._gripper_client

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot1/gripper"]
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(point)
            gripper1_client.send_goal(goal)

            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ["robot2/gripper"]
            point = JointTrajectoryPoint()
            point.positions = [pos]
            point.time_from_start = rospy.Duration(time)
            goal.trajectory.points.append(point)
            gripper2_client.send_goal(goal)

            if gripper1_client.wait_for_result(rospy.Duration(10.0)) and gripper2_client.wait_for_result(rospy.Duration(10.0)):
                rospy.loginfo("Both grippers reached goal pose")
                return gripper1_client.get_result(), gripper2_client.get_result()
            else:
                rospy.logerr("Failed to move gripper")
                
            