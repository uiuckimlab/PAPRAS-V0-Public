#!/usr/bin/env python3

import rospy
from aruco_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped, Twist
import tf2_ros 
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion
import numpy as np
import quaternion
import threading

class VisualServo():
    def __init__(self):
        rospy.init_node('visual_servo_control_law', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        rospy.sleep(1.0)

        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.aruco_callback)
        self.pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)
        self.object_pose = None

        self.lock = threading.Lock()

    
    def aruco_callback(self, data):
        
        frame = data.header.frame_id
        marker = data.markers[0]
        id = marker.id

        self.lock.acquire()
        self.object_pose = marker.pose  
        self.lock.release()

    def calc_deltaX(self):
        '''
        X_ec: transform from camera to eef frame
        X_cm: marker pose in camera frame
        X_gm: 
        '''
        if self.object_pose is None:
            return 
        self.lock.acquire()

        X_gc = self.tfBuffer.lookup_transform( "robot1/end_effector_link", "robot1/camera_link", rospy.Time())
        object_pose = tf2_geometry_msgs.do_transform_pose(self.object_pose, X_gc)
        rot_quat = quaternion_from_euler(-np.pi/2, np.pi/2, 0) 
        obj_orn_quat = [object_pose.pose.orientation.x, 
                        object_pose.pose.orientation.y, 
                        object_pose.pose.orientation.z, 
                        object_pose.pose.orientation.w]        

        servo_pose = PoseStamped()
        servo_pose.header = object_pose.header
        servo_pose.pose.position = object_pose.pose.position
        servo_pose_orn = quaternion_multiply(obj_orn_quat, rot_quat)
        servo_pose.pose.orientation.x = servo_pose_orn[0]
        servo_pose.pose.orientation.y = servo_pose_orn[1]
        servo_pose.pose.orientation.z = servo_pose_orn[2]
        servo_pose.pose.orientation.w = servo_pose_orn[3]

        self.pose_pub.publish(servo_pose)

        servo_pose.pose.position.x -= 0.30
        pose1 = Pose()
        pose1.orientation.w = 1.0
        pose2 = servo_pose.pose
        # pose2 = 0.1*pose2 + 0.9* servo_pose.pose

        twist_cmd = self.pose_interp(1, 0, 1, pose1, pose2)
        self.twist_pub.publish(twist_cmd)
        
        self.lock.release()
        

    def pose_interp(self, t, t1, t2, pose1, pose2):
        '''
        Take poses at times t1 and t2. Interpolate the pose at time t.
        Output: Twist
        '''
        assert( (t1 <= t) and (t <= t2))
        alpha = 0.0
        if t2 != t1:
            alpha = (t - t1) / (t2 - t1)

        trans1 = np.asarray([pose1.position.x, pose1.position.y, pose1.position.z])
        trans2 = np.asarray([pose2.position.x, pose2.position.y, pose2.position.z])

        # import ipdb; ipdb.set_trace()
        q1 = np.quaternion(pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w)
        q2 = np.quaternion(pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w)

        translation = (1.0 - alpha) * trans1 + alpha * trans2
        # Spherical Linear Interpolation of Rotations.
        rotation = quaternion.slerp_evaluate(q1, q2, alpha)

        spatial_vel = TwistStamped()
        spatial_vel.header.frame_id = "robot1/end_effector_link"
        spatial_vel.header.stamp = rospy.Time.now()
        spatial_vel.twist.linear.x = translation[0] / t
        spatial_vel.twist.linear.y = translation[1] / t
        spatial_vel.twist.linear.z = translation[2] / t
        spatial_vel.twist.angular.x = 0#rotation[0] / t
        spatial_vel.twist.angular.y = 0#rotation[1] / t
        spatial_vel.twist.angular.z = 0#rotation[2] / t
        return spatial_vel

        

if __name__ == '__main__':
    vs = VisualServo()

    rate = rospy.Rate(30) # ROS Rate at 5Hz
    while not rospy.is_shutdown():
        vs.calc_deltaX()
        rate.sleep()
