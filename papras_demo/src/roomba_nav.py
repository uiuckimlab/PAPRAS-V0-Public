#!/usr/bin/env python3
'''
Roomba Navigation using ARUCO tags.

Twist
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
'''

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseWithCovariance, PoseStamped

global_aruco_tag_position = [None, None, None]
prev_sequence, curr_sequence = -1, -1
tolerance = 0.025 #m

def aruco_tag_callback(data):
    global global_aruco_tag_position, curr_sequence

    x,y,z = float(data.pose.position.x), float(data.pose.position.y), float(data.pose.position.z)
    global_aruco_tag_position = [x,y,z]

    curr_sequence = int(data.header.seq)

def find_tag(publisher):
    global global_aruco_tag_position, prev_sequence, curr_sequence
    vel_cmd = Twist()

    while prev_sequence == curr_sequence:
        # no updated tag position is being received (i.e. the camera does not detect the aruco tag)
        vel_cmd.angular.z = 0.1
        publisher.publish(vel_cmd)

    vel_cmd.angular.z = 0.0
    publisher.publish(vel_cmd)

    prev_sequence = curr_sequence

def roomba_nav(publisher):
    global global_aruco_tag_position, tolerance

    x,y,z = global_aruco_tag_position

    vel_cmd = Twist()
    vel_cmd.linear.x = 0
    vel_cmd.angular.z = 0

    if x > tolerance:
        # aruco tag to the right of the camera
        vel_cmd.angular.z = -0.1
    elif x < -tolerance:
        # aruco tag to the left of the camera
        vel_cmd.angular.z = 0.1

    if z > 1:
        # aruco tag is more than 1m away
        vel_cmd.linear.x = 0.1
    elif z < 0.5:
        # aruco tag is too close, less than 0.5m away
        vel_cmd.linear.x = -0.1

    publisher.publish(vel_cmd)

def main():
    publisher = rospy.Publisher('/roomba/cmd_vel', Twist, queue_size=10)
    subscriber = rospy.Subscriber("/aruco_tracker/pose", PoseStamped, aruco_tag_callback)
    rospy.init_node('roomba_nav', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        find_tag(publisher)
        roomba_nav(publisher)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Error: Failed to start ROOMBA Navigation!")
