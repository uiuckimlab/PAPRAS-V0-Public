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
from aruco_msgs.msg import MarkerArray, Marker

# global_aruco_tag_position = [None, None, None]
aruco_tags = None
aruco_tag_ids = None
prev_sequence, curr_sequence = -1, -1
tolerance = 0.025 #m

def get_position_from_marker(marker):
    x,y,z = float(marker.pose.pose.position.x), float(marker.pose.pose.position.y), float(marker.pose.pose.position.z)
    return x,y,z

def aruco_tag_callback(data):
    global aruco_tags, aruco_tag_ids, curr_sequence

    aruco_tags = data.markers
    aruco_tag_ids = []
    for tag in data.markers:
        aruco_tag_ids.append(int(tag.id))
    # x,y,z = float(data.pose.position.x), float(data.pose.position.y), float(data.pose.position.z)
    # global_aruco_tag_position = [x,y,z]

    curr_sequence = int(data.header.seq)

def find_tag(publisher, goal_tag_id):
    global aruco_tag_ids, prev_sequence, curr_sequence
    vel_cmd = Twist()

    while prev_sequence == curr_sequence or goal_tag_id not in aruco_tag_ids:
        # no updated tag position is being received (i.e. the camera does not detect the aruco tag)
        # or can't find specific aruco tag id
        vel_cmd.angular.z = 0.15
        publisher.publish(vel_cmd)

    vel_cmd.angular.z = 0.0
    publisher.publish(vel_cmd)

    prev_sequence = curr_sequence

def roomba_nav(publisher, goal_tag_id):
    global aruco_tags, aruco_tag_ids, tolerance

    if goal_tag_id not in aruco_tag_ids:
        print("Can't find the goal aruco tag. Leave nav to stall in search.")

    x,y,z = None, None, None
    for tag in aruco_tags:
        if int(tag.id) == goal_tag_id:
            x,y,z = get_position_from_marker(tag)
            break

    if x == None:
        print("Error! Something is wrong. Leave nav to stall in search.")

    vel_cmd = Twist()
    vel_cmd.linear.x = 0
    vel_cmd.angular.z = 0

    if x > tolerance:
        # aruco tag to the right of the camera
        vel_cmd.angular.z = -0.1
    elif x < -tolerance:
        # aruco tag to the left of the camera
        vel_cmd.angular.z = 0.1

    if z > 0.2:
        # aruco tag is more than 2m away
        vel_cmd.linear.x = 0.2
    elif z > 0.12:
        # aruco tag is more than 1m away
        vel_cmd.linear.x = 0.1
    elif z < 0.08:
        # aruco tag is too close, less than 0.5m away
        vel_cmd.linear.x = -0.1

    publisher.publish(vel_cmd)

def main():
    publisher = rospy.Publisher('/roomba/cmd_vel', Twist, queue_size=10)
    subscriber = rospy.Subscriber("/aruco_tracker/markers", MarkerArray, aruco_tag_callback)
    rospy.init_node('roomba_nav', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    goal_tag_id = 0

    while not rospy.is_shutdown():

        find_tag(publisher, goal_tag_id)
        roomba_nav(publisher, goal_tag_id)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("Error: Failed to start ROOMBA Navigation!")
