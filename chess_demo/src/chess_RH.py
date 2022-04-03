#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def chess_RR():
    publisher = rospy.Publisher('chess_move', String, queue_size=10)
    rospy.init_node('chess_RR', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        move = "Arm1,A1,A2"
        publisher.publish(move)
        rate.sleep()

if __name__ == '__main__':
    try:
        chess_RR()
    except rospy.ROSInterruptException:
        print("Failed to launch chess demo RvR!")




'''
Output: "Arm, Starting Position, Ending Position"
        "Arm1,C3,D8" -- comma delimited, no spaces
'''
