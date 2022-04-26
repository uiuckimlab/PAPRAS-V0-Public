#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from controller_manager_msgs.srv import SwitchController

def switch_controller_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.wait_for_service('controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        resp = switch_controller(["arm1_controller"],["arm1_controller"], 2, False, 0.0)
        print("Service call success: %s"%resp.ok)
        return resp.ok
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def switch_controller_node():
    rospy.init_node('switch_controller_node', anonymous=True)
    rospy.Subscriber("switch_controller", Bool, switch_controller_callback)
    rospy.spin()

if __name__ == '__main__':
    switch_controller_node()