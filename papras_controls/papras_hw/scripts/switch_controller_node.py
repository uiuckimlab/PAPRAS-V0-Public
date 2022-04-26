#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from controller_manager_msgs.srv import SwitchController

def switch_controller_callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", data.data)

    num_arms = data.data
    controller_list = []
    for arm_id in range(num_arms):
        controller_list.append("arm{}_controller".format(arm_id+1))
        controller_list.append("gripper{}_controller".format(arm_id+1))

    print('controller list', controller_list)
    
    rospy.wait_for_service('controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        resp = switch_controller(controller_list, controller_list, 2, False, 0.0)
        print("Service call success: %s"%resp.ok)
        return resp.ok
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
    
def switch_controller_node():
    rospy.init_node('switch_controller_node', anonymous=True)
    rospy.Subscriber("switch_controller", Int32, switch_controller_callback)
    rospy.spin()

if __name__ == '__main__':
    switch_controller_node()