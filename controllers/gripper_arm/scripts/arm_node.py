#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../scripts"))
sys.path.append(SCRIPTS_PATH)
from setter import Setter
from utils import *

# Global variables
host_name = "HAI-1095"

class Controller(Setter):
    def init(self):
        Setter.init(0000)
    def callback(self, cmd_msg):
        Setter.callback(cmd_msg.data)

# main fn
if __name__ == '__main__':
    try:
        # Init the connection with the ROS system
        rospy.init_node("arm_node", anonymous=True)

        controller = Controller(host_name, 4400)

        rospy.Subscriber('/'+resource_name(host_name)+'/set_arm_pos', Float64, controller.callback)
        
        # Start the ROS main loop
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
