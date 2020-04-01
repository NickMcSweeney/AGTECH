#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
#from tf.transformations import euler_from_quaternion
#import tf

import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../scripts"))
sys.path.append(SCRIPTS_PATH)
from setter import Setter
from utils import *

# Global variables
host_name = "hai-1095.local"

class Controller(Setter):
    def init(self):
        Setter.init(self,(1207,1206))
    def callback(self, cmd_msg):
        vel = twist_to_vel(cmd_msg)
        Setter.callback(self,vel)

# main fn
if __name__ == '__main__':
    try:
        # Init the connection with the ROS system
        rospy.init_node("drive_node", anonymous=True)

        controller = Controller(host_name, 4400)
        controller.init()

        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel', Twist, controller.callback)
        
        # Start the ROS main loop
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
