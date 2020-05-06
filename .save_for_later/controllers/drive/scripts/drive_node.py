#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
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
        Setter.init(self,(1206,1207))
    def callback(self, cmd_msg):
        vel = twist_to_vel(cmd_msg)
        Setter.callback(self,vel)
    def set_left(self,cmd_msg):
        vel = cmd_msg.data
        Setter.callback(self,[vel,0])
    def set_right(self,cmd_msg):
        vel = cmd_msg.data
        Setter.callback(self,[0,vel])

# main fn
if __name__ == '__main__':
    try:
        # Init the connection with the ROS system
        rospy.init_node("drive_node", anonymous=True)

        controller = Controller()
        controller.init()

        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel', Twist, controller.callback)
        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel_left', Float64, controller.set_left)
        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel_right', Float64, controller.set_right)
        
        # Start the ROS main loop
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
