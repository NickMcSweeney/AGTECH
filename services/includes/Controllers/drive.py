#!/usr/bin/env python
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from setter import Setter
from utils import *

class DriveController(Setter):
    ###
    # Controller for the robot drive
    # extends the setter parent class
    # handles individual wheel commands as well as a differential drive
    ###
    def callback(self, cmd_msg):
        # differential drive callback. 
        vel = twist_to_vel(cmd_msg)
        Setter.callback(self,vel)
    def set_left(self,cmd_msg):
        # callback to set the left wheel
        vel = cmd_msg.data
        Setter.callback(self,[vel,0])
    def set_right(self,cmd_msg):
        # callback to set the right wheel
        vel = cmd_msg.data
        Setter.callback(self,[0,vel])
    def run(self, rospy, hostname):
        # initialize the connection and create subscribers
        Setter.init(self,(1206,1207))

        rospy.Subscriber('/'+hostname+'/cmd_vel', Twist, self.callback)
        rospy.Subscriber('/'+hostname+'/cmd_vel_left', Float64, self.set_left)
        rospy.Subscriber('/'+hostname+'/cmd_vel_right', Float64, self.set_right)
