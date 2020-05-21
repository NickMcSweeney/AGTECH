#!/usr/bin/env python
from std_msgs.msg import Float64
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from setter import Setter

class GripperController(Setter):
    ###
    # Controller for the robot gripper
    # Extends the Setter class
    ###
    def callback(self, cmd_msg):
        # callback function - extends the setter.callback functionality
        # formats data to be passed to mindprobe
        pos = cmd_msg.data
        if(pos > 100):
            pos = 100
        elif(pos < 0):
            pos = 0
        Setter.callback(self,pos)
    def run(self, rospy, hostname):
        # intializes the mp connection and runs a subscriber for setting the gripper position.
        Setter.init(self,1080)
		
        rospy.Subscriber('/'+hostname+'/set_gripper_pos', Float64, self.callback)
