#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
#from tf.transformations import euler_from_quaternion
#import tf

import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../scripts"))
sys.path.append(SCRIPTS_PATH)
from setter import Setter
from utils import *
from mp import *

# Global variables
host_name = "hai-1095.local"

class DriveController(Setter):
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

class ArmController(Setter):
    def init(self):
        Setter.init(self,1077)
    def callback(self, cmd_msg):
        pos = cmd_msg.data
        if(pos > 100):
            pos = 100
        elif(pos < 0):
            pos = 0
        Setter.callback(self,pos)

class GripperController(Setter):
    def init(self):
        Setter.init(self,1080)
    def callback(self, cmd_msg):
        pos = cmd_msg.data
        if(pos > 100):
            pos = 100
        elif(pos < 0):
            pos = 0
        Setter.callback(self,pos)

# main fn
if __name__ == '__main__':
    try:
        # Init the connection with the ROS system
        rospy.init_node("drive_node", anonymous=True)
        if len(sys.argv) < 2:
            host_name = "hai-1095.local"
        else:
            host_name = sys.argv[1]
        print("connected to {}").format(host_name)

        mp = Mindprobe()
        mp.init(host_name)

		### Drive Controller ###
        drive = DriveController(mp)
        drive.init()

        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel', Twist, drive.callback)
        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel_left', Float64, drive.set_left)
        rospy.Subscriber('/'+resource_name(host_name)+'/cmd_vel_right', Float64, drive.set_right)
		### --------------- ###

		### Arm Controller ###
        arm = ArmController(mp)
        arm.init()

        rospy.Subscriber('/'+resource_name(host_name)+'/set_arm_pos', Float64, arm.callback)
		### --------------- ###

		### Arm Controller ###
        gripper = GripperController(mp)
        gripper.init()

        rospy.Subscriber('/'+resource_name(host_name)+'/set_gripper_pos', Float64, gripper.callback)
		### --------------- ###
        
        # Start the ROS main loop
        rate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            rate.sleep()

        mp.close()
    except rospy.ROSInterruptException:
        mp.close()
        pass
