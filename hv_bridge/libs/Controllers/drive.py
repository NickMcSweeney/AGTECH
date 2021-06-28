#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import sys, os

RESOURCE_PATH = os.path.abspath(os.path.join(sys.path[0], "../../"))
sys.path.append(RESOURCE_PATH)

from libs.Resources import Writer
from libs.Resources.utils import *


class DriveController(Writer):
    ###
    # Controller for the robot drive
    # extends the Writer parent class
    # handles differential drive controll for the robot
    ###
    def __init__(self, hostname):
        Writer.__init__(
            self, ["mob.driveControlWheelSpeeds.l", "mob.driveControlWheelSpeeds.r"]
        )
        # runs a subscriber for setting the wheel velocity.
        rospy.Subscriber("/" + hostname + "/cmd_vel", Twist, self.callback)

    def callback(self, cmd_msg):
        # differential drive callback.
        vel = twist_to_vel(cmd_msg)
        Writer.callback(self, vel)
