#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../scripts"))
sys.path.append(SCRIPTS_PATH)
from utils import *

# Global variables
host_name = "hai-1095.local"
pub = None

# main fn
if __name__ == '__main__':
    try:
        # Init the connection with the ROS system
        rospy.init_node("drive_test", anonymous=True)

        pub = rospy.Publisher('/'+resource_name(host_name)+'/cmd_vel', Twist, queue_size=10)
        
        # Start the ROS main loop
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()
            # Velocity message to send to the robot
            vel_msg = Twist()
            vel_msg.linear.x = 0.0;  # ...linear velocity
            vel_msg.angular.z = 0.0; # ...rotational velocity
            pub.publish(vel_msg);


    except rospy.ROSInterruptException:
        pass
