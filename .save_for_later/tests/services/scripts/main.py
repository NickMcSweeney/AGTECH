#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
#from tf.transformations import euler_from_quaternion
#import tf

# Global variables
pub = None

def init_controller(code):
    rospy.loginfo("Initializing connection")
	# startup controller

	# enable controller
	

def cmd_cb(vel_msg):
    # Uncomment the following line to printout the pose of the robot
    rospy.loginfo("setting a velocity for the Robot")

def looping():
    rospy.loginfo("itteration here")


# main fn
if __name__ == '__main__':
    try:

        # Init the connection with the ROS system
        rospy.init_node("main", anonymous=True)

        # We need to tell ROS that we are interested in receiving 
        # messages that need to be handled by the pose_cb fn.
        rospy.Subscriber('/hai-1095/cmd_vel', Twist, cmd_cb)
        
        # Start the ROS main loop - calls pose_cb once for each iteration. 
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            # get it going
            looping()
            # loop
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
