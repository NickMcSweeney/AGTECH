#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
#import tf

# Global variables
pub = None

'''
pose_cb is callback function that is called every time there is new data 
available on the topic we are interested in listening to. In this case, 
we are interested in the "/robot_0/base_pose_ground_truth" topic as 
specified below when calling the subscribe fn.
'''
def pose_cb(pose_msg):

    # Robot pose (Odometry)
    robot_pose = pose_msg.pose.pose
    pos_x = robot_pose.position.x
    pos_y = robot_pose.position.y
    quat = robot_pose.orientation
    _, _, pos_th = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
    # Uncomment the following line to printout the pose of the robot
    #rospy.loginfo("Robot Pose: [%f; %f; %f]" % (pos_x, pos_y, pos_th))

    # ---------------------
    # Insert your code here 
    # ---------------------

    # Velocity message to send to the robot
    vel_msg = Twist()
    vel_msg.linear.x = 0.0;  # ...linear velocity
    vel_msg.angular.z = 0.0; # ...rotational velocity

    '''
    The publish() function is how you send messages. The parameter is the 
    message object. The type of this object must agree with the type given 
    as a parameter to the rospy.Publisher (see the main fn below).
    '''
    pub.publish(vel_msg);

# main fn
if __name__ == '__main__':
    try:

        # Init the connection with the ROS system
        rospy.init_node("lab1_node", anonymous=True)

        # We need to tell ROS that we are interested in receiving 
        # messages that need to be handled by the pose_cb fn.
        rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, pose_cb)

        # To tell ROS that we are going to publish velocity (Twist) 
        # messages, we use the advertise() fn.
        pub = rospy.Publisher('/robot_0/cmd_vel', Twist, queue_size=10)
        
        # Start the ROS main loop - calls pose_cb once for each iteration. 
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
