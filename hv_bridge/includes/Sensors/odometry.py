#!/usr/bin/env python
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, Twist, Vector3, Quaternion, Point
import rospy
import tf
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from getter import Getter
from utils import resource_name

class Odometer(Getter):
    ###
    # Odometer class extending the getter, for handling the publishing of data 
    ###

    def init(self, ros, hostname): 
        self.hostname = hostname
        # create publisher
        if type(self.name) != tuple:
            self.pub = ros.Publisher('/'+hostname+'/odom/'+resource_name(self.name),Odometry ,queue_size=1)
        else:
            self.pub = ros.Publisher('/'+hostname+'/odom/'+resource_name(os.path.commonprefix(self.name)),Odometry ,queue_size=1)
        # pass to parent to enble probes
        Getter.init(self)

    def run(self, q, current_time):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.hostname
        
        data = []
        labels = self.name
        if type(labels) != tuple:
            labels = (labels)

        for label in labels:
            data.append(q[label])
            
        heading =  tf.transformations.quaternion_from_euler(0. , 0. , data[2])
        odom.pose.pose = Pose(Point(data[0],data[1],0.),Quaternion(heading[0],heading[1],heading[2],heading[3]))
        odom.twist.twist = Twist(Vector3(data[3],0,0),Vector3(0,0,data[4]))
        
        # publish data
        Getter.run(self,odom)
    
