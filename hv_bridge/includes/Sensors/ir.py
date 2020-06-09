#!/usr/bin/env python
from std_msgs.msg import Float32
import rospy
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from getter import Getter
from utils import resource_name

class IrCamera(Getter):
    ###
    # IR sensor class extending the getter, for handling the publishing of data 
    ###

    def init(self, ros, hostname): 
        self.hostname = hostname
        # create publisher
        if type(self.name) != tuple:
            self.pub = ros.Publisher('/'+hostname+'/ir/'+resource_name(self.name),Float32 ,queue_size=1)
        else:
            self.pub = ros.Publisher('/'+hostname+'/ir/'+resource_name(os.path.commonprefix(self.name)),Float32,queue_size=1)
        # pass to parent to enble probes
        Getter.init(self)

    def run(self, q, current_time):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        data = []
        labels = self.name
        if type(labels) != tuple:
            labels = (labels)

        for label in labels:
            data.append(q[label])

        # publish data
        Getter.run(self,data[0])
    
