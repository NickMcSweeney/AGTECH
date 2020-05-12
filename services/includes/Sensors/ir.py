#!/usr/bin/env python
from std_msgs.msg import Float64
import ros
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from getter import Getter

class IrCamera(Getter):
    ###
    # IR sensor class extending the getter, for handling the publishing of data 
    ###

    def init(self, ros, hostname): 
        # create publisher
        self.pub = ros.Publisher('/'+hostname+'/ir/'self.name)
        # pass to parent to enble probes
        Getter.init(self)

    def run(self, q):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        data = q[self.name]
        
        # TODO: format data for publisher
        
        # publish data
        Getter.run(self,data)
    
