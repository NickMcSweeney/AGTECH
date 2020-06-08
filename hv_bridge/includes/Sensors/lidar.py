#!/usr/bin/env python
from std_msgs.msg import Float64
from std_msgs.msg import Int16MultiArray
import time
import ros
import sys, os
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)
from getter import Getter
from utils import resource_name

class LidarCamera(Getter):
    ###
    # Lidar sensor class extending the getter, for handling the publishing of data 
    ###
    def init(self, ros, hostname): 
        # create publisher
        self.pub = ros.Publisher('/'+hostname+'/lidar/'+resource_name(self.name),Int16MultiArray,queue_size=1)
        # pass to parent to enble probes
        #self.mp.write_probe(self.codes,1)
        Getter.init(self)

    def run(self, q):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue


        data = q[self.name]
        
        data = data.split('v=')[-1]
        if (data == ''):
            return
        strarr = data.split(':')
        intarr = []
        for elem in strarr:
            if (elem != ''):
                intarr.append(int(elem,16))
        
        dataout = Int16MultiArray()
        dataout.data = intarr
        # publish data
        Getter.run(self, dataout)

    # enable_probes((3275, 2876,2868,2867,2866,2930))
    # enabling 3275 will collect data from laser sensor


class LidarCameraNearestObstacle(Getter):
    ###
    # Lidar sensor class extending the getter, for handling the publishing of data 
    ###
    def init(self, ros, hostname): 
        # create publisher
        self.pub = ros.Publisher('/'+hostname+'/lidar/'+resource_name(self.name),Float64,queue_size=1)
        # pass to parent to enble probes
        # self.mp.write_probe(self.codes,1)
        # Getter.init(self)

    def run(self, q):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue
        data = q[self.name]
        
        # TODO: format data for publisher
        #print("nearest obs data ", data)
        
        # publish data
        Getter.run(self, data)

    # enable_probes((3275, 2876,2868,2867,2866,2930))
    # enabling 3275 will collect data from laser sensor
