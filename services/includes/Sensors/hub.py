#!/usr/bin/env python
from std_msgs.msg import Float64
import asyncio
import sys, os
import ros
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)

class SensorHub():
    ###
    # collects data from the robot sensors and sorts the output into an object that can then be published
    ###
    def __init__(self, mp):
        # add mp to the hub
        self.mp = mp
        self.queue = {}
        # initialize the ir cameras to colect signals
        # codes are for signalA, signalB
        self.leftFront = IrCamera("LeftFront",(1563,1564),self.mp)
        self.rightFront = IrCamera("RightFront",(1513,1514),self.mp)
        self.leftBack = IrCamera("LeftBack",(1513,1514),self.mp)
        self.rightBack = IrCamera("RightBack",(1513,1514),self.mp)

        # initialize the lidar camera
        # codes: start laser scan collection, collect raw data
        self.lidar = LidarCamera((3275,2868),self.mp)
        # initialize the nearest obstacle detection of the preprocessed laser data
        # codes give: x, y, and distance
        self.forwardObstacle = LidarCameraNearestObstacle((2928,2929,2930),self.mp)

    def start(self, ros, hostname):
        # runs at the begining of ros node
        
        # start the ir camera nodes
        self.leftFront.init(ros, hostname)
        self.rightFront.init(ros, hostname)
        self.leftBack.init(ros, hostname)
        self.rightBack.init(ros, hostname)

        # start the lidar camera
        self.lidar.init(ros, hostname)
        self.forwardObstacle.init(ros, hostname)


    def listen(self):
        # runs in loop on the ros node
        
        # capture data and read it
        self.mp.capture(0.01)
        data_in = self.mp.return_capture()
        # add data to queue
        for item in data_in:
            print(item)
            for (name, value) in item.items():
                print(name, " : ", value)
                if self.queue.has_key(name):
                    self.queue[name] = value

        # run each node (publish ros nodes)
        # start the ir camera nodes
        self.leftFront.run(self.queue)
        self.rightFront.run(self.queue)
        self.leftBack.run(self.queue)
        self.rightBack.run(self.queue)

        # start the lidar camera
        self.lidar.run(self.queue)
        self.forwardObstacle.run(self.queue)
