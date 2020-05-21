#!/usr/bin/env python
from std_msgs.msg import Float64
import sys, os
import ros
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)

from ir import IrCamera
from lidar import LidarCamera, LidarCameraNearestObstacle
from odometry import Odometer

class SensorHub():
    ###
    # collects data from the robot sensors and sorts the output into an object that can then be published
    ###
    def __init__(self, mp):
        # add mp to the hub
        self.mp = mp
        self.queue = {
                "bound.rightFront.signalA":0,
                "bound.leftFront.signalA":0,
                "bound.rightRear.signalA":0,
                "bound.leftRear.signalA":0,
                "bound.rightFront.signalB":0,
                "bound.leftFront.signalB":0,
                "bound.rightRear.signalB":0,
                "bound.leftRear.signalB":0,
                "Raw":0,
                "vis.forwardObstacleDist":0,
                "vis.forwardObstacle.x":0,
                "vis.forwardObstacle.y":0
        }
        # initialize the ir cameras to colect signals
        # codes are for signalA, signalB
        # NOTE: there are many other data elements that can be included here
        # probe id 1513: bound.rightFront.signalA type float length 4
        # probe id 1514: bound.rightFront.signalB type float length 4
        # probe id 1563: bound.leftFront.signalA type float length 4
        # probe id 1564: bound.leftFront.signalB type float length 4
        # probe id 1613: bound.leftRear.signalA type float length 4
        # probe id 1614: bound.leftRear.signalB type float length 4
        # probe id 1662: bound.rightRear.signalA type float length 4
        # probe id 1663: bound.rightRear.signalB type float length 4
        self.rightFrontA = IrCamera("bound.rightFront.signalA",1513,self.mp)
        self.leftFrontA = IrCamera("bound.leftFront.signalA",1563,self.mp)
        self.rightBackA = IrCamera("bound.rightRear.signalA",1613,self.mp)
        self.leftBackA = IrCamera("bound.leftRear.signalA",1662,self.mp)
        self.rightFrontB = IrCamera("bound.rightFront.signalB",1514,self.mp)
        self.leftFrontB = IrCamera("bound.leftFront.signalB",1564,self.mp)
        self.rightBackB = IrCamera("bound.rightRear.signalB",1614,self.mp)
        self.leftBackB = IrCamera("bound.leftRear.signalB",1663,self.mp)

        # initialize the lidar camera
        # codes: start laser scan collection, collect raw data
        # probe id 3275: behaviors.interactiveSickRange.run type bool length 1 <-- this starts the lidar data collection
        # probe id 2866: vis.sick.globalScanData type tlv length 4372
        # probe id 2867: vis.sick.localScanData type tlv length 4372
        # probe id 2868: vis.sick.localScanData_EVERY_TICK type tlv length 4372
        self.lidar = LidarCamera("Raw",3275,self.mp)
        # initialize the nearest obstacle detection of the preprocessed laser data
        # codes give: x, y, and distance
        # probe id 2928: vis.forwardObstacle.x type float length 4
        # probe id 2929: vis.forwardObstacle.y type float length 4
        # probe id 2930: vis.forwardObstacleDist type float length 4
        self.forwardObstacleDist = LidarCameraNearestObstacle("vis.forwardObstacleDist",2930,self.mp)
        self.forwardObstacleX = LidarCameraNearestObstacle("vis.forwardObstacle.x",2928,self.mp)
        self.forwardObstacleY = LidarCameraNearestObstacle("vis.forwardObstacle.y",2929,self.mp)

        # initialize odometery
        # TODO: lookup odometry codes
        # TODO: lookup odometery reset codes
        self.odometery = Odometer("",0000,self.mp)

    def start(self, ros, hostname):
        # runs at the begining of ros node
        self.mp.enable_probes((1513,1514,1563,1564,1613,1614,1662,1663,3275,2868,2928,2929,2930))
        
        # start the ir camera nodes
        self.leftFrontA.init(ros, hostname)
        self.rightFrontA.init(ros, hostname)
        self.leftBackA.init(ros, hostname)
        self.rightBackA.init(ros, hostname)

        # start the lidar camera
        self.lidar.init(ros, hostname)
        self.forwardObstacleDist.init(ros, hostname)
        self.forwardObstacleX.init(ros, hostname)
        self.forwardObstacleY.init(ros, hostname)

        # start the odometery
        self.odometery.init(ros, hostname)

    def listen(self):
        # runs in loop on the ros node
        
        # capture data and read it
        self.mp.capture(0.01)
        data_in = self.mp.return_capture()
        # add data to queue
        for item in data_in:
            # print(item)
            for (name, value) in item.items():
                # print(name, " : ", value)
                if self.queue.has_key(name):
                    self.queue[name] = value

        # print("THE QUEUE: ", self.queue)
        # run each node (publish ros nodes)
        # run the ir camera nodes
        self.leftFrontA.run(self.queue)
        self.rightFrontA.run(self.queue)
        self.leftBackA.run(self.queue)
        self.rightBackA.run(self.queue)

        # run the lidar camera
        self.lidar.run(self.queue)
        self.forwardObstacleDist.run(self.queue)
        self.forwardObstacleX.run(self.queue)
        self.forwardObstacleY.run(self.queue)
    
        # run the odometry
        self.odometery.run(self.queue)
