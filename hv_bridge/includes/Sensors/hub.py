#!/usr/bin/env python
from std_msgs.msg import Float64
import sys, os
import rospy
SCRIPTS_PATH = os.path.abspath(os.path.join(sys.path[0] ,"../../../resources"))
sys.path.append(SCRIPTS_PATH)

from ir import IrCamera
from lidar import LidarCamera, LidarCameraNearestObstacle
from odometry import Odometer
from gripper import Gripper

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
                "vis.sick.localScanData":0,
                "vis.forwardObstacleDist":0,
                "vis.forwardObstacle.x":0,
                "vis.forwardObstacle.y":0,
                "loc.rawOdometry.x":0,
                "loc.rawOdometry.y":0,
                "loc.rawOdometry.h":0,
                "loc.rawOdometry.v":0,
                "loc.rawOdometry.w":0,
                "loc.rawOdometryWGyro.x":0,
                "loc.rawOdometryWGyro.y":0,
                "loc.rawOdometryWGyro.h":0,
                "loc.rawOdometryWGyro.v":0,
                "loc.rawOdometryWGyro.w":0,
                "loc.actual.x":0,
                "loc.actual.y":0,
                "manip.botGotPot":0
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
        # self.rightFrontA = IrCamera("bound.rightFront.signalA",1513,self.mp)
        # self.leftFrontA = IrCamera("bound.leftFront.signalA",1563,self.mp)
        # self.rightBackA = IrCamera("bound.rightRear.signalA",1613,self.mp)
        # self.leftBackA = IrCamera("bound.leftRear.signalA",1662,self.mp)
        # self.rightFrontB = IrCamera("bound.rightFront.signalB",1514,self.mp)
        # self.leftFrontB = IrCamera("bound.leftFront.signalB",1564,self.mp)
        # self.rightBackB = IrCamera("bound.rightRear.signalB",1614,self.mp)
        # self.leftBackB = IrCamera("bound.leftRear.signalB",1663,self.mp)

        self.rightFront = IrCamera(("bound.rightFront.signalA","bound.rightFront.signalB"),(1513,1514),self.mp)
        self.leftFront = IrCamera(("bound.leftFront.signalA","bound.leftFront.signalB"),(1563,1564),self.mp)
        self.rightRear = IrCamera(("bound.rightRear.signalA","bound.rightRear.signalB"),(1613,1614),self.mp)
        self.leftRear = IrCamera(("bound.leftRear.signalA","bound.leftRear.signalB"),(1662,1663),self.mp)

        # initialize the lidar camera
        # codes: start laser scan collection, collect raw data
        # probe id 3275: behaviors.interactiveSickRange.run type bool length 1 <-- this starts the lidar data collection
        # probe id 2866: vis.sick.globalScanData type tlv length 4372
        # probe id 2867: vis.sick.localScanData type tlv length 4372
        # probe id 2868: vis.sick.localScanData_EVERY_TICK type tlv length 4372
        # self.lidar = LidarCamera("vis.sick.localScanData",3275,self.mp)
        # initialize the nearest obstacle detection of the preprocessed laser data
        # codes give: x, y, and distance
        # probe id 2928: vis.forwardObstacle.x type float length 4
        # probe id 2929: vis.forwardObstacle.y type float length 4
        # probe id 2930: vis.forwardObstacleDist type float length 4
        self.forwardObstacleDist = LidarCameraNearestObstacle("vis.forwardObstacleDist",2930,self.mp)
        self.forwardObstacleX = LidarCameraNearestObstacle("vis.forwardObstacle.x",2928,self.mp)
        self.forwardObstacleY = LidarCameraNearestObstacle("vis.forwardObstacle.y",2929,self.mp)

        # initialize odometery
        # probe id 2438: loc.actual.x type float length 4
        # probe id 2439: loc.actual.y type float length 4
        # probe id 2456: loc.rawOdometry.x type float length 4
        # probe id 2457: loc.rawOdometry.y type float length 4
        # probe id 2458: loc.rawOdometry.h type float length 4
        # probe id 2459: loc.rawOdometry.v type float length 4
        # probe id 2460: loc.rawOdometry.w type float length 4
        # probe id 2461: loc.rawOdometry.l type float length 4
        # probe id 2462: loc.rawOdometry.r type float length 4
        # probe id 2463: loc.rawOdometryWGyro.x type float length 4
        # probe id 2464: loc.rawOdometryWGyro.y type float length 4
        # probe id 2465: loc.rawOdometryWGyro.h type float length 4
        # probe id 2466: loc.rawOdometryWGyro.v type float length 4
        # probe id 2467: loc.rawOdometryWGyro.w type float length 4
        # probe id 2472: loc.useGyro type bool length 1
        # probe id 2479: loc.magicGps.x type double length 8
        # probe id 2480: loc.magicGps.y type double length 8
        # probe id 2481: loc.magicGps.h type double length 8
        #self.odometeryX = Odometer("loc.rawOdometry.x",2456,self.mp)
        #self.odometeryY = Odometer("loc.rawOdometry.y",2457,self.mp)
        self.odometery = Odometer(("loc.rawOdometry.x","loc.rawOdometry.y","loc.rawOdometry.h", "loc.rawOdometry.v", "loc.rawOdometry.w"),(2456,2457,2458,2459,2460),self.mp)
        self.odometeryGyro = Odometer(("loc.rawOdometryWGyro.x","loc.rawOdometryWGyro.y","loc.rawOdometryWGyro.h", "loc.rawOdometryWGyro.v", "loc.rawOdometryWGyro.w"),(2463,2464,2465,2466,2467),self.mp)

        #probe id 1100: manip.botGotPot type bool length 1
        self.gripper = Gripper("manip.botGotPot",1100,self.mp)

    def start(self, ros, hostname):
        # runs at the begining of ros node
        # self.mp.enable_probes((1513,
            # 1514,
            # 1563,
            # 1564,
            # 1613,
            # 1614,
            # 1662,
            # 1663,
            # 3275,
            # 2867,
            # 2928,
            # 2929,
            # 2930,
            # 2456,
            # 2457
        # ))

        # turn on the lidar sensor
        #self.mp.write_probes([(3859,1),(3275,1)])
        
        # start the ir camera nodes
        self.leftFront.init(ros, hostname)
        self.rightFront.init(ros, hostname)
        self.leftRear.init(ros, hostname)
        self.rightRear.init(ros, hostname)

        # start the lidar camera
        # self.lidar.init(ros, hostname)
        self.forwardObstacleDist.init(ros, hostname)
        self.forwardObstacleX.init(ros, hostname)
        self.forwardObstacleY.init(ros, hostname)

        # start the odometery
        #self.odometeryX.init(ros, hostname)
        #self.odometeryY.init(ros, hostname)
        self.odometery.init(ros,hostname)
        self.odometeryGyro.init(ros,hostname)

        # start gripper
        self.gripper.init(ros,hostname)

    def listen(self, current_time):
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
        self.leftFront.run(self.queue, current_time)
        self.rightFront.run(self.queue, current_time)
        self.leftRear.run(self.queue, current_time)
        self.rightRear.run(self.queue, current_time)

        # run the lidar camera
        # self.lidar.run(self.queue)
        self.forwardObstacleDist.run(self.queue)
        self.forwardObstacleX.run(self.queue)
        self.forwardObstacleY.run(self.queue)
    
        # run the odometry
        #self.odometeryX.run(self.queue)
        #self.odometeryY.run(self.queue)
        self.odometery.run(self.queue, current_time)
        self.odometeryGyro.run(self.queue, current_time)
        
        # run gripper state collection
        self.gripper.run(self.queue, current_time)

    def scan(self):
        self.mp.write_probe(3275,1)
