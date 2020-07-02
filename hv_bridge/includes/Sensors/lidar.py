#!/usr/bin/env python
from std_msgs.msg import Float64
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import PointCloud2
from nav_msgs.msg import PointField
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
        self.hostname = hostname
        # create publisher
        self.pub = ros.Publisher('/'+hostname+'/lidar/'+resource_name(self.name),Int16MultiArray,queue_size=1)
        # pass to parent to enble probes
        #self.mp.write_probe(self.codes,1)
        Getter.init(self)

    def run(self, q, current_time):
        ## handles the publishing of data, should be run at an acceptable update interval
        # get data from queue


        data = q[self.name]
        
        # get the data from the message
        data = data.split('v=')[-1]
        if (data == ''):
            return

        # create x and y arrays from the base16 string of data
        strarr = data.split(':')
        x=[]
        y=[]
        (lambda a : (x.append(int(a[0],16)),y.append(int(a[1],16)), fx(a[2:])) if a.size > 1  else False)(strarr)
        
        # create pointcloud2 message
        dataout = PointCloud2()
        # add timestamp based on ros time
        msg.header.stamp = current_time
        # add frame 
        msg.header.frame_id = self.hostname
        # format message
        N = len(x)
        xy = np.array(np.hstack([x,y]), dtype=np.float32)
        msg.height = 1
        msg.width = N

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = False
        msg.point_step = 8
        msg.row_step = msg.point_step * N
        msg.is_dense = True;
        msg.data = xy.tostring()


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
