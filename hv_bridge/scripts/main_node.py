#!/usr/bin/env python
import rospy
import sys, os, time

# add local python scripts to the path so that they can be iported
sys.path.append(os.path.abspath(os.path.join(sys.path[0] ,"../../resources")))
sys.path.append(os.path.abspath(os.path.join(sys.path[0] ,"../includes")))

from utils import *
from mp import *
from Controllers import *
from Sensors import *

# Global variables
host_name = "hai-1095.local" # this is the hostname the robot being controlled.

# main fn
if __name__ == '__main__':
###
# Services Main Node
# rosrun services main_node [HOSTNAME (optional)]
# This node must be run to allow the connection between ROS and the HarvestAI bots over
# the Mindprobe protocol
###
    try:
        # Init the connection with the ROS system
        rospy.init_node("hai_main_node", anonymous=True)
        # if present, set the hostname based on the argument inputed by the user
        if len(sys.argv) < 2:
            host_name = "hai-1095.local"
        else:
            host_name = sys.argv[1]
        print("connected to {}").format(host_name)

        # Create the mindprobe instance
        mp = Mindprobe()
        mp.init(host_name)
        # wait while the mp connection is fully established
        while(len(mp.return_probes()) < 3000):
            time.sleep(1)

        ### Create all the Controllers ###
        drive = DriveController(mp)
        arm = ArmController(mp)
        gripper = GripperController(mp)
        ### -------------------------- ###

        ### Create Sensor Hub ###
        hub = SensorHub(mp)
        ### ----------------- ###
        
        time.sleep(1)

		### Drive Controller ###
        drive.run(rospy, resource_name(host_name))
        print("DRIVE control running")
		### --------------- ###

		### Arm Controller ###
        arm.run(rospy, resource_name(host_name))
        print("ARM control running")
		### --------------- ###

		### Gripper Controller ###
        gripper.run(rospy, resource_name(host_name))
        print("GRIPPER control running")
		### --------------- ###

		### Sensor Hub ###
        hub.start(rospy, resource_name(host_name))
        print("sensor HUB running")
		### --------------- ###

        time.sleep(1)

        ### Start Mindprobe Connection ###
        # enables probes and starts mode to allow listening
        mp.start()
        ### -------------------------- ###
        
        time.sleep(3)

        # set rate based on the mindprobe refresh rate.
        # default to 100. (mp runs at 200)
        hz = 100
        if(mp.hz):
            hz = mp.hz
        
        # timer_count_1 = 0
        # timer_count_2 = 0
        # Start the ROS main loop
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            # if(timer_count_1 > 10):
                # # run the sensor data collection
                # hub.listen()
                # timer_count_1 = 0
            # if(timer_count_2 > 100):
                # hub.scan()
                # timer_count_2 = 0
            # if(timer_count_2 == 1):
                # mp.write_probe(3859,1)

            # timer_count_1 = timer_count_1 + 1
            # timer_count_2 = timer_count_2 + 1
            hub.listen(rospy.get_rostime())

            rate.sleep()

        # run the mp disconnect function to safely close down the connection to the robot
        mp.disconnect()
    except rospy.ROSInterruptException:
        mp.disconnect()
        pass
