# AGTECH
bridging application to connect the harvest vehicle robots with the robotics operating system.

### Documentation
see [wiki](https://gitsvn-nt.oru.se/hkan/harvest-automation/wikis/home) for full documentation.

#### Project Notes
This project manages comunication with the Harvest Automation Robots and ROS Melodic. This project has been designed to be run using Ubuntu 18 on a PC or Ubuntu Server 18 on a RaspberryPi 4. No other configurations have been tested, and not other versions of ROS have been tested. 

A preconfigured RaspberryPi disk image (for a 32GB disk) can be downloaded [here](https://cloud.oru.se/s/DJ4sG9yTLsiMZ38), specific instructions for setup can be found in the [Wiki](https://gitsvn-nt.oru.se/hkan/harvest-automation/wikis/setup).

### Open Source
this is being developed with Orebro University and is for academic use.

### Harvest AI
this is for harvest vehicle robots, property of a private company and therefore some of the resources being used for development are not publically available.

### Project Structure
```
*harvey*: demo project written in c++
	|--- include: header files
	|--- src: main 
	|--- srv: service files
	|--- lib: suplemental classes
*hv_bridge*: interface between mindprobe and ros
	|--- libs 
		|--- Controllers
			|--- arm: subcribes to arm commands and writes to robot
			|--- gipper: subcribes to gripper commands and writes to robot
			|--- drive: subcribes to velocity commands and writes to robot
		|--- Resources: utility python scripts for the python nodes
			|--- client: base class for the client application for communicating with the robot
			|--- config: static variables shared by the hv_bridge classes
			|--- listener: class for creating a listener thread in the client connection to the robot
			|--- reader: base class for reading from the robots
			|--- utils: for functions that are used in multiple classes/files
			|--- writer: base class for writing to the robots
		|--- Sensors
			|--- hub: central connection for all sensor listener classes, handles assigning data to each listener
			|--- ir: class for the collection, formating, and publishing of IR camera data
			|--- lidar: class for the collection, formating, and publishing of LIDAR sensor data
			|--- odometry: class for the collection, formatting, and publishing of positional data
			|--- gripper: class for the collection, formatting, and publishing of the gripper and gripper arm states
		|--- Services
			|--- avoid: toggle the avoid obstacle functionality
			|--- follow: toggle the follow person functionality
			|--- pick_target: set a pot location
	|--- scripts
		|--- hv_client: extension of the client application for connecting to the hv robot
		|--- main_node: ros node file
	|--- srv: ros services
		|--- mp_set_target: set xy coordinates for a probe 
		|--- mp_toggle: toggle a boolian value for a probe
*hv_teleop*: ros teleop node
	|--- teleop_node: remote control of the gripper using computer keyboard commands
*hai_launch*: ROS launch files
	|--- hv_bridge: lauch the bridge
	|--- harvey: lauch the bridge + the demo application
	|--- hv_teleop: lauch the bridge + the teleop node
*mindprobe_scripts*: HAi scripts for mindprobe server and client.
```
