# AGTECH
bridging application to connect the harvest vehicle robots with the robotics operating system.

### Documentation
see [wiki](https://github.com/NickMcSweeney/hv-ros-bridge/wiki) for full documentation.

### Open Source
this is being developed with Orebro University and is for academic use.

### Harvest AI
this is for harvest vehicle robots, property of a private company and therefore some of the resources being used for development are not publically available.

### Project Structure
```
*controllers*: nodes to send data to the robots
	|--- *drive*
	|--- *gripper_arm*
*sensors*: nodes to get data from the robots
	|--- 
	|---
*services*: ROS services to control the mindprobe client
	|---
*hai_launch*: ROS launch files
	|---
*tests*: unit tests for the ros nodes
	|--- *controllers*
	|--- *sensors*
	|--- *services*
```
