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

#### Notes: 
I'm a little confused about the context of these questions. From the point of view of MindProbe the issue wasn't getting all of the Sick data, it was displaying the data in a useful way. 
We either displayed a big wedge in front of the robot bounded by the Sick samples or a color coded ray for each sample. The latter was mostly only useful in still frames.

The format of the data should be well documented by Sick. In general, the scan data was in polar coordinates from the center of the Lidar. We got useful information as follows:
- All transformations are assumed to be 2D with the plane of the Lidar sweep parallel to the ground.
- The samples are transformed from polar to cartesian coordinates relative to the Lidar device.
- The time of the start of the scan is either inferred from the arrival time of the data or in the data itself.
- The rate of sweep is well known so the time that each sample was taken can be inferred.
- Wheel odometry is used to localize the robot at the time each sample was taken.
- The samples are finally transformed into "absolute" (field) coordinates given the robot's pose and the Lidar's fixed position relative to the robot's origin.

Scan modes should be well documented by Sick. We only used the one described above.

Data rate was never a big issue. The ethernet connection from the Lidar was adequate. We were also able to transmit the transformed Lidar sample data, including the return intensity to our MindProbe telemetry application over wifi in real time.

