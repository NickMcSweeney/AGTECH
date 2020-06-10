// General includes
#include <cstring>
#include <iostream>
#include <vector>

// ROS includes
#include <ros/ros.h>

using namespace std;

enum states {Follow, TrackBack, ReturnHome, Hold};

class StateMachine {
//////////////////////////////////////////////////////////////
//
// Handles harvey's state
//
//////////////////////////////////////////////////////////////
states state_;

public:
	StateMachine() {
		// Constructor for robot state machine
		this->state_ = Hold; 
	};
	
	void update_state(states new_state) {
		// handle state changes
		this->state_ = new_state;
	};

	void run() {
		// handle state functions

	};

private:
	void manage_state() {
		// handle running states for performing tasks

	}

	void follow() {
		// follow person 
	};
	void return_home() {
		// return to start point
	};
	void track_back() {
		// return to person
	};
	void hold() {
		// wait for processes to start
	};
}

//////////////////////////////////////////////////////////////
//
// Main function
//
/////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

	// Init the connection with the ROS system
	ros::init(argc, argv, "harvey");

	ROS_INFO("STARTING");
	float current_time = ros::Time::now().toSec();
	float last_time = ros::Time::now().toSec();
	float delta_time = current_time - last_time;
	// ROS main loop
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		// update time
		last_time = current_time;
		current_time = ros::Time::now().toSec();
		delta_time = current_time - last_time;

		this->gripper_sub =
				this->nh.subscribe("/gripper/joint_state", 1000, &R2Controller::gripperCallback, this);
		this->arm_sub = this->nh.subscribe("/arm/joint_state", 1000, &R2Controller::armCallback, this);
		this->sensor_pos_sub =
				this->nh.subscribe("/psudo_sensor/pos", 1000, &R2Controller::sensorPosCallback, this);
		this->sensor_force_sub =
				this->nh.subscribe("/psudo_sensor/effort", 1000, &R2Controller::sensorForceCallback, this);
		this->position_sub = this->nh.subscribe("/r2d2_diff_drive_controller/odom", 1000,
																						&R2Controller::positionCallback, this);

		this->gripper_effort_pub = this->nh.advertise<std_msgs::Float32>("/gripper/set_force", 1000);
		this->gripper_pos_pub = this->nh.advertise<geometry_msgs::Point>("/gripper/set_pos", 1000);
		this->arm_pos_pub = this->nh.advertise<geometry_msgs::Point>("/arm/set_pos", 1000);
		this->robot_pos_pub =
				this->nh.advertise<geometry_msgs::Twist>("/r2d2_diff_drive_controller/cmd_vel", 1000);	

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
