// General includes
#include <cstring>
#include <iostream>
#include <vector>

// ROS includes
#include <ros/ros.h>

// local includes
#include "harvey/hv_robot.h"
#include "harvey/robot_state.h"

using namespace std;

//////////////////////////////////////////////////////////////
//
// Main function
// Demo application for the AgTech project using the ROS bridge
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

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
