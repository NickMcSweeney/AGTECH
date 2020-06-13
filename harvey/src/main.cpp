// General includes
#include <cstring>
#include <iostream>
#include <future>

// ROS includes
#include <ros/ros.h>

// local includes
#include "harvey/hv_robot.h"

using  std::function;
using  std::map;
using  std::vector;
using  std::string;
using  std::async;
using  std::future;


//////////////////////////////////////////////////////////////
//
// Main function
// Demo application for the AgTech project using the ROS bridge
//
/////////////////////////////////////////////////////////////
int main(int argc, char **argv) {

  // Init the connection with the ROS system
  ros::init(argc, argv, "harvey");
  
  HvRobot harvey_ = HvRobot();

  ROS_INFO("STARTING");
  float current_time = ros::Time::now().toSec();
  float last_time = ros::Time::now().toSec();
  float delta_time = current_time - last_time;
  // ROS main loop
  int hz = 10;
  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    future<bool> listener = async(&HvRobot::listen,&harvey_,hz);
    // update time
    last_time = current_time;
    current_time = ros::Time::now().toSec();
    delta_time = current_time - last_time;

    // run the robot
    harvey_.run();

    bool ret = listener.get();
    if(ret == true) {
      ROS_INFO("Changed state");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
