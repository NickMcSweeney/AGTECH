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
  
  ROS_INFO("STARTING");
  
  string robot_name = "hai-1095.local";
  //if(argc == 1) robot_name = argv[0];
  double delta_time = 0;
  HvRobot harvey_ = HvRobot(robot_name, &delta_time);
  
  // ros time variable
  double current_time = ros::Time::now().toSec();

  // ROS main loop
  int hz = 100;
  ros::Rate loop_rate(hz);
  while (ros::ok()) {
    future<bool> listener = async(&HvRobot::listen,&harvey_,(0.8/hz));
    // update time
    double last_time = current_time;
    current_time = ros::Time::now().toSec();
    delta_time = delta_time + (current_time - last_time);

    // run the robot
    harvey_.run(1.0);

    bool ret = listener.get();
    if(ret == true) {
      ROS_INFO("Changed state");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
