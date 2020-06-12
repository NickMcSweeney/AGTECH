#ifndef HV_ROBOT_H_
#define HV_ROBOT_H_

// standard includes
#include <map>

// ROS includes
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

// local includes
#include "harvey/states.h"
#include "harvey/robot_state.h"

class HvRobot {
  /**
   * Controller class for the harvest vehicle
   * handles communication with the hv_bridge node
   */

  // ros node handler connection
  ros::NodeHandle nh;
  // Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber ir_sub;
  ros::Subscriber nearest_obj_sub;
  // Publishers
  ros::Publisher arm_pub;
  ros::Publisher gripper_pub;
  ros::Publisher vel_pub;

  RobotState robot_state_manager;
  std::map<char, State> input_codes;

public:
  HvRobot();
  State listen();

private:
  // functions for performing state based tasks
  void follow();
  void return_home();
  void track_back();
  void hold();
  // callback functions for ros subscribers
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void ir_callback(const std_msgs::Float32::ConstPtr &msg);
  // TODO: replace this with a better option
  void nearest_obj_callback(const geometry_msgs::Vector3 &msg);
};

#endif
