#ifndef HV_ROBOT_H_
#define HV_ROBOT_H_

// standard includes
#include <map>
#include <string>

// ROS includes
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>

// local includes
#include "harvey/states.h"
#include "harvey/robot_state.h"
#include "harvey/track.h"
#include "harvey/cartesian_coordinates.h"

class HvRobot {
  /**
   * Controller class for the harvest vehicle
   * handles communication with the hv_bridge node
   */

private:
  // robot name / id
  std::string name_;

  // ros node handler connection
  ros::NodeHandle nh;
  // Subscribers
  ros::Subscriber odom_sub;
  ros::Subscriber ir_sub;
  ros::Subscriber nearest_obj_sub;
  ros::Subscriber gripper_sub;
  // Publishers
  ros::Publisher arm_pub;
  ros::Publisher gripper_pub;
  ros::Publisher vel_pub;
  ros::Publisher path_pub;
  // Services
  ros::ServiceClient follow_srv;
  ros::ServiceClient set_pick_srv;
  ros::ServiceClient pick_srv;

  RobotState robot_state_manager;
  std::map<char, State> input_codes;

  double * dt_;
  const double TO_LOC_THRESHOLD = 0.25;

  bool has_item_;

  Track path_home_;
  Track path_;
  CartesianCoordinate current_pos_ = CartesianCoordinate(0,0);
  CartesianCoordinate goal_pos_ = CartesianCoordinate(0,0);
  double current_vel_; 
  double current_pos_th_;
  double velocity_v;
  double velocity_th;

public:
  HvRobot(std::string name, double * time);
  bool listen(int dt);
  void run(const double INTERVAL);

private:
  //functions for performing state based tasks
  void save_location(Track *t);
  void go_to_loc(Track *t);
  // functions for performing state change based tasks
  void next();
  void follow();
  void return_home();
  void collect_item();
  void track_back();
  void hold();
  // callback functions for ros subscribers
  void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  void ir_callback(const std_msgs::Float32::ConstPtr &msg);
  void gripper_callback(const std_msgs::Bool::ConstPtr &msg);
  // TODO: replace this with a better option
  void nearest_obj_callback(const geometry_msgs::Vector3 &msg);
  // publish functions
  void publish_velocity();
  void publish_path(Track * loc_path);
  // utility functions
  void call_follow_srv(int input_val);
  void call_set_pick_target_srv(int request_x,int request_y);
  void call_pick_srv(int request_val);
};

#endif
