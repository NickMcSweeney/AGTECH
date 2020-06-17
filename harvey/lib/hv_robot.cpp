// general includes
#include <functional>
#include <cmath>
#include <algorithm>
#include <sys/time.h>

// ros includes
#include "tf/transform_datatypes.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

// local includes
#include "harvey/states.h"
#include "harvey/hv_robot.h"
#include "harvey/robot_state.h"
#include "harvey/cartesian_coordinates.h"

#include "harvey/ToggleMP.h"
#include "harvey/SetTargetMP.h"

using  std::map;
using  std::function;
using  std::string;
using  std::replace;

const static float ZERO = 0;
const static float ANGULAR_VEL_MIN = 0.5;
const static float ANGULAR_VEL_MAX = 1.0;
const static float LINEAR_VEL_MIN = 0.2;
const static float LINEAR_VEL_MAX = 2.0;

//////////////////////////////////////////////////////////////
//
// Handles robot control commands
//
//////////////////////////////////////////////////////////////
HvRobot::HvRobot(string name, double * dt) {
  // constructor for the hv robot
  this->dt_ = dt;
  this->velocity_v = 0;
  this->velocity_th = 0;
  this->current_vel_ = 0;
  this->current_pos_th_ = 0;
  this->has_item_ = false;

  name.erase(std::remove(name.begin(), name.end(), '-'), name.end());
  replace( name.begin(), name.end(), '.', '_');

  this->name_ = name;

  this->odom_sub = this->nh.subscribe("/"+name+"/odom/loc_rawodometrywgyro_",
                                      1000, &HvRobot::odom_callback, this);
  this->ir_sub = this->nh.subscribe("/"+name+"/ir", 1000,
                                    &HvRobot::ir_callback, this);
  this->gripper_sub = this->nh.subscribe("/"+name+"/gripper/manip_botgotpot", 1000,
                                    &HvRobot::gripper_callback, this);
  this->nearest_obj_sub = this->nh.subscribe(
      "/"+name+"/lidar", 1000, &HvRobot::nearest_obj_callback, this);
  this->arm_pub =
      this->nh.advertise<std_msgs::Float32>("/"+name+"/set_arm_pos", 1000);
  this->gripper_pub =
      this->nh.advertise<std_msgs::Float32>("/"+name+"/set_gripper_pos", 1000);
  this->vel_pub =
      this->nh.advertise<geometry_msgs::Twist>("/"+name+"/cmd_vel", 1000);
  this->path_pub =
      this->nh.advertise<nav_msgs::Path>("/"+name+"/path", 1000);

  //this->follow_srv = this->nh.serviceClient<hv_bridge::mp_toggle>("FollowMe");
  this->follow_srv = this->nh.serviceClient<harvey::ToggleMP>("/FollowMe");
  this->set_pick_srv = this->nh.serviceClient<harvey::SetTargetMP>("/SetPickTarget");
  this->pick_srv = this->nh.serviceClient<harvey::ToggleMP>("/RunPick");

  // enum states { Follow, TrackBack, ReturnHome, Hold };
  // map functions to states
  map<State, function<void()>> tasks;
  tasks[State::Next] = [this]() { next(); };
  tasks[State::Hold] = [this]() { hold(); };
  tasks[State::Follow] = [this]() { follow(); };
  tasks[State::TrackBack] = [this]() { track_back(); };
  tasks[State::Collect] = [this]() { collect_item(); };
  tasks[State::ReturnHome] = [this]() { return_home(); };
  this->robot_state_manager.load_task_dictionary(tasks);

  // map input key codes to states
  //this->input_codes['\n'] = State::Next;
  this->input_codes['h'] = State::Hold;
  this->input_codes['f'] = State::Follow;
  this->input_codes['r'] = State::ReturnHome;
  this->input_codes['c'] = State::Collect;
  this->input_codes['b'] = State::TrackBack;
}


// -- looping functions -- //

bool HvRobot::listen(int dt) {
  try {
  // listens for keyboard commands to trigger state changes
   fd_set fdset;
   struct timeval timeout;
   int  rc;

   timeout.tv_sec = 0;   /* wait for 6 seconds for data */
   timeout.tv_usec = dt;


   FD_ZERO(&fdset);

   FD_SET(0, &fdset);

   rc = select(1, &fdset, NULL, NULL, &timeout);
   if (rc == -1 || rc == 0)  /* select failed */
   {
       throw -1;
   }
   else 
   {
      if (FD_ISSET(0, &fdset)) 
      {
        try {
          map<char,State>::const_iterator state_ittr = this->input_codes.find(getchar());
          if (state_ittr == this->input_codes.end()) {
            throw -1;
          } else {
            this->robot_state_manager.update_state(state_ittr->second);
            return true;
          }
        } catch(...) {
          throw -1;
        }

      }
   }
   } catch(...) {
    return false;
   }
  
}

void HvRobot::run(const double INTERVAL) {
  // perform functions based on the current state
  switch(this->robot_state_manager.current_state()) {
    case State::TrackBack  :
      // track back to person
      this->go_to_loc(&(this->path_));
      if(*(this->dt_) >= INTERVAL) {
        this->save_location(&(this->path_home_));
        *(this->dt_) = 0;
      }
      this->publish_velocity();
      this->publish_path(&(this->path_));
      break; 
    case State::ReturnHome  :
      // return to home location
      this->go_to_loc(&(this->path_home_));
      if(*(this->dt_) >= INTERVAL) {
        this->save_location(&(this->path_));
        *(this->dt_) = 0;
      }
      this->publish_velocity();
      this->publish_path(&(this->path_home_));
      break;
    case State::Follow  :
      // save current location
      if(*(this->dt_) >= INTERVAL) {
        this->save_location(&(this->path_home_));
        this->publish_path(&(this->path_home_));
        *(this->dt_) = 0;
      }
      break;
    case State::Collect  :
      // save current location
      if(this->has_item_ == true){ 
        this->call_pick_srv(0);
        this->robot_state_manager.update_state(State::Hold);
      }
      break;
  
    default :
      // for all other states do nothing special
      break;
  }
}


// --  Functions to perform tasks -- //

void HvRobot::save_location(Track * loc_path) {
  // save the location of the robot to a vector
  loc_path->add_location(this->current_pos_);
}

void HvRobot::go_to_loc(Track * loc_path) {
  double distance = this->current_pos_.distance(this->goal_pos_);
  if(distance > this->TO_LOC_THRESHOLD) {
    // set velocity to move robot towards location  

    // trigonometric angle between 2 points.
    float opp = std::abs(this->goal_pos_.y - this->current_pos_.y);
    float base = std::abs(this->goal_pos_.x - this->current_pos_.x);
    float hyp = std::sqrt(opp * opp + base * base);
    float angle = asin(opp / hyp);
    if (this->goal_pos_.x < this->current_pos_.x) angle = std::abs(M_PI - angle);
    if (this->goal_pos_.y < this->current_pos_.y) angle = angle * (-1);
    // angle is the amount that the robot needs to turn to be oriented towards the point

    // set linear velocity
    if (loc_path->isEmpty() && this->current_vel_ > LINEAR_VEL_MIN) {
      this->velocity_v = this->current_vel_ - 0.1;
    } else if (this->current_vel_ < LINEAR_VEL_MIN) {
      this->velocity_v = LINEAR_VEL_MIN;
    } else if(this->current_vel_ < LINEAR_VEL_MAX) {
      this->velocity_v = this->current_vel_ + 0.01;
    } 

    // turn robot to align with goal angle to destination.
    if (std::abs(angle - (this->current_pos_th_)) < 0.2)
      this->velocity_th = ZERO;
    else if (std::sin(this->current_pos_th_ - angle) > 0) {
      if (std::abs(angle - (this->current_pos_th_)) > 0.8 ){ 
        this->velocity_th = -ANGULAR_VEL_MAX;
        this->velocity_v = LINEAR_VEL_MIN/2;
      } else
        this->velocity_th = -ANGULAR_VEL_MIN;
    } else if (std::sin(this->current_pos_th_ - angle) < 0) {
      if (std::abs(angle - (this->current_pos_th_)) > 0.8){
        this->velocity_th = ANGULAR_VEL_MAX;
        this->velocity_v = LINEAR_VEL_MIN/2;
      } else
        this->velocity_th = ANGULAR_VEL_MIN;
    }


  } else {
    if(!loc_path->isEmpty()){
      this->goal_pos_ = loc_path->pop();
    } else {
      // NOTE: this may not be the best thing to do when arriving at location
      this->robot_state_manager.update_state(State::Hold);
    }
  }
}

// --  Functions to perform transitions between tasks -- //

void HvRobot::next() {
  // end all running processes
  ROS_INFO("Robot is no longer following you");
  this->call_follow_srv(0);
  
}

void HvRobot::follow() {
  // follow person
  ROS_INFO("Robot is now following you");
  this->path_home_.clear();
  this->path_.clear();
  this->call_follow_srv(1);
}

void HvRobot::return_home() {
  // return to start point
  ROS_INFO("Robot is returning to home location");
  this->goal_pos_ = this->current_pos_;
  this->call_follow_srv(0);
  this->path_home_.pop();
  this->path_home_.shift();
  this->publish_path(&(this->path_home_));
  for(int i = 0; i < (this->path_home_.size()); i ++) {
    ROS_INFO("coordinates[i]: (%f,%f)", this->path_home_.get(i).x,this->path_home_.get(i).y);
  }
  //this->path_.clear();
}

void HvRobot::track_back() {
  // return to person
  ROS_INFO("Robot is following track back to person");
  this->goal_pos_ = this->current_pos_;
  this->call_follow_srv(0);
  this->path_.pop();
  this->publish_path(&(this->path_));
  //this->path_home_.clear();
}

void HvRobot::hold() {
  // wait for processes to start
  ROS_INFO("Robot is holding");
  this->velocity_v = 0;
  this->velocity_th = 0;
  this->call_set_pick_target_srv(this->current_pos_.x,this->current_pos_.y);
  //this->goal_pos_ = this->current_pos_;
}

void HvRobot::collect_item() {
  // identify nearest object and collect it
  // should use call back from collection from nearest object
  double x = this->current_pos_.x;
  double y = this->current_pos_.y;
  double th = this->current_pos_th_;
  
  this->call_set_pick_target_srv(x,y);
  this->call_pick_srv(1);
}


// -- Subscriber callback functions -- //

void HvRobot::odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  // callback function to handle incoming odometry updates
  this->current_pos_.x = msg->pose.pose.position.x;
  this->current_pos_.y = msg->pose.pose.position.y;
  this->current_pos_th_ = tf::getYaw(msg->pose.pose.orientation);
  //ROS_INFO("current linear vel: %f", msg->twist.twist.linear.x);
  //this->current_vel_ = msg->twist.twist.linear.x;
}
void HvRobot::ir_callback(const std_msgs::Float32::ConstPtr &msg) {
  // callback function to handle incoming ir sensor updates
}
// TODO: replace this with a better option
void HvRobot::nearest_obj_callback(const geometry_msgs::Vector3 &msg) {
  // callback function to handle neares object updates

}
void HvRobot::gripper_callback(const std_msgs::Bool::ConstPtr &msg) {
  // callback funtion for if the gripper has an item
  //`bool bot_got = msg->data;
  this->has_item_ = msg->data;
}


// -- Publisher functions -- //

void HvRobot::publish_velocity() {
  geometry_msgs::Twist msg;
  msg.linear.x = this->velocity_v;
  msg.angular.z = this->velocity_th;

  this->current_vel_ = this->velocity_v;
  this->vel_pub.publish(msg);
}

void HvRobot::publish_path(Track * loc_path) {
  // publish path data
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = this->name_;
  int path_size = loc_path->size();
  for(int i = 0; i < path_size; i ++) {
    geometry_msgs::PoseStamped pos_s;
    pos_s.header.stamp = ros::Time::now();
    pos_s.header.frame_id = this->name_;
    pos_s.pose.position.x = loc_path->get(i).x;
    pos_s.pose.position.y = loc_path->get(i).y;
    msg.poses.push_back(pos_s);
  }
  this->path_pub.publish(msg);
}


// -- Utility functions -- //

void HvRobot::call_follow_srv(int request_val) {
  harvey::ToggleMP srv;
  srv.request.val = request_val;
  if(this->follow_srv.call(srv)){
    ROS_INFO("toggled following");
  } else {
    ROS_INFO("request to toggle follow failed");
  }
}

void HvRobot::call_set_pick_target_srv(int request_x,int request_y) {
  harvey::SetTargetMP srv;
  srv.request.x = request_x;
  srv.request.y = request_y;
  if(this->set_pick_srv.call(srv)){
    ROS_INFO("set pick target");
  } else {
    ROS_INFO("request to set pick target failed");
  }
}
void HvRobot::call_pick_srv(int request_val) {
  harvey::ToggleMP srv;
  srv.request.val = request_val;
  if(this->pick_srv.call(srv)){
    ROS_INFO("toggled pick");
  } else {
    ROS_INFO("request to toggle pick failed");
  }
}
