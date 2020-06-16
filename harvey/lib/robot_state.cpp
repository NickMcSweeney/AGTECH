// local includes
#include "harvey/states.h"
#include "harvey/robot_state.h"

using  std::map;
using  std::function;

//////////////////////////////////////////////////////////////
//
// Handles robot state
//
//////////////////////////////////////////////////////////////

RobotState::RobotState() {
  // Constructor for robot state machine
  this->state_ = State::Hold;
}

void RobotState::update_state(State new_state) {
  // handle state changes
  if (new_state == State::Next) {
    // cycle through states
    this->state_ = this->cycle_state(this->state_);
  } else {
    this->tasks_[State::Next]();
    this->state_ = new_state;
  }
  this->manage_state_change();
}

State RobotState::current_state() {
  return this->state_;
}

void RobotState::load_task_dictionary(map<State, function<void()>> new_dictionary) {
  // implements
  this->tasks_ = new_dictionary;
}

void RobotState::manage_state_change() {
  // handle running states for performing tasks
  this->tasks_[this->state_]();
}

State RobotState::cycle_state(State current_state) {
  // use a predefined task order to take the next in the order and return
  for(int i = 0; i < 5; i = i+1) { 
    if (current_state == this->state_order[i]) {
      return this->state_order[(i+1)%5];
    }
  }
  return State::Hold;
}
