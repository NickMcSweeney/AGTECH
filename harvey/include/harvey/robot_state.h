#ifndef ROBOT_STATE_H_
#define ROBOT_STATE_H_

#include <functional>
#include <map>

// local includes
#include "harvey/states.h"

class RobotState {
  /**
   * State management for the hv vehicle
   * executes a follow and fetch protocol
   */

  State state_;
  std::map<State, std::function<void()>> tasks;

public:
  RobotState();
  void update_state(State new_state);
  void load_task_dictonary(std::map<State, std::function<void()>> new_dictionary);
  State current_state();

private:
  void manage_state_change();
};

#endif
