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
  std::map<State, std::function<void()>> tasks_;
  const State state_order [4] = {State::Hold, State::Follow, State::ReturnHome, State::TrackBack};

public:
  RobotState();
  void update_state(State new_state);
  void load_task_dictionary(std::map<State, std::function<void()>> new_dictionary);
  State current_state();

private:
  void manage_state_change();
  State cycle_state(State current_state);
};

#endif
