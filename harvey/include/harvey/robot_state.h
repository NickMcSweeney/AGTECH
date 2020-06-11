#ifdef ROBOT_STATE
#define ROBOT_STATE

#include <function>
#include <map>

// robot states
enum states { Follow, TrackBack, ReturnHome, Hold };

class RobotState {
  /**
   * State management for the hv vehicle
   * executes a follow and fetch protocol
   */
   
  states state_;
  std::map<states, std::function<void()>> tasks;

public:
  RobotState();
  void update_state(states new_state);
  void load_task_dictonary(std::map new_dictionary);

private:
  void manage_state_change();
};

#endif
