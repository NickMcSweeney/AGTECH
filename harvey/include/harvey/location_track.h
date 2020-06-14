#ifndef LOCATION_TRACK_H_
#define LOCATION_TRACK_H_

// general includes
#include <deque>

// local includes
#include "harvey/location.h"

class LocationTrack {
  /**
   * Tracking array for manageing location history of a robot
   */

private:

  std::deque<Location> history_;
  
public:

  LocationTrack();

  void add(double x, double y);
  void add_location(Location new_loc);

  Location shift();
  
  Location pop();
  
  bool isEmpty();
};

#endif
