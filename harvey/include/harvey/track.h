#ifndef LOCATION_TRACK_H_
#define LOCATION_TRACK_H_

// general includes
#include <deque>

// local includes
#include "harvey/cartesian_coordinates.h"

class Track {
  /**
   * Tracking array for manageing location history of a robot
   */

private:

  std::deque<CartesianCoordinate> history_;
  
public:

  Track();

  void add(double x, double y);
  void add_location(CartesianCoordinate new_loc);

  CartesianCoordinate shift();
  
  CartesianCoordinate pop();
  
  bool isEmpty();
};

#endif
