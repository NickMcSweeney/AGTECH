#ifndef LOCATION_H_
#define LOCATION_H_

class CartesianCoordinate {
  /**
   * location on x-y plane
   */

public:
  double x;
  double y;

  CartesianCoordinate(double x, double y);
  double distance(CartesianCoordinate loc);
};

#endif
