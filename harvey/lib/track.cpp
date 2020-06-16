// local includes
#include "harvey/track.h"
#include "harvey/cartesian_coordinates.h"
#include <cmath>

//////////////////////////////////////////////////////////////
//
// Handles location
//
//////////////////////////////////////////////////////////////
CartesianCoordinate::CartesianCoordinate(double x, double y) {
	// constructor for a location
	this->x = x;
	this->y = y;
}
double CartesianCoordinate::distance(CartesianCoordinate loc) {
	return double(std::sqrt(std::pow(this->x-loc.x, 2.0) + std::pow(this->y-loc.y, 2.0)));
}

//////////////////////////////////////////////////////////////
//
// Handles location track
//
//////////////////////////////////////////////////////////////
Track::Track() {
	// constructor for history of locations
}
void Track::add(double x, double y) {
	// add a coordinate to a location
	this->add_location(CartesianCoordinate(x,y));
}
void Track::add_location(CartesianCoordinate new_loc) {
	// add a coordinate to a location
	this->history_.push_back(new_loc);
}

CartesianCoordinate Track::shift() {
	// remove a location from the history's front
	CartesianCoordinate l = this->history_.front();
	this->history_.pop_front();
	return l;
}

CartesianCoordinate Track::pop() {
	// remove a location from the history's end
	CartesianCoordinate l = this->history_.back();
	this->history_.pop_back();
	return l;
}

bool Track::isEmpty() {
	// return TRUE if the track is empty
	return this->history_.size() > 1 ? false : true;
}

int Track::size() {
	// return the length of the list
	return this->history_.size();
}

CartesianCoordinate Track::get(int i) {
	// get an element from list
	return this->history_[i];
}

void Track::clear() {
	this->history_.clear();
}
