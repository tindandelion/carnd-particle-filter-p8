#ifndef __WORLD_H
#define __WORLD_H

#include <math.h>
#include <vector>
#include "math_helpers.hpp"

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
  int id;				// Id of matching landmark in the map.
  double x;			// Local (vehicle coordinates) x position of landmark observation [m]
  double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

struct CartesianPoint {
  double x;
  double y;

  CartesianPoint(double x, double y): x(x), y(y) {}
  
  double distance(const CartesianPoint& other) const {
    return sqrt(square(x - other.x) + square(y - other.y));
  }

  CartesianPoint translate(double dx, double dy) const {
    return CartesianPoint(x + dx, y + dy);
  }
};

struct VehicleState {
  CartesianPoint position;
  double theta;

  VehicleState(const CartesianPoint& position, double yaw_angle): position(position), theta(yaw_angle) {}
  VehicleState(double values[]): position(CartesianPoint(values[0], values[1])), theta(values[3]) {}

  double x() const { return position.x; }
  double y() const { return position.y; }
  LandmarkObs transformToMapCoordinates(const LandmarkObs& observation) const;
};

struct Landmark {
  CartesianPoint position;
  int id;

  Landmark(int id, const CartesianPoint& position): id(id), position(position) {}
  
  double x() const { return position.x; }
  double y() const { return position.y; }
};

class Map {
  std::vector<Landmark> landmarks; 
public:
  void addLandmark(int id, const CartesianPoint& position) { landmarks.push_back(Landmark(id, position));};
  const Landmark& findNearest(const CartesianPoint& point) const;
};



#endif 
