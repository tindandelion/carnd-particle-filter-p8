#ifndef __WORLD_H
#define __WORLD_H

#include <math.h>
#include <vector>
#include "math_helpers.hpp"

struct CartesianPoint {
  double x;
  double y;

  CartesianPoint(): x(0), y(0) {}
  CartesianPoint(double x, double y): x(x), y(y) {}
  
  double distance(const CartesianPoint& other) const {
    return sqrt(square(x - other.x) + square(y - other.y));
  }

  CartesianPoint translate(double dx, double dy) const {
    return CartesianPoint(x + dx, y + dy);
  }

  CartesianPoint translate(const CartesianPoint& origin) const {
    return CartesianPoint(x + origin.x, y + origin.y);
  }

  CartesianPoint rotate(double angle) const {
    double x_new = x * cos(angle) - y * sin(angle);
    double y_new = x * sin(angle) + y * cos(angle);
    return CartesianPoint(x_new, y_new);
  }
};

struct Observation {
  CartesianPoint position;
  
  Observation(const CartesianPoint& position): position(position) { }
  Observation toMapCoordinates(const CartesianPoint& origin, double theta) const {
    return Observation(position.rotate(theta).translate(origin));
  }
};

struct VehicleState {
  CartesianPoint position;
  double theta;

  VehicleState(const CartesianPoint& position, double yaw_angle): position(position), theta(yaw_angle) {}
  VehicleState(double values[]): position(CartesianPoint(values[0], values[1])), theta(values[3]) {}

  double x() const { return position.x; }
  double y() const { return position.y; }
  VehicleState addGaussianNoise(const double* stddev) const;
  VehicleState move(double delta_t, double vel, double yaw_rate) const;
};

struct Landmark {
  int id;
  CartesianPoint position;

  Landmark(int id, const CartesianPoint& position): id(id), position(position) {}
};

class Map {
  std::vector<Landmark> landmarks; 
public:
  void addLandmark(int id, const CartesianPoint& position) { landmarks.push_back(Landmark(id, position));};
  const Landmark& findNearest(const CartesianPoint& point) const;
};


#endif 
