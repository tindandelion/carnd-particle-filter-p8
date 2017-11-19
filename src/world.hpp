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

  CartesianPoint translate(const CartesianPoint& origin) const {
    return CartesianPoint(x + origin.x, y + origin.y);
  }

  CartesianPoint rotate(double angle) const {
    double x_new = x * cos(angle) - y * sin(angle);
    double y_new = x * sin(angle) + y * cos(angle);
    return CartesianPoint(x_new, y_new);
  }
};

class Observation {
  CartesianPoint _position;
  
public: 
  Observation(const CartesianPoint& position): _position(position) { }
  
  Observation toMapCoordinates(const CartesianPoint& origin, double theta) const {
    return Observation(_position.rotate(theta).translate(origin));
  }
  const CartesianPoint& position() const { return _position; }
  double gaussianDistanceFrom(const CartesianPoint& mean, const double* stddev) const;
};

class VehicleState {
  CartesianPoint _position;
  double _theta;

public:
  VehicleState(const CartesianPoint& position, double yaw_angle): _position(position), _theta(yaw_angle) {}

  const CartesianPoint& position() const { return _position; }
  double yaw_angle() const { return _theta; }
  
  VehicleState addGaussianNoise(const double* stddev) const;
  VehicleState move(double delta_t, double vel, double yaw_rate) const;
};

class Landmark {
  int _id;
  CartesianPoint _position;
public:
  Landmark(int id, const CartesianPoint& position): _id(id), _position(position) {}
  
  int id() const { return _id; }
  const CartesianPoint& position() const { return _position; }
};

class Map {
  std::vector<Landmark> landmarks; 
public:
  void addLandmark(int id, const CartesianPoint& position) { landmarks.push_back(Landmark(id, position));};
  const Landmark& findNearest(const CartesianPoint& point) const;
};


#endif 
