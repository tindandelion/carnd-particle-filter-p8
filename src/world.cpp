#include <random>
#include "world.hpp"

static std::default_random_engine random_gen;

inline double gauss(double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(random_gen);
}


VehicleState VehicleState::addGaussianNoise(const VehicleState& stddev) const {
  CartesianPoint new_position;
  new_position.x = gauss(position.x, stddev.position.x);
  new_position.y = gauss(position.y, stddev.position.y);
  double new_theta = gauss(theta, stddev.theta);
  return VehicleState(new_position, new_theta);
}

VehicleState VehicleState::move(double delta_t, double vel, double yaw_rate) const {
  CartesianPoint offset;
  double new_theta = theta + yaw_rate*delta_t;
  
  if (fabs(yaw_rate) < 1e-4) {
    offset.x = vel * cos(theta) * delta_t;
    offset.y = vel * sin(theta) * delta_t;
  } else {
    offset.x = vel/yaw_rate * (sin(new_theta) - sin(theta));
    offset.y = vel/yaw_rate * (-cos(new_theta) + cos(theta));
  }

  CartesianPoint new_position = position.translate(offset);
  return VehicleState(new_position, new_theta);  
};

const Landmark& Map::findNearest(const CartesianPoint& point) const {
  int nearest = 0;
  double nearestDistance = point.distance(landmarks[0].position);

  for(int i = 1; i < landmarks.size(); i++) {
    double distance = point.distance(landmarks[i].position);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearest = i;
    }
  }
  return landmarks[nearest];
}
