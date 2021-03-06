#include <random>
#include "world.hpp"

static std::default_random_engine random_gen;

inline double gauss(double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(random_gen);
}

inline double gaussianDistance(const CartesianPoint& pt, const CartesianPoint& mean, const double* stddev) {
  double scale_factor = 1 / (2 * M_PI * stddev[0] * stddev[1]);
  double xx = square(pt.x - mean.x) / (2 * square(stddev[0]));
  double yy = square(pt.y - mean.y) / (2 * square(stddev[1]));
  return scale_factor * exp(-(xx + yy));
}

VehicleState VehicleState::addGaussianNoise(const double* stddev) const {
  CartesianPoint new_position;
  new_position.x = gauss(_position.x, stddev[0]);
  new_position.y = gauss(_position.y, stddev[1]);
  double new_theta = gauss(_theta, stddev[2]);
  return VehicleState(new_position, new_theta);
}

VehicleState VehicleState::move(double delta_t, double vel, double yaw_rate) const {
  CartesianPoint offset;
  double new_theta = _theta + yaw_rate*delta_t;
  
  if (fabs(yaw_rate) < 1e-4) {
    offset.x = vel * cos(_theta) * delta_t;
    offset.y = vel * sin(_theta) * delta_t;
  } else {
    offset.x = vel/yaw_rate * (sin(new_theta) - sin(_theta));
    offset.y = vel/yaw_rate * (-cos(new_theta) + cos(_theta));
  }

  CartesianPoint new_position = _position.translate(offset);
  return VehicleState(new_position, new_theta);  
};

const Landmark& Map::findNearest(const CartesianPoint& point) const {
  int nearest = 0;
  double nearestDistance = point.distance(landmarks[0].position());

  for(int i = 1; i < landmarks.size(); i++) {
    double distance = point.distance(landmarks[i].position());
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearest = i;
    }
  }
  return landmarks[nearest];
}

double Observation::gaussianDistanceFrom(const CartesianPoint& mean, const double* stddev) const {
  return ::gaussianDistance(_position, mean, stddev);
};
