#include <math.h>
#include <random>

#include "models.hpp"

using std::vector;

std::default_random_engine random_gen;

inline double gauss(double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(random_gen);
}

inline CartesianPoint randomize(const CartesianPoint& mean, const CartesianPoint& stddev) {
  return CartesianPoint(gauss(mean.x, stddev.x), gauss(mean.y, stddev.y));
}

VehicleState randomize(const VehicleState& mean, const VehicleState& stddev) {
  return VehicleState(randomize(mean.position, stddev.position), gauss(mean.theta, stddev.theta));
}

double LandmarkAssoc::calculateWeight(double stddev[]) const {
  double sigma_x = stddev[0], sigma_y = stddev[1];
  double scale_factor = 1 / (2 * M_PI * sigma_x * sigma_y);
  double xx = square(observation.x - landmark.x()) / (2 * square(sigma_x));
  double yy = square(observation.y - landmark.y()) / (2 * square(sigma_y));
    
  return scale_factor * exp(-(xx + yy));
}


VehicleState CtrvMotionModel::init(const VehicleState& mean) {
  return randomize(mean, stddev);
}

VehicleState CtrvMotionModel::predict(const VehicleState& cur, double delta_t, double vel, double yaw_rate) {
  double dx = 0, dy = 0;
  double theta = cur.theta + yaw_rate*delta_t;
  
  if (fabs(yaw_rate) < 1e-4) {
    dx = vel * cos(cur.theta) * delta_t;
    dy = vel * sin(cur.theta) * delta_t;
  } else {
    dx = vel/yaw_rate * (sin(theta) - sin(cur.theta));
    dy = vel/yaw_rate * (-cos(theta) + cos(cur.theta));
  }

  CartesianPoint new_position = cur.position.translate(dx, dy);
  VehicleState new_state(new_position, theta);
  return randomize(new_state, stddev);
}

double ObservationModel::calculateWeight(const VehicleState& state, const vector<LandmarkObs>& observations) {
  vector<LandmarkObs> observations_on_map;
  vector<LandmarkAssoc> associations;

  transformToMapCoordinates(state, observations, observations_on_map);
  associateWithNearestLandmarkOnMap(observations_on_map, associations);
  return calculateTotalWeight(associations);
}

void ObservationModel::transformToMapCoordinates(const VehicleState& state,
						 const vector<LandmarkObs>& observations,
						 vector<LandmarkObs>& result) {
  for (int i = 0; i < observations.size(); i++) {
    result.push_back(state.transformToMapCoordinates(observations[i]));
  }
}

void ObservationModel::associateWithNearestLandmarkOnMap(const vector<LandmarkObs>& observations, vector<LandmarkAssoc>& associations) {
  for(int i = 0; i < observations.size(); i++) {
    const LandmarkObs& obs = observations[i];
    Landmark nearest = map.findNearest(CartesianPoint(obs.x, obs.y));
    associations.push_back(LandmarkAssoc(obs, nearest));
  }
}

double ObservationModel::calculateTotalWeight(const vector<LandmarkAssoc>& associations) {
  double total = 1.0;
  for (int i = 0; i < associations.size(); i++) {
    total *= associations[i].calculateWeight(stddev);
  }
  return total;
}

