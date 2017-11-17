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

inline double calculateWeight(const CartesianPoint& observed_value,
			      const CartesianPoint& true_value,
			      const CartesianPoint& stddev) {
  double scale_factor = 1 / (2 * M_PI * stddev.x * stddev.y);
  double xx = square(observed_value.x - true_value.x) / (2 * square(stddev.x));
  double yy = square(observed_value.y - true_value.y) / (2 * square(stddev.y));
  return scale_factor * exp(-(xx + yy));
}

double LandmarkAssoc::calculateWeight(double stddev[]) const {
  CartesianPoint sigma(stddev[0], stddev[1]);
  return ::calculateWeight(observation.position, landmark.position, sigma);
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

double ObservationModel::calculateWeight(const VehicleState& state, const vector<Observation>& observations) {
  vector<Observation> observations_on_map;
  vector<LandmarkAssoc> associations;

  transformToMapCoordinates(state, observations, observations_on_map);
  associateWithNearestLandmarkOnMap(observations_on_map, associations);
  return calculateTotalWeight(associations);
}

void ObservationModel::transformToMapCoordinates(const VehicleState& state,
						 const vector<Observation>& observations,
						 vector<Observation>& result) {
  for (const Observation& o: observations) {
    result.push_back(o.toMapCoordinates(state.position, state.theta));
  }
}

void ObservationModel::associateWithNearestLandmarkOnMap(const vector<Observation>& observations, vector<LandmarkAssoc>& associations) {
  for(int i = 0; i < observations.size(); i++) {
    const Observation& obs = observations[i];
    Landmark nearest = map.findNearest(obs.position);
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

