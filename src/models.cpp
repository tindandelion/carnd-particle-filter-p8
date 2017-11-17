#include <math.h>
#include <random>

#include "models.hpp"

using std::vector;


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

