#include <math.h>
#include <random>

#include "models.hpp"

using std::vector;

std::default_random_engine random_gen;

double gauss(double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(random_gen);
}

ModelState randomize(const ModelState& mean, const ModelState& stddev) {
  return ModelState(gauss(mean.x, stddev.x),
		    gauss(mean.y, stddev.y),
		    gauss(mean.theta, stddev.theta));
}

LandmarkObs ModelState::transformToMapCoordinates(const LandmarkObs& observation) const {
  LandmarkObs transformed;
  transformed.id = observation.id;
  transformed.x = x + observation.x * cos(theta) - observation.y * sin(theta);
  transformed.y = y + observation.x * sin(theta) + observation.y * cos(theta);
  return transformed;
}

ModelState CtrvMotionModel::init(ModelState const& mean, ModelState const& stddev) {
  return randomize(mean, stddev);
}

ModelState CtrvMotionModel::predict(const ModelState& cur, double delta_t, double vel, double yaw_rate, const ModelState& stddev) {
  double dx = 0, dy = 0;
  double theta = cur.theta + yaw_rate*delta_t;
  
  if (fabs(yaw_rate) < 1e-4) {
    dx = vel * cos(cur.theta) * delta_t;
    dy = vel * sin(cur.theta) * delta_t;
  } else {
    dx = vel/yaw_rate * (sin(theta) - sin(cur.theta));
    dy = vel/yaw_rate * (-cos(theta) + cos(cur.theta));
  }

  ModelState new_state(cur.x + dx, cur.y + dy, theta);
  return randomize(new_state, stddev);
}

double ObservationModel::calculateWeight(const ModelState& state, const vector<LandmarkObs>& observations) {
  vector<LandmarkObs> observations_on_map;
  vector<LandmarkAssoc> associations;

  transformToMapCoordinates(state, observations, observations_on_map);
  associateWithNearestLandmarkOnMap(observations_on_map, associations);
  
  return 1.0;
}

void ObservationModel::transformToMapCoordinates(const ModelState& state,
						 const vector<LandmarkObs>& observations,
						 vector<LandmarkObs>& result) {
  for (int i = 0; i < observations.size(); i++) {
    result.push_back(state.transformToMapCoordinates(observations[i]));
  }
}

void ObservationModel::associateWithNearestLandmarkOnMap(const vector<LandmarkObs>& observations, vector<LandmarkAssoc>& associations) {
  for(int i = 0; i < observations.size(); i++) {
    const LandmarkObs& obs = observations[i];
    Map::Landmark nearest = map.findNearest(obs.x, obs.y);
    associations.push_back(LandmarkAssoc(obs, nearest));
  }
}
