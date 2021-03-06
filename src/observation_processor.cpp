#include "observation_processor.hpp"

void ObservationProcessor::convertToMapCoordinates(const VehicleState& state) {
  map_observations.clear();
  for (const Observation& o: observations) {
    map_observations.push_back(o.toMapCoordinates(state.position(), state.yaw_angle()));
  }
}

void ObservationProcessor::associateWithLandmarks(const Map& map) {
  associations.clear();
  for (const Observation& o: map_observations) {
    associations.push_back(map.findNearest(o.position()));
  }
}

double ObservationProcessor::calculateTotalWeight(const double* stddev) {
  double total = 1.0;
  for (int i = 0; i < map_observations.size(); i++) {
    const Observation& o = map_observations[i];
    const Landmark& lm = associations[i];
    total *= o.gaussianDistanceFrom(lm.position(), stddev);
  }
  return total;
}

void ObservationProcessor::dumpMapObservations(vector<double>& xx, vector<double>& yy) {
  xx.clear();
  yy.clear();
  for (const Observation& o: map_observations) {
    xx.push_back(o.position().x);
    yy.push_back(o.position().y);
  }
}

void ObservationProcessor::dumpAssociations(vector<int>& ids) {
  ids.clear();
  for (const Landmark& lm: associations) {
    ids.push_back(lm.id());
  }
}
