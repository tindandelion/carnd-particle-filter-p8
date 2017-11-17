#include "observation_processor.hpp"

void ObservationProcessor::convertToMapCoordinates(const VehicleState& state) {
  map_observations.clear();
  for (const Observation& o: observations) {
    map_observations.push_back(o.toMapCoordinates(state.position, state.theta));
  }
}
