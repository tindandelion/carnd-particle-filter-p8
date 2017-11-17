#ifndef __OBSERVATION_PROCESSOR_H
#define __OBSERVATION_PROCESSOR_H

#include <vector>
#include "world.hpp"

using std::vector;

class ObservationProcessor {
  const vector<Observation>& observations;
  vector<Observation> map_observations;
public:
  ObservationProcessor(const vector<Observation>& observations): observations(observations) { }

  void convertToMapCoordinates(const VehicleState& state);
  const vector<Observation>& mapped() const { return map_observations; } 
};

#endif
