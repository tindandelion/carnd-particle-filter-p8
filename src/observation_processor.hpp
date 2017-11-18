#ifndef __OBSERVATION_PROCESSOR_H
#define __OBSERVATION_PROCESSOR_H

#include <vector>
#include "world.hpp"

using std::vector;

class ObservationProcessor {
  const vector<Observation>& observations;
  vector<Observation> map_observations;
  vector<Landmark> associations;
public:
  ObservationProcessor(const vector<Observation>& observations): observations(observations) { }

  void convertToMapCoordinates(const VehicleState& state);
  void associateWithLandmarks(const Map& map);
  double calculateTotalWeight(const double* stddev);
  
  const vector<Observation>& mapped() const { return map_observations; }
  const vector<Landmark>& assoc() const { return associations; }
};

#endif
