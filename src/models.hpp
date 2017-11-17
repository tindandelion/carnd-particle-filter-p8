#ifndef __MODELS_H
#define __MODELS_H

#include "world.hpp"
#include "helper_functions.h"

using std::vector;

class LandmarkAssoc {
  const Observation& observation;
  const Landmark& landmark;

public:
  LandmarkAssoc(const Observation& observation, const Landmark& landmark):
    observation(observation), landmark(landmark) {}
  double calculateWeight(double stddev[]) const;
};

class CtrvMotionModel {
public:
  VehicleState stddev;
  CtrvMotionModel(double stddev[]): stddev(stddev) { }
  VehicleState predict(const VehicleState& current, double delta_t, double vel, double yaw_rate);
};

class ObservationModel {
  const Map& map;
  double* stddev;
  
  void transformToMapCoordinates(const VehicleState& state, const vector<Observation>& observations, vector<Observation>& result);
  void associateWithNearestLandmarkOnMap(const vector<Observation>& observations, vector<LandmarkAssoc>& associations);
  double calculateTotalWeight(const vector<LandmarkAssoc>& associations);
public:
  ObservationModel(const Map& map, double stddev[]): map(map), stddev(stddev) {}
  double calculateWeight(const VehicleState& state, const vector<Observation>& observations);
};

#endif
