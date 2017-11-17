#ifndef __MODELS_H
#define __MODELS_H

#include "world.hpp"
#include "helper_functions.h"

using std::vector;

class LandmarkAssoc {
  const LandmarkObs& observation;
  const Landmark& landmark;

public:
  LandmarkAssoc(const LandmarkObs& observation, const Landmark& landmark):
    observation(observation), landmark(landmark) {}
  double calculateWeight(double stddev[]) const;
};

class CtrvMotionModel {
  VehicleState stddev;
public:
  CtrvMotionModel(double stddev[]): stddev(stddev) { }
  VehicleState init(const VehicleState& mean);
  VehicleState predict(const VehicleState& current, double delta_t, double vel, double yaw_rate);
};

class ObservationModel {
  const Map& map;
  double* stddev;
  
  void transformToMapCoordinates(const VehicleState& state, const vector<LandmarkObs>& observations, vector<LandmarkObs>& result);
  void associateWithNearestLandmarkOnMap(const vector<LandmarkObs>& observations, vector<LandmarkAssoc>& associations);
  double calculateTotalWeight(const vector<LandmarkAssoc>& associations);
public:
  ObservationModel(const Map& map, double stddev[]): map(map), stddev(stddev) {}
  double calculateWeight(const VehicleState& state, const vector<LandmarkObs>& observations);
};

#endif
