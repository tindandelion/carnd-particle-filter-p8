#ifndef __MODELS_H
#define __MODELS_H

#include "helper_functions.h"

using std::vector;

struct ModelState {
  double x;
  double y;
  double theta;

  ModelState(double x, double y, double theta): x(x), y(y), theta(theta) {}
  ModelState(double values[]): x(values[0]), y(values[1]), theta(values[3]) {}
  
  LandmarkObs transformToMapCoordinates(const LandmarkObs& observation) const;
};

class LandmarkAssoc {
  const LandmarkObs& observation;
  const Map::Landmark& landmark;

public:
  LandmarkAssoc(const LandmarkObs& observation, const Map::Landmark& landmark):
    observation(observation), landmark(landmark) {}
  double calculateWeight(double stddev[]) const;
};

class CtrvMotionModel {
  ModelState stddev;
public:
  CtrvMotionModel(double stddev[]): stddev(stddev) { }
  ModelState init(const ModelState& mean);
  ModelState predict(const ModelState& current, double delta_t, double vel, double yaw_rate);
};

class ObservationModel {
  const Map& map;
  double* stddev;
  
  void transformToMapCoordinates(const ModelState& state, const vector<LandmarkObs>& observations, vector<LandmarkObs>& result);
  void associateWithNearestLandmarkOnMap(const vector<LandmarkObs>& observations, vector<LandmarkAssoc>& associations);
  double calculateTotalWeight(const vector<LandmarkAssoc>& associations);
public:
  ObservationModel(const Map& map, double stddev[]): map(map), stddev(stddev) {}
  double calculateWeight(const ModelState& state, const vector<LandmarkObs>& observations);
};

#endif
