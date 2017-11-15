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
  const LandmarkObs& prediction;

public:
  LandmarkAssoc(const LandmarkObs& observation, const LandmarkObs& prediction):
    observation(observation), prediction(prediction) {}
};

class CtrvMotionModel {
public:
  ModelState init(const ModelState& mean, const ModelState& stddev);
  ModelState predict(const ModelState& current, double delta_t, double vel, double yaw_rate, const ModelState& stddev);
};

class ObservationModel {
  const Map& map;
  
  void transformToMapCoordinates(const ModelState& state, const vector<LandmarkObs>& observations, vector<LandmarkObs>& result);
  void associateWithNearestLandmarkOnMap(const vector<LandmarkObs>& observations, vector<LandmarkAssoc>& associations);

public:
  ObservationModel(const Map& map): map(map) {}
  double calculateWeight(const ModelState& state, const vector<LandmarkObs>& observations);
};

#endif
