#ifndef __MODELS_H
#define __MODELS_H

struct ModelState {
  double x;
  double y;
  double theta;

  ModelState(double x, double y, double theta): x(x), y(y), theta(theta) {}
  ModelState(double values[]): x(values[0]), y(values[1]), theta(values[3]) {}
};

class CtrvMotionModel {
  double sigma_x;
  double sigma_y;
  double sigma_theta;

public:
  ModelState init(const ModelState& mean, const ModelState& stddev);
  ModelState predict(const ModelState& current, double delta_t, double vel, double yaw_rate, const ModelState& stddev);
};

#endif
