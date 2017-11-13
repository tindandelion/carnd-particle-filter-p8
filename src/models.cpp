#include <math.h>
#include <random>

#include "models.hpp"

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
