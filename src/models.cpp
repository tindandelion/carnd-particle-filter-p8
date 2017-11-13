#include <random>

#include "models.hpp"

double gauss(std::default_random_engine gen, double mean, double stddev) {
  std::normal_distribution<double> dist(mean, stddev);
  return dist(gen);
}

ModelState randomize(const ModelState& mean, const ModelState& stddev) {
  std::default_random_engine gen;
  return ModelState(gauss(gen, mean.x, stddev.x),
		    gauss(gen, mean.y, stddev.y),
		    gauss(gen, mean.theta, stddev.theta));
}

ModelState CtrvMotionModel::init(ModelState const& mean, ModelState const& stddev) {
  return randomize(mean, stddev);
}

ModelState CtrvMotionModel::predict(const ModelState& current, double delta_t, double vel, double yaw_rate, const ModelState& stddev) {
  return current;
}
