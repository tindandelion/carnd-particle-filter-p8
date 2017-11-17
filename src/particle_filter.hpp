/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include "models.hpp"

struct Particle {
public:
  int id;
  VehicleState state;
  double weight;
  std::vector<int> associations;  
  std::vector<double> sense_x;
  std::vector<double> sense_y;

  Particle(): state(VehicleState(CartesianPoint(0, 0), 0)), weight(0) {}
  Particle(const VehicleState& init_state): state(init_state), weight(1) {}

  double x() const { return state.position.x; }
  double y() const { return state.position.y; }
  double theta() const { return state.theta; }
};

class ParticleFilter {
  bool is_initialized = false;
  int num_particles; 
  std::vector<double> weights;

  VehicleState motion_stddev;
  ObservationModel observation_model;
public:
	
  // Set of current particles
  std::vector<Particle> particles;

  ParticleFilter(int num_particles, const Map& map, double motion_stddev[], double observation_stddev[]):
    num_particles(num_particles),
    motion_stddev(motion_stddev),
    observation_model(ObservationModel(map, observation_stddev)) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   */
  void init(double x, double y, double theta);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double velocity, double yaw_rate);
	
  /**
   * updateWeights Updates the weights for each particle based on the likelihood of the 
   *   observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param observations Vector of landmark observations
   */
  void updateWeights(double sensor_range, const std::vector<Observation> &observations);
	
  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();
	
  std::string getAssociations(Particle best);
  std::string getSenseX(Particle best);
  std::string getSenseY(Particle best);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const {
    return is_initialized;
  }
};



#endif /* PARTICLE_FILTER_H_ */
