#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include "helper_functions.h"
#include "observation_processor.hpp"

struct Particle {
public:
  int id;
  VehicleState state;
  double weight;
  std::vector<int> associations;  
  std::vector<double> sense_x;
  std::vector<double> sense_y;

  Particle(): state(VehicleState(CartesianPoint(0, 0), 0)), weight(0) {}
  Particle(const VehicleState& init_state): state(init_state), weight(1.0) {};
  
  double x() const { return state.position.x; }
  double y() const { return state.position.y; }
  double theta() const { return state.theta; }
};

class ParticleFilter {
  bool is_initialized = false;
  int num_particles; 
public:
	
  // Set of current particles
  std::vector<Particle> particles;

  ParticleFilter(int num_particles): num_particles(num_particles) { }


  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);  

  /**
   * updateWeights Updates the weights for each particle based on the likelihood of the 
   *   observed measurements. 
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[], const std::vector<Observation> &observations,
		     const Map &map_landmarks);
	
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
