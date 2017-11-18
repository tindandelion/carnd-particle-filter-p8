/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <assert.h>

#include "particle_filter.hpp"

using namespace std;

static default_random_engine random_gen;

double totalWeight(const vector<Particle>& particles) {
  double total = 0.0;
  for (const Particle& p: particles) {
    total += p.weight;
  }
  return total;
}

void normalizeWeights(vector<Particle>& particles) {
  double total = totalWeight(particles);
  for (Particle& p: particles) {
    p.weight /= total;
  }
}

void collectWeights(const vector<Particle>& particles, vector<double>& weights) {
  for (const Particle& p: particles) {
    weights.push_back(p.weight);
  }
}

void ParticleFilter::init(double x, double y, double theta) {
  VehicleState init_state = VehicleState(CartesianPoint(x, y), theta);
  for (int i = 0; i < num_particles; i++) {
    VehicleState state = init_state.addGaussianNoise(motion_stddev);
    particles.push_back(Particle(state));
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double velocity, double yaw_rate) {
  for (Particle& p: particles) {
    VehicleState new_state = p.state.move(delta_t, velocity, yaw_rate);
    p.state = new_state.addGaussianNoise(motion_stddev);
  }
}

void ParticleFilter::updateWeights(double sensor_range, const std::vector<Observation> &observations) {
  ObservationProcessor op(observations);
  for (Particle& p: particles) {
    op.convertToMapCoordinates(p.state);
    op.associateWithLandmarks(map);
    p.weight = op.calculateTotalWeight(observation_stddev);
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  vector<double> weights;
  collectWeights(particles, weights);

  discrete_distribution<> dist(weights.begin(), weights.end());
  vector<Particle> new_particles;
  
  for (int i = 0; i < num_particles; i++) {
    int index = dist(random_gen);
    new_particles.push_back(particles[index]);
  }
  
  normalizeWeights(new_particles);
  particles = new_particles;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
