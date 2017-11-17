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

#include "particle_filter.hpp"

using namespace std;

void ParticleFilter::init(double x, double y, double theta) {
  for (int i = 0; i < num_particles; i++) {
    VehicleState init_state = motion_model.init(VehicleState(CartesianPoint(x, y), theta));
    particles.push_back(Particle(init_state));
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double velocity, double yaw_rate) {
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    p.setState(motion_model.predict(p.getState(), delta_t, velocity, yaw_rate));
  }
}

void ParticleFilter::updateWeights(double sensor_range, const std::vector<LandmarkObs> &observations) {
  for (int i = 0; i < particles.size(); i++) {
    Particle& p = particles[i];
    p.weight = observation_model.calculateWeight(p.getState(), observations);
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
