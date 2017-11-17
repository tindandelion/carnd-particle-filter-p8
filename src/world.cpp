#include "world.hpp"

LandmarkObs VehicleState::transformToMapCoordinates(const LandmarkObs& observation) const {
  LandmarkObs transformed;
  transformed.id = observation.id;
  transformed.x = position.x + observation.x * cos(theta) - observation.y * sin(theta);
  transformed.y = position.y + observation.x * sin(theta) + observation.y * cos(theta);
  return transformed;
}

const Landmark& Map::findNearest(const CartesianPoint& point) const {
  int nearest = 0;
  double nearestDistance = point.distance(landmarks[0].position);

  for(int i = 1; i < landmarks.size(); i++) {
    double distance = point.distance(landmarks[i].position);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearest = i;
    }
  }
  return landmarks[nearest];
}
