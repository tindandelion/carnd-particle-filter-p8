#include "world.hpp"

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
