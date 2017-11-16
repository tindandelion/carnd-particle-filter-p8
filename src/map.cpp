#include <math.h>
#include "map.hpp"

inline double square(double x) {
  return x * x;
}

inline double distanceTo(const Map::Landmark& landmark, double x, double y) {
  return sqrt(square(landmark.x - x) + square(landmark.y - y));
}

void Map::addLandmark(int id, float x, float y) {
  landmarks.push_back(Landmark(id, x, y));
}

const Map::Landmark& Map::findNearest(float x, float y) const {
  int nearest = 0;
  double nearestDistance = distanceTo(landmarks[0], x, y);

  for(int i = 1; i < landmarks.size(); i++) {
    double distance = distanceTo(landmarks[i], x, y);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearest = i;
    }
  }
  return landmarks[nearest];
}
