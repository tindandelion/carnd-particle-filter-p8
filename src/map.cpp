#include <math.h>
#include "map.hpp"

inline double square(double x) {
  return x * x;
}

inline double distanceTo(const Map::Landmark& landmark, double x, double y) {
  return sqrt(square(landmark.x_f - x) + square(landmark.y_f - y));
}

void Map::addLandmark(int id, float x, float y) {
  landmark_list.push_back(Landmark(id, x, y));
}

const Map::Landmark& Map::findNearest(float x, float y) const {
  int nearest = -1;
  double nearestDistance = -1;

  for(int i = 0; i < landmark_list.size(); i++) {
    double distance = distanceTo(landmark_list[i], x, y);
    if (distance < nearestDistance) {
      nearestDistance = distance;
      nearest = i;
    }
  }
  return landmark_list[nearest];
}
