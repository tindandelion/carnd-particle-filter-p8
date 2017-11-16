#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
 public:
  struct Landmark {

    int id_i ; // Landmark ID
    float x_f; // Landmark x-position in the map (global coordinates)
    float y_f; // Landmark y-position in the map (global coordinates)
    Landmark(int id, float x, float y): id_i(id), x_f(x), y_f(y) {}
  };

  std::vector<Landmark> landmark_list ; // List of landmarks in the map
  
  void addLandmark(int id, float x, float y);
  const Landmark& findNearest(float x, float y) const;
};



#endif
