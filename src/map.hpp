#ifndef MAP_H_
#define MAP_H_

#include <vector>

class Map {
public:
  struct Landmark {
    int id;
    float x;
    float y; 
    Landmark(int id, float x, float y): id(id), x(x), y(y) {}
  };
  
  void addLandmark(int id, float x, float y);
  const Landmark& findNearest(float x, float y) const;
private:
  std::vector<Landmark> landmarks; 
};



#endif
