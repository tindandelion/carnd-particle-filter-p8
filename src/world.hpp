#ifndef __WORLD_H
#define __WORLD_H

#include <math.h>
#include "helper_functions.h"

struct CartesianPoint {
  double x;
  double y;

  CartesianPoint(double x, double y): x(x), y(y) {}
  
  double distance(const CartesianPoint& other) const {
    return sqrt(square(x - other.x) + square(y - other.y));
  }

  CartesianPoint translate(double dx, double dy) const {
    return CartesianPoint(x + dx, y + dy);
  }
};


#endif 
