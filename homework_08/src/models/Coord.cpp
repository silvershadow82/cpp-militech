#include "models/Coord.h"
#include <cmath>

float Coord::length() { return sqrt(x * x + y * y); }

Coord Coord::normalize() {
  float len = length();
  if (len == 0)
    return Coord{0, 0};
  return Coord{x / len, y / len};
}

float Coord::distance(Coord coord1, Coord coord2) {
  return (coord1 - coord2).length();
}

float Coord::angle(Coord coord1, Coord coord2) {
  // Return as arc tangent of coord differences
  Coord diff = coord2 - coord1;
  return atan2(diff.y, diff.x);
}