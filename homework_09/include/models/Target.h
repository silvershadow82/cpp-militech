#pragma once

#include "Coord.h"
#include <vector>

class Target {
private:
  int index;
  float angle;
  std::vector<Coord> timeSteps;

public:
  // Default constructor so we can init an array
  Target();
  Target(const Target &copy);
  Target(int index);

  Coord at(float t, float arrayTimeStep) const;

  int getIndex() const;
  float getAngle() const;

  void setPosAt(int index, float x, float y);
  void setAngle(const float angle);

  // Copy assignment operator
  Target &operator=(const Target &other);
};