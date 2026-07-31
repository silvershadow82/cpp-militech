#include "models/Target.h"
#include "models/Coord.h"
#include <cmath>
#include <vector>

Target::Target()
  : index(0)
  , angle(0.F)
  , timeSteps(std::vector<Coord>{})
{
}

Target::Target(const Target &copy)
  : index(copy.index)
  , angle(copy.angle)
  , timeSteps(copy.timeSteps) {};

Target::Target(int index)
  : index(index)
  , angle(0.F)
  , timeSteps(std::vector<Coord>{})
{
}

void Target::setPosAt(int index, float x, float y)
{
  this->timeSteps.insert(this->timeSteps.begin() + index, Coord{x, y});
}

Coord Target::at(float t, float arrayTimeStep) const
{
  float tLocal = fmod(t, (float)this->timeSteps.size() * arrayTimeStep);
  int idx = static_cast<int>(floor(tLocal / arrayTimeStep));
  int next = (idx + 1) % this->timeSteps.size();
  float frac = (tLocal - idx * arrayTimeStep) / arrayTimeStep;

  return this->timeSteps[idx] + (this->timeSteps[next] - this->timeSteps[idx]) * frac;
}
int Target::getIndex() const
{
  return this->index;
}

float Target::getAngle() const
{
  return this->angle;
};

void Target::setAngle(const float angle)
{
  this->angle = angle;
}

// Copy assignment operator
Target &Target::operator=(const Target &other)
{
  if (this == &other)
    return *this;
  this->index = other.index;
  this->angle = other.angle;
  this->timeSteps = other.timeSteps;
  return *this;
}

Target::~Target() {}