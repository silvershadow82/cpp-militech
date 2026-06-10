#include "target.hpp"

Target::Target()
  : index(0)
{
  this->timeSteps = new Coord[0];
}

Target::Target(const Target &copy)
  : index(copy.index)
  , timeStepCount(copy.timeStepCount)
  , angle(copy.angle)
{
  this->timeSteps = new Coord[this->timeStepCount];
  std::memcpy(this->timeSteps, copy.timeSteps, this->timeStepCount * sizeof(Coord));
};

Target::Target(int index, int timeStepCount)
  : index(index)
  , timeStepCount(timeStepCount)
{
  this->timeSteps = new Coord[this->timeStepCount];
  // this->pos = Coord{0, 0};
  this->angle = 0.F;
}

void Target::setPosAt(int index, float x, float y)
{
  if (index >= 0 && index < this->timeStepCount) {
    this->timeSteps[index] = Coord{x, y};
  }
}

Coord Target::at(float t, float arrayTimeStep) const
{
  float tLocal = fmod(t, (float)this->timeStepCount * arrayTimeStep);
  int idx = static_cast<int>(floor(tLocal / arrayTimeStep));
  int next = (idx + 1) % this->timeStepCount;
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
  delete[] this->timeSteps;
  this->index = other.index;
  this->timeStepCount = other.timeStepCount;
  this->angle = other.angle;
  this->timeSteps = new Coord[this->timeStepCount];
  std::memcpy(this->timeSteps, other.timeSteps, this->timeStepCount * sizeof(Coord));
  return *this;
}

Target::~Target()
{
  if (this->timeSteps != nullptr) {
    delete[] this->timeSteps;
  }
}