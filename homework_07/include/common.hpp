#pragma once

#include <cmath>
#include <cstring>
#include <optional>

const int MAX_AMMO_NAME{32};
const int MAX_STEPS{10000};
const float G{9.81F};

enum DroneState { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord &other) const { return Coord{x + other.x, y + other.y}; }

  Coord operator-(const Coord &other) const { return Coord{x - other.x, y - other.y}; }

  Coord operator*(float scalar) const { return Coord{x * scalar, y * scalar}; }

  Coord operator/(float scalar) const
  {
    if (scalar != 0) {
      return Coord{x / scalar, y / scalar};
    }
    return Coord{x, y};
  }

  bool operator==(const Coord &other) const { return x == other.x && y == other.y; }

  float length() { return sqrt(x * x + y * y); }

  Coord normalize()
  {
    float len = length();
    if (len == 0)
      return Coord{0, 0};
    return Coord{x / len, y / len};
  }

  static float distance(Coord coord1, Coord coord2) { return (coord1 - coord2).length(); }

  static float angle(Coord coord1, Coord coord2)
  {
    // Return as arc tangent of coord differences
    Coord diff = coord2 - coord1;
    return atan2(diff.y, diff.x);
  }

  static inline Coord predict(Coord coord, Coord speed, float time) { return coord + speed * time; }
};

class Target {
private:
  int index;
  int timeStepCount;
  float angle;
  Coord *timeSteps;

public:
  // Default constructor so we can init an array
  Target()
    : index(0)
  {
    this->timeSteps = new Coord[0];
  };
  Target(const Target &copy)
    : index(copy.index)
    , timeStepCount(copy.timeStepCount)
    , angle(copy.angle)
  {
    this->timeSteps = new Coord[this->timeStepCount];
    std::memcpy(this->timeSteps, copy.timeSteps, this->timeStepCount * sizeof(Coord));
  };
  Target(int index, int timeStepCount)
    : index(index)
    , timeStepCount(timeStepCount)
  {
    this->timeSteps = new Coord[this->timeStepCount];
    // this->pos = Coord{0, 0};
    this->angle = 0.F;
  }
  void setPosAt(int index, float x, float y)
  {
    if (index >= 0 && index < this->timeStepCount) {
      this->timeSteps[index] = Coord{x, y};
    }
  }
  Coord at(float t, float arrayTimeStep) const
  {
    float tLocal = fmod(t, (float)this->timeStepCount * arrayTimeStep);
    int idx = static_cast<int>(floor(tLocal / arrayTimeStep));
    int next = (idx + 1) % this->timeStepCount;
    float frac = (tLocal - idx * arrayTimeStep) / arrayTimeStep;

    return this->timeSteps[idx] + (this->timeSteps[next] - this->timeSteps[idx]) * frac;
  }
  int getIndex() const { return this->index; }
  float getAngle() const { return this->angle; };
  void setAngle(const float angle) { this->angle = angle; }

  // Copy assignment operator
  Target &operator=(const Target &other)
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

  ~Target()
  {
    if (this->timeSteps != nullptr) {
      delete[] this->timeSteps;
    }
  }
};

struct PayloadParams {
  float m, m2, m3, m4;
  float d, d2, d3, d4;
  float l, l2, l3, l4;
};

struct AmmoParams {
  char name[MAX_AMMO_NAME];
  float mass;
  float drag;
  float lift;

  PayloadParams payloadParams()
  {
    // Define payload parameters based on the current ammo
    PayloadParams pp{};

    pp.m = mass;
    pp.d = drag;
    pp.l = lift;

    pp.m2 = pp.m * pp.m;
    pp.m3 = pp.m2 * pp.m;
    pp.m4 = pp.m2 * pp.m2;

    pp.d2 = pp.d * pp.d;
    pp.d3 = pp.d2 * pp.d;
    pp.d4 = pp.d2 * pp.d2;

    pp.l2 = pp.l * pp.l;
    pp.l3 = pp.l2 * pp.l;
    pp.l4 = pp.l2 * pp.l2;

    return pp;
  }

  static std::optional<AmmoParams> ammoByName(const char *ammoName, AmmoParams *ammo, int ammoCount)
  {
    if (ammo == nullptr) {
      return std::nullopt;
    }

    for (int i = 0; i < ammoCount; i++) {
      if (strcmp(ammoName, ammo[i].name) == 0) {
        return ammo[i];
      }
    }
    return std::nullopt;
  }
};

struct DroneConfig {
  Coord startPos;
  float altitude;
  float initialDir;
  float attackSpeed;
  float accelPath;
  char ammoName[MAX_AMMO_NAME];
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
};

struct SimState {
  Coord dronePos;
  Coord dropPoint;
  Coord aimPoint;
  Coord targetPos;
  Coord predictedTargetPos;
  int lastTargetIdx;
  float droneSpeed;
  float droneAngle;
  float targetAngle;
  float droneAccel;
  float payloadDropTime;
  int step;
  float t;
  DroneState droneState;
};

struct SimStep {
  Coord pos;
  int targetIdx;
  DroneState state;
  float direction;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
};

// ------------------------- Utility methods -----------------------
int minValueIdx(float array[], size_t len)
{
  int idx{0};
  float value{array[idx]};
  for (int i = 1; i < len; i++) {
    if (value > array[i]) {
      value = array[i];
      idx = i;
    }
  }
  return idx;
}

float timeToDistance(float distance, float currentSpeed, float attackSpeed, float acceleration, float accPath)
{
  float accTime = (attackSpeed - currentSpeed) / acceleration;
  float diff = std::max(distance - accPath, 0.f);
  return static_cast<float>(accTime + diff / attackSpeed);
}

void normalizeAngle(float &angleDiff)
{
  while (angleDiff > M_PI)
    angleDiff -= static_cast<float>(2.f * M_PI);
  while (angleDiff < -M_PI)
    angleDiff += static_cast<float>(2.f * M_PI);
}

float convergeAngle(float &droneAngle, float targetAngle, const DroneConfig &droneConfig)
{
  float angleDiff = targetAngle - droneAngle;

  normalizeAngle(angleDiff);

  float maxAngleStep = droneConfig.angularSpeed * droneConfig.simTimeStep;

  if (fabs(angleDiff) <= maxAngleStep) {
    droneAngle = targetAngle;
  }
  else if (angleDiff > 0) {
    droneAngle += maxAngleStep;
  }
  else {
    droneAngle -= maxAngleStep;
  }

  normalizeAngle(droneAngle);

  return fabs(angleDiff);
}
