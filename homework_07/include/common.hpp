#pragma once

#include <cmath>
#include <cstring>
#include <optional>

constexpr int MAX_AMMO_NAME{32};
constexpr int MAX_STEPS{10000};
constexpr float G{9.81F};

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
