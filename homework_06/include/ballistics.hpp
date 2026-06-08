#pragma once
#include <cmath>

/**
 * Ballistics from homework #1
 */
struct PayloadParams {
  bool ok;
  float m, m2, m3, m4;
  float d, d2, d3, d4;
  float l, l2, l3, l4;
};

const float G{9.81F};
const int AMMO_COUNT = 5;
const int MAX_AMMO_LENGTH = 32;

struct Coord {
  float x;
  float y;

  auto operator+(const Coord &other) const { return Coord{x + other.x, y + other.y}; }

  auto operator-(const Coord &other) const { return Coord{x - other.x, y - other.y}; }

  auto operator*(float scalar) const { return Coord{x * scalar, y * scalar}; }

  auto operator/(float scalar) const
  {
    if (scalar != 0) {
      return Coord{x / scalar, y / scalar};
    }
    return Coord{x, y};
  }

  auto operator==(const Coord &other) const { return x == other.x && y == other.y; }

  auto length() const { return std::sqrt(x * x + y * y); }

  auto normalize() const
  {
    const float len = length();
    if (len == 0) {
      return Coord{0, 0};
    }
    return Coord{x / len, y / len};
  }
};

#pragma pack(push, 1)
struct AmmoParams {
  char name[MAX_AMMO_LENGTH];
  float mass;
  float drag;
  float lift;
};

struct BallisticInput {
  Coord startPos;
  Coord targetPos;
  float altitude;
  float attackSpeed;
  float accelPath;
  char ammoName[MAX_AMMO_LENGTH];
};

struct BallisticResult {
  Coord dronePos;
  Coord dropPoint;
  Coord targetPos;
  float payloadDropTime;
  char ammoName[MAX_AMMO_LENGTH];
};

#pragma pack(pop)

// Helper functions
float distance(Coord coord1, Coord coord2);
void normalizeAngle(float &angleDiff);
auto ammoByName(const char *ammoName, AmmoParams &ammoParams);
auto payloadParams(const char ammoName[MAX_AMMO_LENGTH]);

// Ballistic functions
float payloadTimeOfFlight(const PayloadParams &pp, float altitude, float speed);
float calcHDistance(float t, float speed, const PayloadParams &pp);
int ballistics(BallisticResult &result, const BallisticInput &input);