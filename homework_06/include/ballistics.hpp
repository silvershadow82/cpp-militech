#include <cmath>

/**
 * Ballistics from homework #1
 */
struct PayloadParams {
  float m, m2, m3, m4;
  float d, d2, d3, d4;
  float l, l2, l3, l4;
};

const float G{9.81f};
const int MAX_AMMO_LENGTH = 32;

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
  int ammoCount;
  AmmoParams *ammo;
};

struct BallisticResult {
  Coord dronePos;
  Coord dropPoint;
  Coord targetPos;
  float payloadDropTime;
};

#pragma pack(pop)

// Helper functions
float distance(Coord coord1, Coord coord2);

void normalizeAngle(float &angleDiff);

int ammoByName(const char *ammoName, const AmmoParams *ammo, int ammoCount, AmmoParams &ammoParams);

PayloadParams payloadParams(const char ammoName[MAX_AMMO_LENGTH], const AmmoParams *ammo, int ammoCount);

// Ballistic functions
float payloadTimeOfFlight(const PayloadParams &pp, float altitude, float speed);

float calcHDistance(float t, float speed, const PayloadParams &pp);

int ballistics(BallisticResult &result, const BallisticInput &droneConfig);