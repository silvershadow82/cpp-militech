#define USE_MATH_DEFINES
#include "ballistics.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

// Global ammo definitions so we don't have to read the file
const AmmoParams ammo[AMMO_COUNT] = {
  AmmoParams{.name = "VOG-17", .mass = 0.35, .drag = 0.07, .lift = 0.0},
  AmmoParams{.name = "M67", .mass = 0.6, .drag = 0.1, .lift = 0},
  AmmoParams{.name = "RKG-3", .mass = 1.2, .drag = 0.1, .lift = 0},
  AmmoParams{.name = "GLIDING-VOG", .mass = 0.45, .drag = 0.1, .lift = 1},
  AmmoParams{.name = "GLIDING-RKG", .mass = 1.4, .drag = 0.1, .lift = 1},
};

float distance(Coord coord1, Coord coord2)
{
  return (coord1 - coord2).length();
}

auto ammoByName(const char *ammoName, AmmoParams &ammoParams)
{
  for (auto a : ammo) {
    if (strcmp(ammoName, a.name) == 0) {
      ammoParams = a;
      return 0;
    }
  }
  std::cerr << "Ammo not found: " << ammoName << '\n';
  return 1;
}

auto payloadParams(const char ammoName[MAX_AMMO_LENGTH])
{
  // Define payload parameters based on the name
  AmmoParams selectedAmmo{};
  PayloadParams pp{};

  const int ret_code = ammoByName(ammoName, selectedAmmo);
  if (ret_code > 0) {
    pp.ok = false;
    return pp;
  }

  pp.ok = true;
  pp.m = selectedAmmo.mass;
  pp.d = selectedAmmo.drag;
  pp.l = selectedAmmo.lift;

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

float payloadTimeOfFlight(const PayloadParams &pp, float altitude, float speed)
{
  // Calculate the payload travel time t
  // Using Cardano formula to the cubic equation at^3+bt^2+c = 0

  const auto a = static_cast<float>(pp.d * G * pp.m - 2.0 * pp.d2 * pp.l * speed);
  const auto b = static_cast<float>(-3.0 * G * pp.m2 + 3.0 * pp.d * pp.l * pp.m * speed);
  const auto c = static_cast<float>(6.0 * pp.m2 * altitude);
  const auto a2 = a * a;
  const auto a3 = a2 * a;
  const auto b2 = b * b;
  const auto b3 = b2 * b;

  // Cardano trigonometric solution
  const auto p = static_cast<float>(-b2 / (3.0 * a2));
  const auto q = static_cast<float>((2.0 * b3) / (27.0 * a3) + c / a);
  const auto arg = static_cast<float>(((3.0 * q) / (2.0 * p)) * sqrt(-3.0 / p));

  // Check that arg is within the range [-1, 1]
  if (arg < -1 || arg > 1) {
    std::cerr << "Warning: acos arg is out of range: " << arg << '\n';
    return -1.F;
  }

  const auto phi = acos(arg);

  // Calculate payload travel time
  const auto t = static_cast<float>(2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a));  // NOLINT(misc-include-cleaner)

  return t;
}

float calcHDistance(float t, float speed, const PayloadParams &pp)
{
  const auto t2 = t * t;
  const auto t3 = t2 * t;
  const auto t4 = t2 * t2;
  const auto t5 = t3 * t2;

  const auto e1 = t * speed;
  const auto e2 = static_cast<float>(-t2 * pp.d * speed / (2.0 * pp.m));
  const auto e3 = static_cast<float>(t3 * (6.0 * pp.d * G * pp.l * pp.m - 6.0 * pp.d2 * (pp.l2 - 1.0) * speed) / (36.0 * pp.m2));
  const auto e4 = static_cast<float>(t4 *
                                     (3.0 * pp.d3 * (pp.l2 + 1.0) * pp.l2 * speed + 6.0 * pp.d3 * (pp.l2 + 1.0) * pp.l4 * speed -
                                      6.0 * pp.d2 * G * (pp.l4 + pp.l2 + 1.0) * pp.l * pp.m) /
                                     (36.0 * pow((pp.l2 + 1.0), 2) * pp.m3));
  const auto e5 = static_cast<float>(t5 * (3.0 * pp.d3 * G * pp.l3 * pp.m - 3.0 * pp.d4 * pp.l2 * (pp.l2 + 1.0) * speed) /
                                     (36.0 * (pp.l2 + 1.0) * pp.m4));

  // Calculate the payload travel distance h
  const auto h = e1 + e2 + e3 + e4 + e5;

  return h;
}

int ballistics(BallisticResult &result, const BallisticInput &input)
{
  const PayloadParams pp = payloadParams(
    input.ammoName);  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay) - поки не знаємо про std::string

  if (!pp.ok) {
    return 1;
  }

  // Copy Ammo name for stats
  std::strncpy(
    result.ammoName,  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay) - поки не знаємо про std::string
    input.ammoName,  // NOLINT(cppcoreguidelines-pro-bounds-array-to-pointer-decay,hicpp-no-array-decay) - поки не знаємо про std::string
    MAX_AMMO_LENGTH - 1);

  const float t = payloadTimeOfFlight(pp, input.altitude, input.attackSpeed);

  if (t <= 0) {
    std::cerr << "Invalid t=" << t << '\n';
    return 1;
  }

  const float h = calcHDistance(t, input.attackSpeed, pp);

  if (h <= 0) {
    std::cerr << "Invalid h=" << h << '\n';
    return 1;
  }

  result.dronePos = input.startPos;
  result.targetPos = input.targetPos;
  // Calculate drone to target distance
  float distanceToTarget = distance(result.dronePos, result.targetPos);

  if (distanceToTarget <= 0) {
    std::cout << "Invalid D=" << distanceToTarget << '\n';
    return 1;
  }

  // Check if drone has to maneuvre and calculate new xd, yd

  if (h + input.accelPath > distanceToTarget) {
    if (std::fabs(distanceToTarget) < 1e-6F) {
      result.dronePos.x = input.targetPos.x - (h + input.accelPath);
      result.dronePos.y = input.targetPos.y;
      distanceToTarget = h + input.accelPath;
    }
    else {
      result.dronePos = result.targetPos - (result.targetPos - result.dronePos) * (h + input.accelPath) / distanceToTarget;
      distanceToTarget = distance(result.dronePos, result.targetPos);
    }
  }

  const float ratio = (distanceToTarget - h) / distanceToTarget;

  // Calculate drop point coordinates
  result.dropPoint = result.dronePos + (result.targetPos - result.dronePos) * ratio;
  result.payloadDropTime = t;

  return 0;
}