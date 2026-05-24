#include <cstdint>
#include <cstring>
#include <iostream>
#include "ballistics.hpp"
#include "debug.hpp"

float distance(Coord coord1, Coord coord2)
{
  return (coord1 - coord2).length();
}

int ammoByName(const char *ammoName, const AmmoParams *ammo, int ammoCount, AmmoParams &ammoParams)
{
  if (ammo == nullptr) {
    std::cerr << "Ammo is empty - read the ammo.json first" << std::endl;
    return 1;
  }

  for (uint16_t i = 0; i < ammoCount; i++) {
    if (strcmp(ammoName, ammo[i].name) == 0) {
      ammoParams = ammo[i];
      return 0;
    }
  }
  std::cerr << "Ammo not found: " << ammoName << std::endl;
  return 1;
}

PayloadParams payloadParams(const char ammoName[MAX_AMMO_LENGTH], const AmmoParams *ammo, int ammoCount)
{
  // Define payload parameters based on the name
  AmmoParams selectedAmmo;

  ammoByName(ammoName, ammo, ammoCount, selectedAmmo);

  PayloadParams pp;
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

  float a = static_cast<float>(pp.d * G * pp.m - 2.0 * pp.d2 * pp.l * speed);
  float b = static_cast<float>(-3.0 * G * pp.m2 + 3.0 * pp.d * pp.l * pp.m * speed);
  float c = static_cast<float>(6.0 * pp.m2 * altitude);
  float a2 = a * a, a3 = a2 * a;
  float b2 = b * b, b3 = b2 * b;

  // Cardano trigonometric solution
  float p = static_cast<float>(-b2 / (3.0 * a2));
  float q = static_cast<float>((2.0 * b3) / (27.0 * a3) + c / a);
  float arg = static_cast<float>(((3.0 * q) / (2.0 * p)) * sqrt(-3.0 / p));

  // Check that arg is within the range [-1, 1]
  if (arg < -1 || arg > 1) {
    std::cerr << "Warning: acos arg is out of range: " << arg << std::endl;
    return -1.f;
  }

  float phi = acos(arg);

  // Calculate payload travel time
  float t = static_cast<float>(2.0 * sqrt(-p / 3.0) * cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a));

  return t;
}

float calcHDistance(float t, float speed, const PayloadParams &pp)
{
  float t2 = t * t;
  float t3 = t2 * t;
  float t4 = t2 * t2;
  float t5 = t3 * t2;

  float e1 = t * speed;
  float e2 = static_cast<float>(-t2 * pp.d * speed / (2.0 * pp.m));
  float e3 = static_cast<float>(t3 * (6.0 * pp.d * G * pp.l * pp.m - 6.0 * pp.d2 * (pp.l2 - 1.0) * speed) / (36.0 * pp.m2));
  float e4 = static_cast<float>(t4 *
                                (3.0 * pp.d3 * (pp.l2 + 1.0) * pp.l2 * speed + 6.0 * pp.d3 * (pp.l2 + 1.0) * pp.l4 * speed -
                                 6.0 * pp.d2 * G * (pp.l4 + pp.l2 + 1.0) * pp.l * pp.m) /
                                (36.0 * pow((pp.l2 + 1.0), 2) * pp.m3));
  float e5 = static_cast<float>(t5 * (3.0 * pp.d3 * G * pp.l3 * pp.m - 3.0 * pp.d4 * pp.l2 * (pp.l2 + 1.0) * speed) /
                                (36.0 * (pp.l2 + 1.0) * pp.m4));

  // Calculate the payload travel distance h
  float h = e1 + e2 + e3 + e4 + e5;

  return h;
}

int ballistics(BallisticResult &result, const BallisticInput &input)
{
  PayloadParams pp = payloadParams(input.ammoName, input.ammo, input.ammoCount);

  // Copy Ammo name for stats
  std::strncpy(result.ammoName, input.ammoName, MAX_AMMO_LENGTH - 1);

  float t = payloadTimeOfFlight(pp, input.altitude, input.attackSpeed);

  if (t <= 0) {
    std::cout << "Invalid t=" << t << std::endl;
    return 1;
  }

  float h = calcHDistance(t, input.attackSpeed, pp);

  if (h <= 0) {
    std::cout << "Invalid h=" << h << std::endl;
    return 1;
  }

  result.dronePos = input.startPos;
  result.targetPos = input.targetPos;
  // Calculate drone to target distance
  float distanceToTarget = distance(result.dronePos, result.targetPos);

  if (distanceToTarget <= 0) {
    std::cout << "Invalid D=" << distanceToTarget << std::endl;
    return 1;
  }

  // Check if drone has to maneuvre and calculate new xd, yd

  if (h + input.accelPath > distanceToTarget) {
    if (fabs(distanceToTarget) < 1e-6) {
      result.dronePos.x = input.targetPos.x - (h + input.accelPath);
      result.dronePos.y = input.targetPos.y;

      DEBUG("with intermediate point: NewDronePos=(" << result.dronePos.x << "," << result.dronePos.y << ")");

      distanceToTarget = h + input.accelPath;
    }
    else {
      result.dronePos = result.targetPos - (result.targetPos - result.dronePos) * (h + input.accelPath) / distanceToTarget;

      DEBUG("NewDronePos=(" << result.dronePos.x << ", " << result.dronePos.y << ")");

      distanceToTarget = distance(result.dronePos, result.targetPos);
    }
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;

  // Calculate drop point coordinates
  //   Coord dir = {static_cast<float>(cos(result.droneAngle)), static_cast<float>(sin(result.droneAngle))};
  result.dropPoint = result.dronePos + (result.targetPos - result.dronePos) * ratio;
  //   result.aimPoint = result.dronePos + dir * h;
  result.payloadDropTime = t;

  // fireX = xd + (targetX - xd) * ratio;
  // fireY = yd + (targetY - yd) * ratio;

  return 0;
}