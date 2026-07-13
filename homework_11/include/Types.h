#pragma once

#include "models/Coord.h"
#include <string>

constexpr int MAX_AMMO_NAME{32};
constexpr float G{9.81F};

enum DroneState { STOPPED = 0, ACCELERATING, DECELERATING, TURNING, MOVING };
enum class SolverType { ANALYTICAL, TABLE };
enum class ProviderType { JSON, UART };
enum class LoaderType { FILE, UART };
enum class ConfigSource { FILE };

struct PayloadParams {
  float m, m2, m3, m4;
  float d, d2, d3, d4;
  float l, l2, l3, l4;
};

struct AmmoParams {
  std::string name;
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
};

struct DroneConfig {
  Coord startPos;
  float altitude;
  float initialDir;
  float attackSpeed;
  float accelPath;
  std::string ammoName;
  float arrayTimeStep;
  float simTimeStep;
  float hitRadius;
  float angularSpeed;
  float turnThreshold;
  float physicsTimeStep;
  float timeScale;
};

struct MissionContext {
  Coord dronePos;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTargetPos;
  DroneConfig cfg;
  DroneState state;
  DroneState commandMode;
  float droneSpeed;
  float droneAngle;
  float desiredAngle;
  float targetAngle;
  float droneAccel;
  float timeToStop;
  float payloadDropTime;
  int step;
  int lastTargetIdx;
  float t;
  float timeSecSinceStart;
};

// Pure ballistics output: projectile flight time and horizontal travel distance.
// Fire-point geometry (drop point, aim point, accel-path maneuver) is the mission
// processor's responsibility, not the solver's (SRP).
struct BallisticResult {
  bool ok;
  float t;  // час польоту снаряда
  float h;  // горизонтальна балістична дистанція
};

struct SimStep {
  Coord pos;
  int targetIdx;
  float direction;
  DroneState state;
  Coord dropPoint;
  Coord aimPoint;
  Coord predictedTarget;
  float timeSecSinceStart;
};
