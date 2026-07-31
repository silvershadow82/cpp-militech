#pragma once

#include "ThreadSafeQueue.h"
#include "Types.h"
#include "models/Coord.h"
#include <atomic>
#include <mutex>

struct DroneCommand {
  DroneState mode;    // новий режим руху дрона
  float targetAngle;  // бажаний курс
};

struct DroneTelemetry {
  Coord pos;
  float speed;
  float angle;
  float accel;
  DroneState mode;
  float timeSecSinceStart;
};

class DronePhysics {
private:
  float physicsTimeStep;                       // крок фізики в секундах
  float timeScale;                             // масштаб часу симуляції
  DroneConfig config;                          // копія конфігурації дрона
  ThreadSafeQueue<DroneCommand> commandQueue;  // черга команд для дрона
  std::atomic<bool> stopFlag{false};           // прапорець для зупинки фізики
  std::atomic<bool> ready{false};              // прапорець готовності потоку
  std::atomic<bool> started{false};            // прапорець запуску інтеграції
  mutable std::mutex mtx;                      // м'ютекс для синхронізації доступу до стану дрона

  // Стан дрона (єдиний власник кінематики)
  Coord position;
  DroneState mode{DroneState::STOPPED};
  float speed{0};
  float accel{0};
  float currentAngle{0};
  float targetAngle{0};
  float attackSpeed{0};
  float timeSecSinceStart{0};

public:
  DronePhysics(const DroneConfig& cfg);
  bool isThreadReady();
  void run();
  void start();
  void stop();
  void enqueueCommand(const DroneCommand& command);
  void step();
  DroneTelemetry getTelemetry() const;
  float getPhysicsTimeStep() const { return physicsTimeStep; }
};
