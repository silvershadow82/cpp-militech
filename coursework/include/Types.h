#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace follow::models {

using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pixel {
  double u{0.0};
  double v{0.0};
};

// Axis-aligned box in image pixels: top-left corner plus size.
struct BBox {
  double x{0.0};
  double y{0.0};
  double w{0.0};
  double h{0.0};

  double centerU() const { return this->x + this->w / 2.0; }
  double centerV() const { return this->y + this->h / 2.0; }
  double area() const { return this->w * this->h; }
};

struct AttitudeSample {
  TimePoint t{};
  double roll{0.0};   // rad, right side down positive
  double pitch{0.0};  // rad, nose up positive
  double yaw{0.0};    // rad, clockwise from north
};

// Vehicle position relative to the EKF origin, as in MAVLink LOCAL_POSITION_NED.
struct LocalPositionNed {
  TimePoint t{};
  Vec3 position{};  // m
  Vec3 velocity{};  // m/s
};

struct TargetObservation {
  TimePoint tFrame{};  // capture time of the frame, not the time tracking finished
  BBox box{};
  bool ok{false};
  double confidence{0.0};
};

struct RangeMeasurement {
  TimePoint t{};
  double rangeM{0.0};
  bool valid{false};
};

enum class DistanceSource { Relative, KnownSize, Range };

struct TargetState {
  bool valid{false};
  double bearingRad{0.0};  // positive = target to the right
  double ratio{1.0};       // estimated distance / distance at lock
  std::optional<double> distanceM{};
  DistanceSource source{DistanceSource::Relative};
};

struct VelocityCmd {
  double vx{0.0};       // m/s along body forward
  double yawRate{0.0};  // rad/s, clockwise positive
};

enum class State { Idle, Locking, Following, Lost, Hold, NoFc };

// ArduCopter flight modes as reported in HEARTBEAT.custom_mode.
constexpr uint32_t kModeAltHold = 2;
constexpr uint32_t kModeGuided = 4;
constexpr uint32_t kModeLoiter = 5;

}  // namespace follow::models
