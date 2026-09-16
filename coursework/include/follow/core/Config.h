#pragma once

#include <chrono>
#include <optional>

namespace follow::core {

struct EstimatorConfig {
  double emaAlpha{0.3};
  std::chrono::milliseconds stale{300};
  std::chrono::milliseconds attitudeStale{200};  // newest attitude sample older than this invalidates the target
  double borderMarginPx{2.0};                    // a box closer than this to the image edge may be clipped
  double maxAreaJump{2.0};
  double minConfidence{0.3};
  std::optional<double> targetHeightM{};  // enables the KnownSize distance source
  double rangeGateDeg{2.0};
  std::chrono::milliseconds rangeSync{50};
  std::chrono::milliseconds rangeHold{300};
};

struct ControlConfig {
  double kYaw{1.5};  // 1/s
  double yawRateMaxDps{45.0};
  double yawDeadbandDeg{2.0};
  double kD{0.4};  // 1/s
  double vxMax{1.5};
  double vxSlew{1.5};  // m/s^2
  double dSet{3.0};
  double dNominal{3.0};
  double dMin{1.5};
  double distDeadbandM{0.3};
  double headingGateDeg{25.0};
  bool enableVx{true};
};

struct SupervisorConfig {
  std::chrono::milliseconds fcTimeout{2000};
  std::chrono::milliseconds lockTimeout{1000};
  std::chrono::milliseconds lostTimeout{3000};
};

struct Config {
  EstimatorConfig estimator{};
  ControlConfig control{};
  SupervisorConfig supervisor{};
  double rateHz{20.0};
  double lockBoxFrac{0.20};  // side of the square lock box as a fraction of image height
};

}  // namespace follow::core
