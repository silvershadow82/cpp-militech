#pragma once

#include <cmath>
#include <numbers>

namespace follow::core {

constexpr double degToRad(double deg)
{
  return deg * std::numbers::pi / 180.0;
}

constexpr double radToDeg(double rad)
{
  return rad * 180.0 / std::numbers::pi;
}

// Wraps an angle into (-pi, pi].
inline double wrapPi(double rad)
{
  double wrapped = std::remainder(rad, 2.0 * std::numbers::pi);
  return wrapped <= -std::numbers::pi ? wrapped + 2.0 * std::numbers::pi : wrapped;
}

}  // namespace follow::core
