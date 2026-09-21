#include "models/Intrinsics.h"

#include <cmath>

#include "models/Angles.h"

namespace follow::models {

Intrinsics nominalFisheye(int width, int height, double diagonalFovDeg)
{
  double halfDiagonalPx = std::hypot(width, height) / 2.0;
  double focal = halfDiagonalPx / degToRad(diagonalFovDeg / 2.0);
  return Intrinsics{.width = width, .height = height, .fx = focal, .fy = focal, .cx = width / 2.0, .cy = height / 2.0};
}

}  // namespace follow::models
