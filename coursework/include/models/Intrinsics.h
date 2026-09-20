#pragma once

namespace follow::models {

// Intrinsics in OpenCV's layout. k1..k4 are only used by the fisheye model.
struct Intrinsics {
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double k1{0.0};
  double k2{0.0};
  double k3{0.0};
  double k4{0.0};
};

// Undistorted equidistant fisheye whose image diagonal spans diagonalFovDeg. Used until calibrated.
Intrinsics nominalFisheye(int width, int height, double diagonalFovDeg);

}  // namespace follow::models
