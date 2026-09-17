#pragma once

#include <optional>

#include "follow/core/Types.h"

namespace follow::core {

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

class CameraModel {
public:
  virtual ~CameraModel() = default;

  // Unit ray in the camera optical frame (x right, y down, z forward) through pixel p.
  virtual Vec3 pixelToRay(const Pixel& p) const = 0;
  // Pixel hit by a ray; nullopt if the ray is 90 degrees or more off the optical axis.
  virtual std::optional<Pixel> rayToPixel(const Vec3& ray) const = 0;

  const Intrinsics& intrinsics() const { return this->intr; }
  bool contains(const Pixel& p) const;

protected:
  explicit CameraModel(const Intrinsics& intrinsics)
    : intr(intrinsics)
  {
  }

  Intrinsics intr;
};

class PinholeModel final : public CameraModel {
public:
  explicit PinholeModel(const Intrinsics& intrinsics)
    : CameraModel(intrinsics)
  {
  }

  Vec3 pixelToRay(const Pixel& p) const override;
  std::optional<Pixel> rayToPixel(const Vec3& ray) const override;
};

// Kannala-Brandt equidistant model, identical to OpenCV's cv::fisheye:
// theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8).
class FisheyeKbModel final : public CameraModel {
public:
  explicit FisheyeKbModel(const Intrinsics& intrinsics)
    : CameraModel(intrinsics)
  {
  }

  Vec3 pixelToRay(const Pixel& p) const override;
  std::optional<Pixel> rayToPixel(const Vec3& ray) const override;
};

// Undistorted equidistant fisheye whose image diagonal spans diagonalFovDeg. Used until calibrated.
Intrinsics nominalFisheye(int width, int height, double diagonalFovDeg);

}  // namespace follow::core
