#pragma once

#include <optional>

#include "Types.h"
#include "interfaces/ICameraModel.h"
#include "models/Intrinsics.h"

namespace follow::models {

// Kannala-Brandt equidistant model, identical to OpenCV's cv::fisheye:
// theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8).
class FisheyeKbModel final : public interfaces::ICameraModel {
public:
  explicit FisheyeKbModel(const Intrinsics& intrinsics)
    : ICameraModel(intrinsics)
  {
  }

  Vec3 pixelToRay(const Pixel& p) const override;
  std::optional<Pixel> rayToPixel(const Vec3& ray) const override;
};

}  // namespace follow::models
