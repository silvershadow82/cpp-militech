#pragma once

#include <optional>

#include "Types.h"
#include "interfaces/ICameraModel.h"
#include "models/Intrinsics.h"

namespace follow::models {

// Ideal rectilinear projection: u = fx * x/z + cx. Distortion coefficients are ignored.
class PinholeModel final : public interfaces::ICameraModel {
public:
  explicit PinholeModel(const Intrinsics& intrinsics)
    : ICameraModel(intrinsics)
  {
  }

  Vec3 pixelToRay(const Pixel& p) const override;
  std::optional<Pixel> rayToPixel(const Vec3& ray) const override;
};

}  // namespace follow::models
