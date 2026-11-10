#pragma once

#include <optional>

#include "Types.h"
#include "models/Intrinsics.h"

namespace follow::interfaces {

// The camera's projection, owned by follow_core. Deliberately OpenCV-free: control/TargetEstimator
// holds a reference to one, so an OpenCV include reaching this header would break the default
// FOLLOW_WITH_OPENCV=OFF build, the devcontainer build and the aarch64 cross-build at once.
class ICameraModel {
public:
  virtual ~ICameraModel() = default;

  // Unit ray in the camera optical frame (x right, y down, z forward) through pixel p.
  virtual models::Vec3 pixelToRay(const models::Pixel& p) const = 0;
  // Pixel hit by a ray; nullopt if the ray is 90 degrees or more off the optical axis.
  virtual std::optional<models::Pixel> rayToPixel(const models::Vec3& ray) const = 0;

  const models::Intrinsics& intrinsics() const { return this->intr; }
  bool contains(const models::Pixel& p) const { return p.u >= 0.0 && p.v >= 0.0 && p.u <= this->intr.width && p.v <= this->intr.height; }

protected:
  explicit ICameraModel(const models::Intrinsics& intrinsics)
    : intr(intrinsics)
  {
  }

  models::Intrinsics intr;
};

}  // namespace follow::interfaces
