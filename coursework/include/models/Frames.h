#pragma once

#include "Types.h"

namespace follow::models {

// Frames used by the core:
//   camera - optical frame: x right, y down, z forward
//   body   - FRD: x forward, y right, z down
//   level  - body frame with roll and pitch removed (heading kept)
//   NED    - x north, y east, z down

struct CameraMount {
  double tiltUpDeg{0.0};  // optical axis pitched up from body forward
};

Vec3 cameraToBody(const Vec3& camera, const CameraMount& mount);
Vec3 bodyToCamera(const Vec3& body, const CameraMount& mount);
Vec3 bodyToLevel(const Vec3& body, double roll, double pitch);
Vec3 bodyToNed(const Vec3& body, double roll, double pitch, double yaw);
Vec3 nedToBody(const Vec3& ned, double roll, double pitch, double yaw);

double norm(const Vec3& v);
Vec3 normalized(const Vec3& v);
// Angle between two vectors in radians.
double angleBetween(const Vec3& a, const Vec3& b);

}  // namespace follow::models
