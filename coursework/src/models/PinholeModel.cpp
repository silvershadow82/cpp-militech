#include "models/PinholeModel.h"

#include "models/Frames.h"

namespace follow::models {

namespace {

constexpr double kEpsilon = 1e-12;

}  // namespace

Vec3 PinholeModel::pixelToRay(const Pixel& p) const
{
  return normalized({(p.u - this->intr.cx) / this->intr.fx, (p.v - this->intr.cy) / this->intr.fy, 1.0});
}

std::optional<Pixel> PinholeModel::rayToPixel(const Vec3& ray) const
{
  if (ray.z <= kEpsilon) {
    return std::nullopt;
  }
  return Pixel{this->intr.fx * ray.x / ray.z + this->intr.cx, this->intr.fy * ray.y / ray.z + this->intr.cy};
}

}  // namespace follow::models
