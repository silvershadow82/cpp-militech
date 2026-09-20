#include "models/CameraModel.h"

#include <cmath>
#include <numbers>

#include "models/Frames.h"

namespace follow::models {

namespace {

constexpr double kEpsilon = 1e-12;
constexpr int kNewtonIterations = 10;

double distortTheta(const Intrinsics& k, double theta)
{
  double t2 = theta * theta;
  return theta * (1.0 + t2 * (k.k1 + t2 * (k.k2 + t2 * (k.k3 + t2 * k.k4))));
}

double distortThetaDerivative(const Intrinsics& k, double theta)
{
  double t2 = theta * theta;
  return 1.0 + t2 * (3.0 * k.k1 + t2 * (5.0 * k.k2 + t2 * (7.0 * k.k3 + t2 * 9.0 * k.k4)));
}

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

Vec3 FisheyeKbModel::pixelToRay(const Pixel& p) const
{
  double xd = (p.u - this->intr.cx) / this->intr.fx;
  double yd = (p.v - this->intr.cy) / this->intr.fy;
  double thetaD = std::hypot(xd, yd);
  if (thetaD < kEpsilon) {
    return {0.0, 0.0, 1.0};
  }

  // Newton's method on distortTheta(theta) = thetaD, starting from the undistorted guess.
  double theta = thetaD;
  for (int i = 0; i < kNewtonIterations; ++i) {
    double step = (distortTheta(this->intr, theta) - thetaD) / distortThetaDerivative(this->intr, theta);
    theta -= step;
    if (std::abs(step) < 1e-10) {
      break;
    }
  }

  double scale = std::sin(theta) / thetaD;
  return {xd * scale, yd * scale, std::cos(theta)};
}

std::optional<Pixel> FisheyeKbModel::rayToPixel(const Vec3& ray) const
{
  double rho = std::hypot(ray.x, ray.y);
  double theta = std::atan2(rho, ray.z);
  if (theta >= std::numbers::pi / 2.0) {
    return std::nullopt;
  }
  if (rho < kEpsilon) {
    return Pixel{this->intr.cx, this->intr.cy};
  }
  double thetaD = distortTheta(this->intr, theta);
  double xd = thetaD * ray.x / rho;
  double yd = thetaD * ray.y / rho;
  return Pixel{this->intr.fx * xd + this->intr.cx, this->intr.fy * yd + this->intr.cy};
}

}  // namespace follow::models
