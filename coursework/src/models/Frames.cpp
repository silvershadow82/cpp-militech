#include "models/Frames.h"

#include <algorithm>
#include <cmath>

#include "models/Angles.h"

namespace follow::models {

namespace {

// Active rotations about the x, y and z axes. Body to world is Rz(yaw) * Ry(pitch) * Rx(roll).
Vec3 rotX(const Vec3& v, double angle)
{
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {v.x, c * v.y - s * v.z, s * v.y + c * v.z};
}

Vec3 rotY(const Vec3& v, double angle)
{
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {c * v.x + s * v.z, v.y, -s * v.x + c * v.z};
}

Vec3 rotZ(const Vec3& v, double angle)
{
  double c = std::cos(angle);
  double s = std::sin(angle);
  return {c * v.x - s * v.y, s * v.x + c * v.y, v.z};
}

}  // namespace

Vec3 cameraToBody(const Vec3& camera, const CameraMount& mount)
{
  Vec3 untilted{camera.z, camera.x, camera.y};
  // A positive pitch about body y lifts the forward axis (z is down).
  return rotY(untilted, degToRad(mount.tiltUpDeg));
}

Vec3 bodyToCamera(const Vec3& body, const CameraMount& mount)
{
  Vec3 untilted = rotY(body, -degToRad(mount.tiltUpDeg));
  return {untilted.y, untilted.z, untilted.x};
}

Vec3 bodyToLevel(const Vec3& body, double roll, double pitch)
{
  return rotY(rotX(body, roll), pitch);
}

Vec3 bodyToNed(const Vec3& body, double roll, double pitch, double yaw)
{
  return rotZ(rotY(rotX(body, roll), pitch), yaw);
}

Vec3 nedToBody(const Vec3& ned, double roll, double pitch, double yaw)
{
  return rotX(rotY(rotZ(ned, -yaw), -pitch), -roll);
}

double norm(const Vec3& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 normalized(const Vec3& v)
{
  double n = norm(v);
  return {v.x / n, v.y / n, v.z / n};
}

double angleBetween(const Vec3& a, const Vec3& b)
{
  double cosine = (a.x * b.x + a.y * b.y + a.z * b.z) / (norm(a) * norm(b));
  return std::acos(std::clamp(cosine, -1.0, 1.0));
}

}  // namespace follow::models
