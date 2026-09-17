#include "follow/core/AttitudeHistory.h"

#include <algorithm>
#include <iterator>

#include "follow/core/Angles.h"

namespace follow::core {

void AttitudeHistory::push(const AttitudeSample& sample)
{
  if (!this->samples.empty() && sample.t <= this->samples.back().t) {
    return;
  }
  this->samples.push_back(sample);
  while (this->samples.front().t < sample.t - this->span) {
    this->samples.pop_front();
  }
}

std::optional<AttitudeSample> AttitudeHistory::latest() const
{
  if (this->samples.empty()) {
    return std::nullopt;
  }
  return this->samples.back();
}

std::optional<AttitudeSample> AttitudeHistory::at(TimePoint t) const
{
  if (this->samples.empty() || t < this->samples.front().t) {
    return std::nullopt;
  }
  if (t >= this->samples.back().t) {
    return this->samples.back();
  }

  auto upper =
    std::lower_bound(this->samples.begin(), this->samples.end(), t, [](const AttitudeSample& s, TimePoint value) { return s.t < value; });
  if (upper->t == t) {
    return *upper;
  }
  const AttitudeSample& hi = *upper;
  const AttitudeSample& lo = *std::prev(upper);
  double f = std::chrono::duration<double>(t - lo.t) / std::chrono::duration<double>(hi.t - lo.t);
  return AttitudeSample{
    .t = t,
    .roll = lo.roll + f * (hi.roll - lo.roll),
    .pitch = lo.pitch + f * (hi.pitch - lo.pitch),
    .yaw = wrapPi(lo.yaw + f * wrapPi(hi.yaw - lo.yaw)),
  };
}

}  // namespace follow::core
