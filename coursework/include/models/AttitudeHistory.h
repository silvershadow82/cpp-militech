#pragma once

#include <chrono>
#include <deque>
#include <optional>

#include "Types.h"

namespace follow::models {

// Recent attitude samples, used to look up the attitude at a camera frame's capture time.
class AttitudeHistory {
public:
  explicit AttitudeHistory(std::chrono::milliseconds span = std::chrono::milliseconds{1000})
    : span(span)
  {
  }

  // Appends a sample. Samples not newer than the latest one are ignored.
  void push(const AttitudeSample& sample);

  std::optional<AttitudeSample> latest() const;

  // Attitude at time t: interpolated between samples, the newest sample if t is newer than it,
  // nullopt if t is older than the oldest sample kept.
  std::optional<AttitudeSample> at(TimePoint t) const;

private:
  std::chrono::milliseconds span;
  std::deque<AttitudeSample> samples;
};

}  // namespace follow::models
