#pragma once

#include <chrono>

#include "follow/core/Types.h"

namespace follow::test {

// Deterministic timestamps: `at(1.5)` is 1.5 s after an arbitrary fixed origin.
inline core::TimePoint at(double seconds)
{
  return core::TimePoint{} + std::chrono::hours{1} +
         std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(seconds));
}

}  // namespace follow::test
