#pragma once

#include <chrono>

#include "Types.h"

namespace follow::test {

// Deterministic timestamps: `at(1.5)` is 1.5 s after an arbitrary fixed origin.
inline models::TimePoint at(double seconds)
{
  return models::TimePoint{} + std::chrono::hours{1} +
         std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(seconds));
}

}  // namespace follow::test
