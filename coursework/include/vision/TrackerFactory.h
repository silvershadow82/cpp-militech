#pragma once

#include <memory>
#include <string>

#include "interfaces/ITracker.h"

namespace follow::vision {

// "kcf" or "csrt"; throws std::invalid_argument for any other name.
std::unique_ptr<interfaces::ITracker> makeTracker(const std::string& name);

}  // namespace follow::vision
