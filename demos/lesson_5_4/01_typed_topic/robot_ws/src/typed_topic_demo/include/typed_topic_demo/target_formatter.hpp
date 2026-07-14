#pragma once

#include <string>

#include "typed_topic_demo/msg/target.hpp"

namespace typed_topic_demo {

inline std::string target_label(const msg::Target& target)
{
  if (!target.visible) {
    return "hidden";
  }

  if (target.confidence < 0.8F) {
    return "low_confidence";
  }

  return "tracked";
}

}  // namespace typed_topic_demo
