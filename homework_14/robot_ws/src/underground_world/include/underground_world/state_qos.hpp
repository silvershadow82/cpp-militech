#pragma once

#include "rclcpp/qos.hpp"

namespace underground_world {

// Use the same profile on both sides of world-state topics so late
// subscribers receive the most recently published snapshot.
[[nodiscard]] inline rclcpp::QoS make_state_qos()
{
  return rclcpp::QoS{rclcpp::KeepLast(1)}.reliable().transient_local();
}

}  // namespace underground_world
