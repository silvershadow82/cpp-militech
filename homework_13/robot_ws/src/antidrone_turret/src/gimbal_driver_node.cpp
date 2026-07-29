
#include "antidrone_turret/msg/gimbal_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>

namespace {
constexpr auto gimbalTopic = "/gimbal/cmd";
}

using GimbalCommand = antidrone_turret::msg::GimbalCommand;

class GimbalDriverNode final : public rclcpp::Node {
public:
  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
    rclcpp::QoS qos{10};
    this->subscription =
      create_subscription<GimbalCommand>(gimbalTopic, qos, [this](const GimbalCommand& command) { this->onCommand(command); });
  }

private:
  rclcpp::Subscription<antidrone_turret::msg::GimbalCommand>::SharedPtr subscription;

  void onCommand(const GimbalCommand& command)
  {
    RCLCPP_INFO(get_logger(),
                "received gimbal command - direction=%d target_y=%.2f error_y=%.2f",
                command.direction,
                command.target_y,
                command.error_y);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}