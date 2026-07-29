#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include "antidrone_turret/msg/servo_command.hpp"

namespace {
constexpr auto servoTopic = "/servo/cmd";
}

using ServoCommand = antidrone_turret::msg::ServoCommand;

class YawServoDriverNode final : public rclcpp::Node {
public:
  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
    const rclcpp::QoS qos{10};
    this->subscription =
      create_subscription<ServoCommand>(servoTopic, qos, [this](const ServoCommand& command) { this->onCommand(command); });
  }

private:
  rclcpp::Subscription<ServoCommand>::SharedPtr subscription;
  void onCommand(const ServoCommand& command)
  {
    RCLCPP_INFO(get_logger(), "moving servo dir=%d by=%.2f", command.direction, command.error_x);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}