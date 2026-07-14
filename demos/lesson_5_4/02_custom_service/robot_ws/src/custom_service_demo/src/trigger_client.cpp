#include <chrono>
#include <cstdlib>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "custom_service_demo/srv/trigger_actuator.hpp"

using namespace std::chrono_literals;

namespace {

constexpr auto kTriggerService = "/demo/trigger";

float parse_arg(char** argv, int index, float fallback)
{
  if (argv[index] == nullptr) {
    return fallback;
  }

  return static_cast<float>(std::atof(argv[index]));
}

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  using TriggerActuator = custom_service_demo::srv::TriggerActuator;

  auto node = rclcpp::Node::make_shared("trigger_client");
  auto client = node->create_client<TriggerActuator>(kTriggerService);

  if (!client->wait_for_service(3s)) {
    RCLCPP_ERROR(node->get_logger(), "service %s is not available", kTriggerService);
    rclcpp::shutdown();
    return 1;
  }

  auto request = std::make_shared<TriggerActuator::Request>();
  request->confidence = argc > 1 ? parse_arg(argv, 1, 0.9F) : 0.9F;
  request->distance_m = argc > 2 ? parse_arg(argv, 2, 25.0F) : 25.0F;

  client->async_send_request(request, [node](rclcpp::Client<TriggerActuator>::SharedFuture future) {
    const auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "accepted=%s trigger_count=%u", response->accepted ? "true" : "false", response->trigger_count);
    rclcpp::shutdown();
  });

  rclcpp::spin(node);
  return 0;
}
