#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "custom_service_demo/srv/trigger_actuator.hpp"

namespace {

constexpr auto kTriggerService = "/demo/trigger";

}  // namespace

class TriggerServer final : public rclcpp::Node {
public:
  using TriggerActuator = custom_service_demo::srv::TriggerActuator;

  TriggerServer()
    : Node("trigger_server")
  {
    service_ =
      create_service<TriggerActuator>(kTriggerService,
                                      [this](const std::shared_ptr<TriggerActuator::Request> request,
                                             std::shared_ptr<TriggerActuator::Response> response) { handle_request(request, response); });
  }

private:
  void handle_request(const std::shared_ptr<TriggerActuator::Request>& request, const std::shared_ptr<TriggerActuator::Response>& response)
  {
    const auto accepted = request->confidence >= 0.8F && request->distance_m <= 30.0F;

    if (accepted) {
      ++trigger_count_;
    }

    response->accepted = accepted;
    response->trigger_count = trigger_count_;

    RCLCPP_INFO(get_logger(),
                "request confidence=%.2f distance_m=%.1f accepted=%s trigger_count=%u",
                request->confidence,
                request->distance_m,
                accepted ? "true" : "false",
                trigger_count_);
  }

  std::uint32_t trigger_count_{0};
  rclcpp::Service<TriggerActuator>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TriggerServer>());
  rclcpp::shutdown();
  return 0;
}
