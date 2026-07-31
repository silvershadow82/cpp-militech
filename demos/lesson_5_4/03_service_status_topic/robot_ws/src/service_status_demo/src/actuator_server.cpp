#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "service_status_demo/msg/actuator_status.hpp"
#include "service_status_demo/srv/trigger_actuator.hpp"

namespace {

constexpr auto kStatusTopic = "/demo/actuator_status";
constexpr auto kTriggerService = "/demo/trigger";

}  // namespace

class ActuatorServer final : public rclcpp::Node {
public:
  using ActuatorStatus = service_status_demo::msg::ActuatorStatus;
  using TriggerActuator = service_status_demo::srv::TriggerActuator;

  ActuatorServer()
    : Node("actuator_server")
  {
    reload_ms_ = declare_parameter<int>("reload_ms", 2000);
    const auto status_publish_hz = declare_parameter<double>("status_publish_hz", 2.0);

    status_publisher_ = create_publisher<ActuatorStatus>(kStatusTopic, 10);
    service_ =
      create_service<TriggerActuator>(kTriggerService,
                                      [this](const std::shared_ptr<TriggerActuator::Request> request,
                                             std::shared_ptr<TriggerActuator::Response> response) { on_trigger(request, response); });

    const auto safe_hz = std::max(0.1, status_publish_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / safe_hz));
    status_timer_ = create_wall_timer(period, [this]() { publish_status(); });

    publish_status();
  }

private:
  void on_trigger(const std::shared_ptr<TriggerActuator::Request>& request, const std::shared_ptr<TriggerActuator::Response>& response)
  {
    if (!ready_) {
      response->accepted = false;
      response->trigger_count = trigger_count_;
      publish_status();
      return;
    }

    ++trigger_count_;
    ready_ = false;

    response->accepted = true;
    response->trigger_count = trigger_count_;

    RCLCPP_INFO(get_logger(),
                "shot accepted confidence=%.2f distance_m=%.1f trigger_count=%u",
                request->confidence,
                request->distance_m,
                trigger_count_);

    publish_status();
    schedule_reload();
  }

  void schedule_reload()
  {
    if (reload_timer_) {
      reload_timer_->cancel();
    }

    reload_timer_ = create_wall_timer(std::chrono::milliseconds{std::max(1, reload_ms_)}, [this]() {
      ready_ = true;
      publish_status();
      reload_timer_->cancel();
    });
  }

  void publish_status()
  {
    auto status = ActuatorStatus{};
    status.state = ready_ ? ActuatorStatus::READY : ActuatorStatus::RELOADING;
    status.trigger_count = trigger_count_;
    status_publisher_->publish(status);
  }

  int reload_ms_{2000};
  bool ready_{true};
  std::uint32_t trigger_count_{0};
  rclcpp::Publisher<ActuatorStatus>::SharedPtr status_publisher_;
  rclcpp::Service<TriggerActuator>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr reload_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorServer>());
  rclcpp::shutdown();
  return 0;
}
