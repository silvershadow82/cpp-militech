#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/core_controller.hpp"
#include "antidrone_turret/target_sequence.hpp"

namespace {

constexpr auto targetTopic = "/perception/target";
constexpr auto gimbalTopic = "/gimbal/cmd";
constexpr auto servoTopic = "/servo/cmd";

constexpr auto actuatorServicePath = "/actuator/trigger";
constexpr auto actuatorStatusTopic = "/actuator/status";
constexpr auto turretStatusTopic = "/turret/status";

antidrone_turret::TargetSample toTargetSample(const antidrone_turret::msg::Target& target)
{
  return antidrone_turret::TargetSample{
    .visible = target.visible, .x = target.x, .y = target.y, .distance_m = target.distance_m, .confidence = target.confidence};
};

}  // namespace

using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
using GimbalCommand = antidrone_turret::msg::GimbalCommand;
using Target = antidrone_turret::msg::Target;
using ServoCommand = antidrone_turret::msg::ServoCommand;
using TurretStatus = antidrone_turret::msg::TurretStatus;
using TriggerActuator = antidrone_turret::srv::TriggerActuator;

class TurretControllerNode final : public rclcpp::Node {
public:
  TurretControllerNode()
    : Node("turret_controller_node")
  {
    rclcpp::QoS qos{10};

    this->actuatorStatusSubscription = create_subscription<ActuatorStatus>(
      actuatorStatusTopic, qos, [this](const ActuatorStatus& status) { this->onActuatorStatus(status); });

    this->targetSubscription =
      create_subscription<Target>(targetTopic, qos, [this](const Target& target) { this->onTargetUpdate(target); });

    this->gimbalPublisher = create_publisher<GimbalCommand>(gimbalTopic, qos);
    this->servoPublisher = create_publisher<ServoCommand>(servoTopic, qos);
    this->turretStatusPublisher = create_publisher<TurretStatus>(turretStatusTopic, qos);

    this->actuatorClient = create_client<TriggerActuator>(actuatorServicePath);

    const auto confidence_threshold = declare_parameter<float>("confidence_threshold", 0.8f);
    const auto max_distance_m = declare_parameter<float>("max_distance_m", 30.0f);

    this->coreController = std::make_unique<core::CoreController>(confidence_threshold, max_distance_m);

    RCLCPP_INFO(get_logger(),
                "initialized turret controller node with confidence threshold=%.1f, max_distance=%.1f",
                confidence_threshold,
                max_distance_m);
  }

private:
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuatorStatusSubscription;
  rclcpp::Subscription<antidrone_turret::msg::Target>::SharedPtr targetSubscription;
  rclcpp::Publisher<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbalPublisher;
  rclcpp::Publisher<antidrone_turret::msg::ServoCommand>::SharedPtr servoPublisher;
  rclcpp::Publisher<antidrone_turret::msg::TurretStatus>::SharedPtr turretStatusPublisher;
  rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedPtr actuatorClient;

  std::unique_ptr<core::CoreController> coreController;
  antidrone_turret::ActuatorState lastActuatorState = antidrone_turret::ActuatorState::kReady;

  void processTriggerResult(const antidrone_turret::msg::Target& target, const core::ComputeResult& triggerResult)
  {
    if (triggerResult.action == core::Action::ACTION_TRACK) {
      // Move Gimbal
      auto gimbalCommand = GimbalCommand{};
      gimbalCommand.direction = static_cast<int>(triggerResult.gimbalCommand.gimbalDirection);
      gimbalCommand.target_y = triggerResult.gimbalCommand.targetY;
      gimbalCommand.error_y = triggerResult.gimbalCommand.errorY;

      this->gimbalPublisher->publish(gimbalCommand);

      // Move Servo
      auto servoCommand = ServoCommand{};
      servoCommand.direction = static_cast<int>(triggerResult.servoCommand.servoDirection);
      servoCommand.target_x = triggerResult.servoCommand.targetX;
      servoCommand.error_x = triggerResult.servoCommand.errorX;

      this->servoPublisher->publish(servoCommand);

      if (triggerResult.triggerState == core::TriggerState::TRIGGER_REQUESTED) {
        // Request trigger
        auto request = std::make_shared<TriggerActuator::Request>();
        request->confidence = target.confidence;
        request->distance_m = target.distance_m;
        this->actuatorClient->async_send_request(request, [this, &target](rclcpp::Client<TriggerActuator>::SharedFuture future) {
          auto response = future.get();
          // publish trigger status to the topic
        });
      }
      else if (triggerResult.triggerState == core::TriggerState::TRIGGER_RELOADING) {
        // publish trigger status to the topic
      }
    }
  }

  void onActuatorStatus(const antidrone_turret::msg::ActuatorStatus& status)
  {
    RCLCPP_INFO(get_logger(), "received actuator status in state %d", status.state);
    this->lastActuatorState = static_cast<antidrone_turret::ActuatorState>(status.state);
  }

  void onTargetUpdate(const antidrone_turret::msg::Target& target)
  {
    RCLCPP_INFO(get_logger(), "received target update - confidence: %.2f, x: %.3f, y: %.3f", target.confidence, target.x, target.y);
    core::ComputeResult result = this->coreController->computeTriggerResult(toTargetSample(target), this->lastActuatorState);
    this->processTriggerResult(target, result);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}