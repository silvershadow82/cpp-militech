#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/actuator_model.hpp"
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

  bool triggerInFlight = false;
  std::uint32_t lastTriggerCount = 0;
  std::uint32_t triggerCountAtRequest = 0;

  [[nodiscard]] antidrone_turret::ActuatorState currentActuatorState() const
  {
    return this->triggerInFlight ? antidrone_turret::ActuatorState::kReloading : this->lastActuatorState;
  }

  void processTriggerResult(const antidrone_turret::TargetSample& target, const core::ComputeResult& computeResult)
  {
    if (computeResult.action == core::Action::ACTION_TRACK) {
      // Move Gimbal
      publishGimbalCommand(computeResult);

      // Move Servo
      publishServoCommand(computeResult);

      if (computeResult.triggerState == core::TriggerState::TRIGGER_REQUESTED) {
        this->requestTrigger(target);
      }
    }

    this->publishTurretStatus(target, computeResult);
  }

  void requestTrigger(const antidrone_turret::TargetSample& target)
  {
    if (!this->actuatorClient->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "actuator service %s is not available yet", actuatorServicePath);
      return;
    }

    auto request = std::make_shared<TriggerActuator::Request>();
    request->confidence = target.confidence;
    request->distance_m = target.distance_m;

    this->triggerInFlight = true;
    this->triggerCountAtRequest = this->lastTriggerCount;

    this->actuatorClient->async_send_request(request, [this](rclcpp::Client<TriggerActuator>::SharedFuture future) {
      const auto response = future.get();

      if (!response->accepted) {
        this->triggerInFlight = false;
      }

      RCLCPP_INFO(get_logger(),
                  "actuator service response - accepted: %s, trigger_count: %u",
                  response->accepted ? "true" : "false",
                  response->trigger_count);
    });
  }

  void onActuatorStatus(const antidrone_turret::msg::ActuatorStatus& status)
  {
    RCLCPP_INFO(get_logger(), "received actuator status in state %u with trigger_count %u", status.state, status.trigger_count);
    this->lastActuatorState = static_cast<antidrone_turret::ActuatorState>(status.state);

    if (this->triggerInFlight && status.trigger_count > this->triggerCountAtRequest) {
      this->triggerInFlight = false;
    }

    this->lastTriggerCount = status.trigger_count;
  }

  void onTargetUpdate(const antidrone_turret::msg::Target& target)
  {
    const auto sample = toTargetSample(target);
    RCLCPP_INFO(get_logger(), "received target update - confidence: %.2f, x: %.3f, y: %.3f", sample.confidence, sample.x, sample.y);
    const auto result = this->coreController->computeTriggerResult(sample, this->currentActuatorState());
    this->processTriggerResult(sample, result);
  }

  void publishTurretStatus(const antidrone_turret::TargetSample& target, const core::ComputeResult& computeResult)
  {
    const auto statusView = core::makeTurretStatus(target, computeResult);

    TurretStatus status{};
    status.target_state = static_cast<uint8_t>(statusView.targetState);
    status.trigger_state = static_cast<uint8_t>(statusView.triggerState);
    status.action = static_cast<uint8_t>(statusView.action);
    status.confidence = statusView.confidence;
    status.distance_m = statusView.distanceM;

    this->turretStatusPublisher->publish(status);
  }

  void publishGimbalCommand(const core::ComputeResult& computeResult)
  {
    auto gimbalCommand = GimbalCommand{};
    gimbalCommand.direction = static_cast<std::int8_t>(computeResult.gimbalCommand.gimbalDirection);
    gimbalCommand.target_y = computeResult.gimbalCommand.targetY;
    gimbalCommand.error_y = computeResult.gimbalCommand.errorY;

    this->gimbalPublisher->publish(gimbalCommand);
  }

  void publishServoCommand(const core::ComputeResult& computeResult)
  {
    auto servoCommand = ServoCommand{};
    servoCommand.direction = static_cast<std::int8_t>(computeResult.servoCommand.servoDirection);
    servoCommand.target_x = computeResult.servoCommand.targetX;
    servoCommand.error_x = computeResult.servoCommand.errorX;

    this->servoPublisher->publish(servoCommand);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}