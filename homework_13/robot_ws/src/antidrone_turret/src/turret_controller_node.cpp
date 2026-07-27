
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "antidrone_turret/msg/target.hpp"
#include "antidrone_turret/msg/actuator_status.hpp"
#include "antidrone_turret/msg/gimbal_command.hpp"
#include "antidrone_turret/msg/servo_command.hpp"
#include "antidrone_turret/msg/turret_status.hpp"
#include "antidrone_turret/srv/trigger_actuator.hpp"
#include "antidrone_turret/core_controller.hpp"

namespace {

constexpr auto targetTopic = "/perception/target";
constexpr auto gimbalTopic = "/gimbal/cmd";
constexpr auto servoTopic = "/servo/cmd";

constexpr auto actuatorServicePath = "/actuator/trigger";
constexpr auto actuatorStatusTopic = "/actuator/status";
constexpr auto turretStatusTopic = "/turret/status";

}  // namespace

class TurretControllerNode final : public rclcpp::Node {
public:
  TurretControllerNode()
    : Node("turret_controller_node")
  {
    rclcpp::QoS qos{10};
    using ActuatorStatus = antidrone_turret::msg::ActuatorStatus;
    using GimbalCommand = antidrone_turret::msg::GimbalCommand;
    using Target = antidrone_turret::msg::Target;
    using ServoCommand = antidrone_turret::msg::ServoCommand;
    using TurretStatus = antidrone_turret::msg::TurretStatus;
    using TriggerActuator = antidrone_turret::srv::TriggerActuator;

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
  }

private:
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuatorStatusSubscription;
  rclcpp::Subscription<antidrone_turret::msg::Target>::SharedPtr targetSubscription;
  rclcpp::Publisher<antidrone_turret::msg::GimbalCommand>::SharedPtr gimbalPublisher;
  rclcpp::Publisher<antidrone_turret::msg::ServoCommand>::SharedPtr servoPublisher;
  rclcpp::Publisher<antidrone_turret::msg::TurretStatus>::SharedPtr turretStatusPublisher;
  rclcpp::Client<antidrone_turret::srv::TriggerActuator>::SharedPtr actuatorClient;

  std::unique_ptr<core::CoreController> coreController;

  void onActuatorStatus(const antidrone_turret::msg::ActuatorStatus& status)
  {
    RCLCPP_INFO(get_logger(), "received actuator status in state %d", status.state);
  }

  void onTargetUpdate(const antidrone_turret::msg::Target& target)
  {
    RCLCPP_INFO(get_logger(), "received target update - confidence: %.2f, x: %.3f, y: %.3f", target.confidence, target.x, target.y);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}