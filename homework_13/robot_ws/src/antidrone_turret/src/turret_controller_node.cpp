
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include "antidrone_turret/msg/actuator_status.hpp"

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

    this->actuatorStatusSubscription = create_subscription<ActuatorStatus>(
      actuatorStatusTopic, qos, [this](const ActuatorStatus& status) { this->onActuatorStatus(status); });
  }

private:
  rclcpp::Subscription<antidrone_turret::msg::ActuatorStatus>::SharedPtr actuatorStatusSubscription;
  void onActuatorStatus(const antidrone_turret::msg::ActuatorStatus& status)
  {
    RCLCPP_INFO(get_logger(), "received actuator status in state %d", status.state);
  };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurretControllerNode>());
  rclcpp::shutdown();
  return 0;
}