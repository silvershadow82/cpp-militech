#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

constexpr auto enemyDownTopic = "/payload/enemy_down";
constexpr auto triggerServiceName = "/payload/trigger";

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action_node")
  {
    this->enemyDownPublisher = create_publisher<EnemyDown>(enemyDownTopic, rclcpp::QoS{10});

    this->triggerService =
      create_service<PayloadTrigger>(triggerServiceName,
                                     [this](const std::shared_ptr<PayloadTrigger::Request> request,
                                            std::shared_ptr<PayloadTrigger::Response> response) { onTrigger(*request, *response); });

    RCLCPP_INFO(get_logger(), "payload action ready on %s", triggerServiceName);
  }

private:
  void onTrigger(const PayloadTrigger::Request& request, PayloadTrigger::Response& response)
  {
    EnemyDown message;
    message.contact_id = request.contact_id;
    message.x = request.x;
    message.y = request.y;
    this->enemyDownPublisher->publish(message);

    response.accepted = true;
    response.reason = "payload delivered";

    RCLCPP_INFO(get_logger(), "engaged contact_id=%d at (%d,%d)", request.contact_id, request.x, request.y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemyDownPublisher;
  rclcpp::Service<PayloadTrigger>::SharedPtr triggerService;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
