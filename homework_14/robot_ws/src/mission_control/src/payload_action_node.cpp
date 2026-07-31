#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace {

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action_node")
  {
  }
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}
