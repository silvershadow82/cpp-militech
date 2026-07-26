

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

class GimbalDriverNode final : public rclcpp::Node {
public:
  GimbalDriverNode()
    : Node("gimbal_driver_node")
  {
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalDriverNode>());
  rclcpp::shutdown();
  return 0;
}