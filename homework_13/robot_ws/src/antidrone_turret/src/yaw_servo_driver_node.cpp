#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

class YawServoDriverNode final : public rclcpp::Node {
public:
  YawServoDriverNode()
    : Node("yaw_servo_driver_node")
  {
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YawServoDriverNode>());
  rclcpp::shutdown();
  return 0;
}