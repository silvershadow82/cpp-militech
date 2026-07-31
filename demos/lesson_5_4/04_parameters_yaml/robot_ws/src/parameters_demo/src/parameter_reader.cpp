#include <algorithm>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

class ParameterReader final : public rclcpp::Node {
public:
  ParameterReader()
    : Node("parameter_reader")
  {
    confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.8);
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);
    const auto publish_hz = declare_parameter<double>("publish_hz", 1.0);

    const auto safe_hz = std::max(0.1, publish_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / safe_hz));
    timer_ = create_wall_timer(period, [this]() { print_parameters(); });
  }

private:
  void print_parameters()
  {
    RCLCPP_INFO(get_logger(), "confidence_threshold=%.2f max_distance_m=%.1f", confidence_threshold_, max_distance_m_);
  }

  double confidence_threshold_{0.8};
  double max_distance_m_{30.0};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParameterReader>());
  rclcpp::shutdown();
  return 0;
}
