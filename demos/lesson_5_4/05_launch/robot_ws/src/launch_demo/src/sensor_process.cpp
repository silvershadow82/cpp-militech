#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

class SensorProcess final : public rclcpp::Node {
public:
  SensorProcess()
    : Node("sensor_process")
  {
    source_name_ = declare_parameter<std::string>("source_name", "sim_sensor");
    const auto publish_hz = declare_parameter<double>("publish_hz", 1.0);

    const auto safe_hz = std::max(0.1, publish_hz);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / safe_hz));
    timer_ = create_wall_timer(period, [this]() { tick(); });
  }

private:
  void tick() { RCLCPP_INFO(get_logger(), "source_name=%s", source_name_.c_str()); }

  std::string source_name_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorProcess>());
  rclcpp::shutdown();
  return 0;
}
