#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

constexpr auto kTelemetryTopic = "/robot/telemetry";

class StatusMonitor final : public rclcpp::Node {
public:
  StatusMonitor()
    : Node("status_monitor")
  {
    subscription_ =
      create_subscription<std_msgs::msg::String>(kTelemetryTopic, 10, [this](const std_msgs::msg::String& msg) { on_telemetry(msg); });

    RCLCPP_INFO(get_logger(), "listening on %s", kTelemetryTopic);
  }

private:
  void on_telemetry(const std_msgs::msg::String& msg) { RCLCPP_INFO(get_logger(), "telemetry: %s", msg.data.c_str()); }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusMonitor>());
  rclcpp::shutdown();
  return 0;
}
