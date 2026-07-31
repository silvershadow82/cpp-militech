#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

constexpr auto kTelemetryTopic = "/robot/telemetry";

class TelemetryPublisher final : public rclcpp::Node {
public:
  TelemetryPublisher()
    : Node("telemetry_publisher")
  {
    publisher_ = create_publisher<std_msgs::msg::String>(kTelemetryTopic, 10);
    timer_ = create_wall_timer(100ms, [this]() { publish(); });

    RCLCPP_INFO(get_logger(), "publishing %s at 10 Hz", kTelemetryTopic);
  }

private:
  void publish()
  {
    auto msg = std_msgs::msg::String{};
    const auto battery = 87 - static_cast<int>(sequence_ % 5);
    msg.data = "seq=" + std::to_string(sequence_) + " state=ACTIVE battery=" + std::to_string(battery);

    publisher_->publish(msg);

    if (sequence_ % 10 == 0) {
      RCLCPP_INFO(get_logger(), "published: %s", msg.data.c_str());
    }

    ++sequence_;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t sequence_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TelemetryPublisher>());
  rclcpp::shutdown();
  return 0;
}
