#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "typed_topic_demo/msg/target.hpp"
#include "typed_topic_demo/target_formatter.hpp"

namespace {

constexpr auto kTargetTopic = "/demo/target";

}  // namespace

class TargetSubscriber final : public rclcpp::Node {
public:
  TargetSubscriber()
    : Node("target_subscriber")
  {
    subscription_ = create_subscription<typed_topic_demo::msg::Target>(
      kTargetTopic, 10, [this](const typed_topic_demo::msg::Target& target) { on_target(target); });
  }

private:
  void on_target(const typed_topic_demo::msg::Target& target)
  {
    RCLCPP_INFO(get_logger(),
                "received target label=%s x=%.1f y=%.1f distance_m=%.1f confidence=%.2f",
                typed_topic_demo::target_label(target).c_str(),
                target.x,
                target.y,
                target.distance_m,
                target.confidence);
  }

  rclcpp::Subscription<typed_topic_demo::msg::Target>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSubscriber>());
  rclcpp::shutdown();
  return 0;
}
