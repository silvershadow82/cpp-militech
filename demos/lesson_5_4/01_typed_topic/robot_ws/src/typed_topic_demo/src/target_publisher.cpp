#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "typed_topic_demo/msg/target.hpp"

namespace {

constexpr auto kTargetTopic = "/demo/target";

std::vector<typed_topic_demo::msg::Target> make_samples()
{
  auto first = typed_topic_demo::msg::Target{};
  first.visible = true;
  first.x = 420.0F;
  first.y = 180.0F;
  first.distance_m = 25.0F;
  first.confidence = 0.90F;

  auto second = typed_topic_demo::msg::Target{};
  second.visible = true;
  second.x = 260.0F;
  second.y = 260.0F;
  second.distance_m = 45.0F;
  second.confidence = 0.88F;

  auto third = typed_topic_demo::msg::Target{};
  third.visible = false;
  third.x = 0.0F;
  third.y = 0.0F;
  third.distance_m = 0.0F;
  third.confidence = 0.0F;

  return {first, second, third};
}

}  // namespace

class TargetPublisher final : public rclcpp::Node {
public:
  TargetPublisher()
    : Node("target_publisher")
    , samples_(make_samples())
  {
    publisher_ = create_publisher<typed_topic_demo::msg::Target>(kTargetTopic, 10);
    timer_ = create_wall_timer(std::chrono::milliseconds{1000}, [this]() { publish_next(); });
  }

private:
  void publish_next()
  {
    const auto& sample = samples_[next_index_ % samples_.size()];
    publisher_->publish(sample);

    RCLCPP_INFO(get_logger(),
                "published target visible=%s x=%.1f y=%.1f distance_m=%.1f confidence=%.2f",
                sample.visible ? "true" : "false",
                sample.x,
                sample.y,
                sample.distance_m,
                sample.confidence);

    ++next_index_;
  }

  std::vector<typed_topic_demo::msg::Target> samples_;
  std::size_t next_index_{0};
  rclcpp::Publisher<typed_topic_demo::msg::Target>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPublisher>());
  rclcpp::shutdown();
  return 0;
}
