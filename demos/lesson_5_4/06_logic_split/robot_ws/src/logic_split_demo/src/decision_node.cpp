#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "logic_split_demo/target_decision.hpp"

namespace {

std::vector<logic_split_demo::TargetInput> make_samples()
{
  return {
    logic_split_demo::TargetInput{true, 0.90F, 25.0F},
    logic_split_demo::TargetInput{true, 0.88F, 45.0F},
    logic_split_demo::TargetInput{true, 0.40F, 18.0F},
    logic_split_demo::TargetInput{false, 0.0F, 0.0F},
  };
}

}  // namespace

class DecisionNode final : public rclcpp::Node {
public:
  DecisionNode()
    : Node("decision_node")
    , samples_(make_samples())
  {
    config_.confidence_threshold = static_cast<float>(declare_parameter<double>("confidence_threshold", 0.8));
    config_.max_distance_m = static_cast<float>(declare_parameter<double>("max_distance_m", 30.0));

    timer_ = create_wall_timer(std::chrono::milliseconds{1000}, [this]() { tick(); });
  }

private:
  void tick()
  {
    const auto& sample = samples_[next_index_ % samples_.size()];
    const auto decision = logic_split_demo::decide(sample, config_);

    RCLCPP_INFO(get_logger(),
                "visible=%s confidence=%.2f distance_m=%.1f decision=%s",
                sample.visible ? "true" : "false",
                sample.confidence,
                sample.distance_m,
                logic_split_demo::decision_name(decision));

    ++next_index_;
  }

  logic_split_demo::DecisionConfig config_;
  std::vector<logic_split_demo::TargetInput> samples_;
  std::size_t next_index_{0};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
  return 0;
}
