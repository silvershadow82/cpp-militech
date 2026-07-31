#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

class ControllerProcess final : public rclcpp::Node {
public:
  ControllerProcess()
    : Node("controller_process")
  {
    mode_ = declare_parameter<std::string>("mode", "SIM");
    max_distance_m_ = declare_parameter<double>("max_distance_m", 30.0);

    timer_ = create_wall_timer(std::chrono::milliseconds{1000}, [this]() { tick(); });
  }

private:
  void tick() { RCLCPP_INFO(get_logger(), "mode=%s max_distance_m=%.1f", mode_.c_str(), max_distance_m_); }

  std::string mode_;
  double max_distance_m_{30.0};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerProcess>());
  rclcpp::shutdown();
  return 0;
}
