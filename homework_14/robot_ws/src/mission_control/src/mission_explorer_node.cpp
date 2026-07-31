#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace {

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer_node")
  {
  }
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
