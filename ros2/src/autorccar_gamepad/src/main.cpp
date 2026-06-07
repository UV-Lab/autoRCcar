#include "gamepad_control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GamepadControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
