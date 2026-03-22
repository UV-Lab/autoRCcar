#ifndef GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_
#define GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "autorccar_interfaces/msg/control_command.hpp"
#include <std_msgs/msg/int8.hpp>

class GamepadControlNode : public rclcpp::Node
{
public:
  GamepadControlNode();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Parameters
  int accel_axis_;
  int steer_axis_;
  int stop_axis_;
  int manual_axis_;
  int auto_axis_;
  int tbd_axis_;
  int reset_axis_;
  int reset2_axis_;
  int cmd_mode_;
  double accel_scale_;
  double steer_scale_;
  double deadzone_;

  // Subscriber & Publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmd_pub_;
};

#endif  // GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_