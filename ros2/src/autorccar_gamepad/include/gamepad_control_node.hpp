#ifndef GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_
#define GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "autorccar_interfaces/msg/control_command.hpp"
#include <std_msgs/msg/int8.hpp>
#include <rclcpp/time.hpp>

class GamepadControlNode : public rclcpp::Node
{
public:
  GamepadControlNode();
  void safetyCheck();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Parameters
  int accel_arw_;
  int steer_arw_;
  int steer_axis_;
  int stop_btn_;
  int manual_btn_;
  int auto_btn_;
  int tbd_btn_;
  int reset_btn_;
  int reset2_btn_;
  int cmd_mode_;
  double accel_scale_;
  double steer_scale_;
  double deadzone_;

  double accel_val;
  double steer_val;

  sensor_msgs::msg::Joy prev_joy_;

  rclcpp::Time last_joy_time_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  bool joy_connected_ = false;

  // Subscriber & Publishers
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<autorccar_interfaces::msg::ControlCommand>::SharedPtr control_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr cmd_pub_;
};

#endif  // GAMEPAD_CONTROL__GAMEPAD_CONTROL_NODE_HPP_
