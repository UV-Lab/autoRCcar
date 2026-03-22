#include "gamepad_control_node.hpp"
#include "autorccar_interfaces/msg/control_command.hpp"
#include <cmath>

GamepadControlNode::GamepadControlNode()
: Node("gamepad_control_node")
{
  // ── 파라미터 선언 ────────────────────────────────────────────────
  this->declare_parameter<std::string>("joy_topic",        "/joy");
  this->declare_parameter<std::string>("teleop_control_command_topic",      "/teleop/control_command");
  this->declare_parameter<std::string>("teleop_command_topic",      "/teleop/command");
  this->declare_parameter<int>        ("accel_axis_index", 3);
  this->declare_parameter<int>        ("steer_axis_index", 2);
  this->declare_parameter<int>        ("stop_index", 0);
  this->declare_parameter<int>        ("manual_index", 1);
  this->declare_parameter<int>        ("auto_index", 2);
  this->declare_parameter<int>        ("tbd_index", 3);
  this->declare_parameter<int>        ("reset_index", 4);
  this->declare_parameter<int>        ("reset2_index", 5);
  this->declare_parameter<double>     ("accel_scale",      1.0);
  this->declare_parameter<double>     ("steer_scale",      1.0);
  this->declare_parameter<double>     ("deadzone",         0.05);

  // ── Read Parameter ────────────────────────────────────────────────
  const auto joy_topic   = this->get_parameter("joy_topic").as_string();
  const auto teleop_control_command_topic = this->get_parameter("teleop_control_command_topic").as_string();
  const auto teleop_command_topic = this->get_parameter("teleop_command_topic").as_string();
  accel_axis_  = this->get_parameter("accel_axis_index").as_int();
  steer_axis_  = this->get_parameter("steer_axis_index").as_int();
  stop_axis_  = this->get_parameter("stop_index").as_int();
  manual_axis_  = this->get_parameter("manual_index").as_int();
  auto_axis_  = this->get_parameter("auto_index").as_int();
  tbd_axis_  = this->get_parameter("tbd_index").as_int();
  reset_axis_  = this->get_parameter("reset_index").as_int();
  reset2_axis_  = this->get_parameter("reset2_index").as_int();
  accel_scale_ = this->get_parameter("accel_scale").as_double();
  steer_scale_ = this->get_parameter("steer_scale").as_double();
  deadzone_    = this->get_parameter("deadzone").as_double();

  cmd_mode_ = 0;

  // ── Subscriber ───────────────────────────────────────────────────
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10,
    std::bind(&GamepadControlNode::joyCallback, this, std::placeholders::_1)
  );

  // ── Publisher ────────────────────────────────────────────────────
  control_cmd_pub_ = this->create_publisher<autorccar_interfaces::msg::ControlCommand>(teleop_control_command_topic, 10);
  cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>(teleop_command_topic, 10);

  RCLCPP_INFO(this->get_logger(),
    "\n"
    "  [joy_vehicle_control] Node Start\n"
    "  Joy          Subscribe : %s\n"
    "  Accel/Steer  Publish : %s  (accel : axes[%d] × %.2f | axes[%d] × %.2f)\n"
    "  Control Mode Publish : %s [%d], [%d], [%d], [%d]\n"
    "  Reset      : [%d], [%d]\n"
    "  Deadzone   : ±%.3f",
    joy_topic.c_str(),
    teleop_control_command_topic.c_str(), accel_axis_, accel_scale_, steer_axis_, steer_scale_,
    teleop_command_topic.c_str(), stop_axis_, manual_axis_, auto_axis_, tbd_axis_,
    reset_axis_, reset2_axis_,
    deadzone_
  );
}

void GamepadControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // -- Check axes index range ---------------------------------------
  const int axes_size = static_cast<int>(msg->axes.size());

  if (accel_axis_ >= axes_size) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "accel_axis_index(%d) >= axes size(%d). Please check the parameters.",
      accel_axis_, axes_size);
    return;
  }

  if (steer_axis_ >= axes_size) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "steer_axis_index(%d) >= axes size(%d). Please check the parameters.",
      steer_axis_, axes_size);
    return;
  }

  // -- Read raw values ----------------------------------------------
  const double raw_accel = static_cast<double>(msg->axes[accel_axis_]);
  const double raw_steer = static_cast<double>(msg->axes[steer_axis_]);

  const int raw_stop = static_cast<int>(msg->buttons[stop_axis_]);
  const int raw_manual = static_cast<int>(msg->buttons[manual_axis_]);
  const int raw_auto = static_cast<int>(msg->buttons[auto_axis_]);
  const int raw_tbd = static_cast<int>(msg->buttons[tbd_axis_]);

  const int raw_reset = static_cast<int>(msg->buttons[reset_axis_]);
  const int raw_reset2 = static_cast<int>(msg->buttons[reset2_axis_]);

  // -- Apply deadzone and scaling -----------------------------------
  double accel_val = (std::fabs(raw_accel) > deadzone_) ? raw_accel * accel_scale_ : 0.0;
  double steer_val = (std::fabs(raw_steer) > deadzone_) ? raw_steer * steer_scale_ : 0.0;

  // -- Publish ------------------------------------------------------
  autorccar_interfaces::msg::ControlCommand control_msg;
  std_msgs::msg::Int8 mode_msg;

  if (raw_stop == 1)
    cmd_mode_ = 1;
  if (raw_manual == 1)
    cmd_mode_ = 2;
  if (raw_auto == 1)
    cmd_mode_ = 3;
  if (raw_tbd == 1)
    cmd_mode_ = 4;

  if (raw_reset == 1 || raw_reset2 == 1)
  {
    accel_val = 0;
    steer_val = 0;
    cmd_mode_ = 0;
  }

  control_msg.speed = static_cast<float>(accel_val);
  control_msg.steering_angle = static_cast<float>(steer_val);
  mode_msg.data = cmd_mode_;

  control_cmd_pub_->publish(control_msg);
  cmd_pub_->publish(mode_msg);

  RCLCPP_DEBUG(this->get_logger(),
    "accel=%.3f  steer=%.3f", accel_val, steer_val);
}