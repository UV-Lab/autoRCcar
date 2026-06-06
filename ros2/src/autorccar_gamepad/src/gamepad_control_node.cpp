#include "gamepad_control_node.hpp"
#include "autorccar_interfaces/msg/control_command.hpp"
#include <cmath>
#include <algorithm>

GamepadControlNode::GamepadControlNode()
: Node("gamepad_control_node")
{
  // ── Defind Parameter  ────────────────────────────────────────────────
  this->declare_parameter<int>        ("accel_arw_index", 7);
  this->declare_parameter<int>        ("steer_arw_index", 6);
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
  accel_arw_  = this->get_parameter("accel_arw_index").as_int();
  steer_arw_  = this->get_parameter("steer_arw_index").as_int();
  steer_axis_  = this->get_parameter("steer_axis_index").as_int();

  stop_btn_  = this->get_parameter("stop_index").as_int();
  manual_btn_  = this->get_parameter("manual_index").as_int();
  auto_btn_  = this->get_parameter("auto_index").as_int();
  tbd_btn_  = this->get_parameter("tbd_index").as_int();
  reset_btn_  = this->get_parameter("reset_index").as_int();
  reset2_btn_  = this->get_parameter("reset2_index").as_int();
  accel_scale_ = this->get_parameter("accel_scale").as_double();
  steer_scale_ = this->get_parameter("steer_scale").as_double();
  deadzone_    = this->get_parameter("deadzone").as_double();

  cmd_mode_ = 0;
  accel_val = 0;
  steer_val = 0;

  prev_joy_.buttons.assign(20, 0);
  prev_joy_.axes.assign(20, 0.0f);

  // ── Subscriber ───────────────────────────────────────────────────
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10,
    std::bind(&GamepadControlNode::joyCallback, this, std::placeholders::_1)
  );

  // ── Publisher ────────────────────────────────────────────────────
  control_cmd_pub_ = this->create_publisher<autorccar_interfaces::msg::ControlCommand>("teleop/control_command", 10);
  cmd_pub_ = this->create_publisher<std_msgs::msg::Int8>("teleop/command", 10);

  RCLCPP_INFO(this->get_logger(),
    "\n"
    "  [joy_vehicle_control] Node Start\n"
    "  Accel/Steer arrow index : axes[%d] × %.2f | axes[%d] × %.2f)\n"
    "  Steer stick index       : axes[%d]\n"
    "  Control index           : [%d], [%d], [%d], [%d]\n"
    "  Reset index             : [%d], [%d]\n"
    "  Deadzone                : ±%.3f",
    accel_arw_, accel_scale_, steer_arw_, steer_scale_,
    steer_axis_,
    stop_btn_, manual_btn_, auto_btn_, tbd_btn_,
    reset_btn_, reset2_btn_,
    deadzone_
  );

  last_joy_time_ = this->now();

  // Check connection every 1000ms
  safety_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GamepadControlNode::safetyCheck, this)
  );
}

void GamepadControlNode::safetyCheck()
{
  const double elapsed = (this->now() - last_joy_time_).seconds();

  if (elapsed > 2 && joy_connected_) {
    joy_connected_ = false;
    accel_val = 0.0;
    steer_val = 90.0;
    cmd_mode_ = 0;
    RCLCPP_WARN(this->get_logger(),
      "Gamepad controller disconnected! (%.1f sec no signal) Emergency stop.", elapsed);
  }
}

void GamepadControlNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{

  // -- Update connection status -----------------------------------
  last_joy_time_ = this->now();
  if (!joy_connected_) {
    joy_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Gamepad controller connected.");
  }

  const int axes_size = static_cast<int>(msg->axes.size());

  // -- Check axes index range ---------------------------------------
  const auto check_axis = [&](int idx, const char* name) -> bool {
    if (idx >= axes_size) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "%s(%d) >= axes size(%d).", name, idx, axes_size);
      return false;
    }
    return true;
  };

  if (!check_axis(accel_arw_,  "accel_arw_index") ||
      !check_axis(steer_arw_,  "steer_arw_index")  ||
      !check_axis(steer_axis_, "steer_axis_index")) return;

  // -- Read raw values ----------------------------------------------
  const int    raw_accel     = static_cast<int>(msg->axes[accel_arw_]);
  const int    raw_steer     = static_cast<int>(msg->axes[steer_arw_]);
  const double raw_steer_joy = msg->axes[steer_axis_];

  // -- Rising edge helper -------------------------------------------
  auto rising = [&](int btn_idx) {
    return msg->buttons[btn_idx] == 1 && prev_joy_.buttons[btn_idx] == 0;
  };

  // -- Mode check ---------------------------------------------------
  if (rising(stop_btn_)) {
    cmd_mode_ = 0;  accel_val = 0.0;  steer_val = 0.0;
    RCLCPP_INFO(this->get_logger(), "Teleop: Stopped.");
  }
  if (rising(manual_btn_)) {
    cmd_mode_ = 1;
    RCLCPP_INFO(this->get_logger(), "Teleop: Manual mode ENABLED.");
  }
  if (rising(auto_btn_)) {
    cmd_mode_ = 2;
    RCLCPP_INFO(this->get_logger(), "Teleop: Auto mode ENABLED.");
  }
  if (rising(tbd_btn_)) {
    cmd_mode_ = 3;
    RCLCPP_INFO(this->get_logger(), "Teleop: TBD mode ENABLED.");
  }
  if (rising(reset_btn_)) {
    accel_val = 0.0;  steer_val = 0.0;  cmd_mode_ = 0;
    RCLCPP_INFO(this->get_logger(), "Teleop: Stopped.");
  }
  if (rising(reset2_btn_)) {
    accel_val = 0.0;  steer_val = 0.0;;
    RCLCPP_INFO(this->get_logger(), "autoRccar Stop!");
  }

  // -- Control data -------------------------------------------------
  if (cmd_mode_ == 1) {
    // Arrow key: increment/decrement on rising edge
    if (raw_accel ==  1 && prev_joy_.axes[accel_arw_] == 0) accel_val += accel_scale_;
    if (raw_accel == -1 && prev_joy_.axes[accel_arw_] == 0) accel_val -= accel_scale_;

    // Joystick steer overrides arrow key only when outside deadzone
    if (std::fabs(raw_steer_joy) > deadzone_) {
      // Joystick steer: linear mapping [-1, +1] → [-45, +45]
      steer_val = static_cast<int>(std::round(raw_steer_joy * 45));
    } else {
      // Arrow key steer: increment/decrement on rising edge
      if (raw_steer ==  1 && prev_joy_.axes[steer_arw_] == 0) steer_val += steer_scale_;
      if (raw_steer == -1 && prev_joy_.axes[steer_arw_] == 0) steer_val -= steer_scale_;
    }
  } else {
    accel_val = 0.0;
    steer_val = 0.0;
  }

  // -- Clamp --------------------------------------------------------
  accel_val = std::max(accel_val, 0.0);
  steer_val = std::clamp(steer_val, -45.0, 45.0);

  // -- Update previous state ----------------------------------------
  prev_joy_ = *msg;

  // -- Publish ------------------------------------------------------
  autorccar_interfaces::msg::ControlCommand control_msg;
  std_msgs::msg::Int8 mode_msg;

  control_msg.speed            = static_cast<float>(accel_val);
  control_msg.steering_angle   = static_cast<float>(steer_val * M_PI/180.0);
  mode_msg.data                = cmd_mode_;

  control_cmd_pub_->publish(control_msg);
  cmd_pub_->publish(mode_msg);

  RCLCPP_DEBUG(this->get_logger(), "accel=%.3f  steer=%.3f", accel_val, steer_val);
}
