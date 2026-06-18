// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using sensor_msgs::msg::Joy;
using std_msgs::msg::Bool;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  deadman_timeout_(0, 0),
  command_timeout_(0, 0),
  vesc_state_timeout_(0, 0),
  deadman_active_brake_(false),
  active_braking_enabled_(false),
  runtime_active_braking_enabled_(false),
  runtime_active_braking_default_enabled_(false),
  autonomy_override_enabled_(false),
  autonomy_override_timeout_(0, 0),
  active_braking_enable_timeout_(0, 0),
  deadman_pressed_(false),
  have_joy_(false),
  deadman_state_logged_(false),
  deadman_last_active_(false),
  autonomy_override_active_(false),
  have_autonomy_override_(false),
  have_runtime_active_braking_(false),
  have_command_(false),
  have_vesc_state_(false),
  braking_active_(false),
  measured_speed_mps_(0.0),
  last_brake_current_(0.0),
  last_longitudinal_mode_(""),
  last_joy_time_(0, 0, get_clock()->get_clock_type()),
  last_autonomy_override_time_(0, 0, get_clock()->get_clock_type()),
  last_active_braking_enable_time_(0, 0, get_clock()->get_clock_type()),
  last_command_time_(0, 0, get_clock()->get_clock_type()),
  last_vesc_state_time_(0, 0, get_clock()->get_clock_type()),
  last_brake_update_time_(0, 0, get_clock()->get_clock_type())
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain", 0.0);
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset", 0.0);
  steering_to_servo_gain_ = declare_parameter<double>("steering_angle_to_servo_gain", 0.0);
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset", 0.0);
  joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
  deadman_button_index_ = declare_parameter<int>("deadman_button_index", 4);
  deadman_button_index_alt_ = declare_parameter<int>("deadman_button_index_alt", 5);
  deadman_active_brake_ = declare_parameter<bool>("deadman_active_brake", true);
  autonomy_override_enabled_ = declare_parameter<bool>("autonomy_override_enabled", false);
  autonomy_override_topic_ =
    declare_parameter<std::string>("autonomy_override_topic", "/safety/autonomy_enabled");
  autonomy_override_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("autonomy_override_timeout_sec", 0.5)));
  deadman_brake_current_ =
    std::max(0.0, declare_parameter<double>("deadman_brake_current", 12000.0));
  command_timeout_brake_current_ =
    std::max(
    0.0, declare_parameter<double>(
      "command_timeout_brake_current", deadman_brake_current_));
  active_braking_enabled_ = declare_parameter<bool>("active_braking_enabled", true);
  runtime_active_braking_default_enabled_ = declare_parameter<bool>(
    "runtime_active_braking_default_enabled", active_braking_enabled_);
  active_braking_enable_topic_ = declare_parameter<std::string>(
    "active_braking_enable_topic", "/safety/vesc_active_braking_enabled");
  active_braking_enable_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("active_braking_enable_timeout_sec", 0.5)));
  runtime_active_braking_enabled_ = runtime_active_braking_default_enabled_;
  active_brake_min_current_ =
    std::max(0.0, declare_parameter<double>("active_brake_min_current", 1.0));
  active_brake_max_current_ =
    std::max(
    active_brake_min_current_,
    declare_parameter<double>("active_brake_max_current", 9.0));
  brake_current_per_decel_ =
    std::max(0.0, declare_parameter<double>("brake_current_per_decel", 1.2));
  brake_current_per_speed_error_ =
    std::max(0.0, declare_parameter<double>("brake_current_per_speed_error", 2.4));
  default_decel_mps2_ =
    std::max(0.0, declare_parameter<double>("default_decel_mps2", 1.0));
  brake_enter_speed_error_mps_ =
    std::max(0.0, declare_parameter<double>("brake_enter_speed_error_mps", 0.05));
  brake_release_speed_error_mps_ =
    std::max(0.0, declare_parameter<double>("brake_release_speed_error_mps", 0.02));
  brake_min_vehicle_speed_mps_ =
    std::max(0.0, declare_parameter<double>("brake_min_vehicle_speed_mps", 0.05));
  brake_current_slew_rate_aps_ =
    std::max(0.0, declare_parameter<double>("brake_current_slew_rate_aps", 40.0));
  vesc_state_topic_ =
    declare_parameter<std::string>("vesc_state_topic", "sensors/core");
  deadman_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("deadman_timeout_sec", 0.5)));
  command_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("command_timeout_sec", 0.25)));
  vesc_state_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("vesc_state_timeout_sec", 0.25)));

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  joy_sub_ = create_subscription<Joy>(
    joy_topic_, 10, std::bind(&AckermannToVesc::joyCallback, this, _1));
  autonomy_override_sub_ = create_subscription<Bool>(
    autonomy_override_topic_, 10, std::bind(&AckermannToVesc::autonomyOverrideCallback, this, _1));
  active_braking_enable_sub_ = create_subscription<Bool>(
    active_braking_enable_topic_, 10,
    std::bind(&AckermannToVesc::activeBrakingEnableCallback, this, _1));
  vesc_state_sub_ = create_subscription<VescStateStamped>(
    vesc_state_topic_, 10, std::bind(&AckermannToVesc::vescStateCallback, this, _1));

  longitudinal_timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&AckermannToVesc::longitudinalTimerCallback, this));

  RCLCPP_INFO(
    get_logger(),
    "Active braking enabled=%s current=[%.1f, %.1f]A command_timeout=%.2fs state_timeout=%.2fs",
    active_braking_enabled_ ? "true" : "false",
    active_brake_min_current_,
    active_brake_max_current_,
    command_timeout_.seconds(),
    vesc_state_timeout_.seconds());
  RCLCPP_INFO(
    get_logger(),
    "Runtime active braking topic=%s default=%s timeout=%.2fs",
    active_braking_enable_topic_.c_str(),
    runtime_active_braking_default_enabled_ ? "true" : "false",
    active_braking_enable_timeout_.seconds());
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  last_command_ = *cmd;
  last_command_time_ = now();
  have_command_ = true;

  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;
  servo_pub_->publish(servo_msg);
}

void AckermannToVesc::joyCallback(const Joy::SharedPtr joy_msg)
{
  have_joy_ = true;
  last_joy_time_ = now();

  const bool pressed_primary =
    deadman_button_index_ >= 0 &&
    static_cast<size_t>(deadman_button_index_) < joy_msg->buttons.size() &&
    joy_msg->buttons[deadman_button_index_] == 1;
  const bool pressed_alt =
    deadman_button_index_alt_ >= 0 &&
    static_cast<size_t>(deadman_button_index_alt_) < joy_msg->buttons.size() &&
    joy_msg->buttons[deadman_button_index_alt_] == 1;
  deadman_pressed_ = pressed_primary || pressed_alt;

  const bool active = deadmanActive();
  logDeadmanState(active);
}

void AckermannToVesc::autonomyOverrideCallback(const Bool::SharedPtr msg)
{
  have_autonomy_override_ = true;
  autonomy_override_active_ = msg->data;
  last_autonomy_override_time_ = now();

  const bool active = deadmanActive();
  logDeadmanState(active);
}

void AckermannToVesc::activeBrakingEnableCallback(const Bool::SharedPtr msg)
{
  have_runtime_active_braking_ = true;
  runtime_active_braking_enabled_ = msg->data;
  last_active_braking_enable_time_ = now();
}

void AckermannToVesc::vescStateCallback(const VescStateStamped::SharedPtr state)
{
  if (std::abs(speed_to_erpm_gain_) < 1e-9) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Cannot convert VESC telemetry: speed_to_erpm_gain is zero");
    return;
  }
  measured_speed_mps_ =
    (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  last_vesc_state_time_ = now();
  have_vesc_state_ = true;
}

void AckermannToVesc::longitudinalTimerCallback()
{
  const bool active = deadmanActive();
  logDeadmanState(active);
  if (!active) {
    braking_active_ = false;
    if (deadman_active_brake_) {
      publishBrake(deadman_brake_current_, false);
      logLongitudinalMode("deadman_brake", deadman_brake_current_);
    } else {
      publishSpeed(0.0);
      logLongitudinalMode("deadman_zero_speed", 0.0);
    }
    return;
  }

  if (!commandFresh()) {
    braking_active_ = false;
    publishBrake(command_timeout_brake_current_, false);
    logLongitudinalMode("command_timeout_brake", command_timeout_brake_current_);
    return;
  }

  applyLongitudinalCommand();
}

bool AckermannToVesc::deadmanActive() const
{
  if (autonomy_override_enabled_ && have_autonomy_override_ && autonomy_override_active_) {
    if (
      autonomy_override_timeout_.nanoseconds() <= 0 ||
      (now() - last_autonomy_override_time_) <= autonomy_override_timeout_)
    {
      return true;
    }
  }

  if (!have_joy_) {
    return false;
  }

  if (deadman_timeout_.nanoseconds() > 0) {
    const auto joy_age = now() - last_joy_time_;
    if (joy_age > deadman_timeout_) {
      return false;
    }
  }

  return deadman_pressed_;
}

bool AckermannToVesc::activeBrakingAllowed() const
{
  if (!active_braking_enabled_) {
    return false;
  }

  if (!have_runtime_active_braking_) {
    return runtime_active_braking_default_enabled_;
  }

  if (
    active_braking_enable_timeout_.nanoseconds() > 0 &&
    (now() - last_active_braking_enable_time_) > active_braking_enable_timeout_)
  {
    return runtime_active_braking_default_enabled_;
  }

  return runtime_active_braking_enabled_;
}

bool AckermannToVesc::commandFresh() const
{
  if (!have_command_) {
    return false;
  }
  return command_timeout_.nanoseconds() <= 0 || (now() - last_command_time_) <= command_timeout_;
}

bool AckermannToVesc::vescStateFresh() const
{
  if (!have_vesc_state_) {
    return false;
  }
  return vesc_state_timeout_.nanoseconds() <= 0 ||
         (now() - last_vesc_state_time_) <= vesc_state_timeout_;
}

void AckermannToVesc::applyLongitudinalCommand()
{
  const double target_speed = static_cast<double>(last_command_.drive.speed);
  if (!activeBrakingAllowed()) {
    braking_active_ = false;
    publishSpeed(target_speed);
    logLongitudinalMode("speed", target_speed);
    return;
  }

  if (!vescStateFresh()) {
    braking_active_ = false;
    publishBrake(command_timeout_brake_current_, false);
    logLongitudinalMode("vesc_state_timeout_brake", command_timeout_brake_current_);
    return;
  }

  const double measured_speed_abs = std::abs(measured_speed_mps_);
  const double target_speed_abs = std::abs(target_speed);
  const bool direction_reversal =
    measured_speed_abs > brake_min_vehicle_speed_mps_ &&
    target_speed_abs > brake_min_vehicle_speed_mps_ &&
    measured_speed_mps_ * target_speed < 0.0;
  const double speed_error = measured_speed_abs - target_speed_abs;

  if (braking_active_) {
    const bool reached_target =
      !direction_reversal && speed_error <= brake_release_speed_error_mps_;
    const bool nearly_stopped = measured_speed_abs <= brake_min_vehicle_speed_mps_;
    if (reached_target || nearly_stopped) {
      braking_active_ = false;
    }
  } else {
    if (
      direction_reversal ||
      (measured_speed_abs > brake_min_vehicle_speed_mps_ &&
      speed_error > brake_enter_speed_error_mps_))
    {
      braking_active_ = true;
    }
  }

  if (!braking_active_) {
    last_brake_current_ = 0.0;
    publishSpeed(target_speed);
    logLongitudinalMode("speed", target_speed);
    return;
  }

  const double requested_decel =
    std::max(
    default_decel_mps2_,
    std::abs(static_cast<double>(last_command_.drive.acceleration)));
  const double brake_current =
    computeActiveBrakeCurrent(requested_decel, std::max(0.0, speed_error));
  publishBrake(brake_current, true);
  logLongitudinalMode("active_brake", brake_current);
}

void AckermannToVesc::publishSpeed(double speed_mps)
{
  if (!rclcpp::ok()) {
    return;
  }
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * speed_mps + speed_to_erpm_offset_;
  erpm_pub_->publish(erpm_msg);
  last_brake_current_ = 0.0;
  last_brake_update_time_ = now();
}

void AckermannToVesc::publishBrake(double brake_current, bool apply_slew_limit)
{
  if (!rclcpp::ok()) {
    return;
  }

  double command = std::max(0.0, brake_current);
  const auto current_time = now();
  if (apply_slew_limit && brake_current_slew_rate_aps_ > 0.0 &&
    last_brake_update_time_.nanoseconds() > 0)
  {
    const double dt = std::max(0.0, (current_time - last_brake_update_time_).seconds());
    const double max_delta = brake_current_slew_rate_aps_ * dt;
    command = std::min(command, last_brake_current_ + max_delta);
  }

  Float64 brake_msg;
  brake_msg.data = command;
  brake_pub_->publish(brake_msg);
  last_brake_current_ = command;
  last_brake_update_time_ = current_time;
}

double AckermannToVesc::computeActiveBrakeCurrent(
  double requested_decel, double speed_error) const
{
  const double raw_current =
    brake_current_per_decel_ * requested_decel +
    brake_current_per_speed_error_ * speed_error;
  return std::min(
    active_brake_max_current_,
    std::max(active_brake_min_current_, raw_current));
}

void AckermannToVesc::logLongitudinalMode(const std::string & mode, double value)
{
  if (mode == last_longitudinal_mode_) {
    return;
  }
  last_longitudinal_mode_ = mode;
  RCLCPP_INFO(
    get_logger(),
    "Longitudinal mode=%s value=%.2f target=%.2fm/s measured=%.2fm/s",
    mode.c_str(), value, static_cast<double>(last_command_.drive.speed), measured_speed_mps_);
}

void AckermannToVesc::logDeadmanState(bool active)
{
  if (deadman_state_logged_ && deadman_last_active_ == active) {
    return;
  }

  deadman_state_logged_ = true;
  deadman_last_active_ = active;
  if (active) {
    if (autonomy_override_enabled_ && have_autonomy_override_ && autonomy_override_active_) {
      RCLCPP_INFO(
        get_logger(),
        "Autonomy override active via %s; longitudinal commands enabled",
        autonomy_override_topic_.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "Deadman active on buttons [%d, %d]; longitudinal commands enabled",
        deadman_button_index_,
        deadman_button_index_alt_);
    }
    return;
  }

  RCLCPP_WARN(
    get_logger(),
    "Deadman inactive; publishing active brake current %.1f",
    deadman_brake_current_);
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
