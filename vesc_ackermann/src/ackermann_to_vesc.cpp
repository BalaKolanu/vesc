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
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Bool;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  command_timeout_(0, 0),
  vesc_state_timeout_(0, 0),
  active_braking_enabled_(false),
  runtime_active_braking_enabled_(false),
  runtime_active_braking_default_enabled_(false),
  active_braking_enable_timeout_(0, 0),
  have_runtime_active_braking_(false),
  have_command_(false),
  have_vesc_state_(false),
  braking_active_(false),
  measured_speed_mps_(0.0),
  last_brake_current_(0.0),
  last_longitudinal_mode_(""),
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
  command_timeout_brake_current_ =
    std::max(0.0, declare_parameter<double>("command_timeout_brake_current", 0.0));
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
    "VESC command gate uses muxed Ackermann commands; command_timeout=%.2fs state_timeout=%.2fs",
    command_timeout_.seconds(),
    vesc_state_timeout_.seconds());
  RCLCPP_INFO(
    get_logger(),
    "Negative Ackermann acceleration requests brake current on the mux-selected command");
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
  if (!commandFresh()) {
    braking_active_ = false;
    publishSpeed(0.0);
    logLongitudinalMode("command_timeout_zero_speed", 0.0);
    return;
  }

  applyLongitudinalCommand();
}

bool AckermannToVesc::activeBrakingAllowed() const
{
  if (!active_braking_enabled_) {
    return false;
  }
  if (!have_runtime_active_braking_) {
    return runtime_active_braking_default_enabled_;
  }
  const bool runtime_fresh = active_braking_enable_timeout_.nanoseconds() <= 0 ||
    (now() - last_active_braking_enable_time_) <= active_braking_enable_timeout_;
  if (!runtime_fresh) {
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
  const double requested_brake_current =
    std::max(0.0, -static_cast<double>(last_command_.drive.acceleration));
  if (requested_brake_current > 0.0) {
    braking_active_ = true;
    publishBrake(requested_brake_current, false);
    logLongitudinalMode("commanded_brake", requested_brake_current);
    return;
  }

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

  const rclcpp::Time update_time = now();
  double command = std::max(0.0, brake_current);
  if (apply_slew_limit && brake_current_slew_rate_aps_ > 0.0) {
    double dt = 0.0;
    if (last_brake_update_time_.nanoseconds() > 0) {
      dt = std::max(0.0, (update_time - last_brake_update_time_).seconds());
    }
    const double max_delta = brake_current_slew_rate_aps_ * dt;
    if (max_delta > 0.0) {
      command = std::min(command, last_brake_current_ + max_delta);
    }
  }

  Float64 brake_msg;
  brake_msg.data = command;
  brake_pub_->publish(brake_msg);
  last_brake_current_ = command;
  last_brake_update_time_ = now();
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

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
