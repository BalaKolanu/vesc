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

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using sensor_msgs::msg::Joy;
using std_msgs::msg::Float64;

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options),
  deadman_timeout_(0, 0),
  deadman_pressed_(false),
  have_joy_(false),
  deadman_state_logged_(false),
  deadman_last_active_(false),
  last_joy_time_(0, 0, get_clock()->get_clock_type())
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
  deadman_brake_current_ = declare_parameter<double>("deadman_brake_current", 12000.0);
  deadman_timeout_ = rclcpp::Duration::from_seconds(
    std::max(0.0, declare_parameter<double>("deadman_timeout_sec", 0.5)));

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  brake_pub_ = create_publisher<Float64>("commands/motor/brake", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  joy_sub_ = create_subscription<Joy>(
    joy_topic_, 10, std::bind(&AckermannToVesc::joyCallback, this, _1));

  deadman_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&AckermannToVesc::deadmanTimerCallback, this));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  if (!rclcpp::ok()) {
    return;
  }

  if (deadmanActive()) {
    Float64 erpm_msg;
    erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
    logDeadmanState(true);
    return;
  }

  servo_pub_->publish(servo_msg);
  publishBrake();
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
  if (!active) {
    publishBrake();
  }
}

void AckermannToVesc::deadmanTimerCallback()
{
  const bool active = deadmanActive();
  logDeadmanState(active);
  if (!active) {
    publishBrake();
  }
}

bool AckermannToVesc::deadmanActive() const
{
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

void AckermannToVesc::publishBrake()
{
  if (!deadman_active_brake_ || !rclcpp::ok()) {
    return;
  }

  Float64 brake_msg;
  brake_msg.data = deadman_brake_current_;
  brake_pub_->publish(brake_msg);
}

void AckermannToVesc::logDeadmanState(bool active)
{
  if (deadman_state_logged_ && deadman_last_active_ == active) {
    return;
  }

  deadman_state_logged_ = true;
  deadman_last_active_ = active;
  if (active) {
    RCLCPP_INFO(
      get_logger(),
      "Deadman active on buttons [%d, %d]; speed commands enabled",
      deadman_button_index_,
      deadman_button_index_alt_);
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
