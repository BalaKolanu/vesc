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

#ifndef VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
#define VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_

#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std_msgs::msg::Bool;
using std_msgs::msg::Float64;
using vesc_msgs::msg::VescStateStamped;

class AckermannToVesc : public rclcpp::Node
{
public:
  explicit AckermannToVesc(const rclcpp::NodeOptions & options);

private:
  // ROS parameters
  // conversion gain and offset
  double speed_to_erpm_gain_, speed_to_erpm_offset_;
  double steering_to_servo_gain_, steering_to_servo_offset_;
  double command_timeout_brake_current_;
  double active_brake_min_current_;
  double active_brake_max_current_;
  double brake_current_per_decel_;
  double brake_current_per_speed_error_;
  double default_decel_mps2_;
  double brake_enter_speed_error_mps_;
  double brake_release_speed_error_mps_;
  double brake_min_vehicle_speed_mps_;
  double brake_current_slew_rate_aps_;
  rclcpp::Duration command_timeout_;
  rclcpp::Duration vesc_state_timeout_;
  bool active_braking_enabled_;
  bool runtime_active_braking_enabled_;
  bool runtime_active_braking_default_enabled_;
  std::string vesc_state_topic_;
  std::string active_braking_enable_topic_;
  rclcpp::Duration active_braking_enable_timeout_;

  /** @todo consider also providing an interpolated look-up table conversion */

  // ROS services
  rclcpp::Publisher<Float64>::SharedPtr erpm_pub_;
  rclcpp::Publisher<Float64>::SharedPtr brake_pub_;
  rclcpp::Publisher<Float64>::SharedPtr servo_pub_;
  rclcpp::Subscription<AckermannDriveStamped>::SharedPtr ackermann_sub_;
  rclcpp::Subscription<Bool>::SharedPtr active_braking_enable_sub_;
  rclcpp::Subscription<VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::TimerBase::SharedPtr longitudinal_timer_;

  bool have_runtime_active_braking_;
  bool have_command_;
  bool have_vesc_state_;
  bool braking_active_;
  double measured_speed_mps_;
  double last_brake_current_;
  std::string last_longitudinal_mode_;
  AckermannDriveStamped last_command_;
  rclcpp::Time last_active_braking_enable_time_;
  rclcpp::Time last_command_time_;
  rclcpp::Time last_vesc_state_time_;
  rclcpp::Time last_brake_update_time_;

  // ROS callbacks
  void ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd);
  void activeBrakingEnableCallback(const Bool::SharedPtr msg);
  void vescStateCallback(const VescStateStamped::SharedPtr state);
  void longitudinalTimerCallback();
  bool activeBrakingAllowed() const;
  bool commandFresh() const;
  bool vescStateFresh() const;
  void applyLongitudinalCommand();
  void publishSpeed(double speed_mps);
  void publishBrake(double brake_current, bool apply_slew_limit);
  double computeActiveBrakeCurrent(double requested_decel, double speed_error) const;
  void logLongitudinalMode(const std::string & mode, double value);
};

}  // namespace vesc_ackermann

#endif  // VESC_ACKERMANN__ACKERMANN_TO_VESC_HPP_
