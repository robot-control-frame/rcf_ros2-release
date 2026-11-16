/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "rcf_joint_limits_interface/rcf_joint_limits.hpp"

#include <cmath>
#include <limits>

namespace rcf_joint_limits_interface {
RcfJointLimits::RcfJointLimits(const joint_limits::JointLimits& limits, double* position_ptr, double* velocity_ptr,
                               double* effort_ptr, double* control_mode_ptr, double* position_command_ptr,
                               double* velocity_command_ptr, double* effort_command_ptr)
    : limits_(limits),
      position_ptr_(position_ptr),
      velocity_ptr_(velocity_ptr),
      effort_ptr_(effort_ptr),
      control_mode_ptr_(control_mode_ptr),
      position_command_ptr_(position_command_ptr),
      velocity_command_ptr_(velocity_command_ptr),
      effort_command_ptr_(effort_command_ptr) {
  if (limits_.has_position_limits) {
    min_pos_limit_ = limits_.min_position;
    max_pos_limit_ = limits_.max_position;
  } else {
    min_pos_limit_ = -std::numeric_limits<double>::max();
    max_pos_limit_ = std::numeric_limits<double>::max();
  }
}

void RcfJointLimits::enforce(double dt) {
  using rcf_hardware_interface::ControlMode;
  using rcf_hardware_interface::is_control_mode_value;

  const bool apply_limits = is_control_mode_value(*control_mode_ptr_, ControlMode::MODE_EFFORT) ||
                            is_control_mode_value(*control_mode_ptr_, ControlMode::MODE_VELOCITY) ||
                            is_control_mode_value(*control_mode_ptr_, ControlMode::MODE_POSITION);
  if (!apply_limits) {
    prev_pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
    prev_vel_cmd_ = std::numeric_limits<double>::quiet_NaN();
    return;
  }

  if (std::isnan(prev_pos_cmd_)) {
    prev_pos_cmd_ = *position_ptr_;
  }
  if (std::isnan(prev_vel_cmd_)) {
    prev_vel_cmd_ = *velocity_ptr_;
  }

  // enforce pos command: hard limits first, then rate-limit toward that target.
  // Applying both in one saturate window can collapse to the hard bound in a single
  // cycle when prev_pos_cmd_ is already outside the position limits.
  double pos_cmd = saturate(*position_command_ptr_, min_pos_limit_, max_pos_limit_);
  if (limits_.has_velocity_limits) {
    const double delta_pos = limits_.max_velocity * dt;
    pos_cmd = saturate(pos_cmd, prev_pos_cmd_ - delta_pos, prev_pos_cmd_ + delta_pos);
  }
  *position_command_ptr_ = pos_cmd;
  prev_pos_cmd_ = pos_cmd;

  // enforce vel command: hard velocity limit first, then acceleration rate-limit.
  double vel_cmd = saturate(*velocity_command_ptr_, -limits_.max_velocity, limits_.max_velocity);
  if (limits_.has_acceleration_limits) {
    const double delta_vel = limits_.max_acceleration * dt;
    vel_cmd = saturate(vel_cmd, prev_vel_cmd_ - delta_vel, prev_vel_cmd_ + delta_vel);
  }
  *velocity_command_ptr_ = vel_cmd;
  prev_vel_cmd_ = vel_cmd;

  // enforce effort command
  double min_eff = -limits_.max_effort;
  double max_eff = limits_.max_effort;

  if (limits_.has_position_limits) {
    const double pos = *position_ptr_;
    if (pos < limits_.min_position)
      min_eff = 0.0;
    else if (pos > limits_.max_position)
      max_eff = 0.0;
  }

  const double vel = *velocity_ptr_;
  if (vel < -limits_.max_velocity)
    min_eff = 0.0;
  else if (vel > limits_.max_velocity)
    max_eff = 0.0;
  *effort_command_ptr_ = saturate(*effort_command_ptr_, min_eff, max_eff);
}

double RcfJointLimits::saturate(const double val, const double min_val, const double max_val) {
  return std::min(std::max(val, min_val), max_val);
}
}  // namespace rcf_joint_limits_interface
