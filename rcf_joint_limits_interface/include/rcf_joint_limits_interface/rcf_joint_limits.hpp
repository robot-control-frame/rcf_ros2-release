/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#pragma once

#include <cassert>

#include "joint_limits/joint_limits.hpp"
#include "rcf_hardware_interface/control_modes.hpp"
namespace rcf_joint_limits_interface {

class RcfJointLimits {
 public:
  explicit RcfJointLimits(const joint_limits::JointLimits& limits, double* position_ptr, double* velocity_ptr,
                          double* effort_ptr, double* control_mode_ptr, double* position_command_ptr,
                          double* velocity_command_ptr, double* effort_command_ptr);

  // dt: sample period in seconds (prefer theoretical control period)
  void enforce(double dt);

 private:
  double saturate(const double val, const double min_val, const double max_val);
  std::string joint_name_;
  joint_limits::JointLimits limits_;
  double* position_ptr_ = nullptr;
  double* velocity_ptr_ = nullptr;
  double* effort_ptr_ = nullptr;
  double* control_mode_ptr_ = nullptr;
  double* position_command_ptr_ = nullptr;
  double* velocity_command_ptr_ = nullptr;
  double* effort_command_ptr_ = nullptr;

  double min_pos_limit_, max_pos_limit_;
  double prev_pos_cmd_ = {std::numeric_limits<double>::quiet_NaN()};
  double prev_vel_cmd_ = {std::numeric_limits<double>::quiet_NaN()};
};

}  // namespace rcf_joint_limits_interface
