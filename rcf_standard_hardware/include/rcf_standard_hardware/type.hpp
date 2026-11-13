/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#pragma once
#include <cstdint>
#include <rcf_common/filters/lp_filter.hpp>
#include <rclcpp/time.hpp>
namespace rcf_standard_hardware {

enum ActType {
  UNKNOWN = 0,
  STANDARD_PRISMATIC = 1,
  STANDARD_REVOLUTE = 2,  // [-pi, pi]
};

struct ActData {
  std::string name{};
  std::string type{};  // 工作方式，旋转还是直线
  uint16_t id{};
  rclcpp::Time status_stamp{0, 0, RCL_STEADY_TIME};
  double q_circle{0.0};
  double pos_raw{}, pos_raw_last{};
  double pos{}, vel{}, effort{};
  double cur{};
  rclcpp::Time cmd_stamp{0, 0, RCL_STEADY_TIME};
  double ctrl_mode{0.0};
  double last_ctrl_mode{0.0};
  double cmd_pos{}, cmd_vel{}, cmd_effort{};
  double cmd_v1{}, cmd_v2{}, cmd_v3{};
  double cmd_pos_unlimit{}, cmd_vel_unlimit{}, cmd_effort_unlimit{}, cmd_pos_keep;
  bool use_vel_filter{false};
  std::shared_ptr<rcf_common::filters::LowPassFilter> vel_filter{nullptr};
  double temperature{};
  rclcpp::Time last_printf_jump_command_stamp{0, 0, RCL_STEADY_TIME};
  double jump_cmd_threshold{std::numeric_limits<double>::max()};
  bool allow_homing{true};
  ActType act_type{ActType::UNKNOWN};  // 编码模式
  uint16_t status;
  rclcpp::Time last_timeout_print_stamp{0, 0, RCL_STEADY_TIME};
  bool online{false};
};

struct JointData {
  std::string name;
  double position;
  double velocity;
  double effort;
  double control_mode{0.0};
  double position_command;
  double velocity_command;
  double effort_command;
  double v1;
  double v2;
  double v3;
};

struct ImuData {
  std::string name;
  uint16_t id;
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint64_t seq;
  double orientation[4], angular_vel[3], linear_accel[3];
  double orientation_covariance[3], angular_velocity_covariance[3], linear_acceleration_covariance[3];
  rclcpp::Time last_timeout_print_stamp{0, 0, RCL_STEADY_TIME};
  bool online{false};
};

struct ForceTorqueData {
  std::string name;
  uint16_t id;
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint64_t seq;
  double force[3], torque[3];
  rclcpp::Time last_timeout_print_stamp{0, 0, RCL_STEADY_TIME};
  bool online{false};
};

}  // namespace rcf_standard_hardware
