/*******************************************************************************
 * Copyright (c) 2026/04/17, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once
#include <cstdint>
#include <rcf_hardware_interface/control_modes.hpp>
#include <rclcpp/time.hpp>
namespace rcf_standard_driver_interface {

struct ServoState {
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint32_t id{};
  double position{};     // rad or m
  double velocity{};     // rad/s or m/s
  double effort{};       // Nm or N
  double current{};      // A
  double temperature{};  // °C
  uint32_t error_code{};
};

struct ServoCommand {
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint32_t id{};
  rcf_hardware_interface::ControlMode mode{rcf_hardware_interface::ControlMode::BEGIN};
  double position{};  // rad or m
  double velocity{};  // rad/s or m/s
  double effort{};    // Nm or N
  double v1{};
  double v2{};
  double v3{};
};

struct ForceTorque {
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint32_t id{};
  double force[3];   // N
  double torque[3];  // Nm
};

struct Imu {
  rclcpp::Time stamp{0, 0, RCL_STEADY_TIME};
  uint32_t id{};
  double orientation[4];          // quaternion
  double angular_velocity[3];     // rad/s
  double linear_acceleration[3];  // m/s^2
};

}  // namespace rcf_standard_driver_interface
