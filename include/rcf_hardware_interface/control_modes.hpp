/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

namespace rcf_hardware_interface {

enum class ControlMode : std::int8_t {
  BEGIN = -1,
  MODE_EFFORT = 0,
  MODE_VELOCITY = 1,
  MODE_POSITION = 2,
  MODE_PVT = 3,
  NOMODE = 4,
  EMERGENCY_STOP = 5,
  SWITCHING = 6,
  ERROR = 7,
  CUSTOM_MODE_1 = 8,
  CUSTOM_MODE_2 = 9,
  CUSTOM_MODE_3 = 10,
  UNKNOWN = 11,
};

constexpr double to_interface_value(ControlMode mode) { return static_cast<double>(static_cast<std::int8_t>(mode)); }

constexpr bool is_control_mode_value(double value, ControlMode mode) { return value == to_interface_value(mode); }

// ControlMode 的值域连续：BEGIN=-1 … UNKNOWN=11。
// 编解码直接依赖这个连续性，因此新增模式必须加在 UNKNOWN 之前，或同步更新 kMax。
inline ControlMode to_control_mode(double value) {
  // 非整数（如 2.5）或非有限值（nan/inf）不是合法的模式值，直接判为 UNKNOWN。
  if (!std::isfinite(value) || value != std::trunc(value)) {
    return ControlMode::UNKNOWN;
  }
  // 先在 double 上比较值域，避免 1e30 这种超出整数范围的极大值被转成整数（UB）。
  constexpr double kMin = static_cast<std::int8_t>(ControlMode::BEGIN);
  constexpr double kMax = static_cast<std::int8_t>(ControlMode::UNKNOWN);
  if (value < kMin || value > kMax) {
    return ControlMode::UNKNOWN;
  }
  return static_cast<ControlMode>(static_cast<std::int8_t>(value));
}

}  // namespace rcf_hardware_interface
