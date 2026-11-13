/*******************************************************************************
 * Copyright (c) 2026/04/17, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "type.hpp"
namespace rcf_standard_driver_interface {

class RcfStandardDriverBase {
 public:
  virtual ~RcfStandardDriverBase() = default;

  /**
   * @brief 初始化驱动，加载配置
   * @param config 配置参数列表
   * @return 初始化是否成功
   */
  virtual bool init(std::vector<std::string> config) { return true; }
  /**
   * @brief 启动驱动
   * @return 启动是否成功
   */
  virtual bool starting() { return true; }
  /**
   * @brief 停止驱动
   * @return 停止是否成功
   */
  virtual bool stopping() { return true; }
  /**
   * @brief 使能指定舵机,舵机在使能之前,不会响应控制命令，但是会上报状态
   * @param servos_id 舵机ID列表
   * @return 使能是否成功
   */
  virtual bool servo_enable(std::vector<uint16_t> servos_id) = 0;

  /**
   * @brief 禁用指定舵机,舵机在禁用之后,不会响应控制命令，也不会上报状态
   * @param servos_id 舵机ID列表
   * @return 禁用是否成功
   */
  virtual bool servo_disable(std::vector<uint16_t> servos_id) { return true; }

  /**
   * @brief 获取所有舵机状态
   * @return 舵机状态列表
   */
  virtual std::vector<ServoState> get_servo_state() = 0;
  /**
   * @brief 设置舵机控制命令
   * @param commands 舵机命令列表
   */
  virtual void set_servo_command(std::vector<ServoCommand> commands) = 0;
  /**
   * @brief 获取力/力矩传感器状态
   * @return 力/力矩状态列表
   */
  virtual std::vector<ForceTorque> get_ft_state() = 0;
  /**
   * @brief 获取IMU状态
   * @return IMU状态列表
   */
  virtual std::vector<Imu> get_imu_state() = 0;

  rclcpp::Logger get_logger() const { return *logger_; }
  rclcpp::Clock get_clock() const { return *clock_; }
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;
};

typedef std::shared_ptr<RcfStandardDriverBase> RcfStandardDriverBaseSharedPtr;

}  // namespace rcf_standard_driver_interface
