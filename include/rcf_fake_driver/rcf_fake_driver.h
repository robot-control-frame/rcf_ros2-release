/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#pragma once

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <mutex>
#include <rcf_standard_driver_interface/rcf_standard_driver_base.hpp>
#include <rcf_standard_driver_interface/type.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <unordered_map>
namespace rcf_fake_driver {
class RcfFakeDriver : public rcf_standard_driver_interface::RcfStandardDriverBase {
 public:
  RcfFakeDriver() = default;
  ~RcfFakeDriver();
  virtual bool init(std::vector<std::string> config) override;
  virtual bool starting() override;
  virtual bool stopping() override;

  virtual bool servo_enable(std::vector<uint16_t> servos_id) override;
  virtual bool servo_disable(std::vector<uint16_t> servos_id) override;

  virtual std::vector<rcf_standard_driver_interface::ServoState> get_servo_state() override;
  virtual void set_servo_command(std::vector<rcf_standard_driver_interface::ServoCommand> commands) override;

  virtual std::vector<rcf_standard_driver_interface::ForceTorque> get_ft_state() override;
  virtual std::vector<rcf_standard_driver_interface::Imu> get_imu_state() override;

 private:
  void update();

  std::thread rt_drive_thread_;
  std::atomic<bool> loop_running_ = false;
  std::atomic<bool> driver_exit_ = false;
  std::unordered_map<uint16_t, rcf_standard_driver_interface::ServoCommand> id2servo_commands_;
  std::unordered_map<uint16_t, rcf_standard_driver_interface::ServoState> id2servo_states_;

  std::vector<rcf_standard_driver_interface::ForceTorque> ft_states_;
  std::vector<rcf_standard_driver_interface::Imu> imu_states_;

  std::mutex servo_command_mutex_;
  std::mutex servo_state_mutex_;
  std::mutex ft_state_mutex_;
  std::mutex imu_state_mutex_;

  std::thread spin_thread_;

  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace rcf_fake_driver
