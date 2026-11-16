/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rcf_standard_hardware/type.hpp"

// ros
#include "rclcpp/clock.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

// ros2_control
#include "hardware_interface/handle.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace rcf_standard_hardware {
class TopicImuHardware : public hardware_interface::SensorInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TopicImuHardware)

  TopicImuHardware() = default;
  ~TopicImuHardware() override;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  /// Get the logger of the SensorInterface.
  rclcpp::Logger get_logger() const override { return *logger_; }

  /// Get the clock of the SensorInterface.
  rclcpp::Clock::SharedPtr get_clock() const override { return clock_; }

 private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  bool hardware_online_check();
  hardware_interface::return_type hardware_timeout_check(const rclcpp::Time& time);
  void start_subscriber();
  void stop_subscriber();

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  std::string imu_topic_{"/imu"};
  double timeout_threshold_{0.1};
  bool check_timeout_{false};
  bool hw_init_check_success_{false};

  ImuData imu_data_;
  realtime_tools::RealtimeBuffer<sensor_msgs::msg::Imu> imu_msg_buffer_;
  std::atomic<bool> imu_received_{false};
  std::atomic<uint64_t> imu_msg_seq_{0};
  uint64_t last_read_seq_{0};

  // 独立 node + spinner，避免依赖 controller_manager executor（RT 调度下回调可能饿死）
  rclcpp::Node::SharedPtr imu_node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> imu_executor_;
  std::thread imu_spinner_thread_;
  std::atomic<bool> imu_spinner_running_{false};
};

}  // namespace rcf_standard_hardware
