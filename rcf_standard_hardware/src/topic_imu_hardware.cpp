/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "rcf_standard_hardware/topic_imu_hardware.hpp"

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"

namespace rcf_standard_hardware {

TopicImuHardware::~TopicImuHardware() { stop_subscriber(); }

hardware_interface::CallbackReturn TopicImuHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TopicImuHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  if (hardware_interface::SensorInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.empty()) {
    RCLCPP_ERROR(get_logger(), "No sensor defined in ros2_control tag.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  imu_data_.name = info_.sensors[0].name;
  imu_data_.orientation[0] = 0.0;
  imu_data_.orientation[1] = 0.0;
  imu_data_.orientation[2] = 0.0;
  imu_data_.orientation[3] = 1.0;
  imu_data_.angular_vel[0] = 0.0;
  imu_data_.angular_vel[1] = 0.0;
  imu_data_.angular_vel[2] = 0.0;
  imu_data_.linear_accel[0] = 0.0;
  imu_data_.linear_accel[1] = 0.0;
  imu_data_.linear_accel[2] = 0.0;
  imu_data_.stamp = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  imu_data_.online = false;

  try {
    if (info_.hardware_parameters.count("imu_topic")) {
      imu_topic_ = info_.hardware_parameters.at("imu_topic");
    }
    if (info_.hardware_parameters.count("timeout_threshold")) {
      timeout_threshold_ = std::stod(info_.hardware_parameters.at("timeout_threshold"));
    }
    if (info_.hardware_parameters.count("check_timeout")) {
      const auto& value = info_.hardware_parameters.at("check_timeout");
      check_timeout_ = (value == "true" || value == "True" || value == "1");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "IMU sensor name: %s, topic: %s, timeout_threshold: %.3f, check_timeout: %s",
              imu_data_.name.c_str(), imu_topic_.c_str(), timeout_threshold_, check_timeout_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicImuHardware::start_subscriber() {
  stop_subscriber();

  imu_received_.store(false);
  imu_msg_seq_.store(0);
  last_read_seq_ = 0;

  imu_node_ = std::make_shared<rclcpp::Node>("topic_imu_subscriber");
  // 与 robot_sensors 发布端 QoS(10)/Reliable 对齐，避免 SensorDataQoS(BestEffort) 偶发匹配问题
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  imu_sub_ = imu_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, std::bind(&TopicImuHardware::imu_callback, this, std::placeholders::_1));

  imu_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  imu_executor_->add_node(imu_node_);
  imu_spinner_running_.store(true);
  imu_spinner_thread_ = std::thread([this]() {
    while (rclcpp::ok() && imu_spinner_running_.load()) {
      imu_executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  RCLCPP_INFO(get_logger(), "Subscribed to IMU topic: %s (dedicated spinner thread)", imu_topic_.c_str());
}

void TopicImuHardware::stop_subscriber() {
  imu_spinner_running_.store(false);
  if (imu_executor_) {
    imu_executor_->cancel();
  }
  if (imu_spinner_thread_.joinable()) {
    imu_spinner_thread_.join();
  }
  imu_sub_.reset();
  if (imu_executor_ && imu_node_) {
    imu_executor_->remove_node(imu_node_);
  }
  imu_executor_.reset();
  imu_node_.reset();
}

hardware_interface::CallbackReturn TopicImuHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  start_subscriber();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicImuHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_subscriber();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TopicImuHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "orientation.x", &imu_data_.orientation[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "orientation.y", &imu_data_.orientation[1]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "orientation.z", &imu_data_.orientation[2]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "orientation.w", &imu_data_.orientation[3]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "angular_velocity.x", &imu_data_.angular_vel[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "angular_velocity.y", &imu_data_.angular_vel[1]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "angular_velocity.z", &imu_data_.angular_vel[2]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "linear_acceleration.x", &imu_data_.linear_accel[0]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "linear_acceleration.y", &imu_data_.linear_accel[1]));
  state_interfaces.emplace_back(
      hardware_interface::StateInterface(imu_data_.name, "linear_acceleration.z", &imu_data_.linear_accel[2]));
  return state_interfaces;
}

hardware_interface::CallbackReturn TopicImuHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "TopicImuHardware activating...");
  hw_init_check_success_ = hardware_online_check();
  if (!hw_init_check_success_) {
    RCLCPP_ERROR(get_logger(), "IMU online check failed.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "TopicImuHardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TopicImuHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "TopicImuHardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicImuHardware::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  imu_msg_buffer_.writeFromNonRT(*msg);
  imu_received_.store(true);
  imu_msg_seq_.fetch_add(1, std::memory_order_release);
}

hardware_interface::return_type TopicImuHardware::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  const uint64_t current_seq = imu_msg_seq_.load(std::memory_order_acquire);
  if (current_seq != last_read_seq_) {
    const auto* msg = imu_msg_buffer_.readFromRT();
    if (msg != nullptr) {
      imu_data_.orientation[0] = msg->orientation.x;
      imu_data_.orientation[1] = msg->orientation.y;
      imu_data_.orientation[2] = msg->orientation.z;
      imu_data_.orientation[3] = msg->orientation.w;
      imu_data_.angular_vel[0] = msg->angular_velocity.x;
      imu_data_.angular_vel[1] = msg->angular_velocity.y;
      imu_data_.angular_vel[2] = msg->angular_velocity.z;
      imu_data_.linear_accel[0] = msg->linear_acceleration.x;
      imu_data_.linear_accel[1] = msg->linear_acceleration.y;
      imu_data_.linear_accel[2] = msg->linear_acceleration.z;
      imu_data_.stamp = time;
      imu_data_.seq = current_seq;
      imu_data_.online = true;
      last_read_seq_ = current_seq;
    }
  }

  return hardware_timeout_check(time);
}

bool TopicImuHardware::hardware_online_check() {
  RCLCPP_INFO(get_logger(), "===================================================.");
  RCLCPP_INFO(get_logger(), "开始 IMU 在线自检.");
  hw_init_check_success_ = false;
  imu_data_.online = false;

  auto start_check_time = get_clock()->now();
  rclcpp::Rate check_rate(100);
  // 3秒内收到有效 IMU 数据则认为在线
  while (rclcpp::ok() && (get_clock()->now() - start_check_time).seconds() < 3.0) {
    auto now = get_clock()->now();
    read(now, rclcpp::Duration::from_seconds(0.01));
    if (imu_received_.load() && (now - imu_data_.stamp).seconds() <= timeout_threshold_) {
      imu_data_.online = true;
      hw_init_check_success_ = true;
      break;
    }
    check_rate.sleep();
  }

  if (hw_init_check_success_) {
    RCLCPP_INFO(get_logger(), "IMU 在线自检成功.");
    RCLCPP_INFO(get_logger(), "===================================================.");
    return true;
  }

  RCLCPP_ERROR(get_logger(), "IMU: %s 在 3.0 秒内未收到 topic [%s] 数据, 自检失败.", imu_data_.name.c_str(),
               imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "===================================================.");
  return false;
}

hardware_interface::return_type TopicImuHardware::hardware_timeout_check(const rclcpp::Time& time) {
  if (!hw_init_check_success_ || !check_timeout_) {
    return hardware_interface::return_type::OK;
  }

  const double elapsed = (time - imu_data_.stamp).seconds();
  if (elapsed < timeout_threshold_) {
    imu_data_.online = true;
    return hardware_interface::return_type::OK;
  }

  if (elapsed <= 3.0) {
    if ((time - imu_data_.last_timeout_print_stamp).seconds() > 0.1) {
      RCLCPP_WARN(get_logger(), "IMU %s device timeout %f second.", imu_data_.name.c_str(), elapsed);
      imu_data_.last_timeout_print_stamp = time;
    }
    imu_data_.online = false;
    return hardware_interface::return_type::OK;
  }

  imu_data_.online = false;
  RCLCPP_ERROR(get_logger(), "IMU: %s 超时 %f 秒, IMU 可能离线.", imu_data_.name.c_str(), elapsed);
  return hardware_interface::return_type::ERROR;
}

}  // namespace rcf_standard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rcf_standard_hardware::TopicImuHardware, hardware_interface::SensorInterface)
