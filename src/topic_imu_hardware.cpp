/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "rcf_standard_hardware/topic_imu_hardware.hpp"

#include <algorithm>
#include <cctype>
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
  last_recv_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
  online_state_ = 0.0;
  stamp_sec_ = 0.0;

  try {
    if (info_.hardware_parameters.count("imu_topic")) {
      imu_topic_ = info_.hardware_parameters.at("imu_topic");
    }
    if (info_.hardware_parameters.count("timeout_threshold")) {
      timeout_threshold_ = std::stod(info_.hardware_parameters.at("timeout_threshold"));
    }
    if (info_.hardware_parameters.count("error_timeout")) {
      error_timeout_threshold_ = std::stod(info_.hardware_parameters.at("error_timeout"));
    }
    if (info_.hardware_parameters.count("check_timeout")) {
      const auto& raw = info_.hardware_parameters.at("check_timeout");
      std::string value = raw;
      const auto first = value.find_first_not_of(" \t\r\n");
      const auto last = value.find_last_not_of(" \t\r\n");
      value = (first == std::string::npos) ? std::string() : value.substr(first, last - first + 1);
      std::transform(value.begin(), value.end(), value.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      if (value == "true" || value == "1" || value == "on" || value == "yes") {
        check_timeout_ = true;
      } else if (value == "false" || value == "0" || value == "off" || value == "no") {
        check_timeout_ = false;
      } else {
        RCLCPP_WARN(get_logger(), "Unknown check_timeout value '%s', keep default %s", raw.c_str(),
                    check_timeout_ ? "true" : "false");
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (error_timeout_threshold_ < timeout_threshold_) {
    RCLCPP_WARN(get_logger(),
                "error_timeout(%.3f) < timeout_threshold(%.3f), raise error_timeout to timeout_threshold.",
                error_timeout_threshold_, timeout_threshold_);
    error_timeout_threshold_ = timeout_threshold_;
  }

  RCLCPP_INFO(get_logger(),
              "IMU sensor name: %s, topic: %s, timeout_threshold: %.3f, error_timeout: %.3f, check_timeout: %s",
              imu_data_.name.c_str(), imu_topic_.c_str(), timeout_threshold_, error_timeout_threshold_,
              check_timeout_ ? "true" : "false");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicImuHardware::start_subscriber() {
  stop_subscriber();

  imu_received_.store(false);
  last_recv_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);

  imu_node_ = std::make_shared<rclcpp::Node>("topic_imu_subscriber");
  // 与 robot_sensors 发布端 QoS(10)/Reliable 对齐，避免 SensorDataQoS(BestEffort) 偶发匹配问题
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  imu_sub_ = imu_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos, std::bind(&TopicImuHardware::imu_callback, this, std::placeholders::_1));

  imu_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  imu_executor_->add_node(imu_node_);
  imu_spinner_running_.store(true);
  imu_spinner_thread_ = std::thread([this]() {
    // 事件驱动：spin() 阻塞等待；stop_subscriber()/on_error 里 cancel() 唤醒后返回。
    while (rclcpp::ok() && imu_spinner_running_.load()) {
      imu_executor_->spin();
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
  // 诊断状态：让上层能观测到“是否在线/数据时间戳/序号”，而不是只有内部 online 标志。
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_data_.name, "online", &online_state_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(imu_data_.name, "stamp", &stamp_sec_));
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

hardware_interface::CallbackReturn TopicImuHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/) {
  // on_error 可能在 RT 循环线程中被调用：这里只做非阻塞操作，不 join 线程。
  imu_spinner_running_.store(false);
  if (imu_executor_) {
    imu_executor_->cancel();  // 唤醒 spin()，让其尽快返回
  }
  imu_data_.online = false;
  hw_init_check_success_ = false;
  RCLCPP_WARN(get_logger(), "TopicImuHardware on_error: IMU marked offline, subscriber cancelled (recoverable).");
  // SUCCESS → UNCONFIGURED（可恢复）；FAILURE/ERROR → FINALIZED（不可恢复）。
  return hardware_interface::CallbackReturn::SUCCESS;
}

void TopicImuHardware::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  ImuSample sample;
  sample.msg = *msg;
  // 在回调里给数据打时间戳：用稳定时钟（STEADY），不受 /clock 跳变影响。
  sample.stamp = get_clock()->now();
  imu_msg_buffer_.writeFromNonRT(sample);
  imu_received_.store(true);
}

hardware_interface::return_type TopicImuHardware::read(const rclcpp::Time& time, const rclcpp::Duration& /*period*/) {
  if (imu_received_.load(std::memory_order_acquire)) {
    // RealtimeBuffer::readFromRT() 恒返回有效指针（内部双缓冲），无需判空。
    const auto* sample = imu_msg_buffer_.readFromRT();
    const auto& msg = sample->msg;
    imu_data_.orientation[0] = msg.orientation.x;
    imu_data_.orientation[1] = msg.orientation.y;
    imu_data_.orientation[2] = msg.orientation.z;
    imu_data_.orientation[3] = msg.orientation.w;
    imu_data_.angular_vel[0] = msg.angular_velocity.x;
    imu_data_.angular_vel[1] = msg.angular_velocity.y;
    imu_data_.angular_vel[2] = msg.angular_velocity.z;
    imu_data_.linear_accel[0] = msg.linear_acceleration.x;
    imu_data_.linear_accel[1] = msg.linear_acceleration.y;
    imu_data_.linear_accel[2] = msg.linear_acceleration.z;
    // 时间戳在回调里用稳定时钟打好；sample->stamp 只在新消息到来时变化，故无需 seq 计数。
    imu_data_.stamp = sample->stamp;
    last_recv_time_ = sample->stamp;
    imu_data_.online = true;
  }

  const auto ret = hardware_timeout_check(time);
  online_state_ = imu_data_.online ? 1.0 : 0.0;
  stamp_sec_ = imu_data_.stamp.seconds();
  return ret;
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
    if (imu_received_.load() && (now - last_recv_time_).seconds() <= timeout_threshold_) {
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

  // 断流时长：距最近一条消息的接收时刻（time 与 last_recv_time_ 同为 STEADY 稳定时钟，不会跳变）。
  const double elapsed = (time - last_recv_time_).seconds();

  if (elapsed < timeout_threshold_) {
    imu_data_.online = true;
    return hardware_interface::return_type::OK;
  }

  if (elapsed <= error_timeout_threshold_) {
    // 每 0.1s 至多一条（稳定时钟计时，不受 /clock 跳变影响）。
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100, "IMU %s device timeout %f second (threshold %f).",
                         imu_data_.name.c_str(), elapsed, timeout_threshold_);
    imu_data_.online = false;
    return hardware_interface::return_type::OK;
  }

  imu_data_.online = false;
  // 即使上层迟迟不消费 ERROR，也最多每 0.1s 一条。
  RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 100, "IMU: %s 超时 %f 秒 (error threshold %f), IMU 可能离线.",
                        imu_data_.name.c_str(), elapsed, error_timeout_threshold_);
  return hardware_interface::return_type::ERROR;
}

}  // namespace rcf_standard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rcf_standard_hardware::TopicImuHardware, hardware_interface::SensorInterface)
