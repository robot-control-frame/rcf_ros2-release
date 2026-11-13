/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include <rcf_fake_driver/rcf_fake_driver.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace rcf_fake_driver {
bool RcfFakeDriver::init(std::vector<std::string> config) {
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("RcfFakeDriver"));
  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  RCLCPP_INFO(get_logger(), "RcfFakeDriver init");

  if (config.size() < 2) {
    RCLCPP_ERROR(get_logger(), "RcfFakeDriver expects config: [package_name, relative_config_dir]");
    return false;
  }

  try {
    const std::string config_package_path = ament_index_cpp::get_package_share_directory(config[0]);
    const std::string hw_config_path = config_package_path + "/" + config[1] + "/rcf_std_hardware.yaml";
    const std::string sim_driver_config_path = config_package_path + "/" + config[1] + "/sim_driver.yaml";
    RCLCPP_INFO(get_logger(), "hw_config_path: %s", hw_config_path.c_str());
    RCLCPP_INFO(get_logger(), "sim_driver_config_path: %s", sim_driver_config_path.c_str());

    YAML::Node hw_config = YAML::LoadFile(hw_config_path);
    auto actuators_config = hw_config["rcf_std_hardware"]["actuators"];
    for (auto motor : actuators_config) {
      std::string name = motor.first.as<std::string>();
      YAML::Node motor_config = motor.second;
      uint16_t id = motor_config["id"].as<uint16_t>();
      rcf_standard_driver_interface::ServoState servo_state;
      servo_state.id = id;
      id2servo_states_.insert({id, servo_state});

      rcf_standard_driver_interface::ServoCommand servo_command;
      servo_command.id = id;
      id2servo_commands_.insert({id, servo_command});
    }

    if (hw_config["rcf_std_hardware"]["imus"]) {
      auto imus_config = hw_config["rcf_std_hardware"]["imus"];
      for (auto imu : imus_config) {
        YAML::Node imu_config = imu.second;
        uint16_t id = imu_config["id"].as<uint16_t>();
        rcf_standard_driver_interface::Imu imu_state;
        imu_state.id = id;
        imu_states_.emplace_back(imu_state);
      }
    }

    if (hw_config["rcf_std_hardware"]["force_torque_sensors"]) {
      auto fts_config = hw_config["rcf_std_hardware"]["force_torque_sensors"];
      for (auto ft : fts_config) {
        YAML::Node ft_config = ft.second;
        uint16_t id = ft_config["id"].as<uint16_t>();
        rcf_standard_driver_interface::ForceTorque ft_state;
        ft_state.id = id;
        ft_states_.emplace_back(ft_state);
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to load config: %s", e.what());
    return false;
  }
  // ===================================================================================

  rt_drive_thread_ = std::thread([&]() {
    std::chrono::steady_clock::time_point next_iteration_time{std::chrono::steady_clock::now()};
    unsigned int update_rate = 1000;
    auto const period = std::chrono::nanoseconds(1'000'000'000 / update_rate);
    // todo: 设置线程优先级
    while (rclcpp::ok() && !driver_exit_.load()) {
      if (loop_running_.load()) {
        update();
        next_iteration_time += period;
        std::this_thread::sleep_until(next_iteration_time);
      }
    }
  });

  return true;
}

bool RcfFakeDriver::starting() {
  RCLCPP_INFO(get_logger(), "RcfFakeDriver starting.");
  loop_running_.store(true);
  return true;
}

bool RcfFakeDriver::stopping() {
  loop_running_.store(false);
  RCLCPP_INFO(get_logger(), "RcfFakeDriver stopping.");
  return true;
}

void RcfFakeDriver::update() {
  // receive hardware state from can or else bus.
  {
    // 获取舵机状态
    std::lock_guard<std::mutex> state_lock(servo_state_mutex_);
    std::lock_guard<std::mutex> cmd_lock(servo_command_mutex_);
    // 理想响应
    for (auto& pair : id2servo_commands_) {
      id2servo_states_[pair.first].stamp = get_clock().now();
      id2servo_states_[pair.first].position = pair.second.position;
      id2servo_states_[pair.first].velocity = pair.second.velocity;
      id2servo_states_[pair.first].effort = pair.second.effort;
    }
  }

  {
    // 获取IMU状态
    std::lock_guard<std::mutex> lock(imu_state_mutex_);
    for (auto& state : imu_states_) {
      // 这里构建假数据，要根据实际情况修改
      state.stamp = get_clock().now();
      state.orientation[3] = 1.0;
      state.angular_velocity[0] = 0.1 * sin(get_clock().now().seconds());
      state.linear_acceleration[0] = 0.1 * sin(get_clock().now().seconds());
    }
  }

  {
    // 获取六维力状态
    std::lock_guard<std::mutex> lock(ft_state_mutex_);
    for (auto& state : ft_states_) {
      state.stamp = get_clock().now();
      state.force[0] = 0.1 * sin(get_clock().now().seconds());
      state.torque[0] = 0.1 * sin(get_clock().now().seconds());
    }
  }

  return;
}

bool RcfFakeDriver::servo_enable(std::vector<uint16_t> servos_id) {
  // todo: 发送舵机使能指令指令
  return true;
}
bool RcfFakeDriver::servo_disable(std::vector<uint16_t> servos_id) {
  // todo: 发送舵机标零指令
  return true;
}
std::vector<rcf_standard_driver_interface::ServoState> RcfFakeDriver::get_servo_state() {
  std::lock_guard<std::mutex> lock(servo_state_mutex_);

  std::vector<rcf_standard_driver_interface::ServoState> servo_states_;
  for (const auto& pair : id2servo_states_) {
    servo_states_.push_back(pair.second);
  }
  return servo_states_;
}

void RcfFakeDriver::set_servo_command(std::vector<rcf_standard_driver_interface::ServoCommand> commands) {
  std::lock_guard<std::mutex> lock(servo_command_mutex_);
  for (auto& item : commands) {
    if (id2servo_commands_.find(item.id) != id2servo_commands_.end()) {
      id2servo_commands_[item.id] = item;
    }
  }
}

std::vector<rcf_standard_driver_interface::ForceTorque> RcfFakeDriver::get_ft_state() {
  std::lock_guard<std::mutex> lock(ft_state_mutex_);
  return ft_states_;
}

std::vector<rcf_standard_driver_interface::Imu> RcfFakeDriver::get_imu_state() {
  std::lock_guard<std::mutex> lock(imu_state_mutex_);
  return imu_states_;
}

RcfFakeDriver::~RcfFakeDriver() {
  stopping();
  driver_exit_.store(true);

  if (rt_drive_thread_.joinable()) {
    rt_drive_thread_.join();
  }
  // spin_thread_.join();
}

}  // namespace rcf_fake_driver
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rcf_fake_driver::RcfFakeDriver, rcf_standard_driver_interface::RcfStandardDriverBase)
