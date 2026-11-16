/*******************************************************************************
 * Copyright (c) 2026/04/23, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "rcf_standard_hardware/rcf_standard_hardware.hpp"

#include <cmath>
#include <cstdlib>
#include <hardware_interface/hardware_info.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

namespace rcf_standard_hardware {
hardware_interface::CallbackReturn RcfStandardHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("RcfStandardHardware"));
  clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  std::string driver_classs;
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_init");
  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.rw_rate == 0) {
    RCLCPP_ERROR(get_logger(), "rw_rate is 0, cannot compute control_period");
    return hardware_interface::CallbackReturn::ERROR;
  }
  control_period_ = 1.0 / static_cast<double>(info_.rw_rate);
  if (control_period_ <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid control_period: %.6f (rw_rate=%u)", control_period_, info_.rw_rate);
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "control_period: %.6f s (rw_rate=%u)", control_period_, info_.rw_rate);
  YAML::Node rcf_std_hardware_config, transmissions_config;
  try {
    transmissions_config_path_ = info_.hardware_parameters["transmissions_config_path"];
    RCLCPP_INFO(get_logger(), "transmissions_config_path: %s", transmissions_config_path_.c_str());
    rcf_std_hardware_config_path_ = info_.hardware_parameters["rcf_std_hardware_config_path"];
    RCLCPP_INFO(get_logger(), "rcf_std_hardware_config_path: %s", rcf_std_hardware_config_path_.c_str());

    rcf_std_hardware_config = YAML::LoadFile(rcf_std_hardware_config_path_);
    transmissions_config = YAML::LoadFile(transmissions_config_path_);

    driver_classs = rcf_std_hardware_config["rcf_std_hardware"]["driver"]["class"].as<std::string>("");
    RCLCPP_INFO(get_logger(), "driver_classs: %s", driver_classs.c_str());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception while reading the YAML file: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  pluginlib::ClassLoader<rcf_standard_driver_interface::RcfStandardDriverBase> driver_loader(
      "rcf_standard_driver_interface", "rcf_standard_driver_interface::RcfStandardDriverBase");

  transmission_loader_ = std::make_shared<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
      "transmission_interface", "transmission_interface::TransmissionLoader");
  const char* hw_debug_env = std::getenv("HW_DEBUG");
  const std::string hw_debug_value = hw_debug_env == nullptr ? "" : hw_debug_env;
  hw_debug_ = hw_debug_value == "1" || hw_debug_value == "true" || hw_debug_value == "TRUE" || hw_debug_value == "on" ||
              hw_debug_value == "ON";
  if (hw_debug_) {
    driver_classs = "rcf_fake_driver/RcfFakeDriver";
    RCLCPP_WARN(get_logger(), "HW_DEBUG is enabled, use fake data");
  } else {
    RCLCPP_INFO(get_logger(), "HW_DEBUG is disabled,use driver: %s", driver_classs.c_str());
  }
  try {
    RCLCPP_INFO(get_logger(), "Create driver: %s", driver_classs.c_str());
    driver_ = driver_loader.createSharedInstance(driver_classs);
    std::vector<std::string> driver_config;
    auto driver_cfg_node = rcf_std_hardware_config["rcf_std_hardware"]["driver"]["config"];
    if (driver_cfg_node && driver_cfg_node.IsSequence() && driver_cfg_node.size() > 0) {
      for (const auto& item : driver_cfg_node) {
        driver_config.push_back(item.as<std::string>());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Driver config is not a sequence.");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!driver_->init(driver_config)) {
      RCLCPP_ERROR(get_logger(), "Driver init failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }

  } catch (pluginlib::PluginlibException& ex) {
    RCLCPP_ERROR(get_logger(), "Failed to create driver: %s", ex.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!transmission_parse_.parse(transmissions_config)) {
    RCLCPP_ERROR(get_logger(), "Parse transmissions config FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!setup_motors(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup MOTORS FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!setup_transmission(transmissions_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup TRANSMISSION FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!setup_joint_limits(transmissions_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup JOINT LIMITS FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  transmission_parse_.print_transmission_info();
  transmission_parse_.print_joint_limits();

  if (!setup_imus(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup IMU FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!setup_ft_sensors(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup FT SENSORS FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!setup_check_timeout(rcf_std_hardware_config)) {
    RCLCPP_ERROR(get_logger(), "RcfStandardHardware Setup CHECK TIMEOUT FAILURE");
    return hardware_interface::CallbackReturn::ERROR;
  }
  servo_srv_callback_group_ = get_node()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  servo_enable_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      get_name() + "/servo/enable",
      std::bind(&RcfStandardHardware::servo_enable_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), servo_srv_callback_group_);
  servo_disable_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
      get_name() + "/servo/disable",
      std::bind(&RcfStandardHardware::servo_disable_cb, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), servo_srv_callback_group_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  dynamic_actuator_state_publisher_ = get_node()->create_publisher<control_msgs::msg::DynamicJointState>(
      get_name() + "/actuator_states", rclcpp::SystemDefaultsQoS());
  realtime_dynamic_actuator_state_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<control_msgs::msg::DynamicJointState>>(
          dynamic_actuator_state_publisher_);

  RCLCPP_INFO(get_logger(), "Configuration successful");
  driver_->starting();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RcfStandardHardware::export_state_interfaces() {
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (const auto& joint : joint_data_vector) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint->name, "position", &joint->position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint->name, "velocity", &joint->velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint->name, "effort", &joint->effort));
  }

  for (const auto& imu : imus_data_vec_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.x", &imu->orientation[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.y", &imu->orientation[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.z", &imu->orientation[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(imu->name, "orientation.w", &imu->orientation[3]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.x", &imu->angular_vel[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.y", &imu->angular_vel[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "angular_velocity.z", &imu->angular_vel[2]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.x", &imu->linear_accel[0]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.y", &imu->linear_accel[1]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(imu->name, "linear_acceleration.z", &imu->linear_accel[2]));
  }

  for (const auto& ft : ft_sensor_data_vec_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.x", &ft->force[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.y", &ft->force[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "force.z", &ft->force[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.x", &ft->torque[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.y", &ft->torque[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(ft->name, "torque.z", &ft->torque[2]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RcfStandardHardware::export_command_interfaces() {
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (const auto& joint : joint_data_vector) {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint->name, "control_mode", &joint->control_mode));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint->name, "cmd_position", &joint->position_command));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint->name, "cmd_velocity", &joint->velocity_command));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(joint->name, "cmd_effort", &joint->effort_command));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint->name, "cmd_v1", &joint->v1));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint->name, "cmd_v2", &joint->v2));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(joint->name, "cmd_v3", &joint->v3));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "===================================================.");
  hw_init_check_success_ = hardware_online_check();
  if (!hw_init_check_success_) {
    RCLCPP_ERROR(get_logger(), "Hardware online check failed.");
    RCLCPP_INFO(get_logger(), "===================================================.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  init_dynamic_actuator_state_msg();
  RCLCPP_INFO(get_logger(), "===================================================.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RcfStandardHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "RcfStandardHardware::on_deactivate.");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RcfStandardHardware::read(const rclcpp::Time& time,
                                                          const rclcpp::Duration& /*period*/) {
  // read servo
  auto servo_status = driver_->get_servo_state();
  // RCLCPP_INFO(get_logger(), "Servo status size: %zu", servo_status.size());
  for (auto& item : servo_status) {
    auto iter = id2act_index_.find(item.id);
    if (iter != id2act_index_.end()) {
      auto& act_data = act_data_vector[id2act_index_[item.id]];
      act_data->status_stamp = item.stamp;
      act_data->pos = item.position;
      act_data->pos_raw = item.position;
      if (act_data->use_vel_filter && act_data->vel_filter) {
        act_data->vel_filter->input(item.velocity, control_period_);
        act_data->vel = act_data->vel_filter->output();
      } else {
        act_data->vel = item.velocity;
      }
      act_data->effort = item.effort;
      act_data->cur = item.current;
      act_data->status = item.error_code;
      act_data->temperature = item.temperature;
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown Motor ID: %d", item.id);
    }
  }

  for (auto& transmission : transmissions_) {
    transmission->actuator_to_joint();
  }

  auto imu_states = driver_->get_imu_state();
  for (auto& item : imu_states) {
    auto iter = id2imu_index_.find(item.id);
    if (iter != id2imu_index_.end()) {
      auto& imu_data = imus_data_vec_[id2imu_index_[item.id]];
      imu_data->stamp = item.stamp;
      imu_data->angular_vel[0] = item.angular_velocity[0];
      imu_data->angular_vel[1] = item.angular_velocity[1];
      imu_data->angular_vel[2] = item.angular_velocity[2];
      imu_data->linear_accel[0] = item.linear_acceleration[0];
      imu_data->linear_accel[1] = item.linear_acceleration[1];
      imu_data->linear_accel[2] = item.linear_acceleration[2];
      imu_data->orientation[0] = item.orientation[0];
      imu_data->orientation[1] = item.orientation[1];
      imu_data->orientation[2] = item.orientation[2];
      imu_data->orientation[3] = item.orientation[3];
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown IMU device ID: %d", item.id);
    }
  }

  // read force torque sensors
  auto ft_status = driver_->get_ft_state();
  for (auto& item : ft_status) {
    auto iter = id2ft_index_.find(item.id);
    if (iter != id2ft_index_.end()) {
      auto& ft_data = ft_sensor_data_vec_[id2ft_index_[item.id]];
      ft_data->stamp = item.stamp;
      ft_data->force[0] = item.force[0];
      ft_data->force[1] = item.force[1];
      ft_data->force[2] = item.force[2];
      ft_data->torque[0] = item.torque[0];
      ft_data->torque[1] = item.torque[1];
      ft_data->torque[2] = item.torque[2];

    } else {
      RCLCPP_ERROR(get_logger(), "Unknown FT device ID: %d", item.id);
    }
  }

  return hardware_timeout_check(time);
}

hardware_interface::return_type RcfStandardHardware::write(const rclcpp::Time& /*time*/,
                                                           const rclcpp::Duration& /*period*/) {
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  for (auto& joint_limits : joint_limits_) {
    joint_limits->enforce(control_period_);
  }
  for (auto& transmission : transmissions_) {
    transmission->joint_to_actuator();
  }

  if (servo_init_.load()) {
    std::vector<rcf_standard_driver_interface::ServoCommand> all_servo_cmd;

    for (size_t i = 0; i < act_data_vector.size(); i++) {
      rcf_standard_driver_interface::ServoCommand servo_cmd;
      servo_cmd.stamp = get_clock()->now();
      servo_cmd.id = act_data_vector[i]->id;
      if (is_control_mode_value(act_data_vector[i]->ctrl_mode, rcf_hardware_interface::ControlMode::BEGIN)) {
        if (!is_control_mode_value(act_data_vector[i]->last_ctrl_mode, rcf_hardware_interface::ControlMode::BEGIN)) {
          act_data_vector[i]->cmd_pos_keep = act_data_vector[i]->pos;
        }
        act_data_vector[i]->cmd_pos = act_data_vector[i]->cmd_pos_keep;
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::BEGIN;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      } else if (is_control_mode_value(act_data_vector[i]->ctrl_mode,
                                       rcf_hardware_interface::ControlMode::MODE_EFFORT)) {
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::MODE_EFFORT;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      } else if (is_control_mode_value(act_data_vector[i]->ctrl_mode,
                                       rcf_hardware_interface::ControlMode::MODE_VELOCITY)) {
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::MODE_VELOCITY;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      } else if (is_control_mode_value(act_data_vector[i]->ctrl_mode,
                                       rcf_hardware_interface::ControlMode::MODE_POSITION)) {
        if (use_jump_command_protection_) {
          // 阶跃指令输出log
          if (act_data_vector[i]->act_type == ActType::STANDARD_REVOLUTE) {
            if (std::abs(act_data_vector[i]->cmd_pos - act_data_vector[i]->pos) >=
                act_data_vector[i]->jump_cmd_threshold) {
              if ((get_clock()->now() - act_data_vector[i]->last_printf_jump_command_stamp).seconds() >
                  1.0) {  // 1s内只打印一次
                RCLCPP_ERROR(get_logger(),
                             "Servo %s send a jump command, threshold is 0.087rad, current pos: %f, pos cmd: %f, "
                             "current: %f, err code: %d",
                             act_data_vector[i]->name.c_str(), act_data_vector[i]->pos, act_data_vector[i]->cmd_pos,
                             act_data_vector[i]->cur, act_data_vector[i]->status);
                act_data_vector[i]->last_printf_jump_command_stamp = get_clock()->now();
              }
            }
          } else if (act_data_vector[i]->act_type == ActType::STANDARD_PRISMATIC) {
            if (std::abs(act_data_vector[i]->cmd_pos - act_data_vector[i]->pos) >=
                act_data_vector[i]->jump_cmd_threshold) {
              if ((get_clock()->now() - act_data_vector[i]->last_printf_jump_command_stamp).seconds() > 1.0) {
                RCLCPP_ERROR(get_logger(),
                             "Servo %s send a jump command, threshold is 0.002m, current pos: %f, pos cmd: %f, "
                             "current: %f, err code: %d",
                             act_data_vector[i]->name.c_str(), act_data_vector[i]->pos, act_data_vector[i]->cmd_pos,
                             act_data_vector[i]->cur, act_data_vector[i]->status);
                act_data_vector[i]->last_printf_jump_command_stamp = get_clock()->now();
              }
            }
          } else {
            // Do nothing
          }
        }
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::MODE_POSITION;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      } else if (is_control_mode_value(act_data_vector[i]->ctrl_mode,
                                       rcf_hardware_interface::ControlMode::MODE_PVT)) {  // PVT mode
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::MODE_PVT;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      } else {
        // 不支持其余模式
        servo_cmd.mode = rcf_standard_driver_interface::ControlModes::BEGIN;
        act_data_vector[i]->last_ctrl_mode = act_data_vector[i]->ctrl_mode;
      }

      servo_cmd.position = act_data_vector[i]->cmd_pos;
      servo_cmd.velocity = act_data_vector[i]->cmd_vel;
      servo_cmd.effort = act_data_vector[i]->cmd_effort;
      servo_cmd.v1 = act_data_vector[i]->cmd_v1;
      servo_cmd.v2 = act_data_vector[i]->cmd_v2;
      servo_cmd.v3 = act_data_vector[i]->cmd_v3;
      all_servo_cmd.push_back(servo_cmd);
    }

    driver_->set_servo_command(all_servo_cmd);
  }

  actuator_state_time_since_last_publish_ += control_period_;
  const bool should_publish_actuator_state =
      actuator_state_publish_rate_ <= 0.0 || actuator_state_time_since_last_publish_ >= actuator_state_publish_period_;
  if (should_publish_actuator_state) {
    dynamic_actuator_state_msg_.header.stamp = get_clock()->now();
    for (size_t i = 0; i < act_data_vector.size(); i++) {
      auto& act_data = act_data_vector[i];
      dynamic_actuator_state_msg_.interface_values[i].values[0] = act_data->pos_raw;
      dynamic_actuator_state_msg_.interface_values[i].values[1] = act_data->q_circle;
      dynamic_actuator_state_msg_.interface_values[i].values[2] = act_data->pos;
      dynamic_actuator_state_msg_.interface_values[i].values[3] = act_data->vel;
      dynamic_actuator_state_msg_.interface_values[i].values[4] = act_data->effort;
      dynamic_actuator_state_msg_.interface_values[i].values[5] = act_data->ctrl_mode;
      dynamic_actuator_state_msg_.interface_values[i].values[6] = act_data->cmd_pos;
      dynamic_actuator_state_msg_.interface_values[i].values[7] = act_data->cmd_vel;
      dynamic_actuator_state_msg_.interface_values[i].values[8] = act_data->cmd_effort;
      dynamic_actuator_state_msg_.interface_values[i].values[9] = act_data->cmd_v1;
      dynamic_actuator_state_msg_.interface_values[i].values[10] = act_data->cmd_v2;
      dynamic_actuator_state_msg_.interface_values[i].values[11] = act_data->cmd_v3;
      dynamic_actuator_state_msg_.interface_values[i].values[12] = act_data->cur;
      dynamic_actuator_state_msg_.interface_values[i].values[13] = act_data->status;
      dynamic_actuator_state_msg_.interface_values[i].values[14] = act_data->temperature;
    }
    if (realtime_dynamic_actuator_state_publisher_->try_publish(dynamic_actuator_state_msg_)) {
      actuator_state_time_since_last_publish_ = 0.0;
    }
  }
  return hardware_interface::return_type::OK;
}

bool RcfStandardHardware::setup_motors(YAML::Node& hw_config) {
  try {
    auto actuators_config = hw_config["rcf_std_hardware"]["actuators"];
    size_t act_index = 0;
    for (auto motor : actuators_config) {
      std::string name = motor.first.as<std::string>();
      YAML::Node motor_config = motor.second;
      uint32_t id = motor_config["id"].as<uint32_t>();
      auto data = std::make_unique<ActData>();
      data->name = name;
      data->id = id;
      data->type = motor_config["type"].as<std::string>("");
      data->allow_homing = motor_config["allow_homing"].as<bool>(true);
      if (data->type == "prismatic") {
        data->act_type = ActType::STANDARD_PRISMATIC;
        data->jump_cmd_threshold = motor_config["jump_cmd_threshold"].as<double>(0.002);
      } else if (data->type == "revolute") {
        data->act_type = ActType::STANDARD_REVOLUTE;
        data->jump_cmd_threshold = motor_config["jump_cmd_threshold"].as<double>(0.087);
      } else {
        data->act_type = ActType::UNKNOWN;
      }

      data->use_vel_filter = motor_config["use_vel_filter"].as<bool>(false);
      if (data->use_vel_filter) {
        const double lp_cutoff_frequency = motor_config["lp_cutoff_frequency"].as<double>(70.0);
        data->vel_filter = std::make_shared<rcf_common::filters::LowPassFilter>(lp_cutoff_frequency);
        RCLCPP_INFO(get_logger(), "Actuator %s: velocity low-pass filter enabled, cutoff=%.1f Hz", name.c_str(),
                    lp_cutoff_frequency);
      }

      act_data_vector.push_back(std::move(data));
      id2act_index_.insert(std::make_pair(id, act_index));
      act_index += 1;
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Setup motors success");
  return true;
}

bool RcfStandardHardware::setup_transmission(YAML::Node& transmissions_config) {
  try {
    auto all_transmissions_info = transmission_parse_.get_transmissions();

    for (auto transmission_info : all_transmissions_info) {
      std::shared_ptr<transmission_interface::Transmission> transmission;

      auto loader = transmission_loader_->createSharedInstance(transmission_info.type);

      for (auto& actuator : transmission_info.actuators) {
        auto act_data_iter = std::find_if(act_data_vector.begin(), act_data_vector.end(),
                                          [actuator](const std::shared_ptr<ActData>& ptr) {
                                            if (!ptr) return false;
                                            return ptr->name == actuator.name;
                                          });

        if (act_data_iter == act_data_vector.end()) {
          RCLCPP_ERROR(get_logger(), "Unknown actuator: %s", actuator.name.c_str());
          return false;
        }
      }

      transmission = loader->load(transmission_info);

      std::vector<transmission_interface::JointHandle> joint_handles;
      std::vector<transmission_interface::ActuatorHandle> actuator_handles;

      for (auto& joint_info : transmission_info.joints) {
        auto joint_interface_data = std::make_shared<JointData>();
        joint_interface_data->name = joint_info.name;
        joint_interface_data->position = 0.0;
        joint_interface_data->velocity = 0.0;
        joint_interface_data->effort = 0.0;
        joint_interface_data->control_mode =
            rcf_hardware_interface::to_interface_value(rcf_hardware_interface::ControlMode::BEGIN);
        joint_interface_data->position_command = 0.0;
        joint_interface_data->velocity_command = 0.0;
        joint_interface_data->effort_command = 0.0;
        joint_interface_data->v1 = 0.0;
        joint_interface_data->v2 = 0.0;
        joint_interface_data->v3 = 0.0;
        joint_data_vector.push_back(joint_interface_data);
        transmission_interface::JointHandle joint_state_pos_handle(joint_info.name, "position",
                                                                   &joint_interface_data->position);
        transmission_interface::JointHandle joint_state_vel_handle(joint_info.name, "velocity",
                                                                   &joint_interface_data->velocity);
        transmission_interface::JointHandle joint_state_eff_handle(joint_info.name, "effort",
                                                                   &joint_interface_data->effort);
        transmission_interface::JointHandle joint_control_mode_handle(joint_info.name, "control_mode",
                                                                      &joint_interface_data->control_mode);
        transmission_interface::JointHandle joint_cmd_pos_handle(joint_info.name, "cmd_position",
                                                                 &joint_interface_data->position_command);
        transmission_interface::JointHandle joint_cmd_vel_handle(joint_info.name, "cmd_velocity",
                                                                 &joint_interface_data->velocity_command);
        transmission_interface::JointHandle joint_cmd_eff_handle(joint_info.name, "cmd_effort",
                                                                 &joint_interface_data->effort_command);
        transmission_interface::JointHandle joint_cmd_v1_handle(joint_info.name, "cmd_v1", &joint_interface_data->v1);
        transmission_interface::JointHandle joint_cmd_v2_handle(joint_info.name, "cmd_v2", &joint_interface_data->v2);
        transmission_interface::JointHandle joint_cmd_v3_handle(joint_info.name, "cmd_v3", &joint_interface_data->v3);
        joint_handles.push_back(joint_state_pos_handle);
        joint_handles.push_back(joint_state_vel_handle);
        joint_handles.push_back(joint_state_eff_handle);
        joint_handles.push_back(joint_control_mode_handle);
        joint_handles.push_back(joint_cmd_pos_handle);
        joint_handles.push_back(joint_cmd_vel_handle);
        joint_handles.push_back(joint_cmd_eff_handle);
        joint_handles.push_back(joint_cmd_v1_handle);
        joint_handles.push_back(joint_cmd_v2_handle);
        joint_handles.push_back(joint_cmd_v3_handle);
      }

      for (auto& act_info : transmission_info.actuators) {
        auto act_data_iter = std::find_if(act_data_vector.begin(), act_data_vector.end(),
                                          [act_info](const std::shared_ptr<ActData>& ptr) {
                                            if (!ptr) return false;
                                            return ptr->name == act_info.name;
                                          });

        transmission_interface::ActuatorHandle actuator_state_pos_handle(act_info.name, "position",
                                                                         &(act_data_iter->get()->pos));
        transmission_interface::ActuatorHandle actuator_state_vel_handle(act_info.name, "velocity",
                                                                         &(act_data_iter->get()->vel));
        transmission_interface::ActuatorHandle actuator_state_eff_handle(act_info.name, "effort",
                                                                         &(act_data_iter->get()->effort));
        transmission_interface::ActuatorHandle actuator_cmd_control_mode_handle(act_info.name, "control_mode",
                                                                                &(act_data_iter->get()->ctrl_mode));
        transmission_interface::ActuatorHandle actuator_cmd_pos_handle(act_info.name, "cmd_position",
                                                                       &(act_data_iter->get()->cmd_pos));
        transmission_interface::ActuatorHandle actuator_cmd_vel_handle(act_info.name, "cmd_velocity",
                                                                       &(act_data_iter->get()->cmd_vel));
        transmission_interface::ActuatorHandle actuator_cmd_eff_handle(act_info.name, "cmd_effort",
                                                                       &(act_data_iter->get()->cmd_effort));
        transmission_interface::ActuatorHandle actuator_cmd_v1_handle(act_info.name, "cmd_v1",
                                                                      &(act_data_iter->get()->cmd_v1));
        transmission_interface::ActuatorHandle actuator_cmd_v2_handle(act_info.name, "cmd_v2",
                                                                      &(act_data_iter->get()->cmd_v2));
        transmission_interface::ActuatorHandle actuator_cmd_v3_handle(act_info.name, "cmd_v3",
                                                                      &(act_data_iter->get()->cmd_v3));
        actuator_handles.push_back(actuator_state_pos_handle);
        actuator_handles.push_back(actuator_state_vel_handle);
        actuator_handles.push_back(actuator_state_eff_handle);
        actuator_handles.push_back(actuator_cmd_control_mode_handle);
        actuator_handles.push_back(actuator_cmd_pos_handle);
        actuator_handles.push_back(actuator_cmd_vel_handle);
        actuator_handles.push_back(actuator_cmd_eff_handle);
        actuator_handles.push_back(actuator_cmd_v1_handle);
        actuator_handles.push_back(actuator_cmd_v2_handle);
        actuator_handles.push_back(actuator_cmd_v3_handle);
      }
      transmission->configure(joint_handles, actuator_handles);

      transmissions_.push_back(transmission);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

bool RcfStandardHardware::setup_joint_limits(YAML::Node& transmissions_config) {
  try {
    for (auto& joint : joint_data_vector) {
      auto limit = transmission_parse_.get_joint_limits(joint->name);
      auto joint_limits = std::make_shared<rcf_joint_limits_interface::RcfJointLimits>(
          limit, &(joint->position), &(joint->velocity), &(joint->effort), &(joint->control_mode),
          &(joint->position_command), &(joint->velocity_command), &(joint->effort_command));
      joint_limits_.push_back(joint_limits);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

bool RcfStandardHardware::setup_imus(YAML::Node& hw_config) {
  try {
    auto imus_config = hw_config["rcf_std_hardware"]["imus"];

    size_t imu_index = 0;
    for (auto imu : imus_config) {
      std::string name = imu.first.as<std::string>();
      YAML::Node imu_config = imu.second;
      uint32_t id = imu_config["id"].as<uint32_t>();
      auto data = std::make_shared<rcf_standard_hardware::ImuData>();
      data->name = name;
      data->id = id;
      imus_data_vec_.push_back(data);
      id2imu_index_.insert(std::make_pair(id, imu_index));
      imu_index += 1;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  RCLCPP_INFO(get_logger(), "Setup imus sensors success");
  return true;
}

bool RcfStandardHardware::setup_ft_sensors(YAML::Node& hw_config) {
  try {
    auto fts_config = hw_config["rcf_std_hardware"]["force_torque_sensors"];
    size_t ft_index = 0;
    for (auto ft : fts_config) {
      std::string name = ft.first.as<std::string>();
      YAML::Node ft_config = ft.second;
      uint32_t id = ft_config["id"].as<uint32_t>();
      auto data = std::make_shared<ForceTorqueData>();
      data->name = name;
      data->id = id;
      ft_sensor_data_vec_.push_back(data);
      id2ft_index_.insert(std::make_pair(id, ft_index));
      ft_index += 1;
    }
    RCLCPP_INFO(get_logger(), "Setup ft sensors success");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

bool RcfStandardHardware::setup_check_timeout(YAML::Node& hw_config) {
  try {
    auto standard_hardware_data = hw_config["rcf_std_hardware"];
    timeout_threshold_ = standard_hardware_data["timeout_threshold"].as<double>(0.1);
    check_timeout_ = standard_hardware_data["check_timeout"].as<bool>(true);
    use_jump_command_protection_ = standard_hardware_data["use_jump_command_protection"].as<bool>(true);
    actuator_state_publish_rate_ = standard_hardware_data["actuator_state_publish_rate"].as<double>(50.0);
    if (actuator_state_publish_rate_ > 0.0) {
      actuator_state_publish_period_ = 1.0 / actuator_state_publish_rate_;
    } else {
      // <=0 表示不限频，每个 write 周期都发布
      actuator_state_publish_period_ = 0.0;
    }
    RCLCPP_INFO(get_logger(),
                "timeout_threshold: %.3f, check_timeout: %s, use_jump_command_protection: %s, "
                "actuator_state_publish_rate: %.1f",
                timeout_threshold_, check_timeout_ ? "true" : "false", use_jump_command_protection_ ? "true" : "false",
                actuator_state_publish_rate_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return false;
  }
  return true;
}

void RcfStandardHardware::init_dynamic_actuator_state_msg() {
  dynamic_actuator_state_msg_.header.frame_id = "RcfStandardHardware";
  dynamic_actuator_state_msg_.joint_names.clear();
  dynamic_actuator_state_msg_.interface_values.clear();

  for (const auto& act_data : act_data_vector) {
    dynamic_actuator_state_msg_.joint_names.push_back(act_data->name);
    control_msgs::msg::InterfaceValue if_value;
    if_value.interface_names.emplace_back("position_raw");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("circle");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("position");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("velocity");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("effort");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("control_mode");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_position");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_velocity");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_effort");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_v1");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_v2");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("cmd_v3");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("current");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("status");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    if_value.interface_names.emplace_back("temperature");
    if_value.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
    dynamic_actuator_state_msg_.interface_values.emplace_back(if_value);
  }
}

void RcfStandardHardware::servo_enable_cb(const std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                                          std_srvs::srv::Trigger::Response::SharedPtr resp) {
  servo_init_.store(false);
  for (auto& item : act_data_vector) {
    item->last_ctrl_mode = rcf_hardware_interface::to_interface_value(rcf_hardware_interface::ControlMode::NOMODE);
  }

  std::vector<uint16_t> servos_id;
  for (auto& item : act_data_vector) {
    servos_id.push_back(item->id);
  }

  if (driver_->servo_enable(servos_id)) {
    RCLCPP_INFO(get_logger(), "servo success to enable");
    servo_init_.store(true);
    resp->success = true;
    resp->message = "servo success to enable";

    return;
  } else {
    RCLCPP_ERROR(get_logger(), "servo failed to enable");
    resp->success = false;
    resp->message = "servo failed to enable";
    return;
  }

  return;
}
void RcfStandardHardware::servo_disable_cb(const std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                                           std_srvs::srv::Trigger::Response::SharedPtr resp) {
  if (!servo_init_.load()) {
    RCLCPP_WARN(get_logger(), "servo already disabled");
    resp->success = true;
    resp->message = "servo already disabled";
    return;
  }

  std::vector<uint16_t> servos_id;
  for (auto& item : act_data_vector) {
    servos_id.push_back(item->id);
  }

  if (driver_->servo_disable(servos_id)) {
    RCLCPP_INFO(get_logger(), "servo success to disable");
    servo_init_.store(false);
    resp->message = "servo success to disable";
    resp->success = true;

    return;
  } else {
    RCLCPP_ERROR(get_logger(), "servo failed to disable");
    resp->success = false;
    resp->message = "servo failed to disable";
    return;
  }

  return;
}

bool RcfStandardHardware::hardware_online_check() {
  RCLCPP_INFO(get_logger(), "开始硬件在线自检.");
  auto start_check_time = get_clock()->now();
  rclcpp::Rate check_rate(1000);
  // 3秒内如果在线，则认为硬件在线
  while (rclcpp::ok() and (get_clock()->now() - start_check_time).seconds() < 3.0) {
    bool servo_online = true;
    bool imu_online = true;
    bool ft_online = true;

    auto now = get_clock()->now();

    read(get_clock()->now(), rclcpp::Duration::from_seconds(0.001));
    if (check_timeout_) {
      // servo
      for (auto& item : act_data_vector) {
        if ((now - item->status_stamp).seconds() > timeout_threshold_) {
          item->online = false;
        } else {
          item->online = true;
        }
        servo_online = servo_online && item->online;
      }
    }

    // IMU
    for (auto& item : imus_data_vec_) {
      if ((now - item->stamp).seconds() > timeout_threshold_) {
        item->online = false;
      } else {
        item->online = true;
      }
      imu_online = imu_online && item->online;
    }
    // ft
    for (auto& item : ft_sensor_data_vec_) {
      if ((now - item->stamp).seconds() > timeout_threshold_) {
        item->online = false;
      } else {
        item->online = true;
      }
      ft_online = ft_online && item->online;
    }

    if (servo_online and imu_online and ft_online) {
      hw_init_check_success_ = true;
      break;
    }
    check_rate.sleep();
  }

  if (hw_init_check_success_) {
    RCLCPP_INFO(get_logger(), "硬件在线自检成功.");
    return true;
  } else {
    for (auto& item : act_data_vector) {
      if (!item->online) {
        RCLCPP_ERROR(get_logger(), "舵机: %d, %s 超时.", item->id, item->name.c_str());
      }
    }
    for (auto& item : imus_data_vec_) {
      if (!item->online) {
        RCLCPP_ERROR(get_logger(), "IMU: %d, %s 超时.", item->id, item->name.c_str());
      }
    }
    for (auto& item : ft_sensor_data_vec_) {
      if (!item->online) {
        RCLCPP_ERROR(get_logger(), "六维力: %d, %s 超时.", item->id, item->name.c_str());
      }
    }
    RCLCPP_ERROR(get_logger(), "硬件在线自检失败.");
    return false;
  }
}

hardware_interface::return_type RcfStandardHardware::hardware_timeout_check(const rclcpp::Time& time) {
  // check servo

  if (!hw_init_check_success_) {
    return hardware_interface::return_type::OK;
  }
  bool servo_online = true;
  bool imu_online = true;
  bool ft_online = true;

  if (check_timeout_) {
    for (auto& item : act_data_vector) {
      if ((get_clock()->now() - item->status_stamp).seconds() < timeout_threshold_) {
        // ULOGD("Motor  device mormal.");
      } else if ((time - item->status_stamp).seconds() > timeout_threshold_ and
                 (time - item->status_stamp).seconds() <= 3.0) {
        if ((time - item->last_timeout_print_stamp).seconds() > 0.1) {
          RCLCPP_WARN(get_logger(), "舵机: %d, %s 超时 %f 秒.", item->id, item->name.c_str(),
                      (time - item->status_stamp).seconds());
          item->last_timeout_print_stamp = time;
        }

      } else if ((time - item->status_stamp).seconds() >= 3.0) {
        servo_online = false;
        RCLCPP_ERROR(get_logger(), "舵机: %d, %s 超时 %f 秒, 舵机可能离线.", item->id, item->name.c_str(),
                     (time - item->status_stamp).seconds());
      }
    }

    for (auto& item : imus_data_vec_) {
      if ((time - item->stamp).seconds() < timeout_threshold_) {
        // ULOGD("IMU sensor device mormal.");
      } else if ((time - item->stamp).seconds() > timeout_threshold_ and (time - item->stamp).seconds() <= 3.0) {
        if ((time - item->last_timeout_print_stamp).seconds() > 0.1) {
          RCLCPP_WARN(get_logger(), "IMU %s device timeout %f second.", item->name.c_str(),
                      (time - item->stamp).seconds());
          item->last_timeout_print_stamp = time;
        }
      } else if ((time - item->stamp).seconds() >= 3.0) {
        imu_online = false;
        RCLCPP_ERROR(get_logger(), "IMU: %d, %s 超时 %f 秒, IMU 可能离线.", item->id, item->name.c_str(),
                     (time - item->stamp).seconds());
      }
    }
    // check ft

    for (auto& item : ft_sensor_data_vec_) {
      if ((time - item->stamp).seconds() < timeout_threshold_) {
        // ULOGD("FT sensor device mormal.");
      } else if ((time - item->stamp).seconds() > timeout_threshold_ and (time - item->stamp).seconds() <= 3.0) {
        if ((time - item->last_timeout_print_stamp).seconds() > 0.1) {
          RCLCPP_WARN(get_logger(), "FT %s device timeout %f second.", item->name.c_str(),
                      (time - item->stamp).seconds());
          item->last_timeout_print_stamp = time;
        }
      } else if ((time - item->stamp).seconds() >= 3.0) {
        ft_online = false;
        RCLCPP_ERROR(get_logger(), "FT: %d, %s 超时 %f 秒, FT 可能离线.", item->id, item->name.c_str(),
                     (time - item->stamp).seconds());
      }
    }

    if (!servo_online or !imu_online or !ft_online) {
      RCLCPP_FATAL(get_logger(), "检测到硬件异常, 请检查硬件连接.");
      return hardware_interface::return_type::ERROR;
    }
  }

  last_check_hw_time_ = time;
  return hardware_interface::return_type::OK;
}

}  // namespace rcf_standard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(rcf_standard_hardware::RcfStandardHardware, hardware_interface::SystemInterface)
