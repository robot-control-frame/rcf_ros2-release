/*******************************************************************************
 * Copyright (c) 2026/08/27, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#include "rcf_common/transmission_parse.hpp"

namespace rcf_common {

TransmissionParse::TransmissionParse() {}

TransmissionParse::~TransmissionParse() {}

bool TransmissionParse::parse(const YAML::Node& transmissions_config) {
  try {
    auto transmissions_data = transmissions_config["transmissions"];
    for (auto trans : transmissions_config["transmissions"]) {
      hardware_interface::TransmissionInfo transmission_info;
      transmission_info.name = trans.first.as<std::string>();
      YAML::Node trans_config = trans.second;
      transmission_info.type = trans_config["type"].as<std::string>();

      YAML::Node act_config = trans_config["actuator"];
      if (!act_config.IsMap()) {
        throw std::runtime_error("actuator config must be a map");
      }
      for (auto act : act_config) {
        auto act_name = act.first.as<std::string>();
        hardware_interface::ActuatorInfo act_info;
        act_info.name = act_name;
        act_info.mechanical_reduction = act.second["mechanical_reduction"].as<double>(1.0);
        act_info.role = act.second["role"].as<std::string>("");
        transmission_info.actuators.push_back(act_info);
      }

      YAML::Node joint_config = trans_config["joint"];
      if (!joint_config.IsMap()) {
        throw std::runtime_error("joint config must be a map");
      }

      for (auto joint : joint_config) {
        auto joint_name = joint.first.as<std::string>();
        hardware_interface::JointInfo joint_info;
        joint_info.name = joint_name;
        joint_info.offset = joint.second["offset"].as<double>(0.0);
        joint_info.role = joint.second["role"].as<std::string>("");
        transmission_info.joints.push_back(joint_info);
      }

      transmission_info_map_.insert({transmission_info.name, transmission_info});

      for (auto joint : joint_config) {
        auto joint_name = joint.first.as<std::string>();
        joint_limits::JointLimits limit;
        YAML::Node joint_limit_config = joint.second["limit"];
        if (!joint_limit_config || joint_limit_config.IsNull()) {
          RCLCPP_ERROR(logger_, "The joint %s does not contain limiting parameters.", joint_name.c_str());
        } else {
          auto lower = joint_limit_config["lower"];
          auto upper = joint_limit_config["upper"];
          auto velocity = joint_limit_config["velocity"];
          auto effort = joint_limit_config["effort"];
          if (!lower || lower.IsNull() || !upper || upper.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain position limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_position_limits = true;
            limit.min_position = lower.as<double>() + std::numeric_limits<double>::epsilon();
            limit.max_position = upper.as<double>() - std::numeric_limits<double>::epsilon();
          }

          if (!velocity || velocity.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain velocity limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_velocity_limits = true;
            limit.max_velocity = velocity.as<double>();
          }

          if (!effort || effort.IsNull()) {
            RCLCPP_ERROR(logger_, "The joint %s does not contain effort limiting  parameters.", joint_name.c_str());
          } else {
            limit.has_effort_limits = true;
            limit.max_effort = effort.as<double>();
          }
        }
        joint_limits_map_.insert({joint_name, limit});
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return false;
  }

  return true;
}

joint_limits::JointLimits TransmissionParse::get_joint_limits(const std::string& joint_name) {
  if (joint_limits_map_.find(joint_name) == joint_limits_map_.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("TransmissionParse"), "Joint %s not found in joint_limits_map_",
                 joint_name.c_str());
    throw std::runtime_error("Joint " + joint_name + " not found in joint_limits_map_");
  } else {
    return joint_limits_map_[joint_name];
  }
}

hardware_interface::TransmissionInfo TransmissionParse::get_transmission_info(const std::string& transmission_name) {
  if (transmission_info_map_.find(transmission_name) == transmission_info_map_.end()) {
    RCLCPP_ERROR(logger_, "Transmission %s not found in transmission_info_map_", transmission_name.c_str());
    throw std::runtime_error("Transmission " + transmission_name + " not found in transmission_info_map_");
  } else {
    return transmission_info_map_[transmission_name];
  }
}

std::vector<hardware_interface::TransmissionInfo> TransmissionParse::get_transmissions() {
  std::vector<hardware_interface::TransmissionInfo> transmissions;
  for (auto& transmission : transmission_info_map_) {
    transmissions.push_back(transmission.second);
  }
  return transmissions;
}

void TransmissionParse::print_transmission_info() {
  for (auto& transmission : transmission_info_map_) {
    std::cout << "Transmission: " << transmission.first << ", type: " << transmission.second.type << std::endl;
  }
}

void TransmissionParse::print_joint_limits() {
  for (auto& joint : joint_limits_map_) {
    std::cout << "Joint: " << joint.first << ", limit[lower: " << joint.second.min_position
              << ", upper: " << joint.second.max_position << ", velocity: " << joint.second.max_velocity
              << ", effort: " << joint.second.max_effort << "]" << std::endl;
  }
}

}  // namespace rcf_common
