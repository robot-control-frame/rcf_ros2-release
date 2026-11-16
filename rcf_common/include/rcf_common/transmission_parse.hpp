#pragma once
#include <yaml-cpp/yaml.h>

#include <hardware_interface/hardware_info.hpp>
#include <joint_limits/joint_limits.hpp>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
namespace rcf_common {

class TransmissionParse {
 public:
  TransmissionParse();
  ~TransmissionParse();

  bool parse(const YAML::Node& transmissions_config);

  std::vector<hardware_interface::TransmissionInfo> get_transmissions();
  hardware_interface::TransmissionInfo get_transmission_info(const std::string& transmission_name);
  joint_limits::JointLimits get_joint_limits(const std::string& joint_name);
  void print_transmission_info();
  void print_joint_limits();

 private:
  std::unordered_map<std::string, joint_limits::JointLimits> joint_limits_map_;
  std::unordered_map<std::string, hardware_interface::TransmissionInfo> transmission_info_map_;

  rclcpp::Logger logger_{rclcpp::get_logger("TransmissionParse")};
};
}  // namespace rcf_common
