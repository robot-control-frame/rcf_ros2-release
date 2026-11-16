/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rcf_hardware_interface/control_modes.hpp"

namespace rcf_hardware_interface {

class JointCommandHandle {
 public:
  explicit JointCommandHandle(const std::string& joint_name) : joint_name_(joint_name) {
    interface_names_.push_back(joint_name + "/control_mode");
    interface_names_.push_back(joint_name + "/cmd_position");
    interface_names_.push_back(joint_name + "/cmd_velocity");
    interface_names_.push_back(joint_name + "/cmd_effort");
    interface_names_.push_back(joint_name + "/cmd_v1");
    interface_names_.push_back(joint_name + "/cmd_v2");
    interface_names_.push_back(joint_name + "/cmd_v3");
  }

  std::vector<std::string> get_command_interface_names() const { return interface_names_; }
  bool assign_loaned_command_interfaces(std::vector<hardware_interface::LoanedCommandInterface>& command_interfaces) {
    return controller_interface::get_ordered_interfaces(command_interfaces, interface_names_, "", command_interfaces_);
  }
  void release_interfaces() { command_interfaces_.clear(); }
  bool setCommand(ControlMode mode, double cmd_position, double cmd_velocity, double cmd_effort, double cmd_v1,
                  double cmd_v2, double cmd_v3) {
    bool success = true;
    success &= command_interfaces_[0].get().set_value(to_interface_value(mode));
    success &= command_interfaces_[1].get().set_value(cmd_position);
    success &= command_interfaces_[2].get().set_value(cmd_velocity);
    success &= command_interfaces_[3].get().set_value(cmd_effort);
    success &= command_interfaces_[4].get().set_value(cmd_v1);
    success &= command_interfaces_[5].get().set_value(cmd_v2);
    success &= command_interfaces_[6].get().set_value(cmd_v3);
    return success;
  }

 private:
  std::string joint_name_;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;
};

}  // namespace rcf_hardware_interface
