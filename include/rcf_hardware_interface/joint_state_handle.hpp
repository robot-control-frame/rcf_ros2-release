/*******************************************************************************
 * Copyright (c) 2026/04/13, Liao Lunjia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#pragma once

#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace rcf_hardware_interface {

class JointStateHandle {
 public:
  explicit JointStateHandle(const std::string& name) : name(name) {
    interface_names_.push_back(name + "/position");
    interface_names_.push_back(name + "/velocity");
    interface_names_.push_back(name + "/effort");
  }

  std::vector<std::string> get_state_interface_names() const { return interface_names_; }
  bool assign_loaned_state_interfaces(std::vector<hardware_interface::LoanedStateInterface>& state_interfaces) {
    return controller_interface::get_ordered_interfaces(state_interfaces, interface_names_, "", state_interfaces_);
  }

  double get_position() const {
    if (!state_interfaces_[0].get().get_optional().has_value()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return state_interfaces_[0].get().get_optional().value();
  }
  double get_velocity() const {
    if (!state_interfaces_[1].get().get_optional().has_value()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return state_interfaces_[1].get().get_optional().value();
  }
  double get_effort() const {
    if (!state_interfaces_[2].get().get_optional().has_value()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return state_interfaces_[2].get().get_optional().value();
  }
  void release_interfaces() { state_interfaces_.clear(); }

 protected:
  std::string name;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
};

}  // namespace rcf_hardware_interface
