/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#pragma once
#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "transmission_interface/exception.hpp"
#include "transmission_interface/transmission.hpp"
namespace rcf_transmission_interface {

constexpr auto HW_IF_ABSOLUTE_POSITION = "absolute_position";

class RcfSimpleTransmission : public transmission_interface::Transmission {
 public:
  /**
   * \param[in] joint_to_actuator_reduction Joint to actuator reduction ratio.
   * \param[in] joint_offset Joint position offset used in the position mappings.
   * \pre Nonzero reduction value.
   */
  explicit RcfSimpleTransmission(const double joint_to_actuator_reduction, const double joint_offset = 0.0);

  /// Set up the data the transmission operates on.
  /**
   * \param[in] joint_handles Vector of interface handles of one joint
   * \param[in] actuator_handles Vector of interface handles of one actuator
   * \pre Vectors are nonzero.
   * \pre Joint handles share the same joint name and actuator handles share the same actuator name.
   */
  void configure(const std::vector<transmission_interface::JointHandle>& joint_handles,
                 const std::vector<transmission_interface::ActuatorHandle>& actuator_handles) override;

  /// Transform variables from actuator to joint space.
  /**
   *  This method operates on the handles provided when configuring the transmission.
   *  To call this method it is not required that all supported interface types are provided, e.g.
   * one can supply only velocity handles
   */
  void actuator_to_joint() override;

  /// Transform variables from joint to actuator space.
  /**
   *  This method operates on the handles provided when configuring the transmission.
   *  To call this method it is not required that all supported interface types are provided, e.g.
   * one can supply only velocity handles
   */
  void joint_to_actuator() override;

  std::size_t num_actuators() const override { return 1; }
  std::size_t num_joints() const override { return 1; }

  double get_actuator_reduction() const { return reduction_; }
  double get_joint_offset() const { return jnt_offset_; }

  std::vector<std::string> get_supported_actuator_interfaces() const override {
    return {"position",     "velocity",   "effort", "control_mode", "cmd_position",
            "cmd_velocity", "cmd_effort", "cmd_v1", "cmd_v2",       "cmd_v3"};
  }

  std::vector<std::string> get_supported_joint_interfaces() const override {
    return {"position",     "velocity",   "effort", "control_mode", "cmd_position",
            "cmd_velocity", "cmd_effort", "cmd_v1", "cmd_v2",       "cmd_v3"};
  }

 protected:
  double reduction_;
  double jnt_offset_;

  transmission_interface::JointHandle state_joint_position_ = {"", "", nullptr};
  transmission_interface::JointHandle state_joint_velocity_ = {"", "", nullptr};
  transmission_interface::JointHandle state_joint_effort_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle state_actuator_position_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle state_actuator_velocity_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle state_actuator_effort_ = {"", "", nullptr};

  transmission_interface::JointHandle cmd_joint_control_mode_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_position_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_velocity_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_effort_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_v1_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_v2_ = {"", "", nullptr};
  transmission_interface::JointHandle cmd_joint_v3_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_control_mode_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_position_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_velocity_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_effort_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_v1_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_v2_ = {"", "", nullptr};
  transmission_interface::ActuatorHandle cmd_actuator_v3_ = {"", "", nullptr};
};

inline RcfSimpleTransmission::RcfSimpleTransmission(const double joint_to_actuator_reduction, const double joint_offset)
    : reduction_(joint_to_actuator_reduction), jnt_offset_(joint_offset) {
  if (reduction_ == 0.0) {
    throw transmission_interface::Exception("Transmission reduction ratio cannot be zero.");
  }
}

template <class HandleType>
HandleType get_by_interface(const std::vector<HandleType>& handles, const std::string& interface_name) {
  const auto result = std::find_if(handles.cbegin(), handles.cend(), [&interface_name](const auto handle) {
    return handle.get_interface_name() == interface_name;
  });
  if (result == handles.cend()) {
    return HandleType(handles.cbegin()->get_prefix_name(), interface_name, nullptr);
  }
  return *result;
}

template <class T>
bool are_names_identical(const std::vector<T>& handles) {
  std::vector<std::string> names;
  std::transform(handles.cbegin(), handles.cend(), std::back_inserter(names),
                 [](const auto& handle) { return handle.get_prefix_name(); });
  return std::equal(names.cbegin() + 1, names.cend(), names.cbegin());
}

inline void RcfSimpleTransmission::configure(
    const std::vector<transmission_interface::JointHandle>& joint_handles,
    const std::vector<transmission_interface::ActuatorHandle>& actuator_handles) {
  if (joint_handles.empty()) {
    throw transmission_interface::Exception("No joint handles were passed in");
  }

  if (actuator_handles.empty()) {
    throw transmission_interface::Exception("No actuator handles were passed in");
  }

  if (!are_names_identical(joint_handles)) {
    throw transmission_interface::Exception("Joint names given to transmissions should be identical");
  }

  if (!are_names_identical(actuator_handles)) {
    throw transmission_interface::Exception("Actuator names given to transmissions should be identical");
  }

  state_joint_position_ = get_by_interface(joint_handles, "position");
  state_joint_velocity_ = get_by_interface(joint_handles, "velocity");
  state_joint_effort_ = get_by_interface(joint_handles, "effort");
  cmd_joint_control_mode_ = get_by_interface(joint_handles, "control_mode");
  cmd_joint_position_ = get_by_interface(joint_handles, "cmd_position");
  cmd_joint_velocity_ = get_by_interface(joint_handles, "cmd_velocity");
  cmd_joint_effort_ = get_by_interface(joint_handles, "cmd_effort");
  cmd_joint_v1_ = get_by_interface(joint_handles, "cmd_v1");
  cmd_joint_v2_ = get_by_interface(joint_handles, "cmd_v2");
  cmd_joint_v3_ = get_by_interface(joint_handles, "cmd_v3");

  if (!state_joint_position_ && !state_joint_velocity_ && !state_joint_effort_ && !cmd_joint_control_mode_ &&
      !cmd_joint_position_ && !cmd_joint_velocity_ && !cmd_joint_effort_ && !cmd_joint_v1_ && !cmd_joint_v2_ &&
      !cmd_joint_v3_) {
    throw transmission_interface::Exception(
        "None of the provided joint handles are valid or from the required interfaces");
  }

  state_actuator_position_ = get_by_interface(actuator_handles, "position");
  state_actuator_velocity_ = get_by_interface(actuator_handles, "velocity");
  state_actuator_effort_ = get_by_interface(actuator_handles, "effort");
  cmd_actuator_control_mode_ = get_by_interface(actuator_handles, "control_mode");
  cmd_actuator_position_ = get_by_interface(actuator_handles, "cmd_position");
  cmd_actuator_velocity_ = get_by_interface(actuator_handles, "cmd_velocity");
  cmd_actuator_effort_ = get_by_interface(actuator_handles, "cmd_effort");
  cmd_actuator_v1_ = get_by_interface(actuator_handles, "cmd_v1");
  cmd_actuator_v2_ = get_by_interface(actuator_handles, "cmd_v2");
  cmd_actuator_v3_ = get_by_interface(actuator_handles, "cmd_v3");

  if (!state_actuator_position_ && !state_actuator_velocity_ && !state_actuator_effort_ &&
      !cmd_actuator_control_mode_ && !cmd_actuator_position_ && !cmd_actuator_velocity_ && !cmd_actuator_effort_ &&
      !cmd_actuator_v1_ && !cmd_actuator_v2_ && !cmd_actuator_v3_) {
    throw transmission_interface::Exception(
        "None of the provided actuator handles are valid or from the required interfaces");
  }
}

inline void RcfSimpleTransmission::actuator_to_joint() {
  if (state_joint_effort_ && state_actuator_effort_) {
    state_joint_effort_.set_value(state_actuator_effort_.get_value() * reduction_);
  }

  if (state_joint_velocity_ && state_actuator_velocity_) {
    state_joint_velocity_.set_value(state_actuator_velocity_.get_value() / reduction_);
  }

  if (state_joint_position_ && state_actuator_position_) {
    state_joint_position_.set_value(state_actuator_position_.get_value() / reduction_ + jnt_offset_);
  }
}

inline void RcfSimpleTransmission::joint_to_actuator() {
  if (cmd_joint_control_mode_ && cmd_actuator_control_mode_) {
    cmd_actuator_control_mode_.set_value(cmd_joint_control_mode_.get_value());
  }
  if (cmd_joint_effort_ && cmd_actuator_effort_) {
    cmd_actuator_effort_.set_value(cmd_joint_effort_.get_value() / reduction_);
  }

  if (cmd_joint_velocity_ && cmd_actuator_velocity_) {
    cmd_actuator_velocity_.set_value(cmd_joint_velocity_.get_value() * reduction_);
  }

  if (cmd_joint_position_ && cmd_actuator_position_) {
    cmd_actuator_position_.set_value((cmd_joint_position_.get_value() - jnt_offset_) * reduction_);
  }

  if (cmd_joint_v1_ && cmd_actuator_v1_) {
    cmd_actuator_v1_.set_value(cmd_joint_v1_.get_value());
  }

  if (cmd_joint_v2_ && cmd_actuator_v2_) {
    cmd_actuator_v2_.set_value(cmd_joint_v2_.get_value());
  }

  if (cmd_joint_v3_ && cmd_actuator_v3_) {
    cmd_actuator_v3_.set_value(cmd_joint_v3_.get_value());
  }
}

}  // namespace rcf_transmission_interface
