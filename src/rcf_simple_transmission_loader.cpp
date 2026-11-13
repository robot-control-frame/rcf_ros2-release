/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include "rcf_transmission_interface/rcf_simple_transmission_loader.hpp"

#include <fmt/format.h>

#include <stdexcept>

#include "hardware_interface/hardware_info.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcf_transmission_interface/rcf_simple_transmission.hpp"
#include "rclcpp/logging.hpp"

namespace rcf_transmission_interface {

std::shared_ptr<transmission_interface::Transmission> RcfSimpleTransmissionLoader::load(
    const hardware_interface::TransmissionInfo& transmission_info) {
  try {
    const auto mechanical_reduction = transmission_info.actuators.at(0).mechanical_reduction;
    const auto offset = transmission_info.joints.at(0).offset;
    std::shared_ptr<transmission_interface::Transmission> transmission(
        new RcfSimpleTransmission(mechanical_reduction, offset));
    return transmission;
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rcf_simple_transmission_loader"), "Failed to construct transmission '%s' because",
                 ex.what());
    return std::shared_ptr<transmission_interface::Transmission>();
  }
}

}  // namespace rcf_transmission_interface

PLUGINLIB_EXPORT_CLASS(rcf_transmission_interface::RcfSimpleTransmissionLoader,
                       transmission_interface::TransmissionLoader)
