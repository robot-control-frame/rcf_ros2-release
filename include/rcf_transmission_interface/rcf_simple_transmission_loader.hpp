/*******************************************************************************
 * Copyright (c) 2026/04/21, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#pragma once

#include <memory>

#include "transmission_interface/transmission_loader.hpp"
namespace rcf_transmission_interface {

class RcfSimpleTransmissionLoader : public transmission_interface::TransmissionLoader {
 public:
  std::shared_ptr<transmission_interface::Transmission> load(
      const hardware_interface::TransmissionInfo& transmission_info) override;
};

}  // namespace rcf_transmission_interface
