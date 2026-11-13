/*******************************************************************************
 * Copyright (c) 2025/04/24, Liao LunJia.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace {
std::string resolve_service_name(int argc, char const* argv[]) {
  std::string arg;
  for (int i = 1; i < argc; ++i) {
    const std::string candidate = argv[i];
    if (!candidate.empty() && candidate[0] != '-') {
      arg = candidate;
      break;
    }
  }

  if (arg.empty()) {
    RCLCPP_FATAL(rclcpp::get_logger("servo_disable"), "Usage: servo_disable <hw_name>");
    return "";
  }
  if (arg.front() == '/') {
    // Full service name, e.g. /rcf_standard_hardware/servo/enable
    return arg;
  }
  // Hardware component name, e.g. rcf_standard_hardware
  return "/" + arg + "/servo/disable";
}
}  // namespace

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);

  const std::string service_name = resolve_service_name(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("servo_enable");
  auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Calling servo enable service: %s", service_name.c_str());

  auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(logger, "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
    RCLCPP_INFO(logger, "Service not available, waiting again...");
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  const auto status = rclcpp::spin_until_future_complete(node, future, 10s);
  if (status != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to call service: %s", service_name.c_str());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(logger, "Servo enable succeeded%s%s", response->message.empty() ? "" : ": ", response->message.c_str());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
  }

  RCLCPP_ERROR(logger, "Servo enable failed%s%s", response->message.empty() ? "" : ": ", response->message.c_str());
  rclcpp::shutdown();
  return EXIT_FAILURE;
}
