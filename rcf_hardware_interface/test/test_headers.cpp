#include <gtest/gtest.h>

#include "rcf_hardware_interface/joint_command_handle.hpp"
#include "rcf_hardware_interface/joint_state_handle.hpp"

TEST(RcfHardwareInterfaceHeaders, ControlModeConvertsToInterfaceValue) {
  EXPECT_EQ(rcf_hardware_interface::to_interface_value(rcf_hardware_interface::ControlMode::BEGIN), -1.0);
  EXPECT_EQ(rcf_hardware_interface::to_interface_value(rcf_hardware_interface::ControlMode::MODE_EFFORT), 0.0);
  EXPECT_EQ(rcf_hardware_interface::to_interface_value(rcf_hardware_interface::ControlMode::MODE_PVT), 3.0);
  EXPECT_TRUE(rcf_hardware_interface::is_control_mode_value(2.0, rcf_hardware_interface::ControlMode::MODE_POSITION));
  EXPECT_EQ(rcf_hardware_interface::to_control_mode(2.0), rcf_hardware_interface::ControlMode::MODE_POSITION);
  EXPECT_EQ(rcf_hardware_interface::to_control_mode(2.5), rcf_hardware_interface::ControlMode::UNKNOWN);
  EXPECT_EQ(rcf_hardware_interface::to_control_mode(999.0), rcf_hardware_interface::ControlMode::UNKNOWN);
}

TEST(RcfHardwareInterfaceHeaders, JointStateHandleExportsExpectedInterfaces) {
  rcf_hardware_interface::JointStateHandle handle("joint1");

  const auto interfaces = handle.get_state_interface_names();

  EXPECT_EQ(interfaces.size(), 3u);
  EXPECT_EQ(interfaces[0], "joint1/position");
  EXPECT_EQ(interfaces[1], "joint1/velocity");
  EXPECT_EQ(interfaces[2], "joint1/effort");
}

TEST(RcfHardwareInterfaceHeaders, JointCommandHandleExportsExpectedInterfaces) {
  rcf_hardware_interface::JointCommandHandle handle("joint1");

  const auto interfaces = handle.get_command_interface_names();

  EXPECT_EQ(interfaces.size(), 7u);
  EXPECT_EQ(interfaces[0], "joint1/control_mode");
  EXPECT_EQ(interfaces[1], "joint1/cmd_position");
  EXPECT_EQ(interfaces[2], "joint1/cmd_velocity");
  EXPECT_EQ(interfaces[3], "joint1/cmd_effort");
  EXPECT_EQ(interfaces[4], "joint1/cmd_v1");
  EXPECT_EQ(interfaces[5], "joint1/cmd_v2");
  EXPECT_EQ(interfaces[6], "joint1/cmd_v3");
}
