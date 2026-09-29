// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Regression test for the deprecated-Handle-API migration: the exported state/command
// interface *names* must exactly match what the old raw-pointer export_state_interfaces()/
// export_command_interfaces() used to export, one-for-one. Nothing here exercises hardware or
// the network SDK - only on_init() + export via the hardware_interface::System wrapper (which
// runs the same old-export-empty -> on_export_state_interfaces() fallback the real
// resource_manager uses).

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "kuka_drivers_core/hardware_event.hpp"
#include "kuka_drivers_core/hardware_interface_types.hpp"
#include "kuka_rsi_driver/hardware_interface_eki_rsi.hpp"
#include "kuka_rsi_driver/hardware_interface_mxa_rsi.hpp"
#include "kuka_rsi_driver/hardware_interface_rsi_only.hpp"

namespace
{
constexpr size_t kJointCount = 6;
constexpr const char * kInterfacePrefix = "test/";

hardware_interface::HardwareInfo build_test_hardware_info()
{
  hardware_interface::HardwareInfo info;
  // Required for HardwareComponentInterface::on_init() to parse info.joints/info.gpios into its
  // internal interface maps at all - without this, on_init() still succeeds, but no interfaces
  // beyond the "unlisted" ones would be auto-exported.
  info.type = "system";
  info.hardware_parameters["controller_ip"] = "127.0.0.1";
  info.hardware_parameters["client_ip"] = "127.0.0.1";
  info.hardware_parameters["client_port"] = "59152";
  info.hardware_parameters["mxa_client_port"] = "59151";
  // Fixed, so the expected unlisted interface names below don't depend on HardwareInfo.name.
  info.hardware_parameters["interface_prefix"] = kInterfacePrefix;

  auto make_interface = [](const std::string & name)
  {
    hardware_interface::InterfaceInfo iface{};
    iface.name = name;
    return iface;
  };

  for (size_t i = 1; i <= kJointCount; ++i)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i);
    joint.command_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION),
      make_interface(hardware_interface::HW_IF_VELOCITY),
      make_interface(hardware_interface::HW_IF_EFFORT)};
    joint.state_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION),
      make_interface(hardware_interface::HW_IF_VELOCITY),
      make_interface(hardware_interface::HW_IF_EFFORT)};
    info.joints.push_back(joint);
  }

  hardware_interface::ComponentInfo gpio;
  gpio.name = hardware_interface::IO_PREFIX;
  gpio.state_interfaces = {make_interface("digital_input_1"), make_interface("digital_input_2")};
  gpio.command_interfaces = {make_interface("digital_output_1")};
  info.gpios.push_back(gpio);

  return info;
}

std::vector<std::string> get_names(
  const std::vector<hardware_interface::StateInterface::ConstSharedPtr> & interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto & interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

std::vector<std::string> get_names(
  const std::vector<hardware_interface::CommandInterface::SharedPtr> & interfaces)
{
  std::vector<std::string> names;
  names.reserve(interfaces.size());
  for (const auto & interface : interfaces)
  {
    names.push_back(interface->get_name());
  }
  return names;
}

// Interfaces exported by the removed raw-pointer export_state_interfaces(), common to all three
// RSI hardware interface variants:
//   per joint: <joint>/position, <joint>/velocity, <joint>/effort
//   plus:      gpio/<gpio state name>
//   plus:      test/state/server_state   (interface_prefix + STATE_PREFIX + SERVER_STATE)
std::vector<std::string> expected_base_state_names()
{
  std::vector<std::string> names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  names.push_back(std::string(hardware_interface::IO_PREFIX) + "/digital_input_1");
  names.push_back(std::string(hardware_interface::IO_PREFIX) + "/digital_input_2");
  names.push_back(
    std::string(kInterfacePrefix) + hardware_interface::STATE_PREFIX + "/" +
    hardware_interface::SERVER_STATE);
  return names;
}

std::vector<std::string> expected_base_command_names()
{
  std::vector<std::string> names;
  for (size_t i = 1; i <= kJointCount; ++i)
  {
    const std::string joint = "joint_" + std::to_string(i);
    names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
    names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  names.push_back(std::string(hardware_interface::IO_PREFIX) + "/digital_output_1");
  names.push_back(
    std::string(kInterfacePrefix) + hardware_interface::CONFIG_PREFIX + "/" +
    hardware_interface::INTERPOLATION_COUNT);
  return names;
}

// Additional interfaces exported only by the mxA/EKI variants (via StatusManager and the
// runtime_config control_mode/cycle_time command interfaces).
std::vector<std::string> expected_extended_state_names()
{
  std::vector<std::string> names = expected_base_state_names();
  const std::vector<std::string> status_names = {
    hardware_interface::CONTROL_MODE,    hardware_interface::CYCLE_TIME,
    hardware_interface::DRIVES_POWERED,  hardware_interface::EMERGENCY_STOP,
    hardware_interface::GUARD_STOP,      hardware_interface::IN_MOTION,
    hardware_interface::MOTION_POSSIBLE, hardware_interface::OPERATION_MODE,
    hardware_interface::ROBOT_STOPPED};
  for (const auto & name : status_names)
  {
    names.push_back(std::string(kInterfacePrefix) + hardware_interface::STATE_PREFIX + "/" + name);
  }
  return names;
}

std::vector<std::string> expected_extended_command_names()
{
  std::vector<std::string> names = expected_base_command_names();
  names.push_back(
    std::string(kInterfacePrefix) + hardware_interface::CONFIG_PREFIX + "/" +
    hardware_interface::CONTROL_MODE);
  names.push_back(
    std::string(kInterfacePrefix) + hardware_interface::CONFIG_PREFIX + "/" +
    hardware_interface::CYCLE_TIME);
  return names;
}

template <typename HardwareInterfaceT>
std::unique_ptr<hardware_interface::System> initialize(const rclcpp::Logger & logger)
{
  hardware_interface::HardwareComponentParams params;
  params.hardware_info = build_test_hardware_info();
  params.clock = std::make_shared<rclcpp::Clock>();
  params.logger = logger;

  auto hw = std::make_unique<hardware_interface::System>(std::make_unique<HardwareInterfaceT>());
  const auto state = hw->initialize(params);
  EXPECT_EQ(state.id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    << "on_init() did not succeed - check the test HardwareInfo matches what "
       "KukaRSIHardwareInterfaceBase::on_init() validates.";
  return hw;
}
}  // namespace

TEST(KukaRsiOnlyHardwareInterfaceExportTest, ExportsExpectedInterfaces)
{
  auto hw = initialize<kuka_rsi_driver::KukaRSIHardwareInterface>(
    rclcpp::get_logger("test_kuka_rsi_only_exports"));

  EXPECT_THAT(
    get_names(hw->export_state_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_base_state_names()));
  EXPECT_THAT(
    get_names(hw->export_command_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_base_command_names()));
}

TEST(KukaEkiRsiHardwareInterfaceExportTest, ExportsExpectedInterfaces)
{
  auto hw = initialize<kuka_rsi_driver::KukaEkiRsiHardwareInterface>(
    rclcpp::get_logger("test_kuka_eki_rsi_exports"));

  EXPECT_THAT(
    get_names(hw->export_state_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_extended_state_names()));
  EXPECT_THAT(
    get_names(hw->export_command_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_extended_command_names()));
}

TEST(KukaMxaRsiHardwareInterfaceExportTest, ExportsExpectedInterfaces)
{
  auto hw = initialize<kuka_rsi_driver::KukaMxaRsiHardwareInterface>(
    rclcpp::get_logger("test_kuka_mxa_rsi_exports"));

  EXPECT_THAT(
    get_names(hw->export_state_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_extended_state_names()));
  EXPECT_THAT(
    get_names(hw->export_command_interfaces()),
    ::testing::UnorderedElementsAreArray(expected_extended_command_names()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleMock(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
