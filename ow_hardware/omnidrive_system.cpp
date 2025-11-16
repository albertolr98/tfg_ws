// Copyright 2021 ros2_control Development Team
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

#include "ow_hardware/omnidrive_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tmc5160.hpp"

namespace ow_hardware
{
hardware_interface::CallbackReturn OmnidriveSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // TODO(alr): Validate joint interfaces in info_.joints against what the controller expects
  // (e.g. omni_wheel_drive_controller should expose velocity commands and position/velocity states).

  // TODO(alr): Read driver configuration from info_.hardware_parameters (GPIO pins, wheel radius,
  // reduction, etc.) and keep them as members so you can configure TMC5160 properly.

  // TODO(alr): Instantiate driver_ with the parsed configuration and perform any hardware checks.

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(alr): Reset state/command interfaces and push configuration to the TMC driver
  // (current limits, microstepping, ramp parameters, etc.).

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(alr): Enable the power stage and make sure state == command to avoid jumps.

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(alr): Stop the motors, disable driver outputs, and put the hardware in a safe state.

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmnidriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(alr): Pull encoder/velocity data from driver_ and update the corresponding state
  // interfaces with set_state(name, value).

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ow_hardware ::OmnidriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(alr): Iterate over joint_command_interfaces_, translate the command to motor setpoints,
  // and push them to driver_ (speed/position depending on your control mode).

  return hardware_interface::return_type::OK;
}

}  // namespace ow_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ow_hardware::OmnidriveSystemHardware, hardware_interface::SystemInterface)
