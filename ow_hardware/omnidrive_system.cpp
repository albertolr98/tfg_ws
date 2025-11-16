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

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Omnidrive tiene 1 interfaz de comando (velocidad) y 2 de estado (posición y velocidad)
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  try
  {
  cfg_.front_wheel_cs_gpio = std::stoi(
    info_.hardware_parameters.at("front_wheel_cs_gpio"));
  cfg_.front_wheel_en_gpio = std::stoi(
    info_.hardware_parameters.at("front_wheel_en_gpio"));
  cfg_.left_wheel_cs_gpio = std::stoi(
    info_.hardware_parameters.at("left_wheel_cs_gpio"));
  cfg_.left_wheel_en_gpio = std::stoi(
    info_.hardware_parameters.at("left_wheel_en_gpio"));
  cfg_.right_wheel_cs_gpio = std::stoi(
    info_.hardware_parameters.at("right_wheel_cs_gpio"));
  cfg_.right_wheel_en_gpio = std::stoi(
    info_.hardware_parameters.at("right_wheel_en_gpio"));

  cfg_.spi_device = std::string(
    info_.hardware_parameters.at("spi_device"));
  cfg_.spi_speed_hz = std::stoi(
    info_.hardware_parameters.at("spi_speed_hz"));
    
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Instanciar la comunicación SPI
  comms_ = std::make_unique<SPIBus>();

  // Inicializar los drivers de los motores con los pines configurados
  driver_front_wheel_ = std::make_unique<TMC5160>(
    cfg_.front_wheel_cs_gpio, cfg_.front_wheel_en_gpio);
  driver_left_wheel_ = std::make_unique<TMC5160>(
    cfg_.left_wheel_cs_gpio, cfg_.left_wheel_en_gpio);
  driver_right_wheel_ = std::make_unique<TMC5160>(
    cfg_.right_wheel_cs_gpio, cfg_.right_wheel_en_gpio);
  driver_ptrs_ = {{
    driver_front_wheel_.get(), driver_left_wheel_.get(), driver_right_wheel_.get()}};


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


  // Iniciamos la comunicación SPI
  comms_->init(cfg_.spi_device.c_str(), static_cast<uint32_t>(cfg_.spi_speed_hz));

  // Check de comunicación con los drivers
  if (!driver_front_wheel_->checkComms("Front Wheel")) {
    RCLCPP_ERROR(get_logger(), "Fallo de comunicación con el driver de la rueda frontal");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!driver_left_wheel_->checkComms("Left Wheel")) {
    RCLCPP_ERROR(get_logger(), "Fallo de comunicación con el driver de la rueda izquierda");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!driver_right_wheel_->checkComms("Right Wheel")) {
    RCLCPP_ERROR(get_logger(), "Fallo de comunicación con el driver de la rueda derecha");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Habilitamos los drivers
  driver_front_wheel_->init();
  driver_left_wheel_->init();
  driver_right_wheel_->init();
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Deshabilitamos los drivers
  driver_front_wheel_->shutdown();
  driver_left_wheel_->shutdown();
  driver_right_wheel_->shutdown();  
  
  comms_->close();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmnidriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read wheel states and publish them in joint-state interfaces (radians / rad*s^-1).
  const auto & drivers = driver_ptrs_;

  if (info_.joints.size() != drivers.size())
  {
    RCLCPP_ERROR(
      get_logger(), "Joint count (%zu) does not match driver count (%zu).",
      info_.joints.size(), drivers.size());
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t idx = 0; idx < drivers.size(); ++idx)
  {
    auto * driver = drivers[idx];
    const auto & joint = info_.joints[idx];

    if (driver == nullptr)
    {
      RCLCPP_ERROR(get_logger(), "Driver for joint '%s' not initialized.", joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }

    try
    {
      const double position = static_cast<double>(driver->readPosition(0));
      const double velocity = static_cast<double>(driver->readSpeed(0));

      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, position);
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to read state for joint '%s': %s", joint.name.c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ow_hardware ::OmnidriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(alr): Iterate over joint_command_interfaces_, translate the command to motor setpoints,
  // and push them to driver_ (speed/position depending on your control mode).

  // WRITE_DRIVER_VALUES


  return hardware_interface::return_type::OK;
}

}  // namespace ow_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ow_hardware::OmnidriveSystemHardware, hardware_interface::SystemInterface)
