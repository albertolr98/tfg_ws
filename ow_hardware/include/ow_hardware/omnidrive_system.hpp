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

#ifndef OW_HARDWARE_OMNIDRIVE_SYSTEM_HPP_
#define OW_HARDWARE_OMNIDRIVE_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "spi_bus.hpp"
#include "tmc5160.hpp"

namespace ow_hardware
{
class OmnidriveSystemHardware : public hardware_interface::SystemInterface
{

  static constexpr std::size_t kWheelCount = 3;

struct Config
{
  unsigned int front_wheel_cs_gpio;
  unsigned int front_wheel_en_gpio;
  unsigned int left_wheel_cs_gpio;
  unsigned int left_wheel_en_gpio;
  unsigned int right_wheel_cs_gpio;
  unsigned int right_wheel_en_gpio;

  double accel_time;

  std::string spi_device;
  int spi_speed_hz;
  // Sign multiplier for each wheel (+1 normal, -1 invert direction)
  std::array<double, kWheelCount> wheel_signs{{1.0, 1.0, 1.0}};
};


public:
  RCLCPP_SHARED_PTR_DEFINITIONS(OmnidriveSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<SPIBus> comms_;
  Config cfg_;
  std::unique_ptr<TMC5160> driver_front_wheel_;
  std::unique_ptr<TMC5160> driver_left_wheel_;
  std::unique_ptr<TMC5160> driver_right_wheel_;
  std::array<TMC5160 *, kWheelCount> driver_ptrs_{};

  // Ramp state
  std::array<double, kWheelCount> current_vels_{{0.0, 0.0, 0.0}};
  std::array<double, kWheelCount> target_vels_{{0.0, 0.0, 0.0}};
};

}  // namespace ow_hardware
#endif  // OW_HARDWARE_OMNIDRIVE_SYSTEM_HPP_
