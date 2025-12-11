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
  // Primero llamamos al init del padre para que gestione lo básico
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Verifico que la configuración del URDF (ros2_control.xacro) sea la correcta para mi robot
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

  // Leo los parámetros de hardware del URDF (pines GPIO, bus SPI, etc.)
  try
  {
    cfg_.front_wheel_cs_gpio = std::stoi(info_.hardware_parameters.at("front_wheel_cs_gpio"));
    cfg_.front_wheel_en_gpio = std::stoi(info_.hardware_parameters.at("front_wheel_en_gpio"));
    cfg_.left_wheel_cs_gpio = std::stoi(info_.hardware_parameters.at("left_wheel_cs_gpio"));
    cfg_.left_wheel_en_gpio = std::stoi(info_.hardware_parameters.at("left_wheel_en_gpio"));
    cfg_.right_wheel_cs_gpio = std::stoi(info_.hardware_parameters.at("right_wheel_cs_gpio"));
    cfg_.right_wheel_en_gpio = std::stoi(info_.hardware_parameters.at("right_wheel_en_gpio"));

    cfg_.spi_device = std::string(info_.hardware_parameters.at("spi_device"));
    cfg_.spi_speed_hz = std::stoi(info_.hardware_parameters.at("spi_speed_hz"));

    cfg_.accel_time = std::stod(info_.hardware_parameters.at("accel_time"));

    // Función que permite saber si la rueda tiene el giro invertido
    const auto invert_flag = [&](const std::string & key) {
      auto it = info_.hardware_parameters.find(key);
      if (it == info_.hardware_parameters.end()) return false;
      const auto & v = it->second;
      return (v == "true" || v == "True" || v == "1");
    };

    // Configuro los signos de cada rueda según el parámetro
    cfg_.wheel_signs[0] = invert_flag("front_wheel_inverted") ? -1.0 : 1.0;
    cfg_.wheel_signs[1] = invert_flag("left_wheel_inverted") ? -1.0 : 1.0;
    cfg_.wheel_signs[2] = invert_flag("right_wheel_inverted") ? -1.0 : 1.0;
  }
  catch(const std::exception& e)
  {
    std::cerr << "Error leyendo parámetros: " << e.what() << '\n';
    return hardware_interface::CallbackReturn::ERROR;
  }

  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Aquí podría resetear interfaces o configuraciones extra si hiciera falta en el futuro.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activando OmnidriveSystemHardware...");

  // 1. Instancio e inicializo el Bus SPI. Necesita estar listo antes que los drivers.
  comms_ = std::make_unique<SPIBus>();
  if (!comms_->init(cfg_.spi_device.c_str(), static_cast<uint32_t>(cfg_.spi_speed_hz))) {
      RCLCPP_ERROR(get_logger(), "Fallo al inicializar SPIBus en %s", cfg_.spi_device.c_str());
      return hardware_interface::CallbackReturn::ERROR;
  }

  std::cout<<"Pines de CS configurados: "
           <<cfg_.front_wheel_cs_gpio<<", "
           <<cfg_.left_wheel_cs_gpio<<", "
           <<cfg_.right_wheel_cs_gpio<<std::endl;

  // 2. Ahora sí, instancio los drivers TMC5160.
  // Al construirse ahora, registrarán sus pines CS en el bus ya inicializado.
  // OJO: Si el constructor admite ID, pásalo aquí. Si no, asumo que lo gestiona internamente.
  driver_front_wheel_ = std::make_unique<TMC5160>(
    cfg_.front_wheel_cs_gpio, cfg_.front_wheel_en_gpio);
  driver_left_wheel_ = std::make_unique<TMC5160>(
    cfg_.left_wheel_cs_gpio, cfg_.left_wheel_en_gpio);
  driver_right_wheel_ = std::make_unique<TMC5160>(
    cfg_.right_wheel_cs_gpio, cfg_.right_wheel_en_gpio);

  // 3. Actualizo el array de punteros para usarlos en read/write
  // El orden debe coincidir con info_.joints (hardware en ros2_control.xacro): front, left, right.
  driver_ptrs_ = {{
    driver_front_wheel_.get(), driver_left_wheel_.get(), driver_right_wheel_.get()
  }};

  // 4. Inicializo los drivers (registran CS en el bus)
  if (!driver_front_wheel_->init()) {
    RCLCPP_ERROR(get_logger(), "Fallo al inicializar el driver de la rueda frontal");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!driver_left_wheel_->init()) {
    RCLCPP_ERROR(get_logger(), "Fallo al inicializar el driver de la rueda izquierda");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!driver_right_wheel_->init()) {
    RCLCPP_ERROR(get_logger(), "Fallo al inicializar el driver de la rueda derecha");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 5. Aseguro que arrancan parados tras una reactivación
  driver_front_wheel_->setSpeed(0.0f);
  driver_left_wheel_->setSpeed(0.0f);
  driver_right_wheel_->setSpeed(0.0f);

  // Reset ramp state
  current_vels_.fill(0.0);
  target_vels_.fill(0.0);

  // 5. Verifico que puedo hablar con cada driver (requiere CS ya registrado)
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
  
  RCLCPP_INFO(get_logger(), "Sistema activado y motores listos.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmnidriveSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Desactivando OmnidriveSystemHardware...");

  // Apago los motores por seguridad
  if (driver_front_wheel_) driver_front_wheel_->shutdown();
  if (driver_left_wheel_) driver_left_wheel_->shutdown();
  if (driver_right_wheel_) driver_right_wheel_->shutdown();  
  
  // Cierro la comunicación SPI
  if (comms_) comms_->close();

  // Libero la memoria para asegurar una reinicialización limpia la próxima vez
  driver_front_wheel_.reset();
  driver_left_wheel_.reset();
  driver_right_wheel_.reset();
  comms_.reset();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type OmnidriveSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Leo el estado de las ruedas (posición y velocidad) y lo publico en las interfaces
  const auto & drivers = driver_ptrs_;

  // Comprobación de seguridad por si acaso
  if (info_.joints.size() != drivers.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "Joint count (%zu) does not match driver count (%zu).",
      info_.joints.size(), drivers.size());
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t idx = 0; idx < drivers.size(); ++idx)
  {
    auto * driver = drivers[idx];
    // Ojo: si el driver es null (algo falló en activate), no puedo leer
    if (driver == nullptr) {
        continue; 
    }

    const auto & joint = info_.joints[idx];
    const double sign = cfg_.wheel_signs[idx];

    try
    {
      // Nota: readPosition/Speed devuelve int/float, hago cast a double para ROS
      const double position = static_cast<double>(driver->readPosition()) * sign;
      const double velocity = static_cast<double>(driver->readSpeed()) * sign;

      // std::cout<<"Rueda "<<joint.name<<": Posición leída = "<<position
      //     <<", Velocidad leída = "<<velocity<<std::endl;

      set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, position);
      set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to read state for joint '%s': %s", joint.name.c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ow_hardware::OmnidriveSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Envío los comandos de velocidad (rad/s) a los drivers
  const auto & drivers = driver_ptrs_;

  if (info_.joints.size() != drivers.size())
  {
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  
  // Parametros de aceleracion
  // Asumimos 5.0 rad/s como velocidad maxima de referencia
  const double max_velocity = 5.0; 
  // Evitar division por cero
  const double accel_time = (cfg_.accel_time > 1e-3) ? cfg_.accel_time : 1e-3;
  const double max_accel = max_velocity / accel_time;

  // Paso 1: Leer todos los comandos y calcular el tiempo requerido mas alto
  double max_req_time = 0.0;

  for (std::size_t idx = 0; idx < drivers.size(); ++idx)
  {
    if (drivers[idx] == nullptr) continue;

    const auto & joint = info_.joints[idx];
    try
    {
      double cmd = get_command<double>(joint.name + "/" + hardware_interface::HW_IF_VELOCITY);
      if (std::isnan(cmd)) {
        cmd = 0.0;
      }
      target_vels_[idx] = cmd;
      
      if (std::isnan(current_vels_[idx])) {
        current_vels_[idx] = 0.0;
      }

      double delta_v = std::abs(target_vels_[idx] - current_vels_[idx]);
      double time_needed = delta_v / max_accel;
      
      if (time_needed > max_req_time) {
        max_req_time = time_needed;
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to get command for joint '%s': %s", joint.name.c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  // Paso 2: Calcular el factor de escala para coordinar las ruedas
  // Si el tiempo requerido es mayor que dt, escalamos para que todas tarden 'max_req_time'
  // Esto preserva la direccion del vector de movimiento.
  double scale = 1.0;
  if (max_req_time > dt) {
    scale = dt / max_req_time;
  }

  // Paso 3: Aplicar velocidades
  for (std::size_t idx = 0; idx < drivers.size(); ++idx)
  {
    auto * driver = drivers[idx];
    if (driver == nullptr) continue;

    const double sign = cfg_.wheel_signs[idx];

    // Avanzamos current_vel hacia target_vel usando el factor de escala
    double diff = target_vels_[idx] - current_vels_[idx];
    current_vels_[idx] += diff * scale;

    // Debug print every 1 second (only once per loop, using idx==0 to trigger)
    if (idx == drivers.size() - 1) { 
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
            "dt:%.3f sc:%.2f | T:[%.2f, %.2f, %.2f] C:[%.2f, %.2f, %.2f]",
            dt, scale, 
            target_vels_[0], target_vels_[1], target_vels_[2],
            current_vels_[0], current_vels_[1], current_vels_[2]);
    }

    try
    {
      driver->setSpeed(static_cast<float>(current_vels_[idx] * sign));
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Failed to write command for joint '%s': %s", info_.joints[idx].name.c_str(), e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ow_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ow_hardware::OmnidriveSystemHardware, hardware_interface::SystemInterface)
