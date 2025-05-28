#include "tmc5160_driver/tmc5160_driver.hpp"
#include "tmc5160_driver/spi_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstring>
#include <vector>
#include <cstdint>

// Include TMC-API headers properly - note the order matters
extern "C" {
  #include "tmc/helpers/API_Header.h"
  #include "tmc/ic/TMC5160/TMC5160.h"
}

// Global variables needed for TMC-API integration
// Our singleton driver instance for callbacks
static tmc5160_driver::TMC5160Driver* g_driver_instance = nullptr;

// Implementation of functions required by TMC-API
extern "C" {
  // Function to get bus type for TMC5160
  TMC5160BusType tmc5160_getBusType(uint16_t icID) {
    (void)icID; // Prevent unused parameter warning
    return IC_BUS_SPI; // We're using SPI
  }
  
  // Function to get node address (for UART mode, not used in SPI)
  uint8_t tmc5160_getNodeAddress(uint16_t icID) {
    (void)icID; // Prevent unused parameter warning
    return 0;
  }

  // The SPI communication function the TMC-API will call
  void tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length) {
    (void)icID; // Prevent unused parameter warning
    if (g_driver_instance && g_driver_instance->spi_device_) {
      // Create a buffer for received data
      std::vector<uint8_t> rx_buffer(length, 0);
      g_driver_instance->spi_device_->transfer(data, rx_buffer.data(), length);
      // Copy received data back
      memcpy(data, rx_buffer.data(), length);
    }
  }

  // Add the UART stub function that the TMC-API requires
  bool tmc5160_readWriteUART(uint16_t icID, uint8_t* data, size_t writeLength, size_t readLength) {
    // Since we're only using SPI, this is just a stub
    (void)icID;
    (void)data;
    (void)writeLength;
    (void)readLength;
    return false; // Indicate failure for UART communication
  }
}

namespace tmc5160_driver {

TMC5160Driver::TMC5160Driver() : ic_id_(0) {
  // Store instance for callbacks
  g_driver_instance = this;
}

TMC5160Driver::~TMC5160Driver() {
  shutdown();
  if (g_driver_instance == this) {
    g_driver_instance = nullptr;
  }
}

bool TMC5160Driver::initialize(const std::string& spi_device, int cs_gpio_pin) {
  RCLCPP_INFO(rclcpp::get_logger("tmc5160_driver"), 
              "Initializing TMC5160 on SPI device: %s with CS GPIO: %d", 
              spi_device.c_str(), cs_gpio_pin);
  
  // Create and open SPI device with specific CS GPIO pin
  spi_device_ = std::make_unique<SPIDevice>(spi_device.c_str(), cs_gpio_pin);
  if (!spi_device_->open()) {
    RCLCPP_ERROR(rclcpp::get_logger("tmc5160_driver"), 
                "Failed to open SPI device: %s", spi_device.c_str());
    return false;
  }
  
  // Initialize TMC-API
  initTmcApi();
  
  return true;
}

void TMC5160Driver::shutdown() {
  if (spi_device_) {
    spi_device_->close();
    spi_device_.reset();
  }
}

void TMC5160Driver::initTmcApi() {
  // Call TMC-API initCache function
  tmc5160_initCache();
  
  // Set up basic motor parameters
  // These will need to be adjusted for your specific motor
  
  // Initialize the TMC5160 registers with default values
  tmc5160_writeRegister(ic_id_, TMC5160_GCONF, 0x00000004);        // Enable stealthChop
  tmc5160_writeRegister(ic_id_, TMC5160_CHOPCONF, 0x000100C3);     // Set microstep resolution to 256
  tmc5160_writeRegister(ic_id_, TMC5160_IHOLD_IRUN, 0x00071005);   // Set motor current
  tmc5160_writeRegister(ic_id_, TMC5160_TPOWERDOWN, 0x0000000A);   // Power-down delay
}

void TMC5160Driver::setTargetPosition(int32_t position) {
  // Set positioning mode and target position
  tmc5160_writeRegister(ic_id_, TMC5160_RAMPMODE, 0);             // Positioning mode
  tmc5160_writeRegister(ic_id_, TMC5160_XTARGET, position);       // Set target position
}

int32_t TMC5160Driver::getCurrentPosition() {
  // Read actual position
  return tmc5160_readRegister(ic_id_, TMC5160_XACTUAL);
}

void TMC5160Driver::setSpeed(uint32_t speed) {
  // Set velocity mode and target velocity
  tmc5160_writeRegister(ic_id_, TMC5160_RAMPMODE, 1);             // Velocity mode positive
  tmc5160_writeRegister(ic_id_, TMC5160_VMAX, speed);             // Set maximum speed
}

void TMC5160Driver::stop() {
  // Stop the motor
  tmc5160_writeRegister(ic_id_, TMC5160_VMAX, 0);                 // Set speed to 0
}

} // namespace tmc5160_driver