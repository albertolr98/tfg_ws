#pragma once

#include <string>
#include <memory>
#include <cstdint>

// Forward declarations
class SPIDevice;

// Forward declare the C function that needs access to our class
extern "C" void tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length);

namespace tmc5160_driver {

class TMC5160Driver {
public:
  TMC5160Driver();
  ~TMC5160Driver();
  
  // Initialize with SPI device path and CS GPIO pin
  bool initialize(const std::string& spi_device, int cs_gpio_pin);
  
  // Shut down the driver
  void shutdown();
  
  // Control methods
  void setTargetPosition(int32_t position);
  int32_t getCurrentPosition();
  void setSpeed(uint32_t speed);
  void stop();
  
  // Friend declaration for TMC API callback - give access to private members
  friend void ::tmc5160_readWriteSPI(uint16_t icID, uint8_t* data, size_t length);
  
private:
  // SPI device interface
  std::unique_ptr<SPIDevice> spi_device_;
  
  // TMC5160 IC ID (used by the TMC API)
  uint16_t ic_id_;
  
  // Initialize the TMC API
  void initTmcApi();
};

} // namespace tmc5160_driver