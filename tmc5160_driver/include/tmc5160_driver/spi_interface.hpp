#pragma once

#include <cstdint>
#include <cstddef>

// Forward declare gpiod structures
struct gpiod_chip;
struct gpiod_line;

class SPIDevice {
public:
    SPIDevice(const char* device, int cs_gpio, bool cs_active_low = true);
    ~SPIDevice();
    
    bool open();
    int transfer(uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    void close();
    
private:
    // Listed in order of declaration to avoid initialization warnings
    int fd_;                // SPI file descriptor
    int cs_gpio_;           // GPIO pin number for CS
    bool cs_active_low_;    // CS active low (true) or active high (false)
    const char* device_;    // SPI device path
    
    // libgpiod handles
    struct gpiod_chip* chip_;
    struct gpiod_line* cs_line_;
    
    bool gpio_init(int pin);
    bool gpio_write(int value);
    void gpio_cleanup();
};