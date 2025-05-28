#include "tmc5160_driver/spi_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <cerrno>
#include <cstddef>
#include <gpiod.h>

// SPIDevice implementation
SPIDevice::SPIDevice(const char* device, int cs_gpio, bool cs_active_low)
    : fd_(-1),
      cs_gpio_(cs_gpio),
      cs_active_low_(cs_active_low),
      device_(device),
      chip_(nullptr),
      cs_line_(nullptr)
{
}

SPIDevice::~SPIDevice() {
    close();
}

bool SPIDevice::gpio_init(int pin) {
    // Find the correct chip
    // On Raspberry Pi, GPIO pins are typically on gpiochip0
    chip_ = gpiod_chip_open_by_name("gpiochip0");
    if (!chip_) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to open GPIO chip: %s", strerror(errno));
        return false;
    }
    
    // Get the GPIO line
    cs_line_ = gpiod_chip_get_line(chip_, pin);
    if (!cs_line_) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to get GPIO line: %s", strerror(errno));
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        return false;
    }
    
    // Configure the line for output with initial value (inactive state)
    int initial_value = cs_active_low_ ? 1 : 0;
    if (gpiod_line_request_output(cs_line_, "spi_cs", initial_value) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to configure GPIO as output: %s", strerror(errno));
        cs_line_ = nullptr;
        gpiod_chip_close(chip_);
        chip_ = nullptr;
        return false;
    }
    
    return true;
}

bool SPIDevice::gpio_write(int value) {
    if (!cs_line_) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "GPIO line not initialized");
        return false;
    }
    
    if (gpiod_line_set_value(cs_line_, value) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to set GPIO value: %s", strerror(errno));
        return false;
    }
    
    return true;
}

void SPIDevice::gpio_cleanup() {
    if (cs_line_) {
        gpiod_line_release(cs_line_);
        cs_line_ = nullptr;
    }
    
    if (chip_) {
        gpiod_chip_close(chip_);
        chip_ = nullptr;
    }
}

bool SPIDevice::open() {
    // Open SPI device
    fd_ = ::open(device_, O_RDWR);
    if (fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to open SPI device: %s", strerror(errno));
        return false;
    }

    // Set SPI mode (Mode 3 is common for TMC drivers)
    uint8_t mode = SPI_MODE_3;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to set SPI mode: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set bits per word to 8
    uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to set bits per word: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set max speed (adjust as needed)
    uint32_t speed = 1000000; // 1MHz
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "Failed to set SPI speed: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Setup GPIO CS pin
    if (!gpio_init(cs_gpio_)) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    return true;
}

int SPIDevice::transfer(uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    if (fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), "SPI device not open");
        return -1;
    }

    // Activate CS
    if (!gpio_write(cs_active_low_ ? 0 : 1)) {
        return -1;
    }

    // Perform SPI transfer
    struct spi_ioc_transfer transfer;
    memset(&transfer, 0, sizeof(transfer));  // Clear all fields
    
    transfer.tx_buf = (unsigned long)tx_buf;
    transfer.rx_buf = (unsigned long)rx_buf;
    transfer.len = len;

    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
    if (ret < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("spi_interface"), 
                    "SPI transfer failed: %s", strerror(errno));
    }

    // Deactivate CS
    gpio_write(cs_active_low_ ? 1 : 0);

    return ret;
}

void SPIDevice::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        gpio_cleanup();
    }
}