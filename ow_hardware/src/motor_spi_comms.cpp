#include "ow_hardware/motor_spi_comms.hpp"
#include "TMC5160.h" // From TMC-API
#include <wiringPi.h>
#include <iostream>

// Required for open, O_RDWR
#include <fcntl.h>
// Required for close, read, write (though read/write not directly used here, good practice for SPI)
#include <unistd.h>
// Required for ioctl
#include <sys/ioctl.h>
// Required for SPI_IOC_WR_MODE, SPI_IOC_WR_MAX_SPEED_HZ, SPI_IOC_MESSAGE, struct spi_ioc_transfer
#include <linux/spi/spidev.h>


// Static pointer to the instance for C callbacks
static MotorSpiComms* g_motor_comms_instance = nullptr;

MotorSpiComms::MotorSpiComms(const SPI_Config& config) : config_(config), initialized_(false) {
    g_motor_comms_instance = this;
}

MotorSpiComms::~MotorSpiComms() {
    close_comms();
    g_motor_comms_instance = nullptr;
}

bool MotorSpiComms::setup() {
    if (initialized_) {
        return true;
    }
    if (wiringPiSetup() == -1) {
        std::cerr << "Error: Failed to initialize wiringPi." << std::endl;
        return false;
    }

    if (!configureSpi()) {
        std::cerr << "Error: Failed to configure SPI." << std::endl;
        return false;
    }

    if (!configureGpio()) {
        std::cerr << "Error: Failed to configure GPIO for CS pins." << std::endl;
        if (spi_fd_ != -1) {
            ::close(spi_fd_);
            spi_fd_ = -1;
        }
        return false;
    }

    initialized_ = true;
    return true;
}

void MotorSpiComms::close_comms() {
    if (!initialized_) {
        return;
    }
    if (spi_fd_ != -1) {
        ::close(spi_fd_);
        spi_fd_ = -1;
    }
    for (int pin : config_.motor_cs_wiringpi_pins) {
        if (pin >= 0) {
             pinMode(pin, INPUT);
        }
    }
    initialized_ = false;
}

bool MotorSpiComms::configureSpi() {
    spi_fd_ = ::open(config_.device_path.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        std::cerr << "Error: Can't open SPI device " << config_.device_path << std::endl;
        return false;
    }

    int ret;
    ret = ::ioctl(spi_fd_, SPI_IOC_WR_MODE, &config_.spi_mode);
    if (ret == -1) {
        std::cerr << "Error: Can't set SPI mode." << std::endl;
        ::close(spi_fd_);
        spi_fd_ = -1;
        return false;
    }
    ret = ::ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &config_.spi_speed_hz);
    if (ret == -1) {
        std::cerr << "Error: Can't set SPI speed." << std::endl;
        ::close(spi_fd_);
        spi_fd_ = -1;
        return false;
    }
    return true;
}

bool MotorSpiComms::configureGpio() {
    if (config_.motor_cs_wiringpi_pins.size() != config_.motor_count) {
        std::cerr << "Error: Mismatch between motor_cs_wiringpi_pins size and motor_count." << std::endl;
        return false;
    }
    for (uint8_t i = 0; i < config_.motor_count; ++i) {
        int pin = config_.motor_cs_wiringpi_pins[i];
        if (pin < 0) { 
            std::cerr << "Warning: Invalid pin number for motor CS " << static_cast<int>(i) << std::endl;
            continue; 
        }
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH); 
    }
    return true;
}

void MotorSpiComms::selectMotorCS(uint8_t motor_idx) {
    if (motor_idx < config_.motor_count) {
        int pin = config_.motor_cs_wiringpi_pins[motor_idx];
        if (pin >= 0) {
            digitalWrite(pin, LOW); 
        }
    }
}

void MotorSpiComms::deselectMotorCS(uint8_t motor_idx) {
    if (motor_idx < config_.motor_count) {
        int pin = config_.motor_cs_wiringpi_pins[motor_idx];
        if (pin >= 0) {
            digitalWrite(pin, HIGH); 
        }
    }
}

uint8_t MotorSpiComms::executeSpiTransaction(uint8_t motor_idx, uint8_t* data, size_t length) {
    if (!initialized_ || spi_fd_ < 0) {
        return TMC_ERROR_GENERIC; 
    }
    if (motor_idx >= config_.motor_count) {
        return TMC_ERROR_GENERIC; 
    }

    selectMotorCS(motor_idx);

    struct spi_ioc_transfer tr;
    // Initialize struct members directly for C++ standards before C++20
    // or to avoid -Wpedantic warnings with designated initializers.
    tr.tx_buf = (unsigned long)data;
    tr.rx_buf = (unsigned long)data;
    tr.len = (uint32_t)length;
    tr.speed_hz = config_.spi_speed_hz;
    tr.delay_usecs = 0;
    tr.bits_per_word = 8;
    tr.cs_change = 0; // Explicitly set, though often defaults to 0
    tr.tx_nbits = 0;  // Explicitly set
    tr.rx_nbits = 0;  // Explicitly set
    tr.pad = 0;       // Explicitly set


    if (::ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
        std::cerr << "Error: SPI transaction failed for motor " << static_cast<int>(motor_idx) << std::endl;
        deselectMotorCS(motor_idx);
        return TMC_ERROR_GENERIC; 
    }

    deselectMotorCS(motor_idx);
    return TMC_ERROR_NONE;
}


bool MotorSpiComms::setMotorSpeed(uint8_t motor_idx, int32_t speed) {
    if (!initialized_ || motor_idx >= config_.motor_count) {
        return false;
    }
    // Assuming tmc5160_writeRegister returns a status (e.g., uint8_t)
    // that can be compared with TMC_ERROR_NONE.
    // If tmc5160_writeRegister is truly void in your TMC5160.h, this check is invalid.
    // You'll need to verify the signature in your specific TMC5160.h.
    ::tmc5160_writeRegister(motor_idx, TMC5160_VMAX, speed);
    return true;
}

// Implement other motor control and read functions similarly,
// checking the return status of TMC-API functions if they provide one.

bool MotorSpiComms::initializeMotorDefaults(uint8_t motor_idx) {
    if (!initialized_ || motor_idx >= config_.motor_count) {
        return false;
    }
    // Example: Initialize GCONF register (refer to TMC5160 datasheet for values)
    // This is a placeholder, actual initialization will be more complex.
    ::tmc5160_writeRegister(motor_idx, TMC5160_GCONF, 0x0000000C); // Example: Enable UART, internal RSense

    return true;
}

bool MotorSpiComms::readMotorPosition(uint8_t motor_idx, int32_t& position) {
    if (!initialized_ || motor_idx >= config_.motor_count) {
        return false;
    }
    position = ::tmc5160_readRegister(motor_idx, TMC5160_XACTUAL);
    return true;
}

bool MotorSpiComms::readMotorVelocity(uint8_t motor_idx, int32_t& velocity) {
    if (!initialized_ || motor_idx >= config_.motor_count) {
        return false;
    }
    velocity = ::tmc5160_readRegister(motor_idx, TMC5160_VACTUAL);
    return true;
}


extern "C" {
    TMC5160BusType tmc5160_getBusType(uint8_t icID) {
        (void)icID; 
        // Assuming IC_BUS_SPI is the correct enum member from your TMC5160.h or a related header
        // If not, replace with the actual enum member for SPI (e.g., TMC_BUS_SPI)
        return IC_BUS_SPI; 
    }

    uint8_t tmc5160_readWriteSPI(uint8_t icID, uint8_t *data, size_t length) {
        if (g_motor_comms_instance) {
            return g_motor_comms_instance->executeSpiTransaction(icID, data, length);
        }
        return TMC_ERROR_GENERIC; 
    }

    void tmc5160_readWriteUART(uint8_t icID, uint8_t *data, size_t length, uint16_t *crc) {
        (void)icID;
        (void)data;
        (void)length;
        (void)crc;
    }

    uint8_t tmc5160_getNodeAddress(uint8_t icID) {
        (void)icID;
        return 0xFF; 
    }
}