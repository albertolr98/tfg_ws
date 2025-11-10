#include "ow_hardware/motor_spi_comms.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>

#include <wiringPi.h>

namespace ow_hardware
{
namespace
{
MotorSpiComms * g_motor_comms_instance = nullptr;

bool ensureMotorIndex(const SPI_Config & config, uint8_t motor_idx)
{
  return motor_idx < config.motors.size();
}

}  // namespace

MotorSpiComms::MotorSpiComms(const SPI_Config & config)
: config_(config)
{
  g_motor_comms_instance = this;
}

MotorSpiComms::~MotorSpiComms()
{
  close_comms();
  if (g_motor_comms_instance == this) {
    g_motor_comms_instance = nullptr;
  }
}

size_t MotorSpiComms::motorCount() const noexcept
{
  return config_.motors.size();
}

const MotorChannelConfig * MotorSpiComms::channelConfig(uint8_t motor_idx) const
{
  if (!ensureMotorIndex(config_, motor_idx)) {
    std::cerr << "Error: Invalid motor index " << static_cast<int>(motor_idx) << std::endl;
    return nullptr;
  }
  return &config_.motors[motor_idx];
}

bool MotorSpiComms::setup()
{
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

void MotorSpiComms::close_comms()
{
  if (!initialized_) {
    return;
  }

  if (spi_fd_ != -1) {
    ::close(spi_fd_);
    spi_fd_ = -1;
  }

  for (const auto & motor : config_.motors) {
    if (motor.cs_pin >= 0) {
      pinMode(motor.cs_pin, INPUT);
    }
  }

  initialized_ = false;
}

bool MotorSpiComms::configureSpi()
{
  spi_fd_ = ::open(config_.device_path.c_str(), O_RDWR);
  if (spi_fd_ < 0) {
    std::cerr << "Error: Can't open SPI device " << config_.device_path << std::endl;
    return false;
  }

  if (::ioctl(spi_fd_, SPI_IOC_WR_MODE, &config_.spi_mode) == -1) {
    std::cerr << "Error: Can't set SPI mode." << std::endl;
    ::close(spi_fd_);
    spi_fd_ = -1;
    return false;
  }

  if (::ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &config_.spi_speed_hz) == -1) {
    std::cerr << "Error: Can't set SPI speed." << std::endl;
    ::close(spi_fd_);
    spi_fd_ = -1;
    return false;
  }

  return true;
}

bool MotorSpiComms::configureGpio()
{
  for (const auto & motor : config_.motors) {
    if (motor.cs_pin < 0) {
      std::cerr << "Warning: Invalid CS pin provided. Skipping." << std::endl;
      continue;
    }

    pinMode(motor.cs_pin, OUTPUT);
    digitalWrite(motor.cs_pin, HIGH);
  }

  return true;
}

void MotorSpiComms::selectMotorCS(uint8_t motor_idx)
{
  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return;
  }

  if (channel->cs_pin >= 0) {
    digitalWrite(channel->cs_pin, LOW);
  }
}

void MotorSpiComms::deselectMotorCS(uint8_t motor_idx)
{
  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return;
  }

  if (channel->cs_pin >= 0) {
    digitalWrite(channel->cs_pin, HIGH);
  }
}

uint8_t MotorSpiComms::executeSpiTransaction(uint8_t motor_idx, uint8_t * data, size_t length)
{
  if (!initialized_ || spi_fd_ < 0 || !data || length == 0) {
    return TMC_ERROR_GENERIC;
  }

  if (!ensureMotorIndex(config_, motor_idx)) {
    return TMC_ERROR_GENERIC;
  }

  selectMotorCS(motor_idx);

  struct spi_ioc_transfer transfer;
  transfer.tx_buf = reinterpret_cast<unsigned long>(data);
  transfer.rx_buf = reinterpret_cast<unsigned long>(data);
  transfer.len = static_cast<uint32_t>(length);
  transfer.speed_hz = config_.spi_speed_hz;
  transfer.delay_usecs = 0;
  transfer.bits_per_word = 8;
  transfer.cs_change = 0;
  transfer.tx_nbits = 0;
  transfer.rx_nbits = 0;
  transfer.pad = 0;

  if (::ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &transfer) < 0) {
    std::cerr << "Error: SPI transaction failed for motor " << static_cast<int>(motor_idx) << std::endl;
    deselectMotorCS(motor_idx);
    return TMC_ERROR_GENERIC;
  }

  deselectMotorCS(motor_idx);
  return TMC_ERROR_NONE;
}

bool MotorSpiComms::initializeMotorDefaults(uint8_t motor_idx)
{
  if (!initialized_) {
    return false;
  }

  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return false;
  }

  switch (channel->driver_type) {
    case TmcDriverType::TMC5160:
      tmc5160_initCache();
      tmc5160_writeRegister(motor_idx, TMC5160_GCONF, 0x0000000C);
      tmc5160_writeRegister(motor_idx, TMC5160_CHOPCONF, 0x10410150);
      return true;
#if OW_HAVE_TMC5240
    case TmcDriverType::TMC5240:
      tmc5240_initCache();
      tmc5240_writeRegister(motor_idx, TMC5240_GCONF, 0x0000000C);
      tmc5240_writeRegister(motor_idx, TMC5240_CHOPCONF, 0x10410150);
      return true;
#endif
    default:
      if (channel->driver_type == TmcDriverType::TMC5240) {
        std::cerr << "Error: TMC5240 selected but headers were not available at build time." << std::endl;
      } else {
        std::cerr << "Error: Unsupported driver type for initialization." << std::endl;
      }
      return false;
  }
}

bool MotorSpiComms::setMotorSpeed(uint8_t motor_idx, int32_t speed)
{
  if (!initialized_) {
    return false;
  }

  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return false;
  }

  switch (channel->driver_type) {
    case TmcDriverType::TMC5160:
      tmc5160_writeRegister(motor_idx, TMC5160_VMAX, speed);
      return true;
#if OW_HAVE_TMC5240
    case TmcDriverType::TMC5240:
      tmc5240_writeRegister(motor_idx, TMC5240_VMAX, speed);
      return true;
#endif
    default:
      if (channel->driver_type == TmcDriverType::TMC5240) {
        std::cerr << "Error: TMC5240 selected but headers were not available at build time." << std::endl;
      } else {
        std::cerr << "Error: Unsupported driver type for speed command." << std::endl;
      }
      return false;
  }
}

bool MotorSpiComms::readMotorPosition(uint8_t motor_idx, int32_t & position)
{
  if (!initialized_) {
    return false;
  }

  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return false;
  }

  switch (channel->driver_type) {
    case TmcDriverType::TMC5160:
      position = tmc5160_readRegister(motor_idx, TMC5160_XACTUAL);
      return true;
#if OW_HAVE_TMC5240
    case TmcDriverType::TMC5240:
      position = tmc5240_readRegister(motor_idx, TMC5240_XACTUAL);
      return true;
#endif
    default:
      if (channel->driver_type == TmcDriverType::TMC5240) {
        std::cerr << "Error: TMC5240 selected but headers were not available at build time." << std::endl;
      } else {
        std::cerr << "Error: Unsupported driver type for position read." << std::endl;
      }
      return false;
  }
}

bool MotorSpiComms::readMotorVelocity(uint8_t motor_idx, int32_t & velocity)
{
  if (!initialized_) {
    return false;
  }

  const auto * channel = channelConfig(motor_idx);
  if (!channel) {
    return false;
  }

  switch (channel->driver_type) {
    case TmcDriverType::TMC5160:
      velocity = tmc5160_readRegister(motor_idx, TMC5160_VACTUAL);
      return true;
#if OW_HAVE_TMC5240
    case TmcDriverType::TMC5240:
      velocity = tmc5240_readRegister(motor_idx, TMC5240_VACTUAL);
      return true;
#endif
    default:
      if (channel->driver_type == TmcDriverType::TMC5240) {
        std::cerr << "Error: TMC5240 selected but headers were not available at build time." << std::endl;
      } else {
        std::cerr << "Error: Unsupported driver type for velocity read." << std::endl;
      }
      return false;
  }
}

}  // namespace ow_hardware

extern "C"
{
TMC5160BusType tmc5160_getBusType(uint16_t icID)
{
  (void)icID;
  return IC_BUS_SPI;
}

void tmc5160_readWriteSPI(uint16_t icID, uint8_t * data, size_t length)
{
  if (!g_motor_comms_instance) {
    return;
  }

  (void)g_motor_comms_instance->executeSpiTransaction(static_cast<uint8_t>(icID), data, length);
}

bool tmc5160_readWriteUART(uint16_t icID, uint8_t * data, size_t writeLength, size_t readLength)
{
  (void)icID;
  (void)data;
  (void)writeLength;
  (void)readLength;
  return false;
}

uint8_t tmc5160_getNodeAddress(uint16_t icID)
{
  (void)icID;
  return 0xFF;
}

#if OW_HAVE_TMC5240
TMC5240BusType tmc5240_getBusType(uint16_t icID)
{
  (void)icID;
  return static_cast<TMC5240BusType>(0);
}

void tmc5240_readWriteSPI(uint16_t icID, uint8_t * data, size_t length)
{
  if (!g_motor_comms_instance) {
    return;
  }

  (void)g_motor_comms_instance->executeSpiTransaction(static_cast<uint8_t>(icID), data, length);
}

bool tmc5240_readWriteUART(uint16_t icID, uint8_t * data, size_t writeLength, size_t readLength)
{
  (void)icID;
  (void)data;
  (void)writeLength;
  (void)readLength;
  return false;
}

uint8_t tmc5240_getNodeAddress(uint16_t icID)
{
  (void)icID;
  return 0xFF;
}
#endif
}
