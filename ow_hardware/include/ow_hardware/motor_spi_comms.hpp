#ifndef OW_HARDWARE_MOTOR_SPI_COMMS_HPP
#define OW_HARDWARE_MOTOR_SPI_COMMS_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "TMC5160_HW_Abstraction.h"
#include "tmc/ic/TMC5160/TMC5160.h"

// Guard the 5240 include so the package can still be built on systems that do
// not ship the newer device yet. Users that want to enable the TMC5240 simply
// have to provide the corresponding headers from the TMC-API.
#ifdef __has_include
#  if __has_include("TMC5240_HW_Abstraction.h") && __has_include("tmc/ic/TMC5240/TMC5240.h")
#    define OW_HAVE_TMC5240 1
#    include "TMC5240_HW_Abstraction.h"
#    include "tmc/ic/TMC5240/TMC5240.h"
#  endif
#endif

#ifndef OW_HAVE_TMC5240
#define OW_HAVE_TMC5240 0
#endif

#ifndef TMC_ERROR_NONE
#define TMC_ERROR_NONE 0
#endif
#ifndef TMC_ERROR_GENERIC
#define TMC_ERROR_GENERIC 1
#endif
#ifndef TMC_ERROR_NOT_IMPLEMENTED
#define TMC_ERROR_NOT_IMPLEMENTED 7
#endif

namespace ow_hardware
{

enum class TmcDriverType
{
  TMC5160,
  TMC5240
};

struct MotorChannelConfig
{
  int cs_pin = -1;
  TmcDriverType driver_type = TmcDriverType::TMC5160;
};

struct SPI_Config
{
  std::string device_path;
  uint32_t spi_speed_hz = 0;
  uint8_t spi_mode = 0;
  std::vector<MotorChannelConfig> motors;
};

class MotorSpiComms
{
public:
  explicit MotorSpiComms(const SPI_Config & config);
  ~MotorSpiComms();

  bool setup();
  void close_comms();

  bool initializeMotorDefaults(uint8_t motor_idx);
  bool setMotorSpeed(uint8_t motor_idx, int32_t speed);

  bool readMotorPosition(uint8_t motor_idx, int32_t & position);
  bool readMotorVelocity(uint8_t motor_idx, int32_t & velocity);

  uint8_t executeSpiTransaction(uint8_t motor_idx, uint8_t * data, size_t length);

  size_t motorCount() const noexcept;

private:
  bool configureSpi();
  bool configureGpio();
  void selectMotorCS(uint8_t motor_idx);
  void deselectMotorCS(uint8_t motor_idx);

  const MotorChannelConfig * channelConfig(uint8_t motor_idx) const;

  SPI_Config config_;
  int spi_fd_ = -1;
  bool initialized_ = false;
};

}  // namespace ow_hardware

#endif  // OW_HARDWARE_MOTOR_SPI_COMMS_HPP
