#ifndef OW_HARDWARE_MOTOR_SPI_COMMS_HPP
#define OW_HARDWARE_MOTOR_SPI_COMMS_HPP

#include <string>
#include <vector>
#include <cstdint>
#include "TMC5160.h" // Asegúrate de que esta ruta sea correcta para tu proyecto
// Tipos de ejemplo para TMC-API (reemplazar con los reales)
#ifndef TMC_ERROR_NONE
#define TMC_ERROR_NONE 0
#endif
#ifndef TMC_ERROR_GENERIC
#define TMC_ERROR_GENERIC 1
#endif
#ifndef TMC_ERROR_NOT_IMPLEMENTED
#define TMC_ERROR_NOT_IMPLEMENTED 7
#endif

// wiringPi.h debe incluirse antes que cualquier función de wiringPi sea llamada.
// Usualmente se incluye en el .cpp, pero si alguna constante o tipo de wiringPi
// se necesitara en el header, se pondría aquí. Para este caso, es mejor en el .cpp.

struct SPI_Config {
    std::string device_path; // e.g., "/dev/spidev0.0"
    uint32_t spi_speed_hz;
    uint8_t spi_mode; // SPI mode (0-3)
    // Los números de pin de wiringPi para los CS de cada motor
    std::vector<int> motor_cs_wiringpi_pins;
    uint8_t motor_count; // Número de motores (1-4)
};

class MotorSpiComms {
public:
    MotorSpiComms(const SPI_Config& config);
    ~MotorSpiComms();

    bool setup();
    void close_comms();

    bool initializeMotorDefaults(uint8_t motor_idx);
    bool setMotorSpeed(uint8_t motor_idx, int32_t speed);

    bool readMotorPosition(uint8_t motor_idx, int32_t& position);
    bool readMotorVelocity(uint8_t motor_idx, int32_t& velocity);

    uint8_t executeSpiTransaction(uint8_t motor_idx, uint8_t* data, size_t length);

private:
    bool configureSpi();
    bool configureGpio(); // Usará wiringPi
    void selectMotorCS(uint8_t motor_idx); // Usará wiringPi
    void deselectMotorCS(uint8_t motor_idx); // Usará wiringPi

    SPI_Config config_;
    int spi_fd_ = -1;
    bool initialized_ = false;
    // No se necesitan descriptores de gpiod_chip o gpiod_line
};

#ifdef __cplusplus
extern "C" {
#endif

TMC5160BusType tmc5160_getBusType(uint8_t icID);
uint8_t tmc5160_readWriteSPI(uint8_t icID, uint8_t *data, size_t length);
void tmc5160_readWriteUART(uint8_t icID, uint8_t *data, size_t length, uint16_t *crc);
uint8_t tmc5160_getNodeAddress(uint8_t icID);

#ifdef __cplusplus
}
#endif

#endif // OW_HARDWARE_MOTOR_SPI_COMMS_HPP