#ifndef TMC5160_HW_ABSTRACTION_H
#define TMC5160_HW_ABSTRACTION_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "tmc/ic/TMC5160/TMC5160.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  IC_BUS_SPI = 0,
  IC_BUS_UART = 1,
  IC_BUS_WLAN = 2,
} TMC5160BusType;

// SPI/UART hooks provided by the application layer.
void tmc5160_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength);
bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength);

// Optional hooks queried by the TMC-API.
TMC5160BusType tmc5160_getBusType(uint16_t icID);
uint8_t tmc5160_getNodeAddress(uint16_t icID);

// Compatibility helpers expected by existing code.
void tmc5160_initCache(void);
int32_t tmc5160_readRegister(uint16_t icID, uint8_t address);
void tmc5160_writeRegister(uint16_t icID, uint8_t address, int32_t value);
void tmc5160_readWriteArray(uint8_t channel, uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif  // TMC5160_HW_ABSTRACTION_H
