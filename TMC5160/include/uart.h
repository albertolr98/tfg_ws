#include <stdint.h>
#include <stdbool.h>

bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength);