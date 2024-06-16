#ifndef TRINAMIC_PORTING_H_
#define TRINAMIC_PORTING_H_

#include "API_Header.h"

#define SPI_DEVICE "/dev/spidev0.0"

void initSPI();

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the reply. The first byte sent/received is data[0].
void tmc2130_readWriteArray(uint8_t channel, uint8_t *data, size_t length);
// <= SPI wrapper

#endif /* TRINAMIC_PORTING_H_ */