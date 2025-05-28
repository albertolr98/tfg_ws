#include "TMC5160_HW_Abstraction.h"
#include "TMC5160.h"
#include <linux/spi/spidev.h>
#include <fcntl.h> // For open functions

// Pines CS para libreria WiringPi
#define SPI_CS0 5
#define SPI_CS1 22
#define SPI_CS2 27
#define SPI_CS3 17

//Funcion TMC
void tmc5160_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength);

//Funciones propias
int setup_spi();
void close_spi();