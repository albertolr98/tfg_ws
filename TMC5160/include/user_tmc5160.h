#include "TMC5160_HW_Abstraction.h"
#include "TMC5160.h"

TMC5160BusType bus = IC_BUS_SPI;

uint8_t nodeAddress = 0x00; // TMC5160 address

TMC5160BusType tmc5160_getBusType(uint16_t icID);
uint8_t tmc5160_getNodeAddress(uint16_t icID);

void enable_power();
void tmc5160_init(uint16_t icID);