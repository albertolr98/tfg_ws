#include <stdio.h>
#include "uart.h"

bool tmc5160_readWriteUART(uint16_t icID, uint8_t *data, size_t writeLength, size_t readLength){
    // UART communication code here
    printf("UART communication with IC ID: %u\n", icID);
    for (size_t i = 0; i < writeLength; i++) {
        printf("Write Data[%zu]: %02X\n", i, data[i]);
    }
    // Simulate reading data
    for (size_t i = 0; i < readLength; i++) {
        data[i] = 0xFF; // Example read data
        printf("Read Data[%zu]: %02X\n", i, data[i]);
    }
    
    return true;
}