#include "trinamic_porting.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#define SPI_BUFFER_LENGTH 32

uint8_t tx_buffer[SPI_BUFFER_LENGTH];
uint8_t rx_buffer[SPI_BUFFER_LENGTH];
uint32_t len_data = 32;
uint32_t spi_speed = 1000000;
int fd;
int ret;
struct spi_ioc_transfer trx;
uint8_t looper;
uint32_t scratch32;


// Inicialización del SPI

void initSPI(){

    fd = open(SPI_DEVICE, O_RDWR);
    if(fd < 0) {
        printf("Could not open the SPI device...\r\n");
        exit(EXIT_FAILURE);
    }
    
    ret = ioctl(fd, SPI_IOC_RD_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not read SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    scratch32 |= SPI_MODE_0;

    ret = ioctl(fd, SPI_IOC_WR_MODE32, &scratch32);
    if(ret != 0) {
        printf("Could not write SPI mode...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not read the SPI max speed...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }

    scratch32 = 5000000;

    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &scratch32);
    if(ret != 0) {
        printf("Could not write the SPI max speed...\r\n");
        close(fd);
        exit(EXIT_FAILURE);
    }
}

// => SPI wrapper
// Send [length] bytes stored in the [data] array over SPI and overwrite [data]
// with the reply. The first byte sent/received is data[0].
void tmc2130_readWriteArray(uint8_t channel, uint8_t *data, size_t length){
    uint8_t i;


    if (length > SPI_BUFFER_LENGTH){
        printf("Size of message exceded buffer length\n");
    }

    trx.tx_buf = (unsigned long)tx_buffer;
    trx.rx_buf = (unsigned long)rx_buffer;
    trx.len = length;

    //Cargamos información a enviar por el driver
    for(i=0; i < length; i++) {
        tx_buffer[i]= data[i];
    }

    ret = ioctl(fd, SPI_IOC_MESSAGE(2), &trx);
    if(ret != 0) {
        printf("SPI transfer returned %d...\r\n", ret);
    }

    printf("Received SPI buffer...\r\n");
    for(looper=0; looper<trx.len; ++looper) {
        printf("%02x",rx_buffer[looper]);
    }
    printf("\r\n");
}
