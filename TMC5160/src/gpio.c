#include "gpio.h"
#include <wiringPi.h>

int setup_gpio() {
    // // Initialize wiringPi
    // if (wiringPiSetupGpio() == -1) {
    //     return -1; // Fallo inicialización GPIO
    // }
    
    // // Definimos el modo de numeración de pines
    pinMode(SPI_CS_DRV1, OUTPUT);
    pinMode(SPI_CS_DRV2, OUTPUT);
    pinMode(SPI_CS_DRV3, OUTPUT);
    pinMode(SPI_CS_DRV4, OUTPUT);

    pinMode(EN_PWR, OUTPUT);
    pinMode(DEN_PWR, OUTPUT);

    pinMode(EN_DRV1, OUTPUT);
    pinMode(EN_DRV2, OUTPUT);
    pinMode(EN_DRV3, OUTPUT);
    pinMode(EN_DRV4, OUTPUT);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    
    // Estados iniciales
    digitalWrite(SPI_CS_DRV1, HIGH);
    digitalWrite(SPI_CS_DRV2, HIGH);
    digitalWrite(SPI_CS_DRV3, HIGH);
    digitalWrite(SPI_CS_DRV4, HIGH);

    digitalWrite(EN_PWR, LOW);
    digitalWrite(DEN_PWR, LOW);

    digitalWrite(EN_DRV1, LOW);
    digitalWrite(EN_DRV2, LOW);
    digitalWrite(EN_DRV3, LOW);
    digitalWrite(EN_DRV4, LOW);

    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_BLUE, LOW);

}