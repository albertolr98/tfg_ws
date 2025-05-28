#include <stdio.h>
#include <stdlib.h>
#include "user_tmc5160.h"
#include "spi.h"
#include "gpio.h"
#include <unistd.h>
#include <wiringPi.h>




TMC5160BusType tmc5160_getBusType(uint16_t icID){
        return bus;
}

typedef enum DRIVER {
    WHEEL0 = 0,
    WHEEL1 = 1,
    WHEEL2 = 2,
} DRIVER;

uint8_t tmc5160_getNodeAddress(uint16_t icID){
    switch (icID) {
        case 0:
            return (uint8_t)(0x01);
        case 1:
            return (uint8_t)(0x02);
        case 2:
            return (uint8_t)(0x03);
        default:
            break;
    }
    return (uint8_t)(0xFF); // Invalid ID
}

void enable_power(){
    // Enable power for the TMC5160
    digitalWrite(EN_PWR, HIGH);
}

// Define motor node addresses
#define MOTOR_COUNT 3

void init_tmc5160(uint16_t nodeAddress) {
    // Reset the driver
    tmc5160_writeRegister(nodeAddress, TMC5160_GCONF, 0x00000000); // Reset to default
    tmc5160_writeRegister(nodeAddress, TMC5160_GSTAT, 0x00000007); // Clear status flags

    // Reinitialize registers
    tmc5160_writeRegister(nodeAddress, TMC5160_GCONF, 0x00000004); // Enable StealthChop
    tmc5160_writeRegister(nodeAddress, TMC5160_CHOPCONF, 0x00100C3);
    tmc5160_writeRegister(nodeAddress, TMC5160_IHOLD_IRUN, 0x00061403); // IHOLD=10, IRUN=31, IHOLDDELAY=6
    tmc5160_writeRegister(nodeAddress, TMC5160_TPOWERDOWN, 0x0000000A); // TPOWERDOWN=10
    tmc5160_writeRegister(nodeAddress, TMC5160_TPWMTHRS, 0x000001F4); // TPWM_THRS=500
    tmc5160_writeRegister(nodeAddress, TMC5160_A1, 500); // A1=1000
    tmc5160_writeRegister(nodeAddress, TMC5160_V1, 5000); // V1=50000
    tmc5160_writeRegister(nodeAddress, TMC5160_AMAX, 500); // AMAX=500
    tmc5160_writeRegister(nodeAddress, TMC5160_DMAX, 700); // DMAX=700
    tmc5160_writeRegister(nodeAddress, TMC5160_D1, 1400); // D1=1400
    tmc5160_writeRegister(nodeAddress, TMC5160_VSTOP, 10); // VSTOP=10
    tmc5160_writeRegister(nodeAddress, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS); // Velocity mode
    tmc5160_writeRegister(nodeAddress, TMC5160_VMAX, 0); // VMAX=0 (stop initially)
}

int main(int argc, char *argv[]) {
    // Variables para almacenar las velocidades de los motores
    int32_t motor_speeds[MOTOR_COUNT] = {0, 0, 0};
    
    // Procesar argumentos de línea de comandos
    if (argc != MOTOR_COUNT + 1) {
        fprintf(stderr, "Uso: %s <vel_motor1> <vel_motor2> <vel_motor3>\n", argv[0]);
        fprintf(stderr, "Donde cada velocidad es un entero entre -200000 y 200000\n"); // Actualizado para permitir valores negativos
        return -1;
    }
    
    // Convertir y validar cada argumento
    for (int i = 0; i < MOTOR_COUNT; i++) {
        char *endptr;
        int32_t speed = strtol(argv[i+1], &endptr, 10);
        
        // Verificar si la conversión fue exitosa
        if (*endptr != '\0') {
            fprintf(stderr, "Error: '%s' no es un número válido para la velocidad del motor %d\n", argv[i+1], i+1);
            return -1;
        }
        
        // Verificar si la velocidad está en un rango válido
        if (abs(speed) > 200000) {
            fprintf(stderr, "Error: La velocidad %d está fuera del rango válido (-200000 a 200000)\n", speed);
            return -1;
        }
        
        // Almacenar la velocidad validada
        motor_speeds[i] = speed;
        printf("Motor %d: velocidad = %d\n", i+1, motor_speeds[i]);
    }
    
    // Initialize GPIOs
    if (wiringPiSetupGpio() == -1) {
        fprintf(stderr, "GPIO initialization failed.\n");
        return -1;
    }
    setup_gpio();
    setup_spi();
    enable_power();

    // Initialize all motors
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t node_number = tmc5160_getNodeAddress(i);
        init_tmc5160(node_number);
        usleep(1000); // Small delay between initializations
    }

    // Enable all drivers simultaneously
    // digitalWrite(EN_DRV1, HIGH);
    // digitalWrite(EN_DRV2, HIGH);
    // digitalWrite(EN_DRV3, HIGH);
    
    // Set movement mode and speed for each motor
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t node_number = tmc5160_getNodeAddress(i);
        if (motor_speeds[i] >= 0) {
            tmc5160_writeRegister(node_number, TMC5160_RAMPMODE, TMC5160_MODE_VELPOS); // Set to velocity positive mode
        } else {
            tmc5160_writeRegister(node_number, TMC5160_RAMPMODE, TMC5160_MODE_VELNEG); // Set to velocity negative mode
        }
        tmc5160_writeRegister(node_number, TMC5160_VMAX, abs(motor_speeds[i])); // Set velocity
    }

    printf("Motors are running at steady velocity.\n");
    sleep(5); // Let the motors run for 10 seconds

    // Stop all motors
    for (int i = 0; i < MOTOR_COUNT; i++) {
        uint8_t node_number = tmc5160_getNodeAddress(i);
        tmc5160_writeRegister(node_number, TMC5160_VMAX, 0); // Stop motion
    }

    printf("Motors stopped.\n");
    sleep(1); // Let the motors stop for 1 second
    // Disable all drivers
    close_spi();
    // digitalWrite(EN_DRV1, LOW);
    // digitalWrite(EN_DRV2, LOW);
    // digitalWrite(EN_DRV3, LOW);
    // digitalWrite(EN_DRV4, LOW);

    return 0;
}