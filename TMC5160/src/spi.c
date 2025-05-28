#include "spi.h"
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <wiringPi.h>
#include "gpio.h" // Necesario para las macros de pines CS

// --- Variable estática para el descriptor de archivo SPI ---
// Solo visible dentro de este archivo (spi.c)
static int spi_fd = -1;

// --- Mapeo de icID a pines CS ---
// Asegúrate de que estos pines coincidan con tu hardware y definiciones en gpio.h
static const int cs_pins[] = {
    SPI_CS0, // Índice 0 -> CS0 (Pin 5)
    SPI_CS1, // Índice 1 -> CS1 (Pin 22)
    SPI_CS2, // Índice 2 -> CS2 (Pin 27)
    SPI_CS3  // Índice 3 -> CS3 (Pin 17)
};
static const size_t num_cs_pins = sizeof(cs_pins) / sizeof(cs_pins[0]);


// Ahora setup_spi inicializa la variable estática spi_fd
// Devuelve 0 en caso de éxito, -1 en caso de error
int setup_spi() {
    // Si ya está abierto, ciérralo primero (o devuelve éxito)
    if (spi_fd >= 0) {
        // Opcional: imprimir un warning o simplemente retornar éxito
        // printf("Warning: SPI already initialized.\n");
        return 0;
    }

    const char* device = "/dev/spidev0.0";
    uint8_t mode = SPI_MODE_3 | SPI_NO_CS; // SPI_MODE_3 is CPOL=1, CPHA=1
    uint8_t bits = 8;
    uint32_t speed = 100000;

    int temp_fd = open(device, O_RDWR); // Usa una variable temporal
    if (temp_fd < 0) {
        perror("Failed to open SPI device");
        spi_fd = -1; // Asegúrate de que spi_fd permanezca inválido
        return -1;
    }

    // Configurar modo, bits, velocidad...
    if (ioctl(temp_fd, SPI_IOC_WR_MODE, &mode) == -1) {
        perror("Failed to set SPI mode");
        close(temp_fd);
        spi_fd = -1;
        return -1;
    }
    if (ioctl(temp_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
        perror("Failed to set bits per word");
        close(temp_fd);
        spi_fd = -1;
        return -1;
    }
    if (ioctl(temp_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1) {
        perror("Failed to set max speed");
        close(temp_fd);
        spi_fd = -1;
        return -1;
    }

    // Asigna a la variable estática solo si todo fue exitoso
    spi_fd = temp_fd;
    printf("SPI initialized successfully (fd=%d).\n", spi_fd);
    return 0; // Éxito
}

// close_spi ahora usa la variable estática
void close_spi() {
    if (spi_fd >= 0) {
        close(spi_fd);
        printf("SPI closed (fd=%d).\n", spi_fd);
        spi_fd = -1; // Marcar como cerrado
    }
}

// tmc5160_readWriteSPI usa la variable estática spi_fd
void tmc5160_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength) {
    // Verifica si el SPI está inicializado
    if (spi_fd < 0) {
        fprintf(stderr, "Error: SPI not initialized. Call setup_spi() first.\n");
        // Opcional: llenar data con un patrón de error si es necesario
        // memset(data, 0xFF, dataLength); // Ejemplo: llenar con 0xFF
        return;
    }

    // --- Mapeo de icID a pin CS ---
    if (icID >= num_cs_pins) {
        fprintf(stderr, "Error: Invalid icID %u for CS pin mapping.\n", icID);
        return;
    }
    int cs_pin = cs_pins[icID]; // Obtiene el pin CS correcto del array

    // Estructura para la transferencia SPI
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)data, // Buffer de transmisión
        .rx_buf = (unsigned long)data, // Buffer de recepción (sobrescribe tx_buf)
        .len = (uint32_t)dataLength,   // Longitud de los datos
        .speed_hz = 1000000,           // Velocidad (puede ser 0 para usar la predeterminada)
        .delay_usecs = 0,              // Retardo después de la transferencia
        .bits_per_word = 8,            // Bits por palabra (puede ser 0 para usar predeterminado)
        // .cs_change = 0,              // Mantener CS activo (0) o desactivar (1) después
    };

    // --- Control manual del CS ---
    digitalWrite(cs_pin, LOW); // Activar CS
    // usleep(1); // Pequeño retardo si es necesario

    // Realizar la transferencia SPI usando ioctl
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI transfer failed");
        // Considera cómo manejar el error. ¿Dejar CS bajo? ¿Intentar subirlo?
    }

    // usleep(1); // Pequeño retardo si es necesario
    digitalWrite(cs_pin, HIGH); // Desactivar CS


    // --- Opcional: imprimir datos recibidos ---
    printf("SPI transfer (icID=%u): ", icID);
    for (size_t i = 0; i < dataLength; i++) {
        if (!(i % 6))
            printf("\n");
        printf("%02X ", data[i]);
    }
    printf("\n");
    // --- Fin de la función ---
    
}

