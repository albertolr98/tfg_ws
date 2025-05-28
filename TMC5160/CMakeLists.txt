# CMakeLists.txt
# Colocar en el directorio /home/albertolr98/pruebas/TMC5160/

cmake_minimum_required(VERSION 3.10) # Versión mínima de CMake

# Nombre del proyecto y lenguaje
project(TMC5160_User_Example C)

# Opcional: Establecer el estándar de C
# set(CMAKE_C_STANDARD 11)
# set(CMAKE_C_STANDARD_REQUIRED True)

# Nombre del ejecutable final
set(EXECUTABLE_NAME tmc5160)

# Lista de archivos fuente necesarios
# Asegúrate de que la ruta a TMC5160.c sea correcta
set(SOURCES
    # Archivos fuente de tu proyecto
    src/user_tmc5160.c
    src/spi.c
    src/gpio.c
    src/uart.c

    # API-TMC
    ../TMC-API/tmc/ic/TMC5160/TMC5160.c
)

# Lista de directorios de inclusión necesarios
set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR} # Directorio actual (para user_tmc5160.h)
    ./include
    ../TMC-API/tmc/ic/TMC5160   # Directorio de TMC5160.h
    # --- IMPORTANTE ---
    # Añade otros directorios de inclusión de la API si son necesarios
    # Ejemplo:
    # ../TMC-API/tmc/hal
    # ../TMC-API/tmc/helpers
)

# Crear el objetivo ejecutable
add_executable(${EXECUTABLE_NAME} ${SOURCES})

# Añadir los directorios de inclusión al objetivo
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${INCLUDE_DIRS})


# --- Enlazar librerías necesarias ---
# Busca y enlaza la librería wiringPi (equivalente a -lwiringPi)
find_library(WIRINGPI_LIBRARY wiringPi)
if(WIRINGPI_LIBRARY)
    target_link_libraries(${EXECUTABLE_NAME} PRIVATE ${WIRINGPI_LIBRARY})
    message(STATUS "Enlazando con wiringPi: ${WIRINGPI_LIBRARY}")
else()
    message(FATAL_ERROR "Librería wiringPi no encontrada. Asegúrate de que está instalada y accesible.")
endif()

# --- Configuración del Directorio de Salida ---
# Define dónde se guardará el ejecutable compilado.
# CMAKE_BINARY_DIR es el directorio donde ejecutas 'cmake' (normalmente 'build').
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Asegura que el directorio de salida exista
file(MAKE_DIRECTORY ${EXECUTABLE_OUTPUT_PATH})

# Establece la propiedad para la ubicación de salida del ejecutable
set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${EXECUTABLE_OUTPUT_PATH}
)

# Opcional: Añadir flags de compilación (ej. para warnings)
# target_compile_options(${EXECUTABLE_NAME} PRIVATE -Wall -Wextra)

# Opcional: Añadir librerías si son necesarias (ej. -lm para matemática)
# target_link_libraries(${EXECUTABLE_NAME} PRIVATE m)

# Mensaje informativo
message(STATUS "Ejecutable '${EXECUTABLE_NAME}' se generará en: ${EXECUTABLE_OUTPUT_PATH}")
