cmake_minimum_required(VERSION 3.8)
project(ow_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Encuentra la librería wiringPi
# Esto busca libwiringPi.so en las rutas estándar del sistema.
find_library(WIRINGPI_LIBRARY wiringPi)
if(NOT WIRINGPI_LIBRARY)
  message(FATAL_ERROR "Librería wiringPi no encontrada. Asegúrate de que está instalada y accesible (sudo apt install wiringpi).")
else()
  message(STATUS "wiringPi encontrada: ${WIRINGPI_LIBRARY}")
endif()

# --- 2. Definir Directorios de Inclusión ---
# Añade el directorio 'include' de tu paquete para tus propios headers.
include_directories(include)

# Añade los directorios de inclusión de la TMC-API.
# Ajusta estas rutas si tu estructura de TMC-API es diferente.
# ${CMAKE_CURRENT_SOURCE_DIR} es /home/albertolr98/tfg_ws/src/ow_hardware
set(TMC_API_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API)
include_directories(
  ${TMC_API_DIR}/tmc/ic/TMC5160 # Para TMC5160.h, TMC5160_HW_Abstraction.h
  ${TMC_API_DIR}/tmc/helpers    # Para Functions.h, CRC.h (si los usas)
  # Añade más directorios de TMC-API si es necesario (ej. tmc/hal)
)

# --- 3. Definir Fuentes ---
# Lista los archivos fuente de tu paquete.
set(OW_HARDWARE_SOURCES
  src/motor_spi_comms.cpp
  # Añade aquí otros archivos .cpp de tu paquete si los tienes
)

# Lista los archivos fuente .c de la TMC-API que necesitas compilar.
set(TMC_API_SOURCES
  ${TMC_API_DIR}/tmc/ic/TMC5160/TMC5160.c
  # Descomenta y añade otros si los necesitas:
  # ${TMC_API_DIR}/tmc/helpers/Functions.c
  # ${TMC_API_DIR}/tmc/helpers/CRC.c
)

# --- 4. Crear la Librería (o Ejecutable) ---
# Aquí creamos una librería llamada 'ow_hardware_lib'.
# Puedes cambiar SHARED por STATIC si prefieres una librería estática.
# O usar add_executable si estás construyendo un nodo directamente.
add_library(${PROJECT_NAME}_lib SHARED
  ${OW_HARDWARE_SOURCES}
  ${TMC_API_SOURCES}
)


# --- 5. Enlazar Librerías ---
# Enlaza tu librería con wiringPi.
target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    ${WIRINGPI_LIBRARY}
    rclcpp::rclcpp # Use the exported CMake target from rclcpp
  # If you have other dependencies like hardware_interface or pluginlib,
  # you would add their exported targets here too (e.g., hardware_interface::hardware_interface)
)


# --- 6. Exportar Targets para ROS 2 (si es una librería para otros paquetes) ---
# Esto permite que otros paquetes de ROS 2 encuentren y usen tu librería.
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Exportar también los directorios de inclusión de TMC-API si son necesarios
  # por los headers públicos de tu librería:
  # $<BUILD_INTERFACE:${TMC_API_DIR}/tmc/ic/TMC5160>
  # $<INSTALL_INTERFACE:include/TMC-API/tmc/ic/TMC5160> # Necesitarías instalar los headers de TMC-API
)

# Instalar la librería
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}_lib # Nombre para la exportación
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin # Generalmente no para librerías, pero no hace daño
  INCLUDES DESTINATION include # Esto es para los headers de TU paquete
)

# Instalar los headers de tu paquete
install(
  DIRECTORY include/
  DESTINATION include
)

# --- 7. Configuración de Pruebas (sin cambios respecto a tu original) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- 8. Exportar el Paquete Ament ---
# Exporta la librería para que otros paquetes ament puedan encontrarla.
ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
# Lista las dependencias que otros paquetes necesitarán si usan tu librería.
ament_export_dependencies(
  rclcpp # Si es una dependencia
  # hardware_interface
  # pluginlib
)

ament_package()
