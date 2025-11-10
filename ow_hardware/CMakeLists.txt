cmake_minimum_required(VERSION 3.8)
project(ow_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_library(WIRINGPI_LIBRARY wiringPi)
if(NOT WIRINGPI_LIBRARY)
  message(FATAL_ERROR "Librería wiringPi no encontrada. Asegúrate de que está instalada y accesible (sudo apt install wiringpi).")
else()
  message(STATUS "wiringPi encontrada: ${WIRINGPI_LIBRARY}")
endif()

include_directories(include)

set(TMC_API_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/tmc)
set(TMC5160_DIR ${TMC_API_DIR}/ic/TMC5160)
set(TMC5240_DIR ${TMC_API_DIR}/ic/TMC5240)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API
  ${TMC5160_DIR}
  ${TMC_API_DIR}/helpers
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC5160/include
)

if(EXISTS ${TMC5240_DIR})
  include_directories(${TMC5240_DIR})
endif()

set(OW_HARDWARE_SOURCES
  src/motor_spi_comms.cpp
)

set(TMC_API_SOURCES
  ${TMC5160_DIR}/TMC5160.c
)

if(EXISTS ${TMC5240_DIR}/TMC5240.c)
  list(APPEND TMC_API_SOURCES ${TMC5240_DIR}/TMC5240.c)
endif()

add_library(${PROJECT_NAME}_lib SHARED
  ${OW_HARDWARE_SOURCES}
  ${TMC_API_SOURCES}
)

target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    ${WIRINGPI_LIBRARY}
    rclcpp::rclcpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
)

ament_package()
