cmake_minimum_required(VERSION 3.8)
project(tmc5160_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/tmc/ic/TMC5160/
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/
  ${LIBGPIOD_INCLUDE_DIRS}
  include
)

# Add library target
add_library(${PROJECT_NAME}
  src/tmc5160_driver.cpp
  src/spi_interface.cpp
  # Add only TMC-API source files that exist
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/tmc/ic/TMC5160/TMC5160.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/tmc/helpers/Functions.c
  ${CMAKE_CURRENT_SOURCE_DIR}/../TMC-API/tmc/helpers/CRC.c
)

# Link against required libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)
target_link_libraries(${PROJECT_NAME}
  ${LIBGPIOD_LIBRARIES}
)

# Create the node executable
add_executable(tmc5160_node src/tmc5160_node.cpp)
target_link_libraries(tmc5160_node 
  ${PROJECT_NAME}
  ${LIBGPIOD_LIBRARIES}
)
ament_target_dependencies(tmc5160_node 
  rclcpp
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}
  tmc5160_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()