cmake_minimum_required(VERSION 3.8)
project(tmc4361a_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include include src
)

add_executable(tmc_ros2_driver src/tmc2130_ros2_driver.cpp)
ament_target_dependencies(tmc_ros2_driver rclcpp) 








install(
  TARGETS tmc_ros2_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

#https://www.theconstruct.ai/how-to-create-a-ros2-c-library/
