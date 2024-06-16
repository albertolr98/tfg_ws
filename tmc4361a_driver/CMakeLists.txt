cmake_minimum_required(VERSION 3.8)
project(tmc4361a_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_subdirectory(tmc_lib/)



add_executable(tmc_ros2_driver tmc2130_ros2_driver.cpp)
ament_target_dependencies(tmc_ros2_driver rclcpp)
target_link_libraries(tmc_ros2_driver tmc_lib) 


install(
  TARGETS tmc_ros2_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

