cmake_minimum_required(VERSION 3.16)
project(ow_hardware LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ow_hardware
  SHARED
  omnidrive_system.cpp
)
target_compile_features(ow_hardware PUBLIC cxx_std_17)
target_include_directories(ow_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/ow_hardware>
)
target_link_libraries(ow_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_ow_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/ow_hardware
)

install(TARGETS ow_hardware
  EXPORT export_ow_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES ros2_control_ow_hardware.xml
  DESTINATION share/ow_hardware)


## EXPORTS
ament_export_targets(export_ow_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
