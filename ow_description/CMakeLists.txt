cmake_minimum_required(VERSION 3.8)
project(ow_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Directorios usados por colcon
install(
  DIRECTORY urdf launch rviz meshes config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
