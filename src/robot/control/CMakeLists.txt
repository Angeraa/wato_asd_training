cmake_minimum_required(VERSION 3.10)
project(control)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build library (ControlCore implementation)
add_library(control_lib
  src/control_core.cpp
)

target_include_directories(control_lib
  PUBLIC
  include
)

ament_target_dependencies(control_lib
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Build node executable
add_executable(control_node
  src/control_node.cpp
)

target_link_libraries(control_node
  control_lib
)

ament_target_dependencies(control_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install executables
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files (if you have them)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
