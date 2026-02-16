cmake_minimum_required(VERSION 3.8)
project(aiv_nav)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Controller library
add_library(aiv_nav_controller
  src/controller.cpp
  src/curvature.cpp
  src/modes/mode_a.cpp
  src/modes/mode_c.cpp
  src/modes/mode_g.cpp
  src/companion_disk.cpp
)

target_compile_features(aiv_nav_controller PUBLIC cxx_std_17)
target_include_directories(aiv_nav_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Main executable
add_executable(aiv_nav src/aiv_nav.cpp)

target_compile_features(aiv_nav PUBLIC cxx_std_17)
target_link_libraries(aiv_nav aiv_nav_controller)
ament_target_dependencies(aiv_nav
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs)

# Install
install(TARGETS aiv_nav aiv_nav_controller
  EXPORT aiv_nav_controllerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")

ament_export_targets(aiv_nav_controllerTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()