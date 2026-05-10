cmake_minimum_required(VERSION 3.8)
project(reactive_circumnav)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
add_library(reactive_circumnav_controller
  src/controller.cpp
  src/curvature.cpp
  src/modes/mode_a.cpp
  src/modes/mode_c.cpp
  src/modes/mode_g.cpp
  src/companion_disk.cpp
)

target_compile_features(reactive_circumnav_controller PUBLIC cxx_std_17)
target_include_directories(reactive_circumnav_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# Main executable
add_executable(reactive_circumnav
  src/reactive_circumnav.cpp
  src/debug_markers.cpp
)

target_include_directories(reactive_circumnav PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(reactive_circumnav PUBLIC cxx_std_17)
target_link_libraries(reactive_circumnav reactive_circumnav_controller)
ament_target_dependencies(reactive_circumnav
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
  std_msgs)

add_executable(scan_dt_logger src/scan_dt_logger.cpp)
target_compile_features(scan_dt_logger PUBLIC cxx_std_17)
ament_target_dependencies(scan_dt_logger rclcpp sensor_msgs)

# Install
install(TARGETS reactive_circumnav scan_dt_logger reactive_circumnav_controller
  EXPORT reactive_circumnav_controllerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")

install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  FILES_MATCHING PATTERN "*.sdf")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py")

ament_export_targets(reactive_circumnav_controllerTargets HAS_LIBRARY_TARGET)
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