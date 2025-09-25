cmake_minimum_required(VERSION 3.8)
project(aiv_nav)

# ===== Compiler Settings =====
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Find Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ===== Source Files Organization =====
# Controller library sources
set(CONTROLLER_SOURCES
  src/controller.cpp
  src/modes/mode_a.cpp
  src/modes/mode_c.cpp
  src/modes/mode_g.cpp
)

# Main executable source
set(MAIN_SOURCES
  src/aiv_nav.cpp
)

# ===== Create Controller Library =====
add_library(aiv_nav_controller ${CONTROLLER_SOURCES})

# Set C++ standard for the library
target_compile_features(aiv_nav_controller PUBLIC c_std_99 cxx_std_17)

# Set include directories for the library
target_include_directories(aiv_nav_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# ===== Create Main Executable =====
add_executable(aiv_nav ${MAIN_SOURCES})

# Link the executable with the controller library
target_link_libraries(aiv_nav aiv_nav_controller)

# Set dependencies for the executable
ament_target_dependencies(aiv_nav rclcpp std_msgs sensor_msgs geometry_msgs)

# Set C++ standard for the executable
target_compile_features(aiv_nav PUBLIC c_std_99 cxx_std_17)

# Set include directories for the executable
target_include_directories(aiv_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

# ===== Install Targets =====
# Install the executable
install(TARGETS
  aiv_nav
  DESTINATION lib/${PROJECT_NAME})

# Install the library
install(TARGETS
  aiv_nav_controller
  EXPORT aiv_nav_controllerTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Install header files
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")

# Export the library for other packages
ament_export_targets(aiv_nav_controllerTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs)

# ===== Testing =====
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ===== Finalize Package =====
ament_package()