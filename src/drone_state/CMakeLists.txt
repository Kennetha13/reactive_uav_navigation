cmake_minimum_required(VERSION 3.8)

project(drone_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Reusable DroneState library
add_library(drone_state_lib
  src/drone_state.cpp
)

target_include_directories(drone_state_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(drone_state_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

install(
  TARGETS drone_state_lib
  EXPORT drone_state_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(drone_state_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()