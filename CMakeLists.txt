cmake_minimum_required(VERSION 3.8)

project(reactive_uav_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find package dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(reactive_navigator_node
  src/reactive_navigator_node_main.cpp
  src/reactive_navigator_node.cpp
  src/vss_admissible.cpp
  src/drone_state.cpp
  src/distance_function.cpp
  src/heading_function.cpp
  src/ray_cast.cpp
  src/vss_dynamic_window.cpp
  src/vss_collector.cpp
  src/vss_maximum.cpp
  src/px4_param_reader.cpp
  src/velocity_function.cpp
  src/navigation_state.cpp  
)

target_include_directories(reactive_navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(reactive_navigator_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  pcl_conversions
  std_msgs
  tf2
  mavros_msgs
)

install(
  TARGETS reactive_navigator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # Both tests build without ROS or PCL: test/fake shadows distance_function.hpp
  # (and geometry_msgs) with a stand-in, see test/fake.
  add_executable(test_velocity_pipeline
    test/test_velocity_pipeline.cpp
    src/vss_dynamic_window.cpp
    src/vss_admissible.cpp
    src/vss_collector.cpp
    src/vss_maximum.cpp
  )
  target_include_directories(test_velocity_pipeline BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fake
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(test_velocity_pipeline PRIVATE VSS_NO_ROS)
  add_test(NAME test_velocity_pipeline COMMAND test_velocity_pipeline)

  add_executable(test_compute_vr
    test/test_compute_vr.cpp
    src/vss_dynamic_window.cpp
    src/vss_admissible.cpp
    src/vss_collector.cpp
    src/vss_maximum.cpp
  )
  target_include_directories(test_compute_vr BEFORE PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fake
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_compile_definitions(test_compute_vr PRIVATE VSS_NO_ROS)
  add_test(NAME test_compute_vr COMMAND test_compute_vr)
endif()

ament_package()