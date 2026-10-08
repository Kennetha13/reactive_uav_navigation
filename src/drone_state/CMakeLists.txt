cmake_minimum_required(VERSION 3.8)

project(drone_state)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)


find_package(rclcpp REQUIRED)


find_package(nav_msgs REQUIRED)


find_package(geometry_msgs REQUIRED)

add_executable(drone_state_node
  src/drone_state.cpp
)



target_include_directories(drone_state_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


ament_target_dependencies(drone_state_node
  rclcpp
  nav_msgs
  geometry_msgs
)



install(
  TARGETS drone_state_node
  DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY include/
  DESTINATION include
)



if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)

  
  set(ament_cmake_copyright_FOUND TRUE)

  
  set(ament_cmake_cpplint_FOUND TRUE)

  
  ament_lint_auto_find_test_dependencies()

endif()



ament_package()