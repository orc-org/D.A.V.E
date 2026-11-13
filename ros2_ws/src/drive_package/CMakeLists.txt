cmake_minimum_required(VERSION 3.8)
project(drive_package)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# -----------------------------
# C++ joystick drive node
# -----------------------------
add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(vesc_can_driver_node src/vesc_can_driver_node.cpp)
ament_target_dependencies(vesc_can_driver_node
  rclcpp
  ament_index_cpp
  std_msgs
)
target_link_libraries(vesc_can_driver_node yaml-cpp)

install(TARGETS
  drive_node
  vesc_can_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# Linting
# -----------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
