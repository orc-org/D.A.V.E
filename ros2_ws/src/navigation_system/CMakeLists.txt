cmake_minimum_required(VERSION 3.8)
project(navigation_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Resolve the workspace root so CMake can find built/installed packages
get_filename_component(ROS2_WS "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
list(APPEND CMAKE_PREFIX_PATH "${ROS2_WS}/install")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action)
find_package(rclcpp_components)
find_package(navigation_interfaces REQUIRED)
find_package(serial_interfaces REQUIRED)
find_package(serial_communication REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(raylib QUIET)

install(
  DIRECTORY ${ROS2_WS}/src/serial_communication/include/serial_communication
  DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

set(SHARED_SOURCES
  src/Waypoint.cpp
  src/Obstacle.cpp
  src/ListOfWaypoints.cpp
  ${ROS2_WS}/src/serial_communication/src/SerialPort.cpp
)

set(NAVNODE_SOURCES
  ${SHARED_SOURCES}
  src/NAVnode.cpp
)

set(NAVDISPLAY_SOURCES
  ${SHARED_SOURCES}
  src/Sprite.cpp
  src/NAVDisplay.cpp
)

add_executable(NAVnode ${NAVNODE_SOURCES})
ament_target_dependencies(NAVnode rclcpp rclcpp_action rclcpp_components navigation_interfaces serial_interfaces serial_communication sensor_msgs)
target_include_directories(NAVnode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/navigation_system>
  $<INSTALL_INTERFACE:include/navigation_system>
  $<BUILD_INTERFACE:${ROS2_WS}/src/serial_communication/include/serial_communication>
  $<INSTALL_INTERFACE:include/serial_communication>
)

ament_export_include_directories(include)

install(TARGETS
  NAVnode
  DESTINATION lib/${PROJECT_NAME})

if(raylib_FOUND)
  add_executable(NAVDisplay ${NAVDISPLAY_SOURCES})
  target_link_libraries(NAVDisplay raylib)
  # raylib is linked as a CMake target on the previous line; it is not an ament package
  ament_target_dependencies(NAVDisplay rclcpp rclcpp_action navigation_interfaces serial_communication)
  target_include_directories(NAVDisplay PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/navigation_system>
    $<INSTALL_INTERFACE:include/navigation_system>
    $<BUILD_INTERFACE:${ROS2_WS}/src/serial_communication/include/serial_communication>
    $<INSTALL_INTERFACE:include/serial_communication>
  )
  install(TARGETS
    NAVDisplay
    DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "raylib not found: building NAVnode headless without NAVDisplay")
endif()

install(PROGRAMS
  src/gps_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_nav_math
    test/test_nav_math.cpp
    src/Waypoint.cpp
    src/Obstacle.cpp
    src/ListOfWaypoints.cpp
  )
  target_include_directories(test_nav_math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/navigation_system>
  )
endif()

ament_package()
