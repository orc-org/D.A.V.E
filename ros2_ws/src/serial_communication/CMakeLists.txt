cmake_minimum_required(VERSION 3.8)
project(serial_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_library(serial_port SHARED src/SerialPort.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serial_communication>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/serial_communication>
)

add_executable(XLRnode
  src/XLRnode.cpp
)
ament_target_dependencies(XLRnode rclcpp serial_interfaces)
target_link_libraries(XLRnode serial_port)

add_executable(XLR_UI
  src/XLR_User_input.cpp
)
ament_target_dependencies(XLR_UI rclcpp serial_interfaces)
target_link_libraries(XLR_UI serial_port)

add_executable(morse_bridge_node
  src/MorseBridgeNode.cpp
)
ament_target_dependencies(morse_bridge_node rclcpp std_msgs serial_interfaces)
target_link_libraries(morse_bridge_node serial_port)

ament_export_targets(export_serial_communication HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/serial_communication)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  serial_port
  EXPORT export_serial_communication
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  XLRnode
  morse_bridge_node
  XLR_UI
  DESTINATION lib/${PROJECT_NAME})



ament_package()
