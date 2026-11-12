cmake_minimum_required(VERSION 3.8)
project(user_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include(FetchContent)

find_package(raylib QUIET CONFIG)
if(NOT raylib_FOUND)
  set(BUILD_EXAMPLES OFF)
  set(BUILD_SHARED_LIBS OFF)
  set(PLATFORM "Desktop")
  set(USE_EXTERNAL_GLFW OFF)

  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 02566453880e06bec97f5b8160fa6fe1e98397ca
    GIT_PROGRESS TRUE
  )
  FetchContent_MakeAvailable(raylib)
endif()

FetchContent_Declare(
  raygui
  GIT_REPOSITORY https://github.com/raysan5/raygui.git
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(raygui)

add_executable(main
  src/main.cpp
  src/LaunchNode.cpp
)
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${raygui_SOURCE_DIR})
target_compile_features(main PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(main raylib)
ament_target_dependencies(main rclcpp)
if(TARGET raygui)
  target_link_libraries(main raygui)
endif()

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})



install(FILES include/user_interface/LaunchNode.hpp
  DESTINATION include/user_interface)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ##############
  # Run tests
  ##############

  ament_add_gtest(${PROJECT_NAME}_simple_test test/simple_test.cpp)
  target_include_directories(${PROJECT_NAME}_simple_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  # target_link_libraries(${PROJECT_NAME}_simple_test name_of_local_library)

  ##############
  # Run Linter
  ##############

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  
  #set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  
  #set(ament_cmake_cpplint_FOUND TRUE)

endif()

ament_package()