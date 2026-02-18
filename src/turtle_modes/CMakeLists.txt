cmake_minimum_required(VERSION 3.8)
project(turtle_modes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_modes_interfaces REQUIRED)

add_executable(turtle_mode_controller src/turtle_mode_controller.cpp)
ament_target_dependencies(turtle_mode_controller
  rclcpp rclcpp_action geometry_msgs turtlesim turtle_modes_interfaces
)

install(TARGETS turtle_mode_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

