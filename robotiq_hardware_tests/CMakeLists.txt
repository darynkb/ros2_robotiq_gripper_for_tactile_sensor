# See https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html

cmake_minimum_required(VERSION 3.8)
project(robotiq_hardware_tests LANGUAGES CXX)

# This module provides installation directories as per the GNU coding standards.
include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(robotiq_driver REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

add_executable(full_test
    src/gripper_interface_test.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)

add_executable(gripper_demo
    src/gripper_demo.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)

add_executable(arrow_control
    src/arrow_control.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)

add_executable(grasp_publisher
    src/grasp_publisher.cpp
    src/command_line_utility.hpp
    src/command_line_utility.cpp
)

ament_target_dependencies(full_test robotiq_driver)
ament_target_dependencies(gripper_demo robotiq_driver)
ament_target_dependencies(arrow_control robotiq_driver)
ament_target_dependencies(grasp_publisher robotiq_driver)

install(TARGETS full_test
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS gripper_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arrow_control
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS grasp_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
