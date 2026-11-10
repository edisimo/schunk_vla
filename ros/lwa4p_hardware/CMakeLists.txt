# Copyright (c) 2026 Eduard Zelenay. MIT License, see LICENSE.
cmake_minimum_required(VERSION 3.16)
project(lwa4p_hardware VERSION 0.1.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(lwa4p_canopen STATIC src/can_socket.cpp src/sdo_client.cpp src/cia402.cpp)
target_include_directories(lwa4p_canopen PUBLIC include)
set_target_properties(lwa4p_canopen PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(lwa4p_hardware SHARED src/lwa4p_system.cpp)
target_link_libraries(lwa4p_hardware lwa4p_canopen)
ament_target_dependencies(
  lwa4p_hardware
  controller_manager_msgs hardware_interface pluginlib rclcpp rclcpp_lifecycle
  std_srvs std_msgs)
pluginlib_export_plugin_description_file(hardware_interface lwa4p_hardware.xml)

install(TARGETS lwa4p_hardware LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(joint_trajectory_controller REQUIRED)
  ament_add_gtest(test_canopen test/test_canopen.cpp)
  target_link_libraries(test_canopen lwa4p_canopen)
  ament_add_gtest(test_ip_interpolation test/test_ip_interpolation.cpp)
  ament_target_dependencies(test_ip_interpolation joint_trajectory_controller)
endif()

ament_package()
