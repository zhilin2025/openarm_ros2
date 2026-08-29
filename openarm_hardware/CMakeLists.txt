# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rs_motor_ros2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)

# Find openarm_can library
find_package(OpenArmCAN REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/v10_simple_hardware.cpp
  src/zero_torque_controller.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  OpenArmCAN::openarm_can
)

# 链接本地库包 rs_motor_ros2_lib
if(TARGET rs_motor_ros2::rs_motor_ros2_lib)
  target_link_libraries(${PROJECT_NAME}
    rs_motor_ros2::rs_motor_ros2_lib
  )
elseif(TARGET rs_motor_ros2_lib)
  target_link_libraries(${PROJECT_NAME}
    rs_motor_ros2_lib
  )
else()
  target_link_libraries(${PROJECT_NAME}
    rs_motor_ros2
  )
endif()

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rs_motor_ros2
  sensor_msgs
  std_msgs
)

target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

pluginlib_export_plugin_description_file(hardware_interface openarm_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface openarm_controller_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  )

install(
  FILES openarm_hardware.xml openarm_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  openarm_can
  pluginlib
  rclcpp
  rs_motor_ros2
  controller_interface
  sensor_msgs
  std_msgs
  pinocchio
)

ament_package()
