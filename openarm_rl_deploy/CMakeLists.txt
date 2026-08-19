cmake_minimum_required(VERSION 3.16)
project(openarmx_deploy)

# ============================================================================
# C++17 + LibTorch
# ============================================================================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# --- 查找 LibTorch ---
#   设置 -DTorch_DIR=<libtorch_path>/share/cmake/Torch 或
#   通过 CMAKE_PREFIX_PATH 指定
find_package(Torch REQUIRED)
message(STATUS "LibTorch version: ${Torch_VERSION}")
message(STATUS "LibTorch include: ${TORCH_INCLUDE_DIRS}")

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

# ============================================================================
# Library: openarmx_deploy_core (状态机 + 观测构造)
# ============================================================================
add_library(openarmx_deploy_core STATIC
  src/grasp_state_machine.cpp
  src/observation_builder.cpp
)
target_include_directories(openarmx_deploy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(openarmx_deploy_core PUBLIC cxx_std_17)

# ============================================================================
# Executable: deploy_node
# ============================================================================
add_executable(deploy_node src/deploy_node.cpp)

target_include_directories(deploy_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(deploy_node
  openarmx_deploy_core
  ${TORCH_LIBRARIES}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  trajectory_msgs::trajectory_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  moveit_ros_planning_interface::moveit_move_group_interface
  moveit_msgs::moveit_msgs
)

target_compile_features(deploy_node PRIVATE cxx_std_17)

# ============================================================================
# Install
# ============================================================================
install(TARGETS deploy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_package()
