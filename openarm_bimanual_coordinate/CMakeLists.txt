cmake_minimum_required(VERSION 3.16)
project(openarm_bimanual_coordinate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)

# ---------------------------------------------------------------------------
# Bimanual grasp plugin: command-driven fixed-joint welding between a robot
# link and a world object (attach / detach by topic command).
# ---------------------------------------------------------------------------
add_library(bimanual_grasp_plugin SHARED src/bimanual_grasp_plugin.cpp)
ament_target_dependencies(bimanual_grasp_plugin
  rclcpp
  std_msgs
  gazebo_ros
)
target_link_libraries(bimanual_grasp_plugin ${GAZEBO_LIBRARIES})
target_include_directories(bimanual_grasp_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(bimanual_grasp_plugin PRIVATE ${GAZEBO_LIBRARY_DIRS})

install(TARGETS bimanual_grasp_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------------------------------------------------------------------------
# Python nodes
# ---------------------------------------------------------------------------
install(PROGRAMS
  scripts/perception_node.py
  scripts/carry_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(bimanual_grasp_plugin)
ament_package()
