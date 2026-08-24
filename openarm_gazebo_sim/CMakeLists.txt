# Copyright 2026 OpenArm contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

cmake_minimum_required(VERSION 3.8)
project(openarm_gazebo_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
