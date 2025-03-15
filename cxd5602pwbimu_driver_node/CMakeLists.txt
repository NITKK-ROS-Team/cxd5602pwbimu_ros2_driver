# Copyright (c) 2025 NITK.K ROS-Team.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(cxd5602pwbimu_driver_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# Simple ReadUntil Node ==================================================
set(TARGET cxd5602pwbimu_driver_node)
set(MY_LIB_NAME ${PROJECT_NAME}_${TARGET})
ament_auto_add_library(${MY_LIB_NAME} SHARED
  src/${TARGET}.cpp
  src/imu_class.cpp
  src/crc8.cpp
)

rclcpp_components_register_node(
  ${MY_LIB_NAME}
  PLUGIN "${PROJECT_NAME}::Cxd5602pwbimuDriverNode"
  EXECUTABLE ${TARGET}_exec)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()