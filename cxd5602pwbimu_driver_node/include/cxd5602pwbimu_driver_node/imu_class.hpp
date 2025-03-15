/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ____CXD5602PWBIMU_DRIVER_NODE_IMU_CLASS_HPP__
#define ____CXD5602PWBIMU_DRIVER_NODE_IMU_CLASS_HPP__

#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <tuple>

#include "crc8.hpp"

namespace cxd5602pwbimu_driver_node
{

class ImuClass
{
public:
  ImuClass();

  bool set_data(const uint8_t *, size_t);
  void print_data() const;
  std::tuple<std::array<float, 3>, std::array<float, 3>, uint32_t, uint32_t> get_data() const;

private:
  std::array<float, 3> linear_acceleration_;
  std::array<float, 3> angular_velocity_;
  uint32_t sec_;
  uint32_t msec_;
};

}  // namespace cxd5602pwbimu_driver_node

#endif  // ____CXD5602PWBIMU_DRIVER_NODE_IMU_CLASS_HPP__
