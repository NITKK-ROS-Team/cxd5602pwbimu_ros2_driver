/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "cxd5602pwbimu/imu_class.hpp"

namespace cxd5602pwbimu_node
{

ImuClass::ImuClass()
: linear_acceleration_({0.0, 0.0, 0.0}),
  angular_velocity_({0.0, 0.0, 0.0}),
  sec_(0),
  msec_(0) {}

bool ImuClass::set_data(const uint8_t * data_bytes, size_t length)
{
  if (length != 36) {
    return false;
  }
  if (data_bytes[0] != 'X') {
    return false;
  }

  CRC8 hash_obj(0x07);
  hash_obj.add(data_bytes, 33);

  if (hash_obj.calc() != data_bytes[33]) {
    return false;
  }

  sec_ = data_bytes[1] | (data_bytes[2] << 8) | (data_bytes[3] << 16) | (data_bytes[4] << 24);
  msec_ = data_bytes[5] | (data_bytes[6] << 8) | (data_bytes[7] << 16) | (data_bytes[8] << 24);

  int offset = 9;
  for (int i = 0; i < 3; i++) {
    float value;
    std::memcpy(&value, &data_bytes[offset], sizeof(float));
    linear_acceleration_[i] = value;
    offset += 4;
  }

  for (int i = 0; i < 3; i++) {
    float value;
    std::memcpy(&value, &data_bytes[offset], sizeof(float));
    angular_velocity_[i] = value;
    offset += 4;
  }

  return true;
}

void ImuClass::print_data() const
{
  std::cout << "Time: " << sec_ << "." << msec_ << std::endl;
  std::cout << "Linear Acceleration (x,y,z): ["
            << linear_acceleration_[0] << ", "
            << linear_acceleration_[1] << ", "
            << linear_acceleration_[2] << "]" << std::endl;
  std::cout << "Angular Velocity (x,y,z): ["
            << angular_velocity_[0] << ", "
            << angular_velocity_[1] << ", "
            << angular_velocity_[2] << "]" << std::endl;
}

std::tuple<std::array<float, 3>, std::array<float, 3>, uint32_t,
  uint32_t> ImuClass::get_data() const
{
  return {linear_acceleration_, angular_velocity_, sec_, msec_};
}

}  // namespace cxd5602pwbimu_node
