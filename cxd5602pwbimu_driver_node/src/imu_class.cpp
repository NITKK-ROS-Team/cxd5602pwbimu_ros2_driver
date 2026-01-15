/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "cxd5602pwbimu_driver_node/imu_class.hpp"

namespace cxd5602pwbimu_driver_node
{

ImuClass::ImuClass()
: linear_acceleration_({0.0, 0.0, 0.0}),
  angular_velocity_({0.0, 0.0, 0.0}),
  temperature_(0.0),
  sec_(0),
  msec_(0),
  delimiter_('\n'),
  start_byte_('X') {}

bool ImuClass::set_data(const uint8_t * data_bytes, size_t length)
{
  const size_t expected_length = 39;
  const size_t crc_index = expected_length - 2;
  if (length != expected_length) {
    std::cerr << "Invalid data length: " << length << std::endl;
    return false;
  }
  if (data_bytes[0] != start_byte_) {
    std::cerr << "Invalid start byte: " << static_cast<int>(data_bytes[0]) << std::endl;
    return false;
  }
  if (data_bytes[expected_length - 1] != delimiter_) {
    std::cerr << "Invalid end byte: " << static_cast<int>(data_bytes[expected_length - 1]) << std::endl;
    return false;
  }

  CRC8 hash_obj(0x07);
  hash_obj.add(data_bytes, crc_index);

  if (hash_obj.calc() != data_bytes[crc_index]) {
    std::cerr << "CRC mismatch: calculated " << static_cast<int>(hash_obj.calc())
              << ", received " << static_cast<int>(data_bytes[crc_index]) << std::endl;
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

  float temp;
  std::memcpy(&temp, &data_bytes[offset], sizeof(float));
  temperature_ = temp;

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
  std::cout << "Temperature: " << temperature_ << " C" << std::endl;
}

std::tuple<std::array<float, 3>, std::array<float, 3>, float, uint32_t,
  uint32_t> ImuClass::get_data() const
{
  return {linear_acceleration_, angular_velocity_, temperature_, sec_, msec_};
}

}  // namespace cxd5602pwbimu_driver_node
