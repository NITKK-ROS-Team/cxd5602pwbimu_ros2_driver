/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#ifndef ____CXD5602PWBIMU_DRIVER_NODE_CXD5602PWBIMU_DRIVER_NODE_HPP__
#define ____CXD5602PWBIMU_DRIVER_NODE_CXD5602PWBIMU_DRIVER_NODE_HPP__

#include <functional>
#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "imu_class.hpp"

namespace cxd5602pwbimu_driver_node
{

class Cxd5602pwbimuDriverNode : public rclcpp::Node
{
private:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler port_handler_;
  uint32_t time_offset_;
  const char delimiter_;

  rclcpp::TimerBase::SharedPtr read_timer_;

  std::unique_ptr<ImuClass> imu_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  Cxd5602pwbimuDriverNode() = delete;
  explicit Cxd5602pwbimuDriverNode(const rclcpp::NodeOptions &);
  ~Cxd5602pwbimuDriverNode();

private:
  void timerCallback();


};
}   // namespace cxd5602pwbimu_driver_node
#endif  // ____CXD5602PWBIMU_DRIVER_NODE_CXD5602PWBIMU_DRIVER_NODE_HPP__
