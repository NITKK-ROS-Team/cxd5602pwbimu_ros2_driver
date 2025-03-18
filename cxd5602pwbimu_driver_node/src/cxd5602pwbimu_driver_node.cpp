/*
* Copyright (c) 2025 NITK.K ROS-Team
*
* SPDX-License-Identifier: Apache-2.0
*/

#include "cxd5602pwbimu_driver_node/cxd5602pwbimu_driver_node.hpp"

namespace cxd5602pwbimu_driver_node
{

Cxd5602pwbimuDriverNode::Cxd5602pwbimuDriverNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("imu_publisher", options),
  port_handler_(this->declare_parameter<std::string>("dev", "/dev/ttyUSB0")),
  time_offset_(0),
  delimiter_(this->declare_parameter<char>("delimiter", '\n'))
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int timeout_ms = this->declare_parameter<int>("timeout_ms", 100);
  const int spin_ms = this->declare_parameter<int>("spin_ms", 1);

  imu_ = std::make_unique<ImuClass>();
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data_raw", rclcpp::SensorDataQoS());

  if (!this->port_handler_.configure(baudrate, timeout_ms)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure serial port");
    exit(EXIT_FAILURE);
  }

  if (!this->port_handler_.open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    exit(EXIT_FAILURE);
  }

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(spin_ms), std::bind(&Cxd5602pwbimuDriverNode::timerCallback, this));
}

Cxd5602pwbimuDriverNode::~Cxd5602pwbimuDriverNode()
{
  this->port_handler_.close();
}

void Cxd5602pwbimuDriverNode::timerCallback()
{
  this->timer_->cancel();

  std::stringstream ss;
  this->port_handler_.readUntil(ss, delimiter_);

  std::string buffer = ss.str();

  if (imu_->set_data(reinterpret_cast<const uint8_t *>(buffer.c_str()), buffer.size())) {
    auto [linear_acceleration, angular_velocity, sec, msec] = imu_->get_data();

    auto msg = std::make_unique<sensor_msgs::msg::Imu>();

    if (time_offset_ == 0) {
      time_offset_ = static_cast<uint32_t>(std::time(nullptr));
    }

    msg->header.frame_id = "imu";
    msg->header.stamp.sec = sec + time_offset_;
    msg->header.stamp.nanosec = msec * 1000000;

    msg->linear_acceleration.x = linear_acceleration[0];
    msg->linear_acceleration.y = linear_acceleration[1];
    msg->linear_acceleration.z = linear_acceleration[2];

    msg->angular_velocity.x = angular_velocity[0] * 0.5;
    msg->angular_velocity.y = angular_velocity[1] * 0.5;
    msg->angular_velocity.z = angular_velocity[2] * 0.5;

    publisher_->publish(std::move(msg));
  }

  this->timer_->reset();
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cxd5602pwbimu_driver_node::Cxd5602pwbimuDriverNode)
