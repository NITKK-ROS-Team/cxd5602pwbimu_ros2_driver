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
  delimiter_(this->declare_parameter<char>("delimiter", '\n')),
  running_(true)
{
  const int baudrate = this->declare_parameter<int>("baudrate", 115200);
  const int timeout_ms = this->declare_parameter<int>("timeout_ms", 100);

  imu_ = std::make_unique<ImuClass>();
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());

  if (!port_handler_.configure(baudrate, timeout_ms)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure serial port");
    exit(EXIT_FAILURE);
  }

  if (!port_handler_.open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open serial port");
    exit(EXIT_FAILURE);
  }

  RCLCPP_INFO(get_logger(), "Serial port opened successfully");
  startSerialThread();
}

Cxd5602pwbimuDriverNode::~Cxd5602pwbimuDriverNode()
{
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  port_handler_.close();
}

void Cxd5602pwbimuDriverNode::startSerialThread()
{
  recv_thread_ = std::thread([this]() {
    std::string buf;
    buf.reserve(35);
    char c;

    while (running_) {
        char c;
        int n = port_handler_.read(&c, 1);
        if (n != 1) continue;

        buf.push_back(c);

        if (buf.size() == 35) {
            if (buf.back() == delimiter_) {
                processPacket(reinterpret_cast<const uint8_t*>(buf.data()), buf.size());
            } else {
                char skip;
                do {
                    if (port_handler_.read(&skip, 1) != 1) break;
                    RCLCPP_INFO(get_logger(), "Skipping byte: %c %d", skip, skip==delimiter_);
                } while (skip != delimiter_);
                RCLCPP_INFO(get_logger(), "Resynchronized at delimiter");
            }
            buf.clear();
        }
    }

  });
}

void Cxd5602pwbimuDriverNode::processPacket(const uint8_t* data, size_t size)
{
  if (!imu_->set_data(data, size)) {
    RCLCPP_WARN(get_logger(), "Failed to parse IMU data");
    return;
  }

  auto [linear_acc, angular_vel, sec, msec] = imu_->get_data();

  auto msg = sensor_msgs::msg::Imu();

  if (time_offset_ == 0) {
    time_offset_ = static_cast<uint32_t>(std::time(nullptr));
  }

  msg.header.frame_id = "imu";
  msg.header.stamp.sec = sec + time_offset_;
  msg.header.stamp.nanosec = msec * 1000000;

  msg.linear_acceleration.x = linear_acc[0];
  msg.linear_acceleration.y = linear_acc[1];
  msg.linear_acceleration.z = linear_acc[2];

  msg.angular_velocity.x = angular_vel[0] * 0.5;
  msg.angular_velocity.y = angular_vel[1] * 0.5;
  msg.angular_velocity.z = angular_vel[2] * 0.5;

  publisher_->publish(msg);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(cxd5602pwbimu_driver_node::Cxd5602pwbimuDriverNode)
