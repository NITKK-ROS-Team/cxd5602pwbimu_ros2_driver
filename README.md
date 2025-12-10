# cxd5602pwbimu_ros2_driver
ROS 2 driver for Multi-IMU Add-on board for SONY SPRESENSE™
High speed version up to 1.92 kHz

[![ci_humble](https://github.com/NITKK-ROS-Team/cxd5602pwbimu_ros2_driver/actions/workflows/ci_humble.yml/badge.svg)](https://github.com/NITKK-ROS-Team/cxd5602pwbimu_ros2_driver/actions/workflows/ci_humble.yml)

## Flash
### 1. Setup Arduino IDE for SPRESENSE
Follow the instructions on the official SPRESENSE Arduino IDE setup guide:
[https://developer.sony.com/spresense/development-guides/arduino_set_up_ja](https://developer.sony.com/spresense/development-guides/arduino_set_up_ja)

### flash ino sketch
`firmware/cxd5602pwbimu_imu/cxd5602pwbimu_imu.ino` to SPRESENSE main board using Arduino IDE.

## Build
```bash
git clone <this repo url>
cd cxd5602pwbimu_ros2_driver
git clone -b humble https://github.com/HarvestX/h6x_serial_interface.git
colcon build --symlink-install
```

## Launch
```bash
source install/setup.bash
ros2 launch cxd5602pwbimu_driver_bringup cxd5602pwbimu_driver_launch.py
```