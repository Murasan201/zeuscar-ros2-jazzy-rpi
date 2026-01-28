# ZeusCar ROS 2 Jazzy on Raspberry Pi

A ROS 2 Jazzy workspace for the ZeusCar mecanum wheel robot platform running on Raspberry Pi 4 with Ubuntu 24.04.

## Overview

This project provides a complete ROS 2 setup for the ZeusCar robot, featuring:

- **Motor Control**: Arduino-based mecanum wheel control via USB serial (UART)
- **IMU Integration**: ICM-42688 6-axis IMU sensor via I2C
- **LiDAR Integration**: RPLIDAR A1M8 driver wrapper with custom configuration
- **TF/URDF**: Robot model with accurate sensor positioning based on physical measurements
- **SLAM Ready**: Prepared for slam_toolbox integration (coming soon)

## Hardware Requirements

| Component | Description |
|-----------|-------------|
| Raspberry Pi 4 | 4GB RAM or higher recommended |
| ZeusCar | Mecanum wheel robot platform |
| Arduino Uno R3 | Motor controller (USB serial connection) |
| ICM-42688 IMU | 6-axis gyroscope + accelerometer (I2C) |
| RPLIDAR A1M8 | SLAMTEC 2D LiDAR sensor |
| microSD Card | 32GB or larger |

## Software Requirements

- Ubuntu 24.04 LTS (Noble Numbat)
- ROS 2 Jazzy Jalisco

## Package Structure

```
ros2_ws/src/
├── zeuscar_bringup/       # Launch files for full system startup
├── zeuscar_description/   # URDF model and TF configuration
├── zeuscar_motor/         # Arduino motor controller interface
├── zeuscar_imu/           # ICM-42688 IMU sensor driver
├── zeuscar_lidar/         # RPLIDAR A1M8 driver wrapper
└── zeuscar_slam/          # SLAM configuration (planned)
```

## Motor Control

The robot uses an Arduino Uno R3 for motor control, communicating with Raspberry Pi via USB serial (UART).

### Supported Commands

| Command | Action |
|---------|--------|
| FORWARD | Move forward |
| BACKWARD | Move backward |
| LEFT | Strafe left |
| RIGHT | Strafe right |
| TURNLEFT | Rotate left |
| TURNRIGHT | Rotate right |
| STOP | Stop all motors |

### Serial Configuration

- Port: `/dev/ttyACM0`
- Baud rate: 9600
- Protocol: ASCII commands with newline terminator

## IMU Sensor

The robot uses an ICM-42688 6-axis IMU (3-axis gyroscope + 3-axis accelerometer) connected via I2C.

### IMU Specifications

| Parameter | Value |
|-----------|-------|
| Sensor | ICM-42688 (TDK/InvenSense) |
| Interface | I2C Bus 1 |
| I2C Address | 0x68 |
| Gyroscope Range | ±250 dps (configurable) |
| Accelerometer Range | ±2g (configurable) |

### Wiring (Raspberry Pi 4)

| IMU Pin | RPi Pin | Function |
|---------|---------|----------|
| VCC | Pin 1 (3.3V) | Power |
| GND | Pin 9 (GND) | Ground |
| SCL | Pin 5 (GPIO3) | I2C Clock |
| SDA | Pin 3 (GPIO2) | I2C Data |
| SAO | Pin 9 (GND) | Address select (0x68) |
| CS | Pin 1 (3.3V) | I2C mode (HIGH) |

## TF Tree

```
base_footprint
  └── base_link
        └── laser_frame
```

### Robot Dimensions (Measured)

| Parameter | Value | Description |
|-----------|-------|-------------|
| Body Length | 163mm | X-axis (front-back) |
| Body Width | 177mm | Y-axis (left-right) |

### LiDAR Position (Measured from robot center)

| Parameter | Value | Description |
|-----------|-------|-------------|
| X | +0.0035m | 3.5mm forward |
| Y | -0.0045m | 4.5mm to the right |
| Z | +0.235m | 235mm from ground |
| Yaw | +1.5708 rad | 90° rotated left |

## Quick Start

### 1. Clone the Repository

```bash
cd ~
git clone https://github.com/Murasan201/zeuscar-ros2-jazzy-rpi.git
```

### 2. Build the Workspace

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

### 3. Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

### 4. Launch TF Publisher

```bash
ros2 launch zeuscar_description description.launch.py
```

### 5. Launch LiDAR (in another terminal)

```bash
ros2 launch zeuscar_lidar lidar.launch.py
```

## Documentation

Detailed setup instructions are available in the `docs/` directory:

- [Setup Guide](docs/setup_guide.md) - Complete installation and configuration guide (Japanese)
- [Requirements](docs/zeus_car_ros_2_jazzy_rpi_requirements.md) - Project requirements specification
- [IMU Sensor Specification](docs/hardware/icm42688_imu_sensor.md) - ICM-42688 wiring and test procedures

## Project Status

| Feature | Status |
|---------|--------|
| ROS 2 Workspace Setup | Done |
| Motor Control (Arduino) | Done |
| IMU Integration (ICM-42688) | Done |
| LiDAR Integration | Done |
| TF/URDF Configuration | Done |
| SLAM Integration | Planned |
| RViz Visualization | Planned |

## License

TODO: License declaration

## Author

Created by [Murasan](https://murasan-net.com/)

## Acknowledgments

- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [SLAMTEC RPLIDAR](https://www.slamtec.com/)
- [ZeusCar Platform](https://www.waveshare.com/)
