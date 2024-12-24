# Zaber System

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Packages](#packages)
6. [Usage](#usage)
7. [Technical Details](#technical-details)
8. [License](#license)

## Overview

The **Zaber System** repository provides ROS2 packages for controlling and visualizing Zaber linear motion systems. It includes hardware interface drivers, robot description files, and launch configurations for seamless integration with the ROS2 control framework.

### Features

- Complete ROS2 Control integration for Zaber linear stages
- URDF/XACRO descriptions for visualization
- Hardware interface plugin for direct stage control
- Support for position and velocity control modes
- Real-time performance with proper lifecycle management
- Thread-safe implementations
- Integration with multiple controller types

## Repository Structure

```plaintext
zaber_system/
├── zaber_description/          # Robot description files
│   ├── urdf/                  # URDF/XACRO files
│   ├── meshes/               # Visual/collision meshes
│   └── launch/               # Visualization launch files
├── zaber_robot_driver/        # Hardware interface driver
│   ├── include/              # Header files
│   ├── src/                  # Implementation files
│   └── config/               # Controller configurations
└── README.md                 # This file
```

## Prerequisites

### Required Software

- Ubuntu 22.04 or later
- ROS2 Humble or later
- C++17 compatible compiler
- CMake 3.8+

### Hardware Requirements

- Zaber Linear Stages (Compatible models)
  - X-LSM Series
  - X-LHM Series
  - VSR Series
- USB connection to device
- Proper stage ROM files

### Dependencies

```xml
<depend>controller_interface</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
<depend>geometry_msgs</depend>
<depend>urdf</depend>
<depend>xacro</depend>
```

## Installation

1. Create a workspace:

    ```bash
    mkdir -p zaber_ws/src
    cd zaber_ws/src
    ```

2. Clone the repository:

    ```bash
    git clone https://github.com/your-org/zaber_system.git
    ```

3. Install dependencies:

    ```bash
    cd ..
    rosdep install --from-paths src --ignore-src -r -y
    ```

4. Build the workspace:

    ```bash
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

5. Source the workspace:

    ```bash
    source install/setup.bash
    ```

## Packages

### Zaber Description

Robot description package containing URDF definitions, meshes, and visualization tools. See [zaber_description/README.md](zaber_description/README.md) for details.

### Zaber Robot Driver

Hardware interface implementation for Zaber stages using ROS2 Control. See [zaber_robot_driver/README.md](zaber_robot_driver/README.md) for details.

## Usage

### Basic Launch

```bash
# Launch with visualization
ros2 launch zaber_description view.launch.py

# Launch full system with controllers
ros2 launch zaber_robot_driver zaber.launch.py
```

### Controller Configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

velocity_controller:
  ros__parameters:
    joints:
      - insertion_joint
      - horizontal_joint
      - vertical_joint
```

### Runtime Control

```bash
# List available controllers
ros2 control list_controllers

# Switch controllers
ros2 control switch_controllers --start velocity_controller

# Send movement commands
ros2 topic pub /velocity_controller/commands std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0]"
```

## Technical Details

### Performance Considerations

- Update rates: Configurable up to 100Hz
- Thread-safe implementations
- Real-time compatible
- Lock-free data structures where possible

### Safety Features

- Software position limits
- Velocity limits enforcement
- Hardware homing procedure
- Emergency stop handling
- Proper cleanup on deactivation

### Coordinate Systems

- Internal: millimeter-scale measurements
- ROS2: meter-scale measurements
- Joint frames follow REP-103

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.
