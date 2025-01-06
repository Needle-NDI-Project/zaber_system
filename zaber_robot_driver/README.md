# Zaber Robot Driver

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Table of Contents

1. [Overview](#overview)
2. [Package Structure](#package-structure)
3. [Hardware Interface](#hardware-interface)
4. [Dependencies](#dependencies)
5. [Installation](#installation)
6. [Configuration](#configuration)
7. [Usage](#usage)
8. [Technical Details](#technical-details)
9. [Troubleshooting](#troubleshooting)

## Overview

The Zaber Robot Driver package provides the ROS2 Control hardware interface implementation for Zaber linear motion systems. It enables direct control of Zaber stages through a standardized ROS2 interface, supporting both position and velocity control modes.

## Package Structure

```plaintext
zaber_robot_driver/
├── include/
│   └── zaber_robot_driver/
│       ├── hardware_interface.hpp
│       └── visibility_control.h
├── src/
│   └── hardware_interface.cpp
├── config/
│   └── zaber_controllers.yaml
├── launch/
│   └── zaber.launch.py
└── CMakeLists.txt
```

## Hardware Interface

### Key Components

1. ZaberSystemHardwareInterface
   - Main hardware interface implementation
   - Manages lifecycle states
   - Handles command interfaces

2. Axis Class
   - Encapsulates single axis control
   - Provides movement primitives
   - Handles unit conversions

### Interface Types

```cpp
// Command Interfaces
hardware_interface::CommandInterface
- position
- velocity

// State Interfaces
hardware_interface::StateInterface
- position
```

## Dependencies

### Required Packages

```xml
<depend>controller_manager</depend>
<depend>controller_manager_msgs</depend>
<depend>hardware_interface</depend>
<depend>pluginlib</depend>
<depend>rclcpp</depend>
<depend>rclcpp_lifecycle</depend>
```

### Hardware Requirements

- Zaber stages with ASCII protocol support
- USB serial connection
- Proper device permissions

## Installation

1. Install dependencies:

    ```bash
    sudo apt install ros-$ROS_DISTRO-ros2-control
    sudo apt install ros-$ROS_DISTRO-controller-manager
    ```

2. Download the Zaber Launcher - GUI Tool (Optional):
    - Visit the [Zaber Downloads](https://software.zaber.com/zaber-launcher/download) page.
    - Download the latest version for your operating system.
    - Change ownership and permissions:

        ```bash
        sudo chmod +x ZaberLauncher.AppImage
        ```

    - Run the launcher:

        ```bash
        ./ZaberLauncher.AppImage
        ```

3. Install the Zaber C++ Motion Library:
    - Visit the [Zaber Motion Library - Installation](https://software.zaber.com/motion-library/docs/tutorials/install/cpp) page.
    - Download the respective [Bash Script](https://software.zaber.com/motion-library/docs/tutorials/install/cpp#non-debian-based-distros) for your system.
    - Run the script:

        ```bash
        chmod +x ZaberMotionCppInstaller-7.2.2-Linux_x64.sh
        sudo ./ZaberMotionCppInstaller-7.2.2-Linux_x64.sh --prefix=/usr/local --exclude-subdir
        ```

    - Verify that the following files exist:

        - Library Files:

            ```plaintext
            /usr/local/include/zaber/
            └── motion
                ├── ascii
                ├── binary
                ├── dto
                │   ├── ascii
                │   ├── binary
                │   ├── exceptions
                │   ├── gcode
                │   ├── microscopy
                │   ├── product
                │   └── requests
                ├── exceptions
                ├── gateway
                ├── gcode
                ├── microscopy
                ├── product
                └── utils
            ```

        - Library Binaries:

            ```plaintext
            /usr/local/lib/
            ├── libzaber-motion-core.so.7.2.2
            ├── libzaber-motion.so -> libzaber-motion.so.7.2
            ├── libzaber-motion.so.7.2 -> libzaber-motion.so.7.2.2
            └── libzaber-motion.so.7.2.2
            ```

    **Note:** There is an alternative approach to building the library using [debian packages](https://software.zaber.com/motion-library/docs/tutorials/install/cpp#ubuntu-debian-based). However, that method is **NOT** recommended due to compatibility issues.

4. Build the package:

    ```bash
    colcon build --packages-select zaber_robot_driver
    ```

5. Setup udev rules:

    ```bash
    echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"' | sudo tee /etc/udev/rules.d/99-zaber.rules
    sudo udevadm control --reload-rules
    ```

## Configuration

### Hardware Parameters

```yaml
zaber_system:
  ros__parameters:
    joints:
      - insertion_joint
      - horizontal_joint
      - vertical_joint
    serial_port: /dev/ttyUSB0
    baud_rate: 115200
```

### Controller Configuration

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

velocity_controller:
  ros__parameters:
    joints: ['insertion_joint', 'horizontal_joint', 'vertical_joint']
    interface_name: velocity
```

## Usage

### Launch System

```bash
ros2 launch zaber_robot_driver zaber.launch.py
```

### Controller Management

```bash
# List controllers
ros2 control list_controllers

# Switch controllers
ros2 control switch_controllers --start velocity_controller --stop position_controller

# Send commands
ros2 topic pub /velocity_controller/commands std_msgs/Float64MultiArray "data: [0.1, 0.0, 0.0]"
```

## Technical Details

### State Machine

```mermaid
graph LR
    A[Unconfigured] --> B[Inactive]
    B --> C[Active]
    C --> B
    B --> A
```

### Movement Control

1. Position Control

   ```cpp
   void moveAbs(double position, double velocity, double accel)
   ```

2. Velocity Control

   ```cpp
   void sendVel(double velocity)
   ```

### Safety Features

1. Position Limits

   ```cpp
   bool withinRange(double position)
   {
     return lower_limit_ <= position && position <= upper_limit_;
   }
   ```

2. Emergency Stop

   ```cpp
   void stop()
   {
     axis_.stop(false);
   }
   ```

## Troubleshooting

### Common Issues

1. Connection Errors

    ```plaintext
    Problem: Cannot open serial port
    Solution:
    - Check USB connection
    - Verify device permissions
    - Confirm serial port name
    ```

2. Movement Issues

    ```plaintext
    Problem: Stage not moving
    Solution:
    - Check controller activation
    - Verify command interface
    - Check position limits
    ```

3. Home Position Errors

    ```plaintext
    Problem: Homing failed
    Solution:
    - Clear motion errors
    - Check for obstructions
    - Verify stage ROM files
    ```

### Debugging Tools

```bash
# Monitor hardware interfaces
ros2 control list_hardware_interfaces

# Check controller status
ros2 control list_controllers

# Enable debug logging
ros2 launch zaber_robot_driver zaber.launch.py --ros-args --log-level debug
```

### Error Messages

```cpp
RCLCPP_ERROR(logger, "Failed to connect to device at %s", port_.c_str());
RCLCPP_WARN(logger, "Position limit reached: %f", position);
RCLCPP_INFO(logger, "Successfully homed axis: %s", name_.c_str());
```
