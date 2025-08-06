# ROS2 Kortex Control
[![github](https://img.shields.io/badge/GitHub-ucmercedrobotics-181717.svg?style=flat&logo=github)](https://github.com/ucmercedrobotics)
[![website](https://img.shields.io/badge/Website-UCMRobotics-5087B2.svg?style=flat&logo=telegram)](https://robotics.ucmerced.edu/)
[![ros2](https://img.shields.io/badge/ROS2-Jazzy-22314E.svg?style=flat&logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![docker](https://img.shields.io/badge/Docker-Enabled-2496ED.svg?style=flat&logo=docker&logoColor=white)](https://www.docker.com)
[![pre-commits](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)

A comprehensive Docker-based environment for controlling Kinova Kortex robotic arms using ROS2. This package provides a complete setup with simulation capabilities, vision integration, and hardware control for the Gen3 series robots.

Make sure you initialize the repo with pre-commit hooks:
```bash
make repo-init
```

## Features

- **ROS2 Jazzy** support with full desktop environment
- **Kinova Kortex** robot arm control and simulation
- **Vision integration** with Kortex Vision module
- **Gazebo simulation** with Harmonic support
- **Multi-architecture** Docker builds (ARM64/AMD64)
- **VNC support** for remote visualization
- **MoveIt integration** for motion planning
- **Gripper support** (Robotiq 2F-85)

## Quick Start

### Prerequisites

- Docker with buildx support
- NVIDIA Docker runtime (for GPU acceleration)
- Make

### Docker Setup
Launch an interactive container:
```bash
make bash
```

## Usage

### Simulation Mode

Launch the robot in simulation with fake hardware:
```bash
make sim
```

This will start the Gen3 robot with:
- 6 degrees of freedom
- Vision system enabled
- Robotiq 2F-85 gripper
- Fake hardware interface

### Hardware Control

For real hardware, modify the `sim` target in the Makefile to use your robot's IP address:
```bash
ros2 launch kortex_bringup gen3.launch.py \
    robot_ip:=192.168.1.10 \
    use_fake_hardware:=false \
    dof:=6 \
    vision:=true \
    gripper:=robotiq_2f_85
```

### VNC Access

Start the VNC server for remote visualization:
```bash
make vnc
```

The VNC interface will be available on the host network.
You can access this by going into your browser and putting in `<your_ip>:8080/vnc.html` to see RViz once you run it.

### Development

Access a running container shell:
```bash
make shell
```

Clean build artifacts:
```bash
make clean
```

## Container Architecture

The Docker image includes:

- **Base**: ROS2 Jazzy desktop-full environment
- **Kortex Workspace**: Complete ros2_kortex package with dependencies
- **Vision Workspace**: Kortex vision module for camera integration
- **Development Tools**: Build tools, debugging utilities, and development packages
- **GPU Support**: NVIDIA runtime configuration for hardware acceleration

## Supported Platforms

<!-- - **AMD64** (x86_64) -->
- **ARM64** (ARMv8)

## Related Projects

- [ros2_kortex](https://github.com/ucmercedrobotics/ros2_kortex) - Core Kortex ROS2 packages
- [ros2_kortex_vision](https://github.com/Kinovarobotics/ros2_kortex_vision) - Vision integration
