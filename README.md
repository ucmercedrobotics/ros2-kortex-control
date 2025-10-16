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

## Quick Start

### Prerequisites

- Docker
- NVIDIA Docker runtime (for GPU acceleration)
- CUDA 12.2 (if working on Jetson)
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
make moveit
```

### Hardware Control

For real hardware, configure your network interface to connect to the same subnet as the Kinova
```bash
make config-target-network
```

Then launch moveit,
```bash
make moveit-target
```

### Leaf Sensing

One applied version of vision in this package is leaf segmentation.
To start,
```bash
ros2 launch kortex_vision leaf_segmentation.launch.py
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

### Vision
When testing on real hardware, there is a separate node for enabling camera streams.
Run this in addition to Kortex drivers.
```bash
make vision
```

## Supported Platforms

- **AMD64** (x86_64)
- **ARM64** (ARMv8)

## Related Projects

- [ros2_kortex](https://github.com/ucmercedrobotics/ros2_kortex) - Core Kortex ROS2 packages
- [ros2_kortex_vision](https://github.com/Kinovarobotics/ros2_kortex_vision) - Vision integration
