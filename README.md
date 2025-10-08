# 🛒 CartBot - Autonomous Supermarket Trolley
**PDE4439 Dissertation Project - Autonomous Shopping Cart Prototype**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20ARM64-green)](https://www.linux.org/)

## 📋 Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [System Architecture](#-system-architecture)
- [Hardware Components](#-hardware-components)
- [Software Stack](#-software-stack)
- [Installation](#-installation)
- [Usage](#-usage)
- [Project Structure](#-project-structure)
- [Development](#-development)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

CartBot is an autonomous supermarket trolley prototype developed as part of the PDE4439 dissertation project. This project demonstrates advanced robotics concepts including autonomous navigation, voice control, SLAM mapping, and intelligent shopping assistance features.

### ✨ Key Achievements
- **🗣️ Voice Control**: Natural language navigation commands with real-time processing
- **🗺️ Autonomous Navigation**: SLAM-based mapping and Nav2 navigation stack
- **🎮 Multiple Control Interfaces**: PyQt5 GUI, Web interface, and voice commands
- **🛡️ Safety Features**: Emergency stop, velocity monitoring
- **📍 Smart Waypoints**: Supermarket product-based navigation system

---

## 🚀 Features

### 🧭 Navigation & Mapping
- **SLAM Mapping**: Real-time simultaneous localization and mapping
- **Nav2 Integration**: Production-ready navigation stack
- **Dynamic Path Planning**: Obstacle avoidance and re-routing
- **Multi-Goal Navigation**: Sequential waypoint navigation
- **Voice-Controlled Navigation**: "Go to apple", "Take me to rice" commands

### 🎛️ Control Interfaces
- **PyQt5 Desktop GUI**: Comprehensive robot control with RViz integration
- **Web-Based Interface**: Browser-based control for any device
- **Voice Commands**: Natural language processing for navigation
- **Teleoperation**: Manual control with adjustable speed settings

### 🛡️ Safety & Monitoring
- **Emergency Stop System**: Hardware and software-based safety stops
- **Real-Time Monitoring**: Position, velocity, and system status
- **Collision Detection**: Ultrasonic sensor integration
- **Connection Management**: Robust ROS communication handling

### 🔧 Developer Tools
- **Service Management**: Web-based ROS service control
- **System Diagnostics**: Real-time node and topic monitoring
- **Debug Interfaces**: Comprehensive logging and error handling
- **Modular Architecture**: Easy to extend and customize

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    CartBot System Architecture              │
├─────────────────────┬───────────────────┬───────────────────┤
│   Control Layer    │   Navigation      │   Hardware        │
│                     │   Layer           │   Layer           │
│ ┌─────────────────┐ │ ┌───────────────┐ │ ┌───────────────┐ │
│ │ PyQt5 GUI       │ │ │ Nav2 Stack    │ │ │ Teensy 4.1    │ │
│ │ Web Interface   │ │ │ SLAM/AMCL     │ │ │ RPLiDAR S1    │ │
│ │ Voice Commands  │ │ │ Path Planning │ │ │ IMU Sensor    │ │
│ └─────────────────┘ │ │ Local/Global  │ │ │ Ultrasonic    | |
|                     │ │               | | |(Future)       | |
│                     │ │ Costmaps      │ │ │ Motors+Wheels │ │
│ ┌─────────────────┐ │ └───────────────┘ │ └───────────────┘ │
│ │ ROS 2 Services  │ │                   │                   │
│ │ Topic Bridge    │ │ ┌───────────────┐ │ ┌───────────────┐ │
│ │ WebSocket API   │ │ │ Localization  │ │ │ Micro-ROS     │ │
│ └─────────────────┘ │ │ Mapping       │ │ │ Sensor Fusion │ │
└─────────────────────┴─│ Waypoint Mgmt │─┴─│ Motor Control │─┘
                        └───────────────┘   └───────────────┘
```

---

## 🔧 Hardware Components

| Component | Model | Purpose |
|-----------|-------|---------|
| **Main Controller** | Teensy 4.1 | Micro-ROS firmware, sensor integration |
| **LiDAR** | RPLiDAR S1 | 2D laser scanning for SLAM and navigation |
| **IMU** | MPU6050/9250 | Orientation and motion sensing |
| **Ultrasonic** | HC-SR04 | Collision detection and safety |
| **Motors** | Gear Motors + Encoders | Differential drive system |
| **Computer** | Raspberry Pi/Jetson | ROS 2 stack, navigation processing |
| **Audio** | USB Microphone | Voice command input |

---

## 💻 Software Stack

### Core Technologies
- **ROS 2 Humble**: Robot operating system
- **Nav2**: Navigation framework
- **SLAM Toolbox**: Mapping and localization
- **Micro-ROS**: Embedded ROS communication
- **PyQt5**: Desktop GUI framework
- **WebSocket**: Real-time web communication
- **Vosk**: Offline speech recognition

### Key Packages
```
cartBot/
├── cartBot_base/          # Base controller and hardware interface
├── cartBot_description/   # Robot model and URDF files
├── cartBot_navigation/    # Navigation configuration and launch
├── cartBot_bringup/       # System startup and configuration
└── cartBot/               # Meta-package
```

---

## 🚀 Installation

### Prerequisites
```bash
# ROS 2 Jazzy installation
sudo apt update
sudo apt install ros-jazzy-desktop-full

# Required dependencies
sudo apt install python3-pip python3-colcon-common-extensions
sudo apt install ros-jazzy-nav2-* ros-humble-slam-toolbox
sudo apt install ros-jazzy-micro-ros-* 
```

### Clone and Build
```bash
# Clone the repository
git clone https://github.com/vijithsreevalsam/PDE4439-DIssertation-CartBot.git
cd PDE4439-DIssertation-CartBot

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Install UI Components
```bash
# Install PyQt5 GUI
cd ~/UI
pip3 install -r requirements.txt
python3 setup.py install

# Install Web Interface dependencies
cd ~/websocket_ws
# No additional installation required - pure HTML/JS
```

---

## 📖 Usage

### 1. Hardware Setup
```bash
# Flash Teensy firmware
cd ~/Downloads
# Upload firmware.ino to Teensy 4.1 via Arduino IDE

# Connect hardware components
# - RPLiDAR S1 to USB
# - Teensy 4.1 to USB  
# - Ultrasonic sensors to Teensy pins
# - Motors to motor driver
```

### 2. Launch Robot System
```bash
# Terminal 1: Launch robot base
ros2 launch cartBot_bringup bringup.launch.py madgwick:=true orientation_stddev:=0.01

# Terminal 2: Launch navigation
ros2 launch cartBot_navigation navigation.launch.py

# Terminal 3: Launch SLAM (for mapping)
ros2 launch cartBot_navigation slam.launch.py rviz:=true sim:=false
```

### 3. Control Interfaces

#### PyQt5 GUI (Recommended)
```bash
cd ~/UI/src
python3 main.py
```

#### Web Interface
```bash
cd ~/websocket_ws
python3 service_manager.py
# Open http://localhost:8080/robot_control.html
```

#### Voice Control
```bash
# Included in PyQt5 GUI - click "Start Voice" button
# Say commands like:
# - "Go to apple"
# - "Take me to rice" 
# - "Navigate to checkout"
```

### 4. Mapping New Environments
```bash
# Start SLAM
ros2 launch cartBot_navigation mapping.launch.py

# Drive robot around using GUI or web interface
# Save map when complete
ros2 run nav2_map_server map_saver_cli -f ~/maps/new_map
```

---

## 📁 Project Structure

```
PDE4439-DIssertation-CartBot/
├── 📁 src/cartBot/                    # Main ROS 2 packages
│   ├── 📁 cartBot_base/               # Hardware interface
│   ├── 📁 cartBot_description/        # Robot model (URDF/meshes)
│   ├── 📁 cartBot_navigation/         # Navigation configs
│   └── 📁 cartBot_bringup/            # Launch files
├── 📁 sllidar_ros2/                   # LiDAR driver
├── 📁 src/micro_ros_setup/            # Micro-ROS tools
└── 📄 README.md                       # This file

~/UI/                                  # PyQt5 Desktop Interface
├── 📁 src/                            # Source code
│   ├── 📄 main.py                     # GUI application entry
│   └── 📁 widgets/                    # UI components
├── 📁 config/                         # Configuration files
└── 📄 requirements.txt                # Python dependencies

~/websocket_ws/                        # Web Interface
├── 📄 robot_control.html              # Main web interface
├── 📄 service_manager.py              # Backend service
├── 📄 map_viewer.html                 # Map visualization
└── 📄 supermarket_viewer.html         # Product navigation

~/ros_ws/                              # Development workspace
├── 📁 src/linorobot2/                 # Base robot platform
├── 📁 src/linorobot2_hardware/        # Hardware drivers
└── 📁 maps/                           # Saved maps

~/Downloads/                           # Firmware
├── 📄 firmware.ino                    # Main Teensy firmware
├── 📄 firmware_ultrasound.ino         # Sensor integration
└── 📁 ultrasonic/                     # Sensor libraries
```

---

## 🛠️ Development

### Adding New Voice Commands
```python
# In UI/src/widgets/navigation_widget.py
navigation_triggers = [
    "go to", "take me to", "navigate to", 
    "find", "head to", "travel to"
]

# Add new waypoints in config/waypoints.json
{
    "apple": {"x": 1.5, "y": 2.0},
    "rice": {"x": 3.2, "y": 1.8},
    "checkout": {"x": 0.0, "y": 0.0}
}
```

### Custom Navigation Behaviors
```bash
# Edit navigation parameters
~/PDE4439-DIssertation-CartBot/src/cartBot/cartBot_navigation/config/nav2_params.yaml

# Adjust path planning, recovery behaviors, and safety margins
```

### Hardware Modifications
```cpp
// Modify firmware for new sensors
// ~/Downloads/firmware.ino

// Add new ROS message types
#include <sensor_msgs/msg/your_sensor.h>

// Initialize publishers/subscribers
rcl_publisher_t your_sensor_publisher;
```

---

## 🐛 Troubleshooting

### Common Issues

#### Robot Not Moving
```bash
# Check ROS topics
ros2 topic list
ros2 topic echo /cmd_vel

# Verify hardware connections
ls /dev/ttyACM*  # Should show Teensy connection
ls /dev/ttyUSB*  # Should show LiDAR connection
```

#### Voice Commands Not Working
```bash
# Check audio device
arecord -l
# Ensure microphone is connected and recognized

# Test voice recognition
cd ~/UI/src
python3 -c "from utils.audio_node import AudioProcessingNode; print('Voice module loaded')"
```

#### Navigation Failures
```bash
# Check map and localization
ros2 run rviz2 rviz2 -d ~/cartBot_navigation/rviz/navigation.rviz

# Reset localization
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

#### Web Interface Issues
```bash
# Check service manager
cd ~/websocket_ws
python3 service_manager.py
# Should start on http://localhost:8080

# Check ROS bridge
ros2 run rosbridge_server rosbridge_websocket
```

### Performance Optimization
- **CPU Usage**: Reduce LiDAR scan frequency in `rplidar.launch.py`
- **Memory**: Limit map resolution in `slam_params.yaml`
- **Network**: Use `DDS_DOMAIN_ID` for multiple robots
- **Storage**: Clean old log files with `ros2 log purge`

---

## 🤝 Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Development Guidelines
- Follow ROS 2 [coding standards](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Test all changes in simulation before hardware
- Update documentation for new features
- Add unit tests for critical components

---

## 📚 Documentation Links

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Navigation Framework](https://navigation.ros.org/)
- [Micro-ROS Documentation](https://micro.ros.org/)
- [PyQt5 Documentation](https://doc.qt.io/qtforpython/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

## 👥 Authors

- **Vijith Sreevalsam** - *Lead Developer* - [vijithsreevalsam](https://github.com/vijithsreevalsam)

---

## 🙏 Acknowledgments

- Based on [LinoRobot2](https://github.com/linorobot/linorobot2) platform
- RPLiDAR integration from [Slamtec](https://github.com/Slamtec/sllidar_ros2)
- Navigation stack from [ROS 2 Navigation Working Group](https://github.com/ros-planning/navigation2)
- Voice recognition using [Vosk](https://alphacephei.com/vosk/) speech toolkit
- Lots of thanks to Prof. Dr Sameer Kishore , Prof.Dr Judhi Prasetyo ,Prof.Ziad Burhani

---

## 📊 Project Status

**Current Version**: v1.0.0  
**Status**: ✅ Complete - Dissertation Submission Ready  
**Last Updated**: October 2025

### Completed Features ✅
- [x] Autonomous navigation with SLAM
- [x] Voice command integration
- [x] Multi-interface control (GUI/Web/Voice)
- [x] Safety systems and emergency stops
- [x] Real-time monitoring and diagnostics
- [x] Supermarket-specific waypoint system
- [x] Hardware integration (LiDAR, IMU, Ultrasonic)
- [x] Comprehensive documentation

### Future Enhancements 🚀
- [ ] Machine learning-based path optimization
- [ ] Multi-robot coordination
- [ ] Advanced object detection and recognition
- [ ] Mobile app development
- [ ] Cloud integration for fleet management

---

*This project represents the culmination of advanced robotics research and development, demonstrating practical applications of autonomous systems in retail environments.*
