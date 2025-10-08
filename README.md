# 🛒 CartBot - Autonomous Supermarket Trolley
**PDE4439 Dissertation Project - Autonomous Shopping Cart Prototype**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20ARM64-green)](https://www.linux.org/)
---

#PDE4439 - Dissertation: CartBot

## CartBot — PDE4439 Dissertation

This repository contains the CartBot ROS project developed for the PDE4439 dissertation. It demonstrates a ROS 2-based autonomous cart integrating SLAM, navigation (Nav2), and UI components (Python and Web).

### Table of contents
- Overview
- Branches
- Prerequisites
- Environment
- Launching
    - Bringup
    - Navigation (SLAM + Nav2)
    - Python UI
    - Web UI
- Notes

### Overview
Concise, runnable examples and branch-specific setup are included. Choose the launch method that best fits your environment: use the Python UI for direct operator control or the Web UI for remote/device-independent access.

### Branches
There are four branches in this repository. You are currently on the `master` branch. Each branch isolates a part of the project (hardware bringup, navigation stacks, UI tools, experiments). See each branch’s README for branch-specific setup and troubleshooting.

### Prerequisites
- ROS 2 (distribution used for the project)
- Nav2 and SLAM packages
- Python 3 and required UI dependencies
- Hardware drivers as required by your robot base

Install dependencies per-branch as documented in each branch README.

### Environment
Set the robot base type before launching:
```bash
export cartBot_BASE=2wd
```
Adjust the value to match your platform (e.g., `2wd`, `4wd`, etc.).

### Launching

Bringup
```bash
ros2 launch cartBot_bringup bringup.launch.py madgwick:=true orientation_stddev:=0.01
```
Initializes hardware and sensors required for the robot.

Navigation (SLAM + Nav2)
```bash
ros2 launch cartBot_navigation slam.launch.py rviz:=true sim:=false
```
Launches SLAM and Nav2. Set `rviz:=true` to open RViz and `sim:=true` if running in simulation.

Python UI
1. Change to the folder with the Python UI scripts.
2. Launch:
```bash
bash python main.py
```
This opens RViz with navigation tools for selecting parameters and map data.

Web UI
1. Change to the folder with the web UI scripts.
2. Start the service:
```bash
bash python service_manager.py
```
Provides a device-independent way to start the robot and access the UI from other devices on the network.

### Notes
- Detailed, step-by-step instructions per branch are available in those branches’ READMEs.
- Choose Python UI for local control and debugging; use Web UI for remote access or multi-device testing.
- Logs, parameter files, and maps are stored per-branch; consult branch documentation for their locations.

License and attribution information is included at the repository root.
