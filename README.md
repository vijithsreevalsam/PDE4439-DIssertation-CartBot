# 🛒 CartBot - Autonomous Supermarket Trolley
**PDE4439 Dissertation Project - Autonomous Shopping Cart Prototype**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Linux%20ARM64-green)](https://www.linux.org/)
---

##PDE4439 - Dissertation: CartBot

This repository contains the CartBot project developed as part of the PDE4439 dissertation. It demonstrates a ROS 2-based autonomous cart integrating SLAM, navigation, and UI components.

Branches

There are four branches in this repository. You are currently on the master branch. Each branch contains specific parts of the project, and detailed instructions are available within each branch.

Running the Robot
Set Environment Variable

Before launching, set the robot base type:

export cartBot_BASE=2wd

Launch via ROS 2
1. Bringup

To launch the main robot files:

ros2 launch cartBot_bringup bringup.launch.py madgwick:=true orientation_stddev:=0.01


This will initialize the hardware and sensors.

2. Navigation

You can start navigation using either the UI or the default launch file:

ros2 launch cartBot_navigation slam.launch.py rviz:=true sim:=false


This launch file includes both SLAM and Nav2.

Launch via Python UI

Navigate to the folder containing the Python UI scripts.

Run:

bash
python main.py


This will launch RViz with navigation tools and allow you to select navigation parameters and map data.

Launch via Web UI

Navigate to the folder containing the web UI scripts.

Run:

bash
python service_manager.py


This is a quick way to start the robot and access the UI from any device.

Notes

Comprehensive instructions are available per branch, detailing setup and execution.

Choose the method that best suits your environment: Python UI for direct control, or Web UI for device-independent access.

*This project represents the culmination of advanced robotics research and development, demonstrating practical applications of autonomous systems in retail environments.*
