# 🤖 Robot Control Web Interface -

## 📋 Table of Contents
- [Overview](#overview)
- [Features](#features)
- [File Structure](#file-structure)


---

## 🎯 Overview

This project provides a **consumer-level web interface** for controlling ROS2-based robots. It evolved from a basic waypoint system to a comprehensive robot control platform with emergency safety features, system service management, and developer monitoring tools.

### ✨ Key Achievements
- **Zero CLI Required**: Complete robot operation through web interface
- **Safety First**: Emergency stop with velocity override
- **Consumer Ready**: One-click service management


---

## 🚀 Features

### 🌐 Web Interface (`robot_control.html`)
- **Interactive Map**: Leaflet.js-based mapping with robot visualization
- **Waypoint System**: Single goal and multi-waypoint navigation modes
- **Emergency Stop**: True safety system with velocity command override
- **System Services**: Start/stop ROS services directly from web UI
- **Launch Controls**: SLAM and Navigation system management
- **Real-time Status**: Live monitoring of ROS nodes and topics from browser inspect consoles
- **Connection Management**: Smart ROS connection with manual disconnect option

---

## 📁 File Structure

```
/home/viju/websocket_ws/
├── robot_control.html           # Main web interface
├── service_manager.py        # HTTP server with service management
├── launch_service_node.py    # ROS service node for launch commands
```



## 📝 Summary

This robot control interface represents a complete evolution from basic waypoint functionality to a production-ready, consumer-level robot control system. It provides:

- **Complete Consumer Experience**: No CLI knowledge required
- **Safety-First Design**:  emergency stop with velocity override
- **Map view**: Comprehensive monitoring of robot postion relative to map


The system bridges the gap between complex robotics systems and user-friendly interfaces, making advanced robot control accessible to end users while providing powerful tools for developers.

---


*Project: cartBot - Robot Control Web Interface
*Auther: Vijith viswan