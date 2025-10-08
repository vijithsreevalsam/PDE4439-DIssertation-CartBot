# Robot Control UI

A comprehensive Python-based robot control interface with RViz integration for teleoperation, navigation, and waypoint management.

## Features

🎮 **Teleoperation Control**
- Button control modes
- Adjustable speed settings
- Real-time velocity monitoring
- Emergency stop functionality

🧭 **Navigation System**
- SLAM mapping support
- Nav2 navigation integration
- Map selection and management
- Real-time navigation status

📍 **Waypoint Management**
- Save current robot position as waypoints
- Manual waypoint entry
- One-click navigation to saved points
- Waypoint editing and deletion

📊 **Status Monitoring**
- Real-time robot position and velocity
- ROS connection status
- System performance metrics
- Event logging

🗺️ **RViz Integration**
- Automatic RViz launching
- Navigation goal setting
- Map visualization
- Camera controls

## Installation

### Prerequisites

1. **ROS 2 (Jazzy or Humble)**
   ```bash
   # For Ubuntu 22.04/24.04
   sudo apt update
   sudo apt install ros-jazzy-desktop
   # or
   sudo apt install ros-humble-desktop
   ```

2. **Navigation and SLAM packages**
   ```bash
   sudo apt install ros-$ROS_DISTRO-navigation2
   sudo apt install ros-$ROS_DISTRO-slam-toolbox
   sudo apt install ros-$ROS_DISTRO-rviz2
   sudo apt install ros-$ROS_DISTRO-nav2-simple-commander
   ```

3. **Python dependencies**
   ```bash
   pip3 install PySide6
   ```

### Setup

1. **Clone or copy the UI to your system**
   ```bash
   # The UI is already in /home/viju/UI
   cd /home/viju/UI
   ```

2. **Make launch script executable**
   ```bash
   chmod +x launch_ui.sh
   ```

3. **Install Python requirements**
   ```bash
   pip3 install -r requirements.txt
   ```

## Usage

### Quick Start

1. **Launch the UI**
   ```bash
   ./launch_ui.sh
   ```

2. **Launch Robot Control**
   - Click "🚀 Launch Robot Control Center" in the launcher window
   - This will automatically start RViz and open the main control interface

### Basic Workflow

1. **Setup Navigation (if using real robot)**
   - Go to Navigation tab
   - Start SLAM to create a map, or
   - Start Navigation with an existing map

2. **Teleoperation**
   - Use the Teleop tab for manual robot control
   - Choose between button, keyboard, or joystick modes
   - Adjust speed with sliders

3. **Waypoint Navigation**
   - Add waypoints using current position or manual coordinates
   - Navigate to saved waypoints with one click
   - Manage waypoints (edit/delete)

4. **Monitor Status**
   - Check robot position and velocity in Status tab
   - Monitor ROS connection and system health
   - View event logs

## Configuration

### Waypoints
Waypoints are automatically saved to `/home/viju/UI/config/waypoints.json`

### RViz Configuration
The UI will try to use RViz configs in this order:
1. `/home/viju/my_rviz_config.rviz`
2. `/home/viju/ros_ws/src/cartBot/cartBot_navigation/rviz/cartBot_navigation.rviz`
3. `/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz`
4. Default RViz configuration

### Maps
The UI looks for maps in:
1. `/home/viju/ros_ws/src/cartBot/cartBot_navigation/maps`
2. `/home/viju/maps`
3. `/home/viju`


## RViz Usage

### Setting Navigation Goals
1. Ensure Navigation system is running
2. In RViz, select the "2D Nav Goal" tool
3. Click and drag on the map to set target location and orientation
4. Robot will automatically navigate to the goal

### Setting Initial Pose
1. Select "2D Pose Estimate" tool in RViz
2. Click and drag on map where robot actually is
3. This helps localize the robot on the map

## Troubleshooting

### ROS Connection Issues
- Ensure ROS 2 is properly sourced: `source /opt/ros/jazzy/setup.bash`
- Check if robot nodes are running: `ros2 node list`
- Verify topic availability: `ros2 topic list`

### Navigation Issues
- Clear costmaps using the "Clear Costmaps" button
- Restart navigation system
- Check if map is properly loaded
- Ensure robot localization is working

### RViz Issues
- Install RViz if missing: `sudo apt install ros-jazzy-rviz2`
- Check display plugins: `sudo apt install ros-jazzy-rviz-default-plugins`
- Reset RViz configuration if corrupted

### UI Issues
- Check Python dependencies: `pip3 list | grep PySide6`
- Run with verbose output: `python3 -v src/main.py`
- Check logs in the Status tab

## File Structure

```
UI/
├── src/
│   ├── main.py                 # Main entry point
│   ├── windows/               # UI window classes
│   ├── widgets/               # UI widget components
│   ├── ros/                   # ROS interface
│   └── utils/                 # Utilities and config
|
├── requirements.txt           # Python dependencies
├── setup.py                  # Package setup
├── launch_ui.sh              # Launch script
└── README.md                 # This file
```

## Default Waypoints

The system comes with these predefined waypoints:
- **home**: (0.0, 0.0) - Origin/starting position
- **kitchen**: (2.0, 1.0) - Kitchen area
- **living_room**: (-1.0, 2.0) - Living room
- **fruits**: (3.0, -1.0) - Fruits/grocery area
- **charging_station**: (-2.0, -1.0) - Robot charging station

## Advanced Features

### Custom Map Creation
1. Start SLAM mode
2. Drive robot around to build map
3. Save map using "Save Map" button
4. Use saved map for navigation

### Batch Waypoint Navigation
- Add multiple waypoints
- Navigate sequentially by selecting each one
- Monitor progress in Status tab

### System Integration
- Integrates with existing ROS 2 navigation stack
- Compatible with cartBot and other robot platforms - 
- where cartBot actully forked from Linorobot
- Extensible architecture for custom features

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review ROS 2 navigation documentation
3. Ensure all dependencies are properly installed
4. Check system logs in the Status tab

## License

This project is licensed under the MIT License.