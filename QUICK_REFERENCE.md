# 🚀 Quick Reference Card

## 🎮 End User (Consumer) Commands

### Start the System
```bash
# python service_manager.py
# Open browser to: http://localhost:8080/robot_control.html
# Click: "🚀 Start Launch Service"
# Click: "🌐 Start Rosbridge"
# Wait for "Connected" status
```

### Robot Operations
- **Set Waypoint**: Click on map
- **Navigate**: Click "Go to Waypoint"  
- **Emergency Stop**: Red "EMERGENCY STOP" button
- **Start SLAM**: Click "🗺️ Launch SLAM"
- **Start Navigation**: Click "🧭 Launch Navigation"


## 📊 Status Indicators

### Web Interface Status
- 🟢 **Connected**: ROS communication active
- 🔴 **Disconnected**: No ROS connection
- ⏳ **Connecting**: Connection in progress
- 🟢 **Running (PID: XXX)**: Service active with process ID
- 🔴 **Not Running**: Service inactive

### CLI Status Symbols
- ✅ **Active/Success**: Component working
- ❌ **Inactive/Failed**: Component not working
- 🟢 **Online**: Topic/node available
- 🔴 **Offline**: Topic/node not found
- ⏳ **Processing**: Operation in progress

---

## 🚨 Emergency Procedures

### Robot Emergency Stop
1. **Web Interface**: Click red "EMERGENCY STOP" button
2. **CLI Check**: `ros2 topic echo /cmd_vel` (should show zeros)
3. **Manual CLI**: `ros2 topic pub /cmd_vel geometry_msgs/Twist '{}'`

### System Recovery
```bash
# If system is unresponsive


# If port conflicts
lsof -i :8080
kill <PID>

# If ROS issues
ros2 daemon stop
ros2 daemon start
```

---

## 📂 Key Files

- `robot_control.html` - Main web interface
- `service_manager.py` - HTTP server (port 8080)
- `launch_service_node.py` - ROS launch service
- `README.md` - Complete documentation

---

## 🎯 Quick Workflows

### Consumer Workflow
1. Browser → `http://localhost:8080/robot_control.html`
2. Start Services → "🚀 Start Launch Service" → "🌐 Start Rosbridge"
3. Use Robot → Set waypoints, control navigation, emergency stop





---

*Quick Reference - Robot Control Web Interface*
