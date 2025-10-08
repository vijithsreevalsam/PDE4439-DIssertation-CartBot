# 🚀 Quick Reference Card

## 🎮 End User (Consumer) Commands

### Start the System
```bash
# Open browser to: http://localhost:8080/map_viewer.html
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

---

## 🔧 Developer Commands

### Essential Dev Commands
```bash
cd /home/viju/websocket_ws

# Quick status check
./dev_tools.sh

# Start development environment
./dev_tools.sh start-launch

# Real-time monitoring (Terminal 1)
./dev_tools.sh status -c

# Monitor launch activity (Terminal 2)
./dev_tools.sh logs
```

### Debugging Commands
```bash
# Check specific components
./dev_tools.sh nodes          # List ROS nodes
./dev_tools.sh topics         # List ROS topics
./dev_tools.sh processes      # Show ROS processes
./dev_tools.sh launch-status  # Check launch service

# Service management
./dev_tools.sh start-launch   # Start launch service
./dev_tools.sh stop-launch    # Stop launch service
./dev_tools.sh restart-launch # Restart launch service
```

---

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
./dev_tools.sh restart-launch

# If port conflicts
lsof -i :8080
kill <PID>

# If ROS issues
ros2 daemon stop
ros2 daemon start
```

---

## 📂 Key Files

- `map_viewer.html` - Main web interface
- `service_manager.py` - HTTP server (port 8080)
- `launch_service_node.py` - ROS launch service
- `dev_tools.sh` - Developer CLI toolkit
- `README.md` - Complete documentation

---

## 🎯 Quick Workflows

### Consumer Workflow
1. Browser → `http://localhost:8080/map_viewer.html`
2. Start Services → "🚀 Start Launch Service" → "🌐 Start Rosbridge"
3. Use Robot → Set waypoints, control navigation, emergency stop

### Developer Workflow
1. Terminal → `./dev_tools.sh start-launch`
2. Monitor → `./dev_tools.sh status -c` (Terminal 1)
3. Debug → `./dev_tools.sh logs` (Terminal 2)
4. Test → Use web interface while monitoring

### Troubleshooting Workflow
1. Status → `./dev_tools.sh`
2. Identify → Check specific components
3. Debug → `./dev_tools.sh logs`
4. Fix → Restart services or fix code
5. Verify → `./dev_tools.sh status`

---

*Quick Reference - Robot Control Web Interface*
