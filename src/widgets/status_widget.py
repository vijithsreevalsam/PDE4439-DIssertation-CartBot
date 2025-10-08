from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
                               QGroupBox, QProgressBar, QTextEdit, QScrollArea,
                               QFrame, QGridLayout)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from datetime import datetime
import math

class StatusWidget(QWidget):
    """Widget displaying robot status information."""
    
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.log_messages = []
        self.setup_ui()
        
        # Timer for updating displays
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_displays)
        self.update_timer.start(1000)  # Update every second
        
    def setup_ui(self):
        """Setup the status UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # Robot Position
        pos_group = QGroupBox("📍 Robot Position")
        pos_layout = QGridLayout(pos_group)
        
        pos_layout.addWidget(QLabel("X Position:"), 0, 0)
        self.pos_x_label = QLabel("-- m")
        self.pos_x_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        pos_layout.addWidget(self.pos_x_label, 0, 1)
        
        pos_layout.addWidget(QLabel("Y Position:"), 1, 0)
        self.pos_y_label = QLabel("-- m")
        self.pos_y_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        pos_layout.addWidget(self.pos_y_label, 1, 1)
        
        pos_layout.addWidget(QLabel("Orientation:"), 2, 0)
        self.pos_theta_label = QLabel("-- rad")
        self.pos_theta_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        pos_layout.addWidget(self.pos_theta_label, 2, 1)
        
        layout.addWidget(pos_group)
        
        # Velocity Status
        vel_group = QGroupBox("🚀 Current Velocity")
        vel_layout = QGridLayout(vel_group)
        
        vel_layout.addWidget(QLabel("Linear:"), 0, 0)
        self.vel_linear_label = QLabel("-- m/s")
        self.vel_linear_label.setStyleSheet("font-weight: bold; color: #4CAF50;")
        vel_layout.addWidget(self.vel_linear_label, 0, 1)
        
        vel_layout.addWidget(QLabel("Angular:"), 1, 0)
        self.vel_angular_label = QLabel("-- rad/s")
        self.vel_angular_label.setStyleSheet("font-weight: bold; color: #4CAF50;")
        vel_layout.addWidget(self.vel_angular_label, 1, 1)
        
        layout.addWidget(vel_group)
        
        # System Status
        system_group = QGroupBox("🔧 System Status")
        system_layout = QVBoxLayout(system_group)
        
        # ROS Connection
        connection_layout = QHBoxLayout()
        connection_layout.addWidget(QLabel("ROS Connection:"))
        self.connection_label = QLabel("❌ Disconnected")
        self.connection_label.setStyleSheet("font-weight: bold;")
        connection_layout.addWidget(self.connection_label)
        connection_layout.addStretch()
        system_layout.addLayout(connection_layout)
        
        # Navigation status
        nav_layout = QHBoxLayout()
        nav_layout.addWidget(QLabel("Navigation:"))
        self.nav_status_label = QLabel("⏹️ Idle")
        self.nav_status_label.setStyleSheet("font-weight: bold;")
        nav_layout.addWidget(self.nav_status_label)
        nav_layout.addStretch()
        system_layout.addLayout(nav_layout)
        
        # Robot state
        state_layout = QHBoxLayout()
        state_layout.addWidget(QLabel("Robot State:"))
        self.robot_state_label = QLabel("🤖 Ready")
        self.robot_state_label.setStyleSheet("font-weight: bold; color: #FF9800;")
        state_layout.addWidget(self.robot_state_label)
        state_layout.addStretch()
        system_layout.addLayout(state_layout)
        
        # Uptime
        uptime_layout = QHBoxLayout()
        uptime_layout.addWidget(QLabel("Uptime:"))
        self.uptime_label = QLabel("00:00:00")
        self.uptime_label.setStyleSheet("font-weight: bold; color: #9C27B0;")
        uptime_layout.addWidget(self.uptime_label)
        uptime_layout.addStretch()
        system_layout.addLayout(uptime_layout)
        
        layout.addWidget(system_group)
        
        # Performance metrics
        perf_group = QGroupBox("📊 Performance Metrics")
        perf_layout = QVBoxLayout(perf_group)
        
        # CPU usage (simulated)
        cpu_layout = QHBoxLayout()
        cpu_layout.addWidget(QLabel("CPU Usage:"))
        self.cpu_bar = QProgressBar()
        self.cpu_bar.setRange(0, 100)
        self.cpu_bar.setValue(25)
        self.cpu_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
            }
            QProgressBar::chunk {
                background-color: #4CAF50;
                width: 20px;
            }
        """)
        cpu_layout.addWidget(self.cpu_bar)
        self.cpu_label = QLabel("25%")
        self.cpu_label.setMinimumWidth(40)
        cpu_layout.addWidget(self.cpu_label)
        perf_layout.addLayout(cpu_layout)
        
        # Memory usage (simulated)
        mem_layout = QHBoxLayout()
        mem_layout.addWidget(QLabel("Memory:"))
        self.mem_bar = QProgressBar()
        self.mem_bar.setRange(0, 100)
        self.mem_bar.setValue(45)
        self.mem_bar.setStyleSheet("""
            QProgressBar::chunk {
                background-color: #2196F3;
            }
        """)
        mem_layout.addWidget(self.mem_bar)
        self.mem_label = QLabel("45%")
        self.mem_label.setMinimumWidth(40)
        mem_layout.addWidget(self.mem_label)
        perf_layout.addLayout(mem_layout)
        
        layout.addWidget(perf_group)
        
        # Log display
        log_group = QGroupBox("📝 System Log")
        log_layout = QVBoxLayout(log_group)
        
        self.log_display = QTextEdit()
        self.log_display.setMaximumHeight(200)
        self.log_display.setReadOnly(True)
        self.log_display.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: 'Courier New', monospace;
                font-size: 11px;
                border: 1px solid #555;
            }
        """)
        log_layout.addWidget(self.log_display)
        
        layout.addWidget(log_group)
        
        # Initialize with startup message
        self.add_log_message("🤖 Robot Control UI initialized", "INFO")
        
        # Track startup time
        self.startup_time = datetime.now()
        
    def update_displays(self):
        """Update all status displays."""
        self.update_position_display()
        self.update_velocity_display()
        self.update_connection_status()
        self.update_performance_metrics()
        self.update_uptime()
        
    def update_position_display(self):
        """Update position display."""
        if not self.ros_interface:
            return
            
        pose = self.ros_interface.get_current_pose()
        if pose:
            self.pos_x_label.setText(f"{pose.position.x:.3f} m")
            self.pos_y_label.setText(f"{pose.position.y:.3f} m")
            
            # Convert quaternion to euler for theta
            theta = self.quaternion_to_euler(pose.orientation)
            self.pos_theta_label.setText(f"{theta:.3f} rad ({math.degrees(theta):.1f}°)")
        else:
            self.pos_x_label.setText("-- m")
            self.pos_y_label.setText("-- m")
            self.pos_theta_label.setText("-- rad")
        
    def update_velocity_display(self):
        """Update velocity display."""
        if not self.ros_interface:
            return
            
        vel = self.ros_interface.get_current_velocity()
        if vel:
            self.vel_linear_label.setText(f"{vel.linear.x:.3f} m/s")
            self.vel_angular_label.setText(f"{vel.angular.z:.3f} rad/s")
        else:
            self.vel_linear_label.setText("-- m/s")
            self.vel_angular_label.setText("-- rad/s")
        
    def update_connection_status(self):
        """Update connection status."""
        if self.ros_interface and self.ros_interface.is_connected():
            self.connection_label.setText("✅ Connected")
            self.connection_label.setStyleSheet("color: green; font-weight: bold;")
            self.robot_state_label.setText("🤖 Active")
            self.robot_state_label.setStyleSheet("font-weight: bold; color: #4CAF50;")
        else:
            self.connection_label.setText("❌ Disconnected")
            self.connection_label.setStyleSheet("color: red; font-weight: bold;")
            self.robot_state_label.setText("⚠️ Offline")
            self.robot_state_label.setStyleSheet("font-weight: bold; color: #f44336;")
            
    def update_performance_metrics(self):
        """Update performance metrics (simulated)."""
        import random
        
        # Simulate CPU usage
        cpu_usage = random.randint(20, 80)
        self.cpu_bar.setValue(cpu_usage)
        self.cpu_label.setText(f"{cpu_usage}%")
        
        # Simulate memory usage
        mem_usage = random.randint(30, 70)
        self.mem_bar.setValue(mem_usage)
        self.mem_label.setText(f"{mem_usage}%")
        
        # Update progress bar colors based on usage
        if cpu_usage > 80:
            self.cpu_bar.setStyleSheet(self.cpu_bar.styleSheet().replace("#4CAF50", "#f44336"))
        elif cpu_usage > 60:
            self.cpu_bar.setStyleSheet(self.cpu_bar.styleSheet().replace("#4CAF50", "#FF9800"))
        else:
            self.cpu_bar.setStyleSheet(self.cpu_bar.styleSheet().replace("#f44336", "#4CAF50").replace("#FF9800", "#4CAF50"))
            
    def update_uptime(self):
        """Update system uptime."""
        uptime = datetime.now() - self.startup_time
        total_seconds = int(uptime.total_seconds())
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.uptime_label.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        
    def quaternion_to_euler(self, q):
        """Convert quaternion to euler angle (yaw)."""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def add_log_message(self, message, level="INFO"):
        """Add message to log display."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        # Color code by level
        color_map = {
            "INFO": "#ffffff",
            "WARN": "#FFA500", 
            "ERROR": "#ff6b6b",
            "SUCCESS": "#4CAF50"
        }
        color = color_map.get(level, "#ffffff")
        
        formatted_message = f'<span style="color: #888">[{timestamp}]</span> <span style="color: {color}">[{level}]</span> {message}'
        
        self.log_display.append(formatted_message)
        self.log_messages.append((timestamp, level, message))
        
        # Keep only last 100 messages
        if len(self.log_messages) > 100:
            self.log_messages.pop(0)
        
        # Auto-scroll to bottom
        scrollbar = self.log_display.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
        
    def log_navigation_event(self, event_type, details=""):
        """Log navigation-related events."""
        if event_type == "goal_set":
            self.add_log_message(f"🎯 Navigation goal set: {details}", "INFO")
            self.nav_status_label.setText("🧭 Navigating")
            self.nav_status_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        elif event_type == "goal_reached":
            self.add_log_message(f"✅ Navigation goal reached: {details}", "SUCCESS")
            self.nav_status_label.setText("✅ Goal Reached")
            self.nav_status_label.setStyleSheet("font-weight: bold; color: #4CAF50;")
        elif event_type == "goal_cancelled":
            self.add_log_message(f"❌ Navigation cancelled: {details}", "WARN")
            self.nav_status_label.setText("⏹️ Idle")
            self.nav_status_label.setStyleSheet("font-weight: bold; color: #666;")
        elif event_type == "slam_started":
            self.add_log_message("🗺️ SLAM mapping started", "INFO")
        elif event_type == "slam_stopped":
            self.add_log_message("⏹️ SLAM mapping stopped", "INFO")
        elif event_type == "nav_started":
            self.add_log_message(f"🧭 Navigation system started: {details}", "INFO")
        elif event_type == "nav_stopped":
            self.add_log_message("⏹️ Navigation system stopped", "INFO")
            
    def log_teleop_event(self, event_type, details=""):
        """Log teleoperation events."""
        if event_type == "movement_start":
            self.add_log_message(f"🚀 Robot movement started: {details}", "INFO")
        elif event_type == "movement_stop":
            self.add_log_message("⏹️ Robot movement stopped", "INFO")
        elif event_type == "emergency_stop":
            self.add_log_message("🛑 EMERGENCY STOP activated!", "ERROR")
            
    def log_system_event(self, event_type, details=""):
        """Log system events."""
        if event_type == "rviz_launched":
            self.add_log_message("🔍 RViz visualization launched", "INFO")
        elif event_type == "rviz_closed":
            self.add_log_message("❌ RViz visualization closed", "INFO")
        elif event_type == "ros_connected":
            self.add_log_message("✅ ROS connection established", "SUCCESS")
        elif event_type == "ros_disconnected":
            self.add_log_message("❌ ROS connection lost", "ERROR")
        elif event_type == "waypoint_added":
            self.add_log_message(f"📍 Waypoint added: {details}", "INFO")
        elif event_type == "waypoint_deleted":
            self.add_log_message(f"🗑️ Waypoint deleted: {details}", "INFO")