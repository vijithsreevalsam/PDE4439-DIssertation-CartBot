from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                               QSlider, QLabel, QGroupBox, QRadioButton, QButtonGroup,
                               QGridLayout, QFrame, QProgressBar,QMessageBox)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont
import subprocess
import os
import time

from .navigation_widget import NavigationWidget

try:
    from ..ros.ros_interface import Twist
except ImportError:
    # Fallback for when ROS is not available
    class Twist:
        def __init__(self):
            self.linear = type('Linear', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()
            self.angular = type('Angular', (), {'x': 0.0, 'y': 0.0, 'z': 0.0})()

class TeleopWidget(QWidget):
    """Widget for robot teleoperation control."""

    def __init__(self, ros_interface, navigation_widget):
        super().__init__()
        self.ros_interface = ros_interface
        self.current_twist = Twist()
        self.is_moving = False
        self.navigation_widget = navigation_widget
        self.setup_ui()
        
        # Timer for continuous movement
        self.move_timer = QTimer()
        self.move_timer.timeout.connect(self.publish_twist)
        
        # Key press handling
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        
    def setup_ui(self):
        """Setup the teleoperation UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # # Control mode selection
        # mode_group = QGroupBox("🎮 Control Mode")
        # mode_layout = QVBoxLayout(mode_group)
        
        # self.button_group = QButtonGroup()
        # self.button_mode = QRadioButton("🔘 Button Control")
        # self.joystick_mode = QRadioButton("🕹️ Joystick Control")
        # self.keyboard_mode = QRadioButton("⌨️ Keyboard Control")
        # self.button_mode.setChecked(True)
        
        # self.button_group.addButton(self.button_mode)
        # self.button_group.addButton(self.joystick_mode)
        # self.button_group.addButton(self.keyboard_mode)
        
        # mode_layout.addWidget(self.button_mode)
        # mode_layout.addWidget(self.joystick_mode)
        # mode_layout.addWidget(self.keyboard_mode)
        # layout.addWidget(mode_group)

        # SLAM Control
        slam_group = QGroupBox("🗺️ SLAM (Mapping)")
        # slam_group.setMinimumWidth(500)  # or a value that fits your text
        slam_layout = QVBoxLayout(slam_group)
        slam_buttons_layout = QHBoxLayout()
        
        self.start_slam_btn = QPushButton("🗺️ Start SLAM")
        self.start_slam_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #45a049; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.start_slam_btn.clicked.connect(self.start_slam)
        slam_buttons_layout.addWidget(self.start_slam_btn)
        
        self.stop_slam_btn = QPushButton("⏹️ Stop SLAM")
        self.stop_slam_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #d32f2f; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.stop_slam_btn.clicked.connect(self.stop_slam)
        self.stop_slam_btn.setEnabled(False)
        slam_buttons_layout.addWidget(self.stop_slam_btn)
        
        self.save_map_btn = QPushButton("💾 Save Map")
        self.save_map_btn.clicked.connect(self.save_map)
        slam_buttons_layout.addWidget(self.save_map_btn)
        
        slam_layout.addLayout(slam_buttons_layout)
        layout.addWidget(slam_group)
        









        
        # Speed control
        speed_group = QGroupBox("⚡ Speed Control")
        speed_layout = QVBoxLayout(speed_group)
        
        # Linear speed
        linear_layout = QHBoxLayout()
        linear_layout.addWidget(QLabel("Linear:"))
        self.linear_slider = QSlider(Qt.Orientation.Horizontal)
        self.linear_slider.setRange(10, 100)  # 0.1 to 1.0 m/s
        self.linear_slider.setValue(50)  # 0.5 m/s default
        self.linear_label = QLabel("0.5 m/s")
        self.linear_label.setMinimumWidth(60)
        linear_layout.addWidget(self.linear_slider)
        linear_layout.addWidget(self.linear_label)
        speed_layout.addLayout(linear_layout)
        
        # Angular speed
        angular_layout = QHBoxLayout()
        angular_layout.addWidget(QLabel("Angular:"))
        self.angular_slider = QSlider(Qt.Orientation.Horizontal)
        self.angular_slider.setRange(10, 100)  # 0.1 to 1.0 rad/s
        self.angular_slider.setValue(50)  # 0.5 rad/s default
        self.angular_label = QLabel("0.5 rad/s")
        self.angular_label.setMinimumWidth(60)
        angular_layout.addWidget(self.angular_slider)
        angular_layout.addWidget(self.angular_label)
        speed_layout.addLayout(angular_layout)
        
        # Connect sliders
        self.linear_slider.valueChanged.connect(self.update_linear_speed)
        self.angular_slider.valueChanged.connect(self.update_angular_speed)
        
        layout.addWidget(speed_group)
        
        # Movement buttons
        move_group = QGroupBox("🚀 Movement Control")
        move_layout = QVBoxLayout(move_group)
        
        # Create button grid
        button_grid = QGridLayout()
        
        # Forward
        self.forward_btn = QPushButton("⬆️\\nForward\\n(W)")
        self.forward_btn.setMinimumHeight(60)
        self.forward_btn.pressed.connect(lambda: self.start_movement(1, 0))
        self.forward_btn.released.connect(self.stop_movement)
        button_grid.addWidget(self.forward_btn, 0, 1)
        
        # Left, Stop, Right
        self.left_btn = QPushButton("⬅️\\nLeft\\n(A)")
        self.left_btn.setMinimumHeight(60)
        self.left_btn.pressed.connect(lambda: self.start_movement(0, 1))
        self.left_btn.released.connect(self.stop_movement)
        button_grid.addWidget(self.left_btn, 1, 0)
        
        self.stop_btn = QPushButton("🛑\\nSTOP\\n(S)")
        self.stop_btn.setMinimumHeight(60)
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-weight: bold;
                border-radius: 8px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        self.stop_btn.clicked.connect(self.emergency_stop)
        button_grid.addWidget(self.stop_btn, 1, 1)
        
        self.right_btn = QPushButton("➡️\\nRight\\n(D)")
        self.right_btn.setMinimumHeight(60)
        self.right_btn.pressed.connect(lambda: self.start_movement(0, -1))
        self.right_btn.released.connect(self.stop_movement)
        button_grid.addWidget(self.right_btn, 1, 2)
        
        # Backward
        self.backward_btn = QPushButton("⬇️\\nBackward\\n(S)")
        self.backward_btn.setMinimumHeight(60)
        self.backward_btn.pressed.connect(lambda: self.start_movement(-1, 0))
        self.backward_btn.released.connect(self.stop_movement)
        button_grid.addWidget(self.backward_btn, 2, 1)
        
        move_layout.addLayout(button_grid)
        layout.addWidget(move_group)
        
        # Status indicators
        status_group = QGroupBox("📊 Control Status")
        status_layout = QVBoxLayout(status_group)
        
        # Movement indicator
        movement_layout = QHBoxLayout()
        movement_layout.addWidget(QLabel("Movement:"))
        self.movement_indicator = QLabel("⏹️ Stopped")
        self.movement_indicator.setStyleSheet("color: #666; font-weight: bold;")
        movement_layout.addWidget(self.movement_indicator)
        movement_layout.addStretch()
        status_layout.addLayout(movement_layout)
        
        # Connection status
        connection_layout = QHBoxLayout()
        connection_layout.addWidget(QLabel("ROS Status:"))
        self.connection_status = QLabel("❌ Disconnected")
        self.connection_status.setStyleSheet("color: red; font-weight: bold;")
        connection_layout.addWidget(self.connection_status)
        connection_layout.addStretch()
        status_layout.addLayout(connection_layout)
        
        # Current velocity display
        velocity_layout = QVBoxLayout()
        self.velocity_linear_label = QLabel("Linear: 0.00 m/s")
        self.velocity_angular_label = QLabel("Angular: 0.00 rad/s")
        velocity_layout.addWidget(self.velocity_linear_label)
        velocity_layout.addWidget(self.velocity_angular_label)
        status_layout.addLayout(velocity_layout)
        
        layout.addWidget(status_group)
        
        # Instructions
        instructions_group = QGroupBox("ℹ️ Instructions")
        instructions_layout = QVBoxLayout(instructions_group)
        
        instructions_text = QLabel("""
        🎮 Button Mode: Click and hold movement buttons
        ⌨️ Keyboard Mode: Use WASD keys (focus on this tab)
        🕹️ Joystick Mode: Click and drag in movement area
        
        W/⬆️ - Forward    S/⬇️ - Backward
        A/⬅️ - Left       D/➡️ - Right
        Space/🛑 - Emergency Stop
        """)
        instructions_text.setWordWrap(True)
        instructions_text.setStyleSheet("color: #666; font-size: 11px;")
        instructions_layout.addWidget(instructions_text)
        
        layout.addWidget(instructions_group)
        
        # Update connection status initially
        self.update_connection_status()

    def start_slam(self):
        """Start SLAM process."""
        try:
            # Try different SLAM launch commands
            slam_commands = [
                ["ros2", "launch", "cartBot_navigation", "slam.launch.py", "rviz:=false", "sim:=false"],
                ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
                ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node"]
            ]
            
            for cmd in slam_commands:
                try:
                    self.slam_process = subprocess.Popen(cmd)
                    break
                except FileNotFoundError:
                    continue
            
            if self.slam_process is None:
                raise Exception("Could not start SLAM - no valid command found")
                
            self.start_slam_btn.setEnabled(False)
            self.stop_slam_btn.setEnabled(True)
            QMessageBox.information(self, "SLAM", "SLAM mapping started successfully!\\n\\nDrive the robot around to build the map.")
            
            # Log event
            if hasattr(self.parent(), 'status_widget'):
                self.parent().status_widget.log_navigation_event("slam_started")
                
        except Exception as e:
            QMessageBox.warning(self, "SLAM Error", f"Failed to start SLAM: {str(e)}\\n\\nPlease ensure SLAM packages are installed:\\nsudo apt install ros-jazzy-slam-toolbox")
            
    def stop_slam(self):
        """Stop SLAM process."""
        if self.slam_process:
            self.slam_process.terminate()
            self.slam_process.wait()
            self.slam_process = None
            
        self.start_slam_btn.setEnabled(True)
        self.stop_slam_btn.setEnabled(False)
        QMessageBox.information(self, "SLAM", "SLAM mapping stopped successfully!")
        
        # Log event
        if hasattr(self.parent(), 'status_widget'):
            self.parent().status_widget.log_navigation_event("slam_stopped")
            
    def save_map(self):
        """Save the current map."""
        map_name = f"map_{int(time.time())}"
        
        try:
            # Create maps directory if it doesn't exist
            maps_dir = "/home/viju/maps"
            os.makedirs(maps_dir, exist_ok=True)
            
            # Save map using ros2 map_server
            subprocess.run([
                "ros2", "run", "nav2_map_server", "map_saver_cli",
                "-f", os.path.join(maps_dir, map_name), "--ros-args", "-p", "save_map_timeout:=10000"
            ])
            
            QMessageBox.information(self, "Map Saved", f"Map saved as {map_name}.yaml\\n\\nLocation: {maps_dir}")
            self.navigation_widget.refresh_map_list()
            
        except Exception as e:
            QMessageBox.warning(self, "Map Save Error", f"Failed to save map: {str(e)}")
        
        
    def update_linear_speed(self, value):
        """Update linear speed display."""
        speed = value / 100.0
        self.linear_label.setText(f"{speed:.1f} m/s")
        
    def update_angular_speed(self, value):
        """Update angular speed display."""
        speed = value / 100.0
        self.angular_label.setText(f"{speed:.1f} rad/s")
        
    def start_movement(self, linear_factor, angular_factor):
        """Start robot movement."""
        linear_speed = self.linear_slider.value() / 100.0
        angular_speed = self.angular_slider.value() / 100.0
        
        self.current_twist.linear.x = linear_factor * linear_speed
        self.current_twist.angular.z = angular_factor * angular_speed
        
        self.is_moving = True
        self.move_timer.start(50)  # Publish at 20Hz
        
        # Update movement indicator
        direction = ""
        if linear_factor > 0:
            direction = "🚀 Moving Forward"
        elif linear_factor < 0:
            direction = "🔙 Moving Backward"
        elif angular_factor > 0:
            direction = "↪️ Turning Left"
        elif angular_factor < 0:
            direction = "↩️ Turning Right"
            
        self.movement_indicator.setText(direction)
        self.movement_indicator.setStyleSheet("color: #4CAF50; font-weight: bold;")
        
    def stop_movement(self):
        """Stop robot movement."""
        self.move_timer.stop()
        self.current_twist = Twist()  # Zero velocity
        self.publish_twist()
        self.is_moving = False
        
        # Update movement indicator
        self.movement_indicator.setText("⏹️ Stopped")
        self.movement_indicator.setStyleSheet("color: #666; font-weight: bold;")
        
    def emergency_stop(self):
        """Emergency stop."""
        self.stop_movement()
        if self.ros_interface:
            self.ros_interface.emergency_stop()
        
    def publish_twist(self):
        """Publish current twist to ROS."""
        if self.ros_interface:
            self.ros_interface.publish_cmd_vel(self.current_twist)
            
        # Update velocity display
        self.velocity_linear_label.setText(f"Linear: {self.current_twist.linear.x:.2f} m/s")
        self.velocity_angular_label.setText(f"Angular: {self.current_twist.angular.z:.2f} rad/s")
        
    def update_connection_status(self):
        """Update ROS connection status."""
        if self.ros_interface and self.ros_interface.is_connected():
            self.connection_status.setText("✅ Connected")
            self.connection_status.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.connection_status.setText("❌ Disconnected")
            self.connection_status.setStyleSheet("color: red; font-weight: bold;")
            
    def keyPressEvent(self, event):
        """Handle keyboard input for teleoperation."""
        if not self.keyboard_mode.isChecked():
            return
            
        key = event.key()
        
        if key == Qt.Key.Key_W or key == Qt.Key.Key_Up:
            self.start_movement(1, 0)
        elif key == Qt.Key.Key_S or key == Qt.Key.Key_Down:
            self.start_movement(-1, 0)
        elif key == Qt.Key.Key_A or key == Qt.Key.Key_Left:
            self.start_movement(0, 1)
        elif key == Qt.Key.Key_D or key == Qt.Key.Key_Right:
            self.start_movement(0, -1)
        elif key == Qt.Key.Key_Space:
            self.emergency_stop()
            
    def keyReleaseEvent(self, event):
        """Handle keyboard release for teleoperation."""
        if not self.keyboard_mode.isChecked():
            return
            
        # Stop movement when key is released
        if event.key() in [Qt.Key.Key_W, Qt.Key.Key_S, Qt.Key.Key_A, Qt.Key.Key_D,
                          Qt.Key.Key_Up, Qt.Key.Key_Down, Qt.Key.Key_Left, Qt.Key.Key_Right]:
            self.stop_movement()
            
    def showEvent(self, event):
        """Handle widget show event."""
        super().showEvent(event)
        self.update_connection_status()
        
    def update_displays(self):
        """Update all displays (called from main window timer)."""
        self.update_connection_status()