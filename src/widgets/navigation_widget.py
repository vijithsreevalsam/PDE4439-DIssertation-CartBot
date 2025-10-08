from PySide6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                               QGroupBox, QListWidget, QLineEdit, QLabel,
                               QComboBox, QMessageBox, QScrollArea, QFrame,
                               QGridLayout, QListWidgetItem, QDoubleSpinBox)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont
from PySide6.QtCore import QEventLoop
import subprocess
import os
import json
import time
from ..utils.audio_node import AudioProcessingNode
import time

# Import ROS message types if available
try:
    from std_msgs.msg import String
except ImportError:
    # Define a dummy String class for non-ROS environments
    class String:
        def __init__(self):
            self.data = ""

class NavigationWidget(QWidget):
    """Widget for navigation control and waypoint management."""
    
    def __init__(self, ros_interface):
        super().__init__()
        self.ros_interface = ros_interface
        self.audio_node = AudioProcessingNode() if ros_interface else None
        self.waypoints = {}
        self.slam_process = None
        self.nav_process = None
        self.last_recognized_text = None
        self.waypoints_file = "/home/viju/UI/config/waypoints.json"
        
        # Connect voice text signal if available
        # if hasattr(self.ros_interface, 'voice_text_received'):
        #     self.ros_interface.voice_text_received.connect(self.update_voice_text)
        
        # Start the product navigator node automatically
        self.start_product_navigator()
        self.voice_recognition_active = False
        self.setup_ui()
        self.load_waypoints()
        
    def setup_ui(self):
        """Setup the navigation UI."""
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # SLAM Control
        slam_group = QGroupBox("🗺️ SLAM (Mapping)")
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
        
        # Navigation Control
        nav_group = QGroupBox("🧭 Navigation Control")
        nav_layout = QVBoxLayout(nav_group)
        
        # Map selection
        map_layout = QHBoxLayout()
        map_layout.addWidget(QLabel("Map File:"))
        self.map_combo = QComboBox()
        self.map_combo.setEditable(True)
        self.refresh_map_list()
        map_layout.addWidget(self.map_combo)
        
        self.refresh_maps_btn = QPushButton("🔄")
        self.refresh_maps_btn.setMaximumWidth(40)
        self.refresh_maps_btn.clicked.connect(self.refresh_map_list)
        map_layout.addWidget(self.refresh_maps_btn)
        
        nav_layout.addLayout(map_layout)
        # Param Selection
        param_layout = QHBoxLayout()
        param_layout.addWidget(QLabel("Param File:"))
        self.param_combo = QComboBox()
        self.param_combo.setEditable(True)
        self.refresh_param_list()
        param_layout.addWidget(self.param_combo)

        self.refresh_params_btn = QPushButton("🔄")
        self.refresh_params_btn.setMaximumWidth(40)
        self.refresh_params_btn.clicked.connect(self.refresh_param_list)
        param_layout.addWidget(self.refresh_params_btn)

        nav_layout.addLayout(param_layout)
        # map_layout.addWidget(self.refresh_maps_btn)
        
        # nav_layout.addLayout(map_layout)


        
        # Navigation buttons
        nav_buttons_layout = QHBoxLayout()
        
        self.start_nav_btn = QPushButton("🧭 Start Navigation")
        self.start_nav_btn.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:disabled { background-color: #cccccc; }
        """)
        self.start_nav_btn.clicked.connect(self.start_navigation)
        nav_buttons_layout.addWidget(self.start_nav_btn)
        
        self.stop_nav_btn = QPushButton("⏹️ Stop Navigation")
        self.stop_nav_btn.setStyleSheet("""
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
        self.stop_nav_btn.clicked.connect(self.stop_navigation)
        self.stop_nav_btn.setEnabled(False)
        nav_buttons_layout.addWidget(self.stop_nav_btn)
        
        nav_layout.addLayout(nav_buttons_layout)
        layout.addWidget(nav_group)
        
        # Waypoint Management
        waypoint_group = QGroupBox("📍 Waypoint Management")
        waypoint_layout = QVBoxLayout(waypoint_group)
        
        # Add waypoint from current position
        current_pos_layout = QHBoxLayout()
        self.waypoint_name = QLineEdit()
        self.waypoint_name.setPlaceholderText("Enter waypoint name (e.g., 'fruits', 'kitchen')")
        current_pos_layout.addWidget(self.waypoint_name)
        
        self.add_current_btn = QPushButton("📍 Add Current Position")
        self.add_current_btn.setStyleSheet("""
            QPushButton {
                background-color: #FF9800;
                color: white;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #F57C00; }
        """)
        self.add_current_btn.clicked.connect(self.add_current_waypoint)
        current_pos_layout.addWidget(self.add_current_btn)
        
        waypoint_layout.addLayout(current_pos_layout)
        
        # Manual waypoint entry
        manual_layout = QGridLayout()
        
        manual_layout.addWidget(QLabel("Name:"), 0, 0)
        self.manual_name = QLineEdit()
        self.manual_name.setPlaceholderText("Waypoint name")
        manual_layout.addWidget(self.manual_name, 0, 1, 1, 2)
        
        manual_layout.addWidget(QLabel("X:"), 1, 0)
        self.x_input = QDoubleSpinBox()
        self.x_input.setRange(-100.0, 100.0)
        self.x_input.setDecimals(3)
        self.x_input.setSuffix(" m")
        manual_layout.addWidget(self.x_input, 1, 1)
        
        manual_layout.addWidget(QLabel("Y:"), 1, 2)
        self.y_input = QDoubleSpinBox()
        self.y_input.setRange(-100.0, 100.0)
        self.y_input.setDecimals(3)
        self.y_input.setSuffix(" m")
        manual_layout.addWidget(self.y_input, 1, 3)
        
        self.add_manual_btn = QPushButton("➕ Add Manual Waypoint")
        self.add_manual_btn.clicked.connect(self.add_manual_waypoint)
        manual_layout.addWidget(self.add_manual_btn, 2, 0, 1, 2)
        
        # waypoint_layout.addLayout(manual_layout)
        self.btn_state  = False
        self.btn_talk = QPushButton("🎤 Push to Talk")
        self.btn_talk.setCheckable(True)
        self.btn_talk.pressed.connect(self.start_voice_recognition)
        self.btn_talk.released.connect(self.stop_voice_recognition)
        # if self.btn_talk.isDown():
        
        # if self.btn_state:
        #     self.check_for_navigation_command(self.last_recognized_text)
        #     if self.btn_talk.isUp():
        #         self.btn_state = False
        # self.btn_talk.released.connect(self.stop_voice_recognition)
        self.btn_talk.setStyleSheet("""
            QPushButton {
                background-color: #2196F3;
                color: white;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #1976D2; }
            QPushButton:pressed, QPushButton:checked {
                background-color: #FF5722;
            }
        """)
        manual_layout.addWidget(self.btn_talk, 2, 3, 1, 2)
        
        # Text label to show recognized speech
        self.voice_text_label = QLabel("Press the button and speak")
        self.voice_text_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                padding: 8px;
                border-radius: 4px;
                border: 1px solid #ddd;
                min-height: 20px;
            }
        """)
        manual_layout.addWidget(self.voice_text_label, 3, 0, 1, 5)

        waypoint_layout.addLayout(manual_layout)
        
        # Waypoint list
        waypoint_layout.addWidget(QLabel("📋 Saved Waypoints:"))
        self.waypoint_list = QListWidget()
        self.waypoint_list.setSelectionMode(QListWidget.MultiSelection)
        self.waypoint_list.setMaximumHeight(150)
        self.waypoint_list.setStyleSheet("""
            QListWidget {
                border: 1px solid #ccc;
                border-radius: 4px;
                background-color: #fafafa;
            }
            QListWidget::item {
                padding: 5px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #2196F3;
                color: white;
            }
            QListWidget::item:hover {
                background-color: #e3f2fd;
            }
        """)
        waypoint_layout.addWidget(self.waypoint_list)
        
        # Waypoint action buttons
        waypoint_buttons_layout = QHBoxLayout()
        
        self.goto_btn = QPushButton("🎯 Go To Selected")
        self.goto_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #45a049; }
        """)
        self.goto_btn.clicked.connect(self.goto_selected_waypoint)
        waypoint_buttons_layout.addWidget(self.goto_btn)
        
        self.edit_btn = QPushButton("✏️ Edit Selected")
        self.edit_btn.clicked.connect(self.edit_selected_waypoint)
        waypoint_buttons_layout.addWidget(self.edit_btn)
        
        self.delete_btn = QPushButton("🗑️ Delete Selected")
        self.delete_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                padding: 8px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #d32f2f; }
        """)
        self.delete_btn.clicked.connect(self.delete_selected_waypoint)
        waypoint_buttons_layout.addWidget(self.delete_btn)
        
        waypoint_layout.addLayout(waypoint_buttons_layout)
        
        # Quick navigation buttons
        quick_nav_layout = QHBoxLayout()
        
        self.cancel_nav_btn = QPushButton("❌ Cancel Navigation")
        self.cancel_nav_btn.clicked.connect(self.cancel_navigation)
        quick_nav_layout.addWidget(self.cancel_nav_btn)
        
        self.clear_costmaps_btn = QPushButton("🧹 Clear Costmaps")
        self.clear_costmaps_btn.clicked.connect(self.clear_costmaps)
        quick_nav_layout.addWidget(self.clear_costmaps_btn)
        
        waypoint_layout.addLayout(quick_nav_layout)
        
        layout.addWidget(waypoint_group)
        
        # Predefined waypoints
        self.load_default_waypoints()
        
    def refresh_map_list(self):
        """Refresh the list of available maps."""
        self.map_combo.clear()
        
        possible_map_dirs = [
            "/home/viju/ros_ws/src/cartBot/cartBot_navigation/maps",
            "/home/viju/maps",
            "/home/viju"
        ]
        
        for map_dir in possible_map_dirs:
            if os.path.exists(map_dir):
                for file in os.listdir(map_dir):
                    if file.endswith('.yaml'):
                        self.map_combo.addItem(os.path.join(map_dir, file))
        
        # Add common map names
        if self.map_combo.count() == 0:
            self.map_combo.addItems([
                "maha.yaml",
                "my_room.yaml", 
                "tb_viju.yaml",
                "office.yaml"
            ])

    def refresh_param_list(self):
        """Refresh the list of available parameter files."""
        self.param_combo.clear()
        
        possible_param_dirs = [
            "/home/viju/ros_ws/src/cartBot/cartBot_navigation/params",
            "/home/viju/params",
            "/home/viju"
        ]
        
        for param_dir in possible_param_dirs:
            if os.path.exists(param_dir):
                for file in os.listdir(param_dir):
                    if file.endswith('.yaml'):
                        self.param_combo.addItem(os.path.join(param_dir, file))

        # Add common parameter names
        if self.param_combo.count() == 0:
            self.param_combo.addItems([
                ".yaml",
                "nav2_params.yaml", 
                "navigation.yaml",
                
            ])
            




            
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
                "-f", os.path.join(maps_dir, map_name),"--ros-args", "-p", "save_map_timeout:=10000"
            ])
            QMessageBox.information(self, "Map Saved", f"Map saved as {map_name}.yaml\\n\\nLocation: {maps_dir}")
            self.refresh_map_list()
            
        except Exception as e:
            QMessageBox.warning(self, "Map Save Error", f"Failed to save map: {str(e)}")
            
    def start_navigation(self):
        """Start navigation with selected map."""
        selected_map = self.map_combo.currentText()
        selected_param = self.param_combo.currentText()
        
        if not selected_map:
            QMessageBox.warning(self, "No Map", "Please select a map file first!")
            return
        if not selected_param:
            QMessageBox.warning(self, "No Param", "Please select a parameter file first!")
            return
            
        try:
            # Try different navigation launch commands
            nav_commands = [
                ["ros2", "launch", "cartBot_navigation", "navigation.launch.py", f"map:={selected_map}", f"params:={selected_param}", "rviz:=false", "use_sim_time:=true", "autostart:=true"],
                ["ros2", "launch", "nav2_bringup", "navigation_launch.py", f"params:={selected_param}", "use_sim_time:=false", "autostart:=true"],
            ]
            
            for cmd in nav_commands:
                try:
                    self.nav_process = subprocess.Popen(cmd)
                    break
                except FileNotFoundError:
                    continue
                    
            if self.nav_process is None:
                raise Exception("Could not start navigation - no valid command found")
                
            self.start_nav_btn.setEnabled(False)
            self.stop_nav_btn.setEnabled(True)
            QMessageBox.information(self, "Navigation", f"Navigation started successfully!\\n\\nMap: {os.path.basename(selected_map)}\\n\\nUse RViz 2D Nav Goal tool to set targets.")
            
            # Log event
            if hasattr(self.parent(), 'status_widget'):
                self.parent().status_widget.log_navigation_event("nav_started", os.path.basename(selected_map))
                
        except Exception as e:
            QMessageBox.warning(self, "Navigation Error", f"Failed to start navigation: {str(e)}\\n\\nPlease ensure Nav2 packages are installed:\\nsudo apt install ros-jazzy-navigation2")
            
    def stop_navigation(self):
        """Stop navigation process."""
        if self.nav_process:
            self.nav_process.terminate()
            self.nav_process.wait()
            self.nav_process = None
            
        self.start_nav_btn.setEnabled(True)
        self.stop_nav_btn.setEnabled(False)
        QMessageBox.information(self, "Navigation", "Navigation system stopped successfully!")
        
        # Log event
        if hasattr(self.parent(), 'status_widget'):
            self.parent().status_widget.log_navigation_event("nav_stopped")
            
    def load_default_waypoints(self):
        """Load some default waypoints."""
        self.waypoints = {
            "home": {"x": 0.0, "y": 0.0},
            "kitchen": {"x": 2.0, "y": 1.0},
            "living_room": {"x": -1.0, "y": 2.0},
            "fruits": {"x": 3.0, "y": -1.0},
            "charging_station": {"x": -2.0, "y": -1.0}
        }
        self.update_waypoint_list()
        
    def load_waypoints(self):
        """Load waypoints from file."""
        try:
            if os.path.exists(self.waypoints_file):
                with open(self.waypoints_file, 'r') as f:
                    self.waypoints = json.load(f)
                self.update_waypoint_list()
        except Exception as e:
            print(f"Could not load waypoints: {e}")
            self.load_default_waypoints()
            
    def save_waypoints(self):
        """Save waypoints to file."""
        try:
            os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
            with open(self.waypoints_file, 'w') as f:
                json.dump(self.waypoints, f, indent=2)
        except Exception as e:
            print(f"Could not save waypoints: {e}")
            
    def update_waypoint_list(self):
        """Update the waypoint list display."""
        self.waypoint_list.clear()
        for name, coords in self.waypoints.items():
            item_text = f"📍 {name}: ({coords['x']:.3f}, {coords['y']:.3f})"
            item = QListWidgetItem(item_text)
            item.setData(Qt.ItemDataRole.UserRole, name)
            self.waypoint_list.addItem(item)
            
    def add_current_waypoint(self):
        """Add current robot position as waypoint."""
        name = self.waypoint_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Please enter a waypoint name!")
            return
            
        if name in self.waypoints:
            reply = QMessageBox.question(
                self, "Waypoint Exists", 
                f"Waypoint '{name}' already exists. Overwrite?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
            
        if not self.ros_interface:
            QMessageBox.warning(self, "No ROS", "ROS interface not available!")
            return
            
        pose = self.ros_interface.get_current_pose()
        print(f"DEBUG: Current robot pose: {pose}")
        if pose:
            self.waypoints[name] = {
                "x": pose.position.x,
                "y": pose.position.y
            }
            self.waypoint_name.clear()
            self.update_waypoint_list()
            self.save_waypoints()
            QMessageBox.information(self, "Waypoint Added", f"Waypoint '{name}' added at current position!\\n\\nPosition: ({pose.position.x:.3f}, {pose.position.y:.3f})")
            
            # Log event
            if hasattr(self.parent(), 'status_widget'):
                self.parent().status_widget.log_system_event("waypoint_added", f"{name} at ({pose.position.x:.3f}, {pose.position.y:.3f})")
        else:
            QMessageBox.warning(self, "Position Error", "Could not get current robot position!\\n\\nEnsure robot localization is running.")

    def start_product_navigator(self):
        """Start the product navigator node in a separate process."""
        try:
            import subprocess
            import os
            
            product_navigator_path = os.path.expanduser("~/UI/src/utils/product_navigator.py")
            
            # Make sure the script is executable
            if os.path.exists(product_navigator_path):
                os.chmod(product_navigator_path, 0o755)
                
                # Start the product navigator process
                print(f"Starting product navigator from {product_navigator_path}")
                self.product_navigator_process = subprocess.Popen(
                    [product_navigator_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                print(f"Product navigator started with PID: {self.product_navigator_process.pid}")
                
                # Create a subscription to the navigation_waypoint topic
                if self.ros_interface and self.ros_interface.ros_available:
                    self.navigation_sub = self.ros_interface.node.create_subscription(
                        String, 
                        'navigation_waypoint',
                        self.navigation_waypoint_callback,
                        10
                    )
                    print("Subscribed to navigation_waypoint topic")
                
                return True
            else:
                print(f"Warning: Product navigator not found at {product_navigator_path}")
                return False
        except Exception as e:
            print(f"Error starting product navigator: {e}")
            import traceback
            traceback.print_exc()
            return False

    def navigation_waypoint_callback(self, msg):
        """Handle navigation waypoint commands from the product navigator."""
        try:
            print(f"Received navigation waypoint command: {msg.data}")
            
            # Parse the command format: waypoint_name:x:y
            parts = msg.data.split(':')
            if len(parts) == 3:
                waypoint_name = parts[0]
                x = float(parts[1])
                y = float(parts[2])
                
                # Find the waypoint in the list and select it
                for i in range(self.waypoint_list.count()):
                    item = self.waypoint_list.item(i)
                    if item.data(Qt.ItemDataRole.UserRole) == waypoint_name:
                        self.waypoint_list.clearSelection()
                        item.setSelected(True)
                        break
                
                # Navigate to the coordinates
                if self.ros_interface:
                    self.ros_interface.navigate_to_point(x, y)
                    QMessageBox.information(self, "Voice Navigation", 
                                         f"Navigating to '{waypoint_name}' at ({x:.2f}, {y:.2f})")
            else:
                print(f"Invalid navigation command format: {msg.data}")
        except Exception as e:
            print(f"Error processing navigation command: {e}")
            import traceback
            traceback.print_exc()
    
    def start_voice_recognition(self):
        """Start voice recognition when button is pressed."""
        if not self.ros_interface or not self.audio_node:
            return
        
        # Ensure the button release signal is connected
        try:
            self.btn_talk.released.disconnect()  # Disconnect any existing connections
        except TypeError:
            pass  # No existing connections
        self.btn_talk.released.connect(self.stop_voice_recognition)
        
        # Reset the label style
        self.voice_text_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                padding: 8px;
                border-radius: 4px;
                border: 1px solid #FF5722;
                min-height: 20px;
                color: #D84315;
                font-weight: bold;
            }
        """)
        self.voice_text_label.setText("🎙️ Listening... (speak now)")
        
        # Start audio processing
        success = self.audio_node.start_voice_processing()
        # print(f"DEBUG: Audio node start_voice_processing returned: {self.audio_node.get_latest_text()}")
        if success:
            print("DEBUG: Voice recognition started {}")
            # Start a timer to periodically check for new recognized text
            if not hasattr(self, 'voice_text_timer'):
                self.voice_text_timer = QTimer()
                self.voice_text_timer.timeout.connect(self.check_voice_text)

            self.voice_text_timer.start(10)  # Check every 10ms
        else:
            self.voice_text_label.setText("Failed to start voice recognition")

    def check_voice_text(self):
        """Check for new voice recognition results"""
        if hasattr(self, 'audio_node') and self.audio_node:
            text = self.audio_node.get_latest_text()
            if text:
                self.update_voice_text(text)

    # def update_voice_text(self, text):
    #     """Update the voice text label with recognized text."""
    #     if not text:
    #         return
            
    #     print(f"DEBUG: Updating voice text with: {text}")
        
    #     # Update the label
    #     self.voice_text_label.setText(f"Hearing: {text}")
        
    #     # Quick check for navigation intent
    #     navigation_triggers = ["go to", "take me to", "navigate to", "move to", "find"]
    #     if any(trigger in text.lower() for trigger in navigation_triggers):
    #         # Found potential navigation command, highlight it in the UI
    #         self.voice_text_label.setStyleSheet("""
    #             QLabel {
    #                 background-color: #e3f2fd;
    #                 padding: 8px;
    #                 border-radius: 4px;
    #                 border: 1px solid #2196F3;
    #                 min-height: 20px;
    #                 color: #0d47a1;
    #             }
    #         """)
            
    #         # Indicate that a navigation command was detected
    #         self.voice_text_label.setText(f"Hearing navigation new command: {text}")   


    # def start_voice_recognition(self):
    #     """Start voice recognition when button is pressed."""
    #     if not self.ros_interface:
    #         QMessageBox.warning(self, "No ROS", "ROS interface not available!")
    #         return
        
    #     # Reset the label style
    #     self.voice_text_label.setStyleSheet("""
    #         QLabel {
    #             background-color: #f0f0f0;
    #             padding: 8px;
    #             border-radius: 4px;
    #             border: 1px solid #FF5722;
    #             min-height: 20px;
    #             color: #D84315;
    #             font-weight: bold;
    #         }
    #     """)
    #     self.voice_text_label.setText("🎙️ Listening... (speak now)")
    #     get_the_voice_to_text = self.audio_node.process_audio()
    #     # Start audio processing
    #     # success = self.audio_node.start_voice_processing()
    #     if get_the_voice_to_text:
    #         print("DEBUG: Voice recognition started")
    #         # if hasattr(self.audio_node, 'start_voice_processing'):
    #         self.voice_text_label.setText(f"🎙️ Listening... {get_the_voice_to_text}")

    #     else:
    #         self.voice_text_label.setText("❌ Failed to start listening")
    #         print("DEBUG: Failed to start voice recognition")


            
        # if success:
        #     print("DEBUG: Voice recognition started")
            
        #     # Connect the signal for voice text updates if not already connected
        #     if hasattr(self.ros_interface, 'voice_text_received'):
        #         try:
        #             # Disconnect first to avoid multiple connections
        #             self.ros_interface.voice_text_received.disconnect(self.update_voice_text)
        #         except:
        #             pass
        #         # Reconnect
        #         self.ros_interface.voice_text_received.connect(self.update_voice_text)
        #         self.voice_text_label.setText(f"🎙️ Listening... {self.ros_interface.current_voice_text()}")
        # else:
        #     self.voice_text_label.setText("❌ Failed to start listening")
        #     print("DEBUG: Failed to start voice recognition")
    def stop_voice_recognition(self):
        result = self.audio_node.stop()
        
        # Stop the voice text timer if it's running
        if hasattr(self, 'voice_text_timer') and self.voice_text_timer.isActive():
            self.voice_text_timer.stop()
            
        if self.btn_state:
            # if self.btn_talk.isUp():
                # result = self.audio_node.stop()
            if result:
                self.voice_recognition_active = False
                print("DEBUG: Voice recognition stopped successfully")
                # Reset to default UI state  
                self.voice_text_label.setText("Press the button and speak")
                self.voice_text_label.setStyleSheet("""
                    QLabel {
                        background-color: #f0f0f0;
                        padding: 8px;
                        border-radius: 4px;
                        border: 1px solid #ccc;
                        min-height: 20px;
                        color: #333;
                    }
                """)
            else:
                print("DEBUG: Failed to stop voice recognition")
                time.sleep(0.2)
            self.btn_state = False

        # if self.btn_talk.isUp():
        #     self.check_for_navigation_command(self.last_recognized_text)
        #     print(f"DEBUG: Final recognized text: {self.last_recognized_text}")
        #     self.btn_state = False

            # disconnect so it doesn’t trigger again
            try:
                self.btn_talk.released.disconnect(self.stop_voice_recognition)
            except TypeError:
                pass
            return


    # def stop_voice_recognition(self):
    #     if self.btn_state:
    #             """Stop voice recognition when button is released."""
    #             result=self.audio_node.stop()
    #             if result:
    #                 self.voice_recognition_active = False
    #                 print("DEBUG: Voice recognition stopped")
    #                 self.voice_text_label.setText(f"Stopped Mic: ")
    #             else:
    #                 print("DEBUG: Failed to stop voice recognition")
    #                 time.sleep(0.2)
            
    #     if self.btn_talk.isUp():
    #             self.check_for_navigation_command(self.last_recognized_text)    

    #             print(f"DEBUG: Final recognized text: {self.last_recognized_text}")
    #             self.btn_state = False
    #             return

        # self.voice_text_label.setText(f"Last Msg: {self.check_for_navigation_command(self.last_recognized_text)}")
        # self.check_for_navigation_command(self.last_recognized_text)

        # if not self.ros_interface:
        #     return
            
        # # Stop audio processing
        # success = self.ros_interface.stop_voice_processing()
        # if success:
        #     print("DEBUG: Voice recognition stopped")
            
        #     # Reset the label style
        #     self.voice_text_label.setStyleSheet("""
        #         QLabel {
        #             background-color: #f0f0f0;
        #             padding: 8px;
        #             border-radius: 4px;
        #             border: 1px solid #ddd;
        #             min-height: 20px;
        #         }
        #     """)
            
        #     # Get the final recognized text
        #     voice_command = self.ros_interface.current_voice_text()
        #     if voice_command and voice_command != "No voice text available":
        #         self.voice_text_label.setText(f"✅ Recognized: {voice_command}")
                
        #         # Check if any product or waypoint name is in the voice command
        #         # and navigate there automatically
        #         self.process_voice_command(voice_command)
        #     else:
        #         self.voice_text_label.setText("❓ No speech detected")
        # else:
        #     self.voice_text_label.setText("❌ Error stopping recognition")
        #     print("DEBUG: Failed to stop voice recognition")
    
    def update_voice_text(self, text):
        """Update the voice text label with real-time recognition results."""
        if text:
            # self.voice_text_label.setText(f"Hearing: {text}")
            # print(f"Voice recognition update: {text}")
            # self.check_for_navigation_command(text)
            self.last_recognized_text = text
            
            # If currently pressing the button, we can immediately check for navigation commands
            if self.btn_talk.isDown():
                self.voice_text_label.setText(f"Navigation command received: {text}")
                self.last_recognized_text = text
                self.check_for_navigation_command(self.last_recognized_text)   
                self.btn_state = True
            # if self.btn_state:
               
            #     if self.btn_talk.isUp():
            #         self.btn_state = False
            
                

    def check_for_navigation_command(self, text):
        """Check text for navigation commands in real-time."""
        text = text.lower()
        
        # Quick check for navigation intent
        navigation_triggers = ["go to", "take me to", "navigate to", "move to", "find"]
        if any(trigger in text for trigger in navigation_triggers):
            # Found potential navigation command, highlight it in the UI
            self.voice_text_label.setStyleSheet("""
                QLabel {
                    background-color: #e3f2fd;
                    padding: 8px;
                    border-radius: 4px;
                    border: 1px solid #2196F3;
                    min-height: 20px;
                    color: #0d47a1;
                }
            """)
            
            # We don't actually navigate yet - wait for button release
            # But we can indicate that a navigation command was detected
            self.voice_text_label.setText(f"Hearing navigation command: {text}")
            self.process_voice_command(text)
    
    def process_voice_command(self, command):
        """Process voice commands to navigate to waypoints using NLP matching."""
        command = command.lower()
        print(f"DEBUG: Processing voice command: {command}")
        
        # Define navigation trigger phrases for NLP matching
        navigation_triggers = ["go to", "take me to", "navigate to", "move to", "find", "head to", "travel to"]
        
        # Check if command contains navigation intent
        is_navigation_command = any(trigger in command for trigger in navigation_triggers)
        
        if is_navigation_command:
            print(f"DEBUG: Navigation command detected in: {command}")
            
            # Look for matching waypoint names in the command
            found_waypoint = None
            for waypoint_name in self.waypoints.keys():
                # Check if waypoint name appears in the command (case-insensitive)
                if waypoint_name.lower() in command:
                    found_waypoint = waypoint_name
                    print(f"DEBUG: Found waypoint '{waypoint_name}' in command")
                    break
            
            if found_waypoint:
                # Update UI to show command was processed
                self.voice_text_label.setText(f"✅ Navigating to: {found_waypoint}")
                self.voice_text_label.setStyleSheet("""
                    QLabel {
                        background-color: #c8e6c9;
                        padding: 8px;
                        border-radius: 4px;
                        border: 1px solid #4caf50;
                        min-height: 20px;
                        color: #1b5e20;
                    }
                """)
                
                # Call goto_voice_waypoint with the detected waypoint
                print(f"DEBUG: Calling goto_voice_waypoint with: {found_waypoint}")
                self.goto_voice_waypoint(found_waypoint)
                
                # Reset the UI after a delay to allow for new voice commands
                QTimer.singleShot(3000, self.reset_voice_ui_for_new_command)
                return
            
            # If no matching waypoint found
            self.voice_text_label.setText(f"❌ No matching waypoint found for: {command}")
            self.voice_text_label.setStyleSheet("""
                QLabel {
                    background-color: #ffcdd2;
                    padding: 8px;
                    border-radius: 4px;
                    border: 1px solid #f44336;
                    min-height: 20px;
                    color: #c62828;
                }
            """)
            print("DEBUG: No matching waypoint found in command")
            
            # Reset the UI after a delay even for failed commands
            QTimer.singleShot(3000, self.reset_voice_ui_for_new_command)
        else:
            # Not a navigation command
            print(f"DEBUG: No navigation intent detected in: {command}")
    
    def reset_voice_ui_for_new_command(self):
        """Reset the voice UI to ready state for new commands."""
        # Reset label text and style to default/ready state
        self.voice_text_label.setText("Press the button and speak")
        self.voice_text_label.setStyleSheet("""
            QLabel {
                background-color: #f0f0f0;
                padding: 8px;
                border-radius: 4px;
                border: 1px solid #ccc;
                min-height: 20px;
                color: #333;
            }
        """)
        
        # Reset any internal state variables
        self.last_recognized_text = None
        self.btn_state = False
        
        print("DEBUG: Voice UI reset for new commands")

    def add_manual_waypoint(self):
        """Add manually entered waypoint."""
        name = self.manual_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Invalid Name", "Please enter a waypoint name!")
            return
            
        if name in self.waypoints:
            reply = QMessageBox.question(
                self, "Waypoint Exists", 
                f"Waypoint '{name}' already exists. Overwrite?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
            
        x = self.x_input.value()
        y = self.y_input.value()
        
        self.waypoints[name] = {"x": x, "y": y}
        self.manual_name.clear()
        self.x_input.setValue(0.0)
        self.y_input.setValue(0.0)
        self.update_waypoint_list()
        self.save_waypoints()
        QMessageBox.information(self, "Waypoint Added", f"Waypoint '{name}' added manually!\\n\\nPosition: ({x:.3f}, {y:.3f})")
        
        # Log event
        if hasattr(self.parent(), 'status_widget'):
            self.parent().status_widget.log_system_event("waypoint_added", f"{name} at ({x:.3f}, {y:.3f})")
            
    # def goto_selected_waypoint(self):
    #     """Navigate to selected waypoint."""
    #     selected_items = self.waypoint_list.selectedItems()
    #     if not selected_items:
    #         QMessageBox.warning(self, "No Selection", "Please select a waypoint from the list!")
    #         return
            
    #     waypoint_name = selected_items[0].data(Qt.ItemDataRole.UserRole)
        
    #     if waypoint_name in self.waypoints:
    #         coords = self.waypoints[waypoint_name]
            
    #         if not self.ros_interface:
    #             QMessageBox.information(self, "Simulation Mode", f"Would navigate to '{waypoint_name}' at ({coords['x']:.3f}, {coords['y']:.3f})")
    #             return
                
    #         success = self.ros_interface.navigate_to_point(coords["x"], coords["y"])
    #         if success:
    #             QMessageBox.information(self, "Navigation Started", f"🎯 Navigating to '{waypoint_name}'\\n\\nTarget: ({coords['x']:.3f}, {coords['y']:.3f})\\n\\nMonitor progress in RViz.")
                
    #             # Log event
    #             if hasattr(self.parent(), 'status_widget'):
    #                 self.parent().status_widget.log_navigation_event("goal_set", f"{waypoint_name} at ({coords['x']:.3f}, {coords['y']:.3f})")
    #         else:
    #             QMessageBox.warning(self, "Navigation Error", "Failed to send navigation goal!\\n\\nEnsure navigation system is running.")


    def goto_selected_waypoint(self):
        """Navigate to multiple selected waypoints with user confirmation."""
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "No Selection", "Please select one or more waypoints from the list!")
            return

        # Get the list of waypoint names in order
        waypoint_names = [item.data(Qt.ItemDataRole.UserRole) for item in selected_items if item.data(Qt.ItemDataRole.UserRole) in self.waypoints]
        print(f"DEBUG: Selected waypoints: {waypoint_names}")
        
        if not waypoint_names:
            QMessageBox.warning(self, "No Valid Waypoints", "No valid waypoints selected!")
            return

        for idx, waypoint_name in enumerate(waypoint_names):
            coords = self.waypoints[waypoint_name]
            print(f"DEBUG: Processing waypoint '{waypoint_name}' at {coords}")
            
            if not self.ros_interface:
                QMessageBox.information(self, "Simulation Mode", f"Would navigate to '{waypoint_name}' at ({coords['x']:.3f}, {coords['y']:.3f})")
                continue

            # Send navigation goal
            print(f"DEBUG: Sending navigation goal to ({coords['x']:.3f}, {coords['y']:.3f})")
            success = self.ros_interface.navigate_to_point(coords["x"], coords["y"])
            goal_status, is_active = self.ros_interface.get_goal_status()
            print(f"DEBUG: navigate_to_point returned: {success}")
            print(f"DEBUG: Goal status: {goal_status}, Active: {is_active}")
            
            if not success:
                QMessageBox.warning(self, "Navigation Error", f"Failed to send navigation goal for '{waypoint_name}'!\n\nEnsure navigation system is running.")
                break
            
            # Inform user that navigation has started
            status_message = self.ros_interface.get_goal_status_string()
            QMessageBox.information(self, "Navigation Started", f"🎯 {status_message}\n\nTarget: '{waypoint_name}' at ({coords['x']:.3f}, {coords['y']:.3f})\n\nWaiting for robot to reach destination...")
            
            # Wait for goal to be reached using QEventLoop
            loop = QEventLoop()
            def on_goal_reached():
                print("DEBUG: Goal reached signal received!")
                final_status, _ = self.ros_interface.get_goal_status()
                print(f"DEBUG: Final goal status: {final_status}")
                loop.quit()
            
            self.ros_interface.goal_reached.connect(on_goal_reached)
            print("DEBUG: Waiting for goal to be reached...")
            loop.exec_()  # Wait here until goal_reached is emitted
            self.ros_interface.goal_reached.disconnect(on_goal_reached)
            print(f"DEBUG: Finished waiting for goal '{waypoint_name}'")

            # Get final status and show result to user
            final_status, _ = self.ros_interface.get_goal_status()
            if final_status == "success":
                result_msg = f"✅ Successfully reached '{waypoint_name}'!"
            elif final_status == "failed":
                result_msg = f"❌ Failed to reach '{waypoint_name}'"
                QMessageBox.warning(self, "Navigation Failed", result_msg)
                break
            elif final_status == "canceled":
                result_msg = f"⏹️ Navigation to '{waypoint_name}' was canceled"
                QMessageBox.information(self, "Navigation Canceled", result_msg)
                break
            else:
                result_msg = f"⚠️ Navigation to '{waypoint_name}' completed with status: {final_status}"

            # Ask user if they want to continue to next waypoint (only if more waypoints remain)
            if idx < len(waypoint_names) - 1:
                reply = QMessageBox.question(
                    self, "Continue?", f"{result_msg}\n\nMove to next waypoint '{waypoint_names[idx + 1]}'?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                )
                if reply != QMessageBox.StandardButton.Yes:
                    QMessageBox.information(self, "Navigation Stopped", "Multi-waypoint navigation stopped by user.")
                    break
            else:
                # This is the last waypoint
                final_status, _ = self.ros_interface.get_goal_status()
                if final_status == "success":
                    QMessageBox.information(self, "Navigation Complete", f"✅ Successfully reached final waypoint '{waypoint_name}'!\n\nAll selected waypoints have been visited.")
                else:
                    QMessageBox.information(self, "Navigation Complete", f"Navigation completed with status: {final_status}")
                
        print("DEBUG: goto_selected_waypoint completed")

    def goto_voice_waypoint(self, waypoint_name):
            """Navigate to multiple selected waypoints with user confirmation."""
            gotoWaypoints = waypoint_name
            if not gotoWaypoints:
                QMessageBox.warning(self, "No Selection", "Please select one or more waypoints from the list!")
                return

            # Get the list of waypoint names in order
            # waypoint_names = [item.data(Qt.ItemDataRole.UserRole) for item in  if item.data(Qt.ItemDataRole.UserRole) in self.waypoints]
            # print(f"DEBUG: Selected waypoints: {waypoint_names}")
            waypoint_names = [gotoWaypoints] if gotoWaypoints in self.waypoints else []
            
            if not waypoint_names:
                QMessageBox.warning(self, "No Valid Waypoints", "No valid waypoints selected!")
                return

            for idx, waypoint_name in enumerate(waypoint_names):
                coords = self.waypoints[waypoint_name]
                print(f"DEBUG: Processing waypoint '{waypoint_name}' at {coords}")
                
                if not self.ros_interface:
                    QMessageBox.information(self, "Simulation Mode", f"Would navigate to '{waypoint_name}' at ({coords['x']:.3f}, {coords['y']:.3f})")
                    continue

                # Send navigation goal
                print(f"DEBUG: Sending navigation goal to ({coords['x']:.3f}, {coords['y']:.3f})")
                success = self.ros_interface.navigate_to_point(coords["x"], coords["y"])
                goal_status, is_active = self.ros_interface.get_goal_status()
                print(f"DEBUG: navigate_to_point returned: {success}")
                print(f"DEBUG: Goal status: {goal_status}, Active: {is_active}")
                
                if not success:
                    QMessageBox.warning(self, "Navigation Error", f"Failed to send navigation goal for '{waypoint_name}'!\n\nEnsure navigation system is running.")
                    break
                
                # Inform user that navigation has started
                status_message = self.ros_interface.get_goal_status_string()
                QMessageBox.information(self, "Navigation Started", f"🎯 {status_message}\n\nTarget: '{waypoint_name}' at ({coords['x']:.3f}, {coords['y']:.3f})\n\nWaiting for robot to reach destination...")
                
                # Wait for goal to be reached using QEventLoop
                loop = QEventLoop()
                def on_goal_reached():
                    print("DEBUG: Goal reached signal received!")
                    final_status, _ = self.ros_interface.get_goal_status()
                    print(f"DEBUG: Final goal status: {final_status}")
                    loop.quit()
                
                self.ros_interface.goal_reached.connect(on_goal_reached)
                print("DEBUG: Waiting for goal to be reached...")
                loop.exec_()  # Wait here until goal_reached is emitted
                self.ros_interface.goal_reached.disconnect(on_goal_reached)
                print(f"DEBUG: Finished waiting for goal '{waypoint_name}'")

                # Get final status and show result to user
                final_status, _ = self.ros_interface.get_goal_status()
                if final_status == "success":
                    result_msg = f"✅ Successfully reached '{waypoint_name}'!"
                elif final_status == "failed":
                    result_msg = f"❌ Failed to reach '{waypoint_name}'"
                    QMessageBox.warning(self, "Navigation Failed", result_msg)
                    break
                elif final_status == "canceled":
                    result_msg = f"⏹️ Navigation to '{waypoint_name}' was canceled"
                    QMessageBox.information(self, "Navigation Canceled", result_msg)
                    break
                else:
                    result_msg = f"⚠️ Navigation to '{waypoint_name}' completed with status: {final_status}"

                # Ask user if they want to continue to next waypoint (only if more waypoints remain)
                if idx < len(waypoint_names) - 1:
                    reply = QMessageBox.question(
                        self, "Continue?", f"{result_msg}\n\nMove to next waypoint '{waypoint_names[idx + 1]}'?",
                        QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
                    )
                    if reply != QMessageBox.StandardButton.Yes:
                        QMessageBox.information(self, "Navigation Stopped", "Multi-waypoint navigation stopped by user.")
                        break
                else:
                    # This is the last waypoint
                    final_status, _ = self.ros_interface.get_goal_status()
                    if final_status == "success":
                        QMessageBox.information(self, "Navigation Complete", f"✅ Successfully reached final waypoint '{waypoint_name}'!\n\nAll selected waypoints have been visited.")
                    else:
                        QMessageBox.information(self, "Navigation Complete", f"Navigation completed with status: {final_status}")
                    
            print("DEBUG: goto_selected_waypoint completed")


    def edit_selected_waypoint(self):
        """Edit selected waypoint."""
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "No Selection", "Please select a waypoint to edit!")
            return
            
        waypoint_name = selected_items[0].data(Qt.ItemDataRole.UserRole)
        
        if waypoint_name in self.waypoints:
            coords = self.waypoints[waypoint_name]
            self.manual_name.setText(waypoint_name)
            self.x_input.setValue(coords["x"])
            self.y_input.setValue(coords["y"])
            
    def delete_selected_waypoint(self):
        """Delete selected waypoint."""
        selected_items = self.waypoint_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "No Selection", "Please select a waypoint to delete!")
            return
            
        waypoint_name = selected_items[0].data(Qt.ItemDataRole.UserRole)
        
        reply = QMessageBox.question(
            self, "Delete Waypoint", 
            f"Are you sure you want to delete waypoint '{waypoint_name}'?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes and waypoint_name in self.waypoints:
            del self.waypoints[waypoint_name]
            self.update_waypoint_list()
            self.save_waypoints()
            QMessageBox.information(self, "Waypoint Deleted", f"Waypoint '{waypoint_name}' has been deleted.")
            
            # Log event
            if hasattr(self.parent(), 'status_widget'):
                self.parent().status_widget.log_system_event("waypoint_deleted", waypoint_name)
                
    def cancel_navigation(self):
        """Cancel current navigation goal."""
        if self.ros_interface:
            success = self.ros_interface.cancel_navigation()
            if success:
                QMessageBox.information(self, "Navigation Cancelled", "Current navigation goal has been cancelled.")
                
                # Log event
                if hasattr(self.parent(), 'status_widget'):
                    self.parent().status_widget.log_navigation_event("goal_cancelled")
            else:
                QMessageBox.warning(self, "Cancel Error", "Failed to cancel navigation goal.")
        else:
            QMessageBox.information(self, "Simulation Mode", "Would cancel current navigation goal.")
            
    def clear_costmaps(self):
        """Clear costmaps to resolve navigation issues."""
        if self.ros_interface:
            success = self.ros_interface.clear_costmaps()
            if success:
                QMessageBox.information(self, "Costmaps Cleared", "Navigation costmaps have been cleared successfully!")
            else:
                QMessageBox.warning(self, "Clear Error", "Failed to clear costmaps.")
        else:
            QMessageBox.information(self, "Simulation Mode", "Would clear navigation costmaps.")
            
    def __del__(self):
        """Clean up resources when the widget is destroyed."""
        try:
            # Terminate the product navigator if it's running
            if hasattr(self, 'product_navigator_process') and self.product_navigator_process:
                print("Terminating product navigator process")
                self.product_navigator_process.terminate()
                self.product_navigator_process.wait(timeout=1.0)
        except Exception as e:
            print(f"Error cleaning up NavigationWidget resources: {e}")
            
    def shutdown(self):
        """Explicitly shutdown all processes started by this widget."""
        try:
            # Stop SLAM process if running
            if self.slam_process:
                self.slam_process.terminate()
                self.slam_process = None
                
            # Stop navigation process if running
            if self.nav_process:
                self.nav_process.terminate()
                self.nav_process = None
                
            # Terminate the product navigator if it's running
            if hasattr(self, 'product_navigator_process') and self.product_navigator_process:
                self.product_navigator_process.terminate()
                self.product_navigator_process = None
                
            print("NavigationWidget shutdown complete")
        except Exception as e:
            print(f"Error during NavigationWidget shutdown: {e}")
            import traceback

            traceback.print_exc()