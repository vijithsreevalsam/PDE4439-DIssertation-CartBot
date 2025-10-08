import sys
import os
import subprocess
from PySide6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                               QTabWidget, QLabel, QFrame, QMessageBox, 
                               QPushButton, QSplitter)
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont

from ..widgets.teleop_widget import TeleopWidget
from ..widgets.status_widget import StatusWidget
from ..widgets.navigation_widget import NavigationWidget
# from ..widgets.map_widget import MapWidget
from ..ros.ros_interface import ROSInterface

class ControlWindow(QMainWindow):
    """Main robot control window with all functionalities."""
    
    def __init__(self):
        super().__init__()
        self.rviz_process = None
        
        # Initialize ROS interface
        try:
            self.ros_interface = ROSInterface()
        except Exception as e:
            print(f"Warning: Could not initialize ROS interface: {e}")
            self.ros_interface = None
        
        self.setup_ui()
        
        # Setup timer for status updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_status)
        self.update_timer.start(100)  # Update every 100ms
        
    def setup_ui(self):
        """Setup the main control UI."""
        self.setWindowTitle("🤖 Robot Control Center")
        self.setGeometry(100, 100, 400, 300)  # x, y, width, height
        
        # Set window style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QTabWidget::pane {
                border: 1px solid #c0c0c0;
                background-color: white;
            }
            QTabWidget::tab-bar {
                alignment: center;
            }
            QTabBar::tab {
                background-color: #e0e0e0;
                padding: 10px 20px;
                margin: 2px;
                border-radius: 5px;
            }
            QTabBar::tab:selected {
                background-color: #4CAF50;
                color: white;
            }
            QTabBar::tab:hover {
                background-color: #45a049;
                color: white;
            }
            QFrame {
                background-color: white;
                border-radius: 5px;
            }
            QPushButton {
                background-color: #2196F3;
                color: white;
                border: none;
                padding: 16px 16px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0d47a1;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout with splitter
        main_layout = QHBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Left panel - Controls
        left_panel = self.create_control_panel()
        splitter.addWidget(left_panel)
        
        # Right panel - Map and visualization
        right_panel = self.create_visualization_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter proportions
        splitter.setSizes([100, 1000])
        
        main_layout.addWidget(splitter)
        
    def create_control_panel(self):
        """Create the left control panel."""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        panel.setMaximumWidth(450)
        layout = QVBoxLayout(panel)
        
        # Title
        title_label = QLabel("🎮 Robot Controls")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setStyleSheet("color: #2c3e50; margin: 10px; padding: 10px;")
        layout.addWidget(title_label)
        
        # Control tabs
        self.control_tabs = QTabWidget()
        
        self.navigation_widget = NavigationWidget(self.ros_interface)
        
        # Create widgets (they'll handle their own ROS interface checking)
        self.teleop_widget = TeleopWidget(self.ros_interface, self.navigation_widget)
        self.control_tabs.addTab(self.teleop_widget, "🎮 Teleop")
        self.control_tabs.addTab(self.navigation_widget, "🧭 Navigation")
        
        
        self.status_widget = StatusWidget(self.ros_interface)
        self.control_tabs.addTab(self.status_widget, "📊 Status")
        
        layout.addWidget(self.control_tabs)
        
        # Emergency stop button
        emergency_frame = QFrame()
        emergency_layout = QVBoxLayout(emergency_frame)
        
        self.emergency_stop_btn = QPushButton("🛑 EMERGENCY STOP")
        self.emergency_stop_btn.setMinimumHeight(50)
        self.emergency_stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                font-size: 16px;
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
        self.emergency_stop_btn.clicked.connect(self.emergency_stop)
        emergency_layout.addWidget(self.emergency_stop_btn)
        
        layout.addWidget(emergency_frame)
        
        return panel
        
    def create_visualization_panel(self):
        """Create the right visualization panel."""
        panel = QFrame()
        panel.setFrameStyle(QFrame.Shape.StyledPanel)
        layout = QVBoxLayout(panel)
        
        # Title and RViz controls
        header_layout = QHBoxLayout()
        
        title_label = QLabel("🗺️ Map & Visualization")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet("color: #2c3e50;")
        # header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
        # RViz control buttons
        self.launch_rviz_btn = QPushButton("🔍 Launch RViz")
        self.launch_rviz_btn.clicked.connect(self.toggle_rviz)
        # header_layout.addWidget(self.launch_rviz_btn)
        
        self.close_rviz_btn = QPushButton("❌ Close RViz")
        self.close_rviz_btn.clicked.connect(self.close_rviz)
        self.close_rviz_btn.setEnabled(False)
        # header_layout.addWidget(self.close_rviz_btn)
        
        layout.addLayout(header_layout)
        
        # Map widget
        # self.map_widget = MapWidget(self.ros_interface)
        # layout.addWidget(self.map_widget)
        
        return panel
        
    def toggle_rviz(self):
        """Toggle RViz on/off."""
        if self.rviz_process is None or self.rviz_process.poll() is not None:
            self.launch_rviz()
        else:
            self.close_rviz()
            
    def launch_rviz(self):
        """Launch RViz with navigation configuration."""
        # Try different possible RViz config locations
        possible_configs = [
            "/home/viju/my_rviz_config.rviz",
            "/home/viju/ros_ws/src/cartBot/cartBot_navigation/rviz/cartBot_navigation.rviz",
            "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
        ]
        
        config_to_use = None
        for config in possible_configs:
            if os.path.exists(config):
                config_to_use = config
                break
        
        try:
            if config_to_use:
                self.rviz_process = subprocess.Popen(["rviz2", "-d", config_to_use])
            else:
                self.rviz_process = subprocess.Popen(["rviz2"])
                
            self.launch_rviz_btn.setText("🔄 Restart RViz")
            self.close_rviz_btn.setEnabled(True)
            
            QMessageBox.information(
                self, 
                "RViz Launched", 
                "RViz launched successfully!\n\nUse the 2D Nav Goal tool to set navigation targets."
            )
            
        except Exception as e:
            QMessageBox.warning(
                self, 
                "RViz Error", 
                f"Could not launch RViz: {e}\n\nPlease ensure RViz2 is installed:\nsudo apt install ros-jazzy-rviz2"
            )
            
    def close_rviz(self):
        """Close RViz."""
        if self.rviz_process:
            self.rviz_process.terminate()
            self.rviz_process.wait()
            self.rviz_process = None
            
        self.launch_rviz_btn.setText("🔍 Launch RViz")
        self.close_rviz_btn.setEnabled(False)
        
    def emergency_stop(self):
        """Emergency stop - halt all robot movement."""
        if self.ros_interface:
            self.ros_interface.emergency_stop()
        
        # Also stop any running processes
        self.teleop_widget.emergency_stop()
        
        QMessageBox.warning(
            self, 
            "Emergency Stop", 
            "🛑 EMERGENCY STOP ACTIVATED!\n\nAll robot movement has been halted."
        )
        
    def update_status(self):
        """Update all status displays."""
        if self.status_widget:
            self.status_widget.update_displays()
            
    def closeEvent(self, event):
        """Handle window close event."""
        # Stop update timer
        if self.update_timer:
            self.update_timer.stop()
            
        # Close RViz
        self.close_rviz()
        
        # Shutdown ROS interface
        if self.ros_interface:
            self.ros_interface.shutdown()
            
        event.accept()