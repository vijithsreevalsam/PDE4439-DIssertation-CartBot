import sys
import subprocess
from PySide6.QtWidgets import (QMainWindow, QPushButton, QVBoxLayout, QWidget, 
                               QLabel, QMessageBox)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from .control_window import ControlWindow

class LauncherWindow(QMainWindow):
    """Initial launcher window that starts RViz and opens control panel."""
    
    def __init__(self):
        super().__init__()
        self.rviz_process = None
        self.setup_ui()
        
    def setup_ui(self):
        """Setup the launcher UI."""
        self.setWindowTitle("🤖 Robot Control Launcher")
        self.setGeometry(100, 80, 500, 300) 
        
        # Set window style
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 15px;
                font-size: 14px;
                border-radius: 8px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QLabel {
                color: #333;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        layout.setSpacing(20)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Title
        title_label = QLabel("🤖 Robot Control System")
        title_font = QFont()
        title_font.setPointSize(24)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        title_label.setStyleSheet("color: #2c3e50; margin: 20px;")
        layout.addWidget(title_label)
        
        # Description
        desc_label = QLabel("Complete robot teleoperation and navigation control")
        desc_font = QFont()
        desc_font.setPointSize(12)
        desc_label.setFont(desc_font)
        desc_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        desc_label.setStyleSheet("color: #7f8c8d; margin-bottom: 30px;")
        layout.addWidget(desc_label)
        
        # Features list
        features_label = QLabel("""
        ✅ RViz Visualization
        ✅ Teleoperation Control  
        ✅ Navigation & SLAM
        ✅ Waypoint Management
        ✅ Real-time Status Monitoring
        """)
        features_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        features_label.setStyleSheet("color: #34495e; font-size: 11px; margin: 10px;")
        layout.addWidget(features_label)
        
        # Launch button
        self.launch_button = QPushButton("🚀 Launch Robot Control Center")
        self.launch_button.setMinimumHeight(60)
        self.launch_button.clicked.connect(self.launch_control_center)
        layout.addWidget(self.launch_button)
        
        # Info label
        info_label = QLabel("This will start RViz and open the control panel")
        info_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        info_label.setStyleSheet("color: #95a5a6; font-size: 10px; margin-top: 10px;")
        layout.addWidget(info_label)
        
    def launch_control_center(self):
        """Launch RViz and open the main control window."""
        try:
            # First launch RViz
            self.launch_rviz()
            
            # Then open control window
            self.control_window = ControlWindow()
            self.control_window.show()
            
            # Close launcher
            self.close()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to launch control center: {str(e)}")
    
    def launch_rviz(self):
        """Launch RViz with navigation configuration."""
        rviz_config = "/home/viju/my_rviz_config.rviz"  # Default config
        
        # Try different possible RViz config locations
        possible_configs = [
            "/home/viju/my_rviz_config.rviz",
            "/home/viju/ros_ws/src/cartBot/cartBot_navigation/rviz/cartBot_navigation.rviz",
            "/opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz"
        ]
        
        config_to_use = None
        for config in possible_configs:
            import os
            if os.path.exists(config):
                config_to_use = config
                break
        
        try:
            if config_to_use:
                self.rviz_process = subprocess.Popen(["rviz2", "-d", config_to_use])
            else:
                self.rviz_process = subprocess.Popen(["rviz2"])
                
            print("RViz launched successfully!")
            
        except Exception as e:
            # RViz launch failed, but continue anyway
            print(f"Warning: Could not launch RViz: {e}")
            QMessageBox.warning(
                self, 
                "RViz Warning", 
                f"Could not launch RViz automatically: {e}\n\nYou can launch it manually later."
            )
    
    def closeEvent(self, event):
        """Handle window close event."""
        # Don't terminate RViz when closing launcher - let control window handle it
        event.accept()